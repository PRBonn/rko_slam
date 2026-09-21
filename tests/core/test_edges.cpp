#include <algorithm>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <g2o/core/jacobian_workspace.h>
#include <g2o/core/optimizable_graph.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <sophus/se3.hpp>

#include "rko_slam/core/edges.hpp"
#include "rko_slam/core/pose_graph.hpp"
#include "rko_slam/core/slam.hpp"
#include "rko_slam/core/sub_map.hpp"
#include "rko_slam/core/sub_map_builder.hpp"

using Catch::Matchers::WithinAbs;
using rko_slam::core::EdgeGravity;
using rko_slam::core::FinishedSubMap;
using rko_slam::core::ImuSample;
using rko_slam::core::KeyposeId;
using rko_slam::core::Nsec;
using rko_slam::core::PoseGraph;
using rko_slam::core::SLAM;
using rko_slam::core::SubMap;
using rko_slam::core::SubMapBuilder;
using rko_slam::core::to_isometry;

namespace {

constexpr double kGravity = 9.81;

// Deterministic poses with rotations up to about 1.2 rad about varying axes.
Sophus::SE3d pose_at(const int index) {
  const double phase = 0.37 * static_cast<double>(index);
  const Eigen::Vector3d rotation(1.2 * std::sin(phase), 0.9 * std::cos(1.3 * phase), 0.7 * std::sin(2.1 * phase));
  const Eigen::Vector3d translation(20.0 * std::cos(phase), -15.0 * std::sin(0.7 * phase), 3.0 * std::cos(phase));
  return {Sophus::SO3d::exp(rotation), translation};
}

// The edge's own Jacobian; g2o maps it onto a workspace the optimizer otherwise provides.
template <typename Edge>
Eigen::MatrixXd analytic_jacobian(Edge& edge) {
  g2o::JacobianWorkspace workspace;
  workspace.updateSize(&edge);
  workspace.allocate();
  g2o::OptimizableGraph::Edge& graph_edge = edge;
  graph_edge.linearizeOplus(workspace);
  return edge.template jacobianOplusXn<0>();
}

// Central differences of the edge's error over g2o's increment of the vertex.
template <typename Edge>
Eigen::MatrixXd numeric_jacobian(Edge& edge, g2o::VertexSE3& vertex) {
  constexpr double kStep = 1e-6;
  Eigen::MatrixXd jacobian(edge.dimension(), 6);
  for (int column = 0; column < 6; ++column) {
    Eigen::Matrix<double, 6, 1> delta = Eigen::Matrix<double, 6, 1>::Zero();
    std::array<Eigen::VectorXd, 2> errors;
    for (int side = 0; side < 2; ++side) {
      delta(column) = side == 0 ? kStep : -kStep;
      vertex.push();
      vertex.oplus(delta.data());
      edge.computeError();
      errors.at(side) = edge.error();
      vertex.pop();
    }
    jacobian.col(column) = (errors.at(0) - errors.at(1)) / (2.0 * kStep);
  }
  return jacobian;
}

double tilt(const Sophus::SE3d& pose) {
  return std::acos(std::clamp((pose.so3() * Eigen::Vector3d::UnitZ()).z(), -1.0, 1.0));
}

} // namespace

TEST_CASE("gravity edge: the analytic Jacobian matches central differences", "[gravity]") {
  for (int index = 0; index < 40; ++index) {
    g2o::VertexSE3 vertex;
    vertex.setEstimate(to_isometry(pose_at(index)));
    EdgeGravity edge(3.0 * (pose_at(index + 11).so3() * Eigen::Vector3d::UnitZ()));
    edge.setVertex(0, &vertex);
    const Eigen::MatrixXd analytic = analytic_jacobian(edge);
    CAPTURE(index);
    CHECK((analytic - numeric_jacobian(edge, vertex)).cwiseAbs().maxCoeff() < 1e-6);
  }
}

TEST_CASE("gravity edge: zero for the keypose's own up axis, whatever its length and the heading", "[gravity]") {
  const Sophus::SE3d pose = pose_at(3);
  const Eigen::Vector3d up_in_keypose = pose.so3().inverse() * Eigen::Vector3d::UnitZ();
  g2o::VertexSE3 vertex;
  EdgeGravity edge(4.2 * up_in_keypose);
  edge.setVertex(0, &vertex);
  for (const double heading : {0.0, 0.9, -2.5}) {
    vertex.setEstimate(to_isometry(Sophus::SE3d(Sophus::SO3d::rotZ(heading), Eigen::Vector3d::Zero()) * pose));
    edge.computeError();
    CHECK(edge.error().norm() < 1e-12);
  }
  // a tilt shows, as sin(tilt) in the tangent plane
  vertex.setEstimate(to_isometry(Sophus::SE3d(Sophus::SO3d::rotX(0.2), Eigen::Vector3d::Zero()) * pose));
  edge.computeError();
  CHECK_THAT(edge.error().norm(), WithinAbs(std::sin(0.2), 1e-9));
}

TEST_CASE("pose_graph: gravity levels a map started tilted, keeping the first position and heading", "[gravity]") {
  constexpr KeyposeId kKeyposes = 5;
  constexpr double kHeading = 0.3;
  const Sophus::SE3d start(Sophus::SO3d::rotZ(kHeading) * Sophus::SO3d::rotX(0.09) * Sophus::SO3d::rotY(-0.05),
                           Eigen::Vector3d(4.0, -2.0, 1.0));
  // the true relative poses are level
  const auto level_step = [](const KeyposeId keypose_id) {
    return Sophus::SE3d(Sophus::SO3d::rotZ(0.1 * static_cast<double>(keypose_id)), Eigen::Vector3d(10.0, 0.0, 0.0));
  };

  PoseGraph pose_graph{PoseGraph::Config{.max_iterations = 50}};
  Sophus::SE3d pose = start;
  pose_graph.add_keypose(0, pose);
  pose_graph.add_gauge_edge(0);
  for (KeyposeId keypose_id = 0; keypose_id < kKeyposes; ++keypose_id) {
    pose_graph.add_gravity_edge(keypose_id, Eigen::Vector3d(0.0, 0.0, kGravity));
    if (keypose_id + 1 < kKeyposes) {
      pose = pose * level_step(keypose_id);
      pose_graph.add_keypose(keypose_id + 1, pose);
      pose_graph.add_odom_edge(keypose_id, keypose_id + 1, level_step(keypose_id));
    }
  }
  REQUIRE(pose_graph.se3_edges().size() == kKeyposes - 1);
  REQUIRE(pose_graph.optimize());

  const Sophus::SE3d first = pose_graph.get_keypose(0);
  CHECK((first.translation() - start.translation()).norm() < 1e-6);
  const Eigen::Vector3d forward = first.so3() * Eigen::Vector3d::UnitX();
  // a heading under two tilts is defined up to their product
  CHECK_THAT(std::atan2(forward.y(), forward.x()), WithinAbs(kHeading, 0.09 * 0.05));
  Sophus::SE3d expected = first;
  for (KeyposeId keypose_id = 0; keypose_id < kKeyposes; ++keypose_id) {
    const Sophus::SE3d keypose = pose_graph.get_keypose(keypose_id);
    CAPTURE(keypose_id);
    CHECK(tilt(keypose) < 1e-6);
    CHECK((keypose.inverse() * expected).log().norm() < 1e-6);
    expected = expected * level_step(keypose_id);
  }
}

TEST_CASE("pose_graph: the gauge holds position and heading, and leaves roll and pitch to gravity", "[gravity]") {
  const Eigen::Vector3d translation(4.0, -2.0, 1.0);
  const Sophus::SO3d level = Sophus::SO3d::rotZ(0.3);
  const Sophus::SO3d tilted = Sophus::SO3d::rotZ(0.3) * Sophus::SO3d::rotX(0.09) * Sophus::SO3d::rotY(-0.05);
  const auto heading = [](const Sophus::SE3d& pose) {
    const Eigen::Vector3d forward = pose.so3() * Eigen::Vector3d::UnitX();
    return std::atan2(forward.y(), forward.x());
  };

  // an anchor with no tilt is the case where the yaw axis coincides with the map's, and a wrong projection still
  // looks right; the tilted one is the case that separates them
  for (const Sophus::SO3d& rotation : {level, tilted}) {
    const Sophus::SE3d anchor(rotation, translation);
    CAPTURE(tilt(anchor));
    const Eigen::Vector3d measured_up =
        kGravity * (Sophus::SO3d::rotX(0.05) * Sophus::SO3d::rotY(-0.04) * Eigen::Vector3d::UnitZ());

    PoseGraph pose_graph{PoseGraph::Config{.max_iterations = 50}};
    pose_graph.add_keypose(0, anchor);
    pose_graph.add_gauge_edge(0);
    pose_graph.add_gravity_edge(0, measured_up);
    REQUIRE(pose_graph.optimize());
    const Sophus::SE3d solved = pose_graph.get_keypose(0);

    CHECK((solved.translation() - translation).norm() < 1e-6);
    // roll and pitch were free: the map's up in the keypose frame ends on the measured one
    CHECK((solved.so3().inverse() * Eigen::Vector3d::UnitZ() - measured_up.normalized()).norm() < 1e-4);
    // a heading under two tilts is defined up to their product
    CHECK_THAT(heading(solved), WithinAbs(heading(anchor), 0.09 * 0.05));
  }
}

TEST_CASE("pose_graph: a saved graph carries its measured up directions back", "[gravity]") {
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() / "rko_slam_test_graph_round_trip" / "graph.g2o";
  std::filesystem::remove_all(path.parent_path());
  std::filesystem::create_directories(path.parent_path());
  // g2o's text format writes doubles at the default 6 significant digits
  constexpr double kTextPrecision = 1e-5;
  const std::array<Eigen::Vector3d, 3> ups = {
      Eigen::Vector3d(0.3141592653, -0.1234567891, 9.8066499123),
      Eigen::Vector3d(0.0, 0.0, kGravity),
      Eigen::Vector3d(-0.4567891234, 0.2718281828, 9.7912345678),
  };

  PoseGraph written{PoseGraph::Config{}};
  Sophus::SE3d pose(Sophus::SO3d::rotZ(0.2) * Sophus::SO3d::rotX(0.05), Eigen::Vector3d(1.0, 2.0, 3.0));
  for (KeyposeId keypose_id = 0; keypose_id < ups.size(); ++keypose_id) {
    written.add_keypose(keypose_id, pose);
    written.add_gravity_edge(keypose_id, ups.at(keypose_id));
    if (keypose_id + 1 < ups.size()) {
      const Sophus::SE3d step(Sophus::SO3d::rotZ(0.1), Eigen::Vector3d(5.0, 0.0, 0.0));
      written.add_odom_edge(keypose_id, keypose_id + 1, step);
      pose = pose * step;
    }
  }
  written.add_gauge_edge(0);
  REQUIRE(written.save(path));

  PoseGraph read{PoseGraph::Config{}};
  REQUIRE(read.load(path));
  const std::vector<PoseGraph::GravityEdgeView> back = read.gravity_edges();
  REQUIRE(back.size() == ups.size());
  for (const PoseGraph::GravityEdgeView& gravity : back) {
    CAPTURE(gravity.keypose_id);
    CHECK((gravity.measured_up - ups.at(gravity.keypose_id)).norm() < kTextPrecision * kGravity);
  }
  std::filesystem::remove_all(path.parent_path());
}

TEST_CASE("SLAM: the first gravity edge of the run takes the first keypose off its fixed anchor", "[gravity]") {
  const rko_slam::core::VoxelHashMap::Config voxel_map_config{.voxel_size = 0.5F, .max_points_per_voxel = 20};
  const SLAM::Config slam_config;
  SLAM slam(slam_config, voxel_map_config);

  // enough geometry that closure detection has something to project
  std::vector<Eigen::Vector3f> cloud;
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      for (int k = 0; k < 6; ++k) {
        cloud.emplace_back(0.1F * static_cast<float>(i), 0.1F * static_cast<float>(j), 0.1F * static_cast<float>(k));
      }
    }
  }

  const auto sub_map_at = [](const KeyposeId keypose_id, const std::optional<Eigen::Vector3f>& measured_up) {
    auto sub_map = std::make_unique<SubMap>();
    sub_map->id = keypose_id;
    sub_map->odom_T_keypose = Sophus::SE3f::transX(static_cast<float>(keypose_id));
    sub_map->odom_T_next_keypose = Sophus::SE3f::transX(static_cast<float>(keypose_id) + 1.0F);
    sub_map->local_trajectory = {Sophus::SE3f{}};
    sub_map->scan_times = {Nsec{keypose_id}};
    sub_map->measured_up = measured_up;
    return sub_map;
  };

  slam.process_finished_sub_map(sub_map_at(0, std::nullopt), cloud);
  CHECK(slam.pose_graph.is_keypose_fixed(0));
  CHECK(slam.pose_graph.gravity_edges().empty());

  slam.process_finished_sub_map(sub_map_at(1, Eigen::Vector3f(0.0F, 0.0F, static_cast<float>(kGravity))), cloud);
  CHECK(!slam.pose_graph.is_keypose_fixed(0));
  REQUIRE(slam.pose_graph.gravity_edges().size() == 1);
  CHECK(slam.pose_graph.gravity_edges().front().keypose_id == 1);
}

TEST_CASE("SubMapBuilder: up is gravity's reaction while the platform accelerates and turns", "[gravity]") {
  // 0.5 * |acceleration| * t^2 crosses 4 m at the 31st scan, which seals the first sub-map
  const Sophus::SO3f odom_R_keypose = Sophus::SO3f::rotZ(0.4F) * Sophus::SO3f::rotY(0.1F);
  const Eigen::Vector3f acceleration(0.8F, -0.3F, 0.1F);
  const Eigen::Vector3f gravity_reaction(0.0F, 0.0F, static_cast<float>(kGravity));
  constexpr float kYawRate = 0.5F;
  constexpr std::int64_t kScanPeriod = 100'000'000;
  constexpr std::int64_t kSamplesPerScan = 10;
  const auto odom_R_base = [&](const std::int64_t nanoseconds) {
    return odom_R_keypose * Sophus::SO3f::rotZ(kYawRate * 1e-9F * static_cast<float>(nanoseconds));
  };

  // each scan interval's samples arrive `delay` scans late, newest first
  for (const std::int64_t delay : {0, 2}) {
    CAPTURE(delay);
    SubMapBuilder builder({.splitting_distance = 4.0F});
    std::optional<Eigen::Vector3f> measured_up;
    for (std::int64_t scan = 0; scan < 40 && !measured_up; ++scan) {
      std::vector<ImuSample> samples;
      for (std::int64_t index = kSamplesPerScan; index >= 1 && scan > delay; --index) {
        const std::int64_t time = ((scan - delay - 1) * kScanPeriod) + (index * kScanPeriod / kSamplesPerScan);
        samples.push_back(
            {.time = Nsec{time}, .specific_force = odom_R_base(time).inverse() * (acceleration + gravity_reaction)});
      }
      const float time = 1e-9F * static_cast<float>(scan * kScanPeriod);
      const Sophus::SE3f odom_T_base(odom_R_base(scan * kScanPeriod), 0.5F * acceleration * time * time);
      for (const ImuSample& sample : samples) {
        builder.add_imu_sample(sample);
      }
      if (auto finished = builder.add_to_live_map({}, Nsec{scan * kScanPeriod}, odom_T_base)) {
        REQUIRE(finished->sub_map->scan_times.size() == 31);
        measured_up = finished->sub_map->measured_up;
        REQUIRE(measured_up.has_value());
      }
    }
    REQUIRE(measured_up.has_value());
    const Eigen::Vector3f expected = odom_R_keypose.inverse() * gravity_reaction;
    CHECK((measured_up.value_or(Eigen::Vector3f::Zero()) - expected).norm() < 1e-3F);
  }
}

TEST_CASE("SubMapBuilder: a run too short to split is still levelled, at finalize", "[gravity]") {
  const Sophus::SO3f odom_R_keypose = Sophus::SO3f::rotZ(0.4F) * Sophus::SO3f::rotY(0.1F);
  const Eigen::Vector3f acceleration(0.8F, -0.3F, 0.1F);
  const Eigen::Vector3f gravity_reaction(0.0F, 0.0F, static_cast<float>(kGravity));
  constexpr float kYawRate = 0.5F;
  constexpr std::int64_t kScanPeriod = 100'000'000;
  constexpr std::int64_t kSamplesPerScan = 10;
  const auto odom_R_base = [&](const std::int64_t nanoseconds) {
    return odom_R_keypose * Sophus::SO3f::rotZ(kYawRate * 1e-9F * static_cast<float>(nanoseconds));
  };

  // 1000 m apart: the drive never reaches a split, so the only sub-map is the one finalize() closes
  SubMapBuilder builder({.splitting_distance = 1000.0F});
  for (std::int64_t scan = 0; scan < 20; ++scan) {
    for (std::int64_t index = 1; index <= kSamplesPerScan && scan > 0; ++index) {
      const std::int64_t time = ((scan - 1) * kScanPeriod) + (index * kScanPeriod / kSamplesPerScan);
      builder.add_imu_sample(
          {.time = Nsec{time}, .specific_force = odom_R_base(time).inverse() * (acceleration + gravity_reaction)});
    }
    const float time = 1e-9F * static_cast<float>(scan * kScanPeriod);
    const Sophus::SE3f odom_T_base(odom_R_base(scan * kScanPeriod), 0.5F * acceleration * time * time);
    REQUIRE(!builder.add_to_live_map({}, Nsec{scan * kScanPeriod}, odom_T_base).has_value());
  }

  const std::optional<FinishedSubMap> finished = builder.finalize();
  REQUIRE(finished.has_value());
  const std::optional<Eigen::Vector3f>& measured_up = finished->sub_map->measured_up;
  REQUIRE(measured_up.has_value());
  const Eigen::Vector3f expected = odom_R_keypose.inverse() * gravity_reaction;
  CHECK((measured_up.value_or(Eigen::Vector3f::Zero()) - expected).norm() < 1e-3F);
}

TEST_CASE("SubMapBuilder: no up before the first scan, without samples, or inside one scan interval", "[gravity]") {
  const ImuSample at_or_before_the_first_scan{.time = Nsec{0}, .specific_force = Eigen::Vector3f(0.0F, 0.0F, 9.81F)};

  SubMapBuilder without_samples({.splitting_distance = 1.0F});
  REQUIRE(!without_samples.add_to_live_map({}, Nsec{0}, Sophus::SE3f{}).has_value());
  without_samples.add_imu_sample(at_or_before_the_first_scan);
  bool has_up = true;
  if (auto finished = without_samples.add_to_live_map({}, Nsec{1}, Sophus::SE3f::transX(2.0F))) {
    has_up = finished->sub_map->measured_up.has_value();
  }
  CHECK(!has_up);

  SubMapBuilder one_interval({.splitting_distance = 1000.0F});
  one_interval.add_to_live_map({}, Nsec{0}, Sophus::SE3f{});
  one_interval.add_imu_sample({.time = Nsec{5}, .specific_force = Eigen::Vector3f(0.0F, 0.0F, 9.81F)});
  one_interval.add_to_live_map({}, Nsec{10}, Sophus::SE3f{});
  has_up = true;
  if (auto finished = one_interval.finalize()) {
    has_up = finished->sub_map->measured_up.has_value();
  }
  CHECK(!has_up);
}
