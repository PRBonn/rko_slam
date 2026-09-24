#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <sophus/se3.hpp>

#include "rko_slam/core/sub_map_builder.hpp"
#include "rko_slam/core/voxel_hash_map.hpp"

using rko_slam::core::fill_sub_map;
using rko_slam::core::FinishedSubMap;
using rko_slam::core::ImuSample;
using rko_slam::core::Nsec;
using rko_slam::core::SubMap;
using rko_slam::core::SubMapBuilder;
using rko_slam::core::VoxelHashMap;

using Catch::Matchers::WithinAbs;

TEST_CASE("fill_sub_map: empty map fills nothing", "[sub_map]") {
  const VoxelHashMap map({.voxel_size = 1.0F, .max_points_per_voxel = 20});
  REQUIRE(map.voxels.empty());
  SubMap filled;
  fill_sub_map(map, filled);
  REQUIRE(map.points().empty());
  REQUIRE(filled.centroids.empty());
  REQUIRE(filled.normals.empty());
}

TEST_CASE("fill_sub_map: a voxel below the min point count yields nothing", "[sub_map]") {
  // Two voxels, two well-separated points each; the map keeps them all, fill_sub_map gates on the count.
  VoxelHashMap map({.voxel_size = 1.0F, .max_points_per_voxel = 10});
  map.add_points(std::vector<Eigen::Vector3f>{
      {0.1F, 0.1F, 0.1F},
      {0.9F, 0.9F, 0.9F}, // voxel (0,0,0)
      {5.1F, 5.1F, 5.1F},
      {5.9F, 5.9F, 5.9F},
  }); // voxel (5,5,5)
  REQUIRE(map.voxels.size() == 2);
  SubMap filled;
  fill_sub_map(map, filled);
  REQUIRE(map.points().size() == 4);
  // Below kMinPointsForCovariance (10) per voxel -> no centroids/normals.
  REQUIRE(filled.centroids.empty());
  REQUIRE(filled.normals.empty());
}

TEST_CASE("fill_sub_map: single-voxel centroid + covariance shape", "[sub_map]") {
  // One voxel with >= 10 points -> one centroid (the arithmetic mean) and one
  // normal. The twelve points are 5 m apart, beyond the 4.47 m dedup radius.
  VoxelHashMap map({.voxel_size = 20.0F, .max_points_per_voxel = 20});
  std::vector<Eigen::Vector3f> points;
  for (int grid_x = 0; grid_x < 3; ++grid_x) {
    for (int grid_y = 0; grid_y < 4; ++grid_y) {
      points.emplace_back(static_cast<float>(grid_x) * 5.0F, static_cast<float>(grid_y) * 5.0F,
                          static_cast<float>(grid_x + grid_y));
    }
  }
  map.add_points(points);
  REQUIRE(map.voxels.size() == 1);
  REQUIRE(map.points().size() == 12);

  SubMap filled;
  fill_sub_map(map, filled);
  REQUIRE(map.points().size() == 12);
  REQUIRE(filled.centroids.size() == 1);
  REQUIRE(filled.normals.size() == 1); // index-aligned with centroids

  // The map stores in float, so the double-precision oracle needs an explicit
  // cast and a float tolerance.
  Eigen::Vector3d expected_mean = Eigen::Vector3d::Zero();
  for (const auto& point : points) {
    expected_mean += point.cast<double>();
  }
  expected_mean /= static_cast<double>(points.size());
  REQUIRE_THAT((filled.centroids.at(0).cast<double>() - expected_mean).norm(), WithinAbs(0.0, 1e-5));

  // Normal is a unit vector (smallest eigenvector of a symmetric 3x3).
  REQUIRE_THAT(filled.normals.at(0).cast<double>().norm(), WithinAbs(1.0, 1e-5));
}

TEST_CASE("fill_sub_map: planar patch -> normal along plane normal", "[sub_map]") {
  // A dense planar patch in the z=2 plane inside ONE voxel: the smallest
  // eigenvector of the in-voxel covariance must point along +/- z.
  const unsigned int cap = 200;
  VoxelHashMap map({.voxel_size = 20.0F, .max_points_per_voxel = cap});
  std::vector<Eigen::Vector3f> points;
  for (int grid_x = 0; grid_x < 6; ++grid_x) {
    for (int grid_y = 0; grid_y < 6; ++grid_y) {
      // 2 m spacing keeps neighbours outside the 1.41 m dedup radius.
      points.emplace_back(static_cast<float>(grid_x) * 2.0F, static_cast<float>(grid_y) * 2.0F, 2.0F);
    }
  }
  map.add_points(points);
  REQUIRE(map.voxels.size() == 1);
  REQUIRE(map.points().size() == 36);

  SubMap filled;
  fill_sub_map(map, filled);
  REQUIRE(filled.centroids.size() == 1);
  REQUIRE(filled.normals.size() == 1);

  const Eigen::Vector3d normal = filled.normals.at(0).cast<double>();
  REQUIRE_THAT(std::abs(normal.z()), WithinAbs(1.0, 1e-5));
  REQUIRE_THAT(normal.x(), WithinAbs(0.0, 1e-5));
  REQUIRE_THAT(normal.y(), WithinAbs(0.0, 1e-5));

  REQUIRE_THAT(static_cast<double>(filled.centroids.at(0).z()), WithinAbs(2.0, 1e-5));
}

TEST_CASE("fill_sub_map: covariance matches the solver oracle on a patch", "[sub_map]") {
  // Independent oracle: rebuild the per-voxel covariance with the documented
  // (n-1) normalisation and confirm fill_sub_map's normal is its smallest eigenvector.
  VoxelHashMap map({.voxel_size = 20.0F, .max_points_per_voxel = 400});
  const std::vector<Eigen::Vector3f> points{
      {1.0F, 0.0F, 0.5F}, {4.0F, 1.0F, 0.5F},  {2.0F, 5.0F, 0.5F},  {6.0F, 3.0F, 0.5F},  {3.0F, 2.0F, 0.5F},
      {8.0F, 6.0F, 2.5F}, {10.0F, 9.0F, 1.0F}, {5.0F, 12.0F, 3.0F}, {12.0F, 3.0F, 4.0F}, {15.0F, 14.0F, 2.0F},
  };
  map.add_points(points);
  REQUIRE(map.points().size() == points.size());

  SubMap filled;
  fill_sub_map(map, filled);
  REQUIRE(filled.centroids.size() == 1);

  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto& point : points) {
    mean += point.cast<double>();
  }
  mean /= static_cast<double>(points.size());
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto& point : points) {
    const Eigen::Vector3d deviation = point.cast<double>() - mean;
    covariance += deviation * deviation.transpose();
  }
  covariance /= static_cast<double>(points.size() - 1);

  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance, Eigen::ComputeEigenvectors);
  const Eigen::Vector3d oracle_normal = solver.eigenvectors().col(0);

  // Sign-tolerant (eigenvector sign is arbitrary).
  REQUIRE(std::abs(filled.normals.at(0).cast<double>().dot(oracle_normal)) > 1.0 - 1e-5);
}

TEST_CASE("SubMapBuilder: the splitting scan opens the next sub-map", "[sub_map]") {
  // Scans 1 m apart along +x, so scan 10 is the first at or past the 10 m splitting distance.
  SubMapBuilder builder({.voxel_map = {.voxel_size = 1.0F, .max_points_per_voxel = 20}, .splitting_distance = 10.0F});
  std::vector<std::unique_ptr<SubMap>> sealed;
  for (std::int64_t scan = 0; scan <= 12; ++scan) {
    Sophus::SE3f odom_T_base;
    odom_T_base.translation() = Eigen::Vector3f(static_cast<float>(scan), 0.0F, 0.0F);
    if (auto finished = builder.add_to_live_map({}, Nsec{scan}, odom_T_base)) {
      sealed.push_back(std::move(finished->sub_map));
    }
  }
  REQUIRE(sealed.size() == 1);
  const auto finalized = builder.finalize();
  REQUIRE(finalized);
  const SubMap& live = *finalized->sub_map;

  // Scan 10 is the keypose of the new sub-map and its first scan, not the last scan of the sealed one.
  REQUIRE(sealed.at(0)->scan_times.size() == 10);
  REQUIRE(sealed.at(0)->scan_times.back() == Nsec{9});
  REQUIRE(live.scan_times.front() == Nsec{10});
  REQUIRE_THAT(static_cast<double>(live.odom_T_keypose.translation().x()), WithinAbs(10.0, 1e-6));
  const SubMap& split_sealed = *sealed.at(0);
  REQUIRE(split_sealed.odom_T_next_keypose.has_value());
  REQUIRE_FALSE(live.odom_T_next_keypose.has_value());
  // value_or, not a dereference: Catch2 hands REQUIRE's condition to its decomposer rather than
  // branching on it, so bugprone-unchecked-optional-access never sees a guard at the call site.
  const Sophus::SE3f odom_T_next = split_sealed.odom_T_next_keypose.value_or(Sophus::SE3f{});
  REQUIRE_THAT(static_cast<double>(odom_T_next.translation().x()), WithinAbs(10.0, 1e-6));

  // Every sub-map's trajectory starts at its own keypose, and no scan is shared or dropped.
  REQUIRE_THAT(static_cast<double>(sealed.at(0)->local_trajectory.front().translation().norm()), WithinAbs(0.0, 1e-6));
  REQUIRE_THAT(static_cast<double>(live.local_trajectory.front().translation().norm()), WithinAbs(0.0, 1e-6));
  REQUIRE(sealed.at(0)->local_trajectory.size() + live.local_trajectory.size() == 13);
}

namespace {

constexpr float kGravity = 9.81F;
constexpr std::int64_t kScanPeriod = 100'000'000;
constexpr std::int64_t kSamplesPerScan = 10;

} // namespace

TEST_CASE("SubMapBuilder: up is gravity's reaction while the platform accelerates and turns", "[sub_map]") {
  // 0.5 * |acceleration| * t^2 crosses 4 m at the 31st scan, which seals the first sub-map
  const Sophus::SO3f odom_R_keypose = Sophus::SO3f::rotZ(0.4F) * Sophus::SO3f::rotY(0.1F);
  const Eigen::Vector3f acceleration(0.8F, -0.3F, 0.1F);
  const Eigen::Vector3f gravity_reaction(0.0F, 0.0F, kGravity);
  constexpr float kYawRate = 0.5F;
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

TEST_CASE("SubMapBuilder: a run too short to split is still levelled, at finalize", "[sub_map]") {
  const Sophus::SO3f odom_R_keypose = Sophus::SO3f::rotZ(0.4F) * Sophus::SO3f::rotY(0.1F);
  const Eigen::Vector3f acceleration(0.8F, -0.3F, 0.1F);
  const Eigen::Vector3f gravity_reaction(0.0F, 0.0F, kGravity);
  constexpr float kYawRate = 0.5F;
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

TEST_CASE("SubMapBuilder: no up before the first scan, without samples, or inside one scan interval", "[sub_map]") {
  const ImuSample at_or_before_the_first_scan{
      .time = Nsec{0},
      .specific_force = Eigen::Vector3f(0.0F, 0.0F, kGravity),
  };

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
  one_interval.add_imu_sample({.time = Nsec{5}, .specific_force = Eigen::Vector3f(0.0F, 0.0F, kGravity)});
  one_interval.add_to_live_map({}, Nsec{10}, Sophus::SE3f{});
  has_up = true;
  if (auto finished = one_interval.finalize()) {
    has_up = finished->sub_map->measured_up.has_value();
  }
  CHECK(!has_up);
}
