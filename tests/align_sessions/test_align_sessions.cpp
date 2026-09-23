// End-to-end coverage for align_sessions over a seeded pillar constellation,
// texture-rich in BEV so MapClosures' ORB pipeline has keypoints to match.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_exception.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <format>
#include <fstream>
#include <optional>
#include <utility>

#include <cmath>
#include <filesystem>
#include <numbers>
#include <random>
#include <stdexcept>
#include <string>

#include "rko_slam/align_sessions/align_sessions.hpp"
#include "rko_slam/pgo/io.hpp"
#include "rko_slam/core/closure.hpp"
#include "rko_slam/core/run_artifacts.hpp"
#include "rko_slam/core/sub_map_builder.hpp"
#include "rko_slam/pgo/pose_graph.hpp"

namespace fs = std::filesystem;
using namespace rko_slam::core;
using rko_slam::align_sessions::align;
using rko_slam::align_sessions::AlignResult;

namespace {

const rko_slam::pgo::PoseGraph::Config kPoseGraph{.max_iterations = 100};
const ClosureDetector::Config kDetector{.no_of_sub_maps_to_skip = 0};
constexpr float kOverlapThreshold = ClosureRefinement::kDefaultOverlapThreshold;

fs::path temp_dir(const std::string& tag) {
  auto dir = fs::temp_directory_path() / ("rko_slam_test_align_" + tag + "_" + std::to_string(std::random_device{}()));
  fs::create_directories(dir);
  return dir;
}

// Repeated uniform shapes hit MapClosures' self-similarity filter, and the footprint must exceed
// ORB's 31-px edge_threshold at 0.5 m/px to yield any keypoints.
std::vector<Eigen::Vector3f> pillar_world(const unsigned seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> position(5.0F, 145.0F);
  std::uniform_real_distribution<float> angle(0.0F, std::numbers::pi_v<float>);
  std::uniform_real_distribution<float> wall_len(4.0F, 25.0F);
  std::uniform_real_distribution<float> jitter(-0.15F, 0.15F);
  std::uniform_real_distribution<float> height(0.0F, 3.0F);
  std::uniform_real_distribution<float> radius(0.2F, 1.4F);
  std::uniform_int_distribution<int> pillar_pts(100, 600);
  std::vector<Eigen::Vector3f> cloud;
  // GroundAlign needs a dominant horizontal plane, or the fit latches onto a wall.
  constexpr float kGroundSpacing = 0.45F;
  constexpr int kGroundSamples = 334; // 150 m at kGroundSpacing
  for (int index_x = 0; index_x < kGroundSamples; ++index_x) {
    const float ground_x = static_cast<float>(index_x) * kGroundSpacing;
    for (int index_y = 0; index_y < kGroundSamples; ++index_y) {
      const float ground_y = static_cast<float>(index_y) * kGroundSpacing;
      cloud.emplace_back(ground_x + jitter(rng), ground_y + jitter(rng), 0.05F * jitter(rng));
    }
  }
  for (int wall = 0; wall < 60; ++wall) {
    const float center_x = position(rng);
    const float center_y = position(rng);
    const float wall_angle = angle(rng);
    const float len = wall_len(rng);
    const float direction_x = std::cos(wall_angle);
    const float direction_y = std::sin(wall_angle);
    std::uniform_real_distribution<float> spacing(0.04F, 0.15F);
    const float step = spacing(rng);
    const int offset_count = static_cast<int>(std::ceil(len / step));
    for (int offset_index = 0; offset_index < offset_count; ++offset_index) {
      const float wall_offset = (-len / 2.0F) + (static_cast<float>(offset_index) * step);
      for (int rep = 0; rep < 3; ++rep) {
        cloud.emplace_back(center_x + (wall_offset * direction_x) + jitter(rng),
                           center_y + (wall_offset * direction_y) + jitter(rng), height(rng));
      }
    }
  }
  for (int pillar = 0; pillar < 30; ++pillar) {
    const float center_x = position(rng);
    const float center_y = position(rng);
    const float pillar_radius = radius(rng);
    const int point_count = pillar_pts(rng);
    std::uniform_real_distribution<float> within(-pillar_radius, pillar_radius);
    for (int i = 0; i < point_count; ++i) {
      cloud.emplace_back(center_x + within(rng), center_y + within(rng), height(rng));
    }
  }
  return cloud;
}

// `clouds` are sub_map-local, `keyposes` session-frame, same count. The config records the builder
// defaults, which is what align rebuilds at.
void write_session(const fs::path& run_dir,
                   const std::vector<std::vector<Eigen::Vector3f>>& clouds,
                   const std::vector<Sophus::SE3f>& keyposes,
                   const std::int64_t t0_ns,
                   const std::vector<std::optional<Eigen::Vector3f>>& measured_ups = {}) {
  REQUIRE(clouds.size() == keyposes.size());
  constexpr std::size_t kScansPerSubMap = 3;

  std::vector<TrajectorySample> tum;
  std::int64_t time_ns = t0_ns;
  for (std::size_t k = 0; k < clouds.size(); ++k) {
    REQUIRE(write_ply_xyz(run_dir / "sub_maps" / std::format("sub_map_{:06}_{}.ply", k, time_ns), clouds.at(k)));
    for (std::size_t scan_index = 0; scan_index < kScansPerSubMap; ++scan_index, time_ns += 100'000'000LL) {
      const Sophus::SE3f local = Sophus::SE3f::trans(0.1F * static_cast<float>(scan_index), 0.0F, 0.0F);
      tum.push_back({.time = Nsec{time_ns}, .pose = keyposes.at(k) * local});
    }
  }

  rko_slam::pgo::PoseGraph pose_graph;
  for (const Sophus::SE3f& keypose : keyposes) {
    pose_graph.add_keypose(keypose.cast<double>());
  }
  for (std::size_t k = 0; k + 1 < keyposes.size(); ++k) {
    pose_graph.add_odometry_edge(k, k + 1, (keyposes.at(k).inverse() * keyposes.at(k + 1)).cast<double>());
  }
  for (std::size_t k = 0; k < measured_ups.size(); ++k) {
    if (measured_ups.at(k)) {
      pose_graph.add_gravity_edge(k, measured_ups.at(k)->cast<double>());
    }
  }
  pose_graph.anchor_at(0);
  const std::string stem = run_dir.filename().string();
  {
    const VoxelHashMap::Config voxel_map = SubMapBuilder::Config{}.voxel_map;
    std::ofstream config(run_dir / (stem + "_config.yaml"));
    config << std::format("voxel_size: {}\nmax_points_per_voxel: {}\n", voxel_map.voxel_size,
                          voxel_map.max_points_per_voxel);
  }
  REQUIRE(rko_slam::pgo::save(pose_graph, run_dir / (stem + "_keypose_graph.g2o")));
  REQUIRE(write_tum(run_dir / (stem + "_tum.txt"), tum));
}

Sophus::SE3f yaw_xy(const double yaw, const double x_metres, const double y_metres) {
  return Sophus::SE3d{Sophus::SO3d::rotZ(yaw), Eigen::Vector3d{x_metres, y_metres, 0.0}}.cast<float>();
}

} // namespace

TEST_CASE("align: cloud-replay parity - dumped+reloaded clouds reproduce the closure", "[align_sessions]") {
  const auto dir = temp_dir("parity");
  const std::vector<Eigen::Vector3f> shared = pillar_world(7);
  const std::vector<Eigen::Vector3f> other = pillar_world(99);

  ClosureDetector::Config detector_config;
  detector_config.no_of_sub_maps_to_skip = 0;
  ClosureDetector live(detector_config);
  REQUIRE(live.query_all(0, shared).empty());
  (void)live.query_all(1, other);
  const auto live_candidates = live.query_all(2, shared); // revisit of map 0
  REQUIRE(!live_candidates.empty());
  REQUIRE(live_candidates.front().source_id == 0);

  // Round-trip both clouds through the ply dump, replay into a fresh detector.
  REQUIRE(write_ply_xyz(dir / "shared.ply", shared));
  REQUIRE(write_ply_xyz(dir / "other.ply", other));
  const std::vector<Eigen::Vector3f> shared_back = read_ply_xyz(dir / "shared.ply");
  const std::vector<Eigen::Vector3f> other_back = read_ply_xyz(dir / "other.ply");
  ClosureDetector replay(detector_config);
  (void)replay.query_all(0, shared_back);
  (void)replay.query_all(1, other_back);
  const auto replay_candidates = replay.query_all(2, shared_back);
  REQUIRE(!replay_candidates.empty());
  REQUIRE(replay_candidates.front().source_id == 0);
  REQUIRE(std::cmp_greater_equal(replay_candidates.front().number_of_inliers, detector_config.inliers_threshold));
  fs::remove_all(dir);
}

TEST_CASE("closure detector: gapped caller ids are safe and translate back", "[align_sessions][map_closures]") {
  // Non-contiguous ids, as an online split-drop produces: the dense-id mapping must survive the gap
  // and report the caller's id 0, not an internal dense id.
  const std::vector<Eigen::Vector3f> shared = pillar_world(7);
  const std::vector<Eigen::Vector3f> other = pillar_world(99);
  ClosureDetector::Config detector_config;
  detector_config.no_of_sub_maps_to_skip = 0;
  ClosureDetector detector(detector_config);
  REQUIRE(detector.query_all(0, shared).empty());
  (void)detector.query_all(1, other);
  const auto candidates = detector.query_all(50, shared); // gap 2..49 never queried
  REQUIRE(!candidates.empty());
  REQUIRE(candidates.front().source_id == 0);
  REQUIRE(candidates.front().target_id == 50);
}

TEST_CASE("align: two sessions with a shared place align to the known offset", "[align_sessions]") {
  const auto dir = temp_dir("two");
  const std::vector<Eigen::Vector3f> shared = pillar_world(7);
  const std::vector<Eigen::Vector3f> unique_a = pillar_world(11);
  const std::vector<Eigen::Vector3f> unique_b = pillar_world(23);

  // Session A in its own frame; session B's frame is offset in the world by
  // `world_offset` (what align should recover, up to the closure residual).
  const Sophus::SE3f world_offset = yaw_xy(0.3, 25.0, -10.0);
  const Sophus::SE3f a_keypose0 = yaw_xy(0.0, 0.0, 0.0);
  const Sophus::SE3f a_keypose1 = yaw_xy(0.1, 50.0, 5.0);
  // A is the world here, so B's session-frame keypose must satisfy world_offset * b_T_kp == a_T_kp.
  const Sophus::SE3f b_keypose0 = world_offset.inverse() * a_keypose1;
  const Sophus::SE3f b_keypose1 = b_keypose0 * yaw_xy(-0.2, 40.0, 0.0);

  write_session(dir / "sess_a_0", {unique_a, shared}, {a_keypose0, a_keypose1}, 1'000'000'000LL);
  write_session(dir / "sess_b_0", {shared, unique_b}, {b_keypose0, b_keypose1}, 2'000'000'000'000LL);

  const std::optional<AlignResult> aligned =
      align(kDetector, kOverlapThreshold, kPoseGraph, {dir / "sess_a_0", dir / "sess_b_0"}, dir / "out", "aligned");
  REQUIRE(aligned.has_value());
  const AlignResult& result = *aligned;
  INFO(result.accepted_closures << " accepted");
  REQUIRE(result.dropped_sessions.empty());
  REQUIRE(fs::exists(result.out_run_dir / (result.out_run_name + "_joint_keypose_graph.g2o")));

  // Session B's re-anchored TUM must land where the world (= session A
  // frame) expects it: world_pose = world_offset * session_pose.
  const auto tum_b = read_tum(result.out_run_dir / (result.out_run_name + "_session_1_tum.txt"));
  REQUIRE(tum_b.size() == 6);
  const Sophus::SE3f expected_first = world_offset * b_keypose0;
  // closure residual scale, not exactness
  REQUIRE_THAT((tum_b.front().pose.inverse() * expected_first).log().norm(), Catch::Matchers::WithinAbs(0.0, 0.5));
  fs::remove_all(dir);
}

TEST_CASE("align: the sub-maps' up directions level sessions recorded tilted", "[align_sessions]") {
  const auto dir = temp_dir("gravity");
  const std::vector<Eigen::Vector3f> shared = pillar_world(7);
  // Session A's frame is `tilt` off the world about its first keypose; B is placed as in the two-session case.
  const Sophus::SO3f tilt = Sophus::SO3f::rotX(0.1F) * Sophus::SO3f::rotY(-0.05F);
  const Sophus::SE3f world_offset = yaw_xy(0.3, 25.0, -10.0);
  const std::vector<Sophus::SE3f> a_keyposes{yaw_xy(0.0, 0.0, 0.0), yaw_xy(0.1, 50.0, 5.0)};
  const Sophus::SE3f b_keypose0 = world_offset.inverse() * a_keyposes.at(1);
  const std::vector<Sophus::SE3f> b_keyposes{b_keypose0, b_keypose0 * yaw_xy(-0.2, 40.0, 0.0)};
  const auto up_in = [](const Sophus::SO3f& world_R_keypose) {
    return Eigen::Vector3f(world_R_keypose.inverse() * Eigen::Vector3f::UnitZ());
  };
  const auto measured_ups_of = [&](const Sophus::SE3f& a_T_session, const std::vector<Sophus::SE3f>& keyposes) {
    std::vector<std::optional<Eigen::Vector3f>> measured_ups;
    measured_ups.reserve(keyposes.size());
    for (const Sophus::SE3f& keypose : keyposes) {
      measured_ups.emplace_back(9.81F * up_in(tilt * a_T_session.so3() * keypose.so3()));
    }
    return measured_ups;
  };
  write_session(dir / "a_0", {pillar_world(11), shared}, a_keyposes, 1'000'000'000LL,
                measured_ups_of(Sophus::SE3f{}, a_keyposes));
  write_session(dir / "b_0", {shared, pillar_world(23)}, b_keyposes, 2'000'000'000'000LL,
                measured_ups_of(world_offset, b_keyposes));

  const std::optional<AlignResult> aligned =
      align(kDetector, kOverlapThreshold, kPoseGraph, {dir / "a_0", dir / "b_0"}, dir / "out", "aligned");
  REQUIRE(aligned.has_value());
  const AlignResult& result = *aligned;
  REQUIRE(result.reference_session == 0);
  const auto tum_a = read_tum(result.out_run_dir / (result.out_run_name + "_session_0_tum.txt"));
  const auto tum_b = read_tum(result.out_run_dir / (result.out_run_name + "_session_1_tum.txt"));
  REQUIRE((up_in(a_keyposes.at(0).so3()) - up_in(tilt)).norm() > 0.1F); // recorded tilted
  CHECK((up_in(tum_a.front().pose.so3()) - up_in(tilt)).norm() < 0.01F);
  CHECK((up_in(tum_b.front().pose.so3()) - up_in(tilt * world_offset.so3() * b_keypose0.so3())).norm() < 0.01F);
  CHECK(tum_a.front().pose.translation().norm() < 1e-4F);
  fs::remove_all(dir);
}

TEST_CASE("align: disconnected session is dropped, connected pair survives", "[align_sessions]") {
  const auto dir = temp_dir("drop");
  const std::vector<Eigen::Vector3f> shared = pillar_world(7);
  write_session(dir / "a_0", {pillar_world(11), shared}, {yaw_xy(0, 0, 0), yaw_xy(0, 50, 0)}, 1'000'000'000LL);
  write_session(dir / "b_0", {shared, pillar_world(23)}, {yaw_xy(0.2, 3, 4), yaw_xy(0.2, 44, 4)}, 2'000'000'000'000LL);
  // Session c shares nothing with a or b.
  write_session(dir / "c_0", {pillar_world(51), pillar_world(67)}, {yaw_xy(0, 0, 0), yaw_xy(0, 50, 0)},
                3'000'000'000'000LL);

  const std::optional<AlignResult> aligned =
      align(kDetector, kOverlapThreshold, kPoseGraph, {dir / "a_0", dir / "b_0", dir / "c_0"}, dir / "out", "aligned");
  REQUIRE(aligned.has_value());
  const AlignResult& result = *aligned;
  INFO(result.accepted_closures << " accepted");
  REQUIRE(result.dropped_sessions == std::vector<std::size_t>{2});
  REQUIRE(fs::exists(result.out_run_dir / (result.out_run_name + "_session_0_tum.txt")));
  REQUIRE(fs::exists(result.out_run_dir / (result.out_run_name + "_session_1_tum.txt")));
  REQUIRE(!fs::exists(result.out_run_dir / (result.out_run_name + "_session_2_tum.txt")));
  fs::remove_all(dir);
}

TEST_CASE("align: a second connected component is dropped whole", "[align_sessions]") {
  const auto dir = temp_dir("two_components");
  const std::vector<Eigen::Vector3f> shared_ab = pillar_world(7);
  const std::vector<Eigen::Vector3f> shared_cd = pillar_world(13);
  write_session(dir / "a_0", {pillar_world(11), shared_ab}, {yaw_xy(0, 0, 0), yaw_xy(0, 50, 0)}, 1'000'000'000LL);
  write_session(dir / "b_0", {shared_ab, pillar_world(23)}, {yaw_xy(0.2, 3, 4), yaw_xy(0.2, 44, 4)},
                2'000'000'000'000LL);
  write_session(dir / "c_0", {pillar_world(51), shared_cd}, {yaw_xy(0, 0, 0), yaw_xy(0, 50, 0)}, 3'000'000'000'000LL);
  write_session(dir / "d_0", {shared_cd, pillar_world(67)}, {yaw_xy(0.2, 3, 4), yaw_xy(0.2, 44, 4)},
                4'000'000'000'000LL);

  const std::optional<AlignResult> aligned =
      align(kDetector, kOverlapThreshold, kPoseGraph, {dir / "a_0", dir / "b_0", dir / "c_0", dir / "d_0"}, dir / "out",
            "aligned");
  REQUIRE(aligned.has_value());
  const AlignResult& result = *aligned;
  INFO(result.accepted_closures << " accepted");
  REQUIRE(result.dropped_sessions == std::vector<std::size_t>{2, 3});
  REQUIRE(result.accepted_closures == 2);
  REQUIRE(fs::exists(result.out_run_dir / (result.out_run_name + "_session_0_tum.txt")));
  REQUIRE(fs::exists(result.out_run_dir / (result.out_run_name + "_session_1_tum.txt")));
  REQUIRE(!fs::exists(result.out_run_dir / (result.out_run_name + "_session_2_tum.txt")));
  REQUIRE(!fs::exists(result.out_run_dir / (result.out_run_name + "_session_3_tum.txt")));

  std::ifstream joint(result.out_run_dir / (result.out_run_name + "_joint_keypose_graph.g2o"));
  std::size_t vertices = 0;
  std::size_t pose_edges = 0;
  for (std::string line; std::getline(joint, line);) {
    vertices += static_cast<std::size_t>(line.starts_with("VERTEX_SE3"));
    pose_edges += static_cast<std::size_t>(line.starts_with("EDGE_SE3"));
  }
  REQUIRE(vertices == 4);
  REQUIRE(pose_edges == 3);
  fs::remove_all(dir);
}

TEST_CASE("align: an isolated first session does not block the rest", "[align_sessions]") {
  const auto dir = temp_dir("isolated_first");
  const std::vector<Eigen::Vector3f> shared = pillar_world(7);
  write_session(dir / "a_0", {pillar_world(51), pillar_world(67)}, {yaw_xy(0, 0, 0), yaw_xy(0, 50, 0)},
                1'000'000'000LL);
  write_session(dir / "b_0", {pillar_world(11), shared}, {yaw_xy(0, 0, 0), yaw_xy(0, 50, 0)}, 2'000'000'000'000LL);
  write_session(dir / "c_0", {shared, pillar_world(23)}, {yaw_xy(0.2, 3, 4), yaw_xy(0.2, 44, 4)}, 3'000'000'000'000LL);

  const std::optional<AlignResult> aligned =
      align(kDetector, kOverlapThreshold, kPoseGraph, {dir / "a_0", dir / "b_0", dir / "c_0"}, dir / "out", "aligned");
  REQUIRE(aligned.has_value());
  const AlignResult& result = *aligned;
  INFO(result.accepted_closures << " accepted");
  REQUIRE(result.dropped_sessions == std::vector<std::size_t>{0});
  REQUIRE(!fs::exists(result.out_run_dir / (result.out_run_name + "_session_0_tum.txt")));
  REQUIRE(fs::exists(result.out_run_dir / (result.out_run_name + "_session_1_tum.txt")));
  REQUIRE(fs::exists(result.out_run_dir / (result.out_run_name + "_session_2_tum.txt")));
  fs::remove_all(dir);
}

TEST_CASE("align: zero inter-session closures yields no alignment and writes nothing", "[align_sessions]") {
  const auto dir = temp_dir("none");
  write_session(dir / "a_0", {pillar_world(11), pillar_world(31)}, {yaw_xy(0, 0, 0), yaw_xy(0, 50, 0)},
                1'000'000'000LL);
  write_session(dir / "b_0", {pillar_world(23), pillar_world(41)}, {yaw_xy(0, 0, 0), yaw_xy(0, 50, 0)},
                2'000'000'000'000LL);

  REQUIRE_FALSE(
      align(kDetector, kOverlapThreshold, kPoseGraph, {dir / "a_0", dir / "b_0"}, dir / "out", "aligned").has_value());
  REQUIRE(!fs::exists(dir / "out" / "aligned_0"));
  fs::remove_all(dir);
}
