// A writer feeds synthetic scans to SLAM while a reader polls latest_keyposes().
// Run under TSan to surface races on the atomic shared_ptr.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <thread>

#include <Eigen/Core>
#include <catch2/catch_test_macros.hpp>
#include <sophus/se3.hpp>

#include "rko_slam/core/slam.hpp"
#include "rko_slam/core/sub_map_builder.hpp"

using namespace std::chrono_literals;

namespace {

// Just enough geometry that the voxel hash and normal estimation do not trip on degenerate input.
std::vector<Eigen::Vector3f> cube_cloud(float side, std::size_t per_axis) {
  std::vector<Eigen::Vector3f> points;
  points.reserve(per_axis * per_axis * per_axis);
  const auto step = side / static_cast<float>(per_axis - 1);
  for (std::size_t i = 0; i < per_axis; ++i) {
    for (std::size_t j = 0; j < per_axis; ++j) {
      for (std::size_t k = 0; k < per_axis; ++k) {
        points.emplace_back(static_cast<float>(i) * step, static_cast<float>(j) * step, static_cast<float>(k) * step);
      }
    }
  }
  return points;
}

} // namespace

TEST_CASE("process_finished_sub_map: keyposes publication is atomic and monotonic", "[closure-thread]") {
  rko_slam::core::SubMapBuilder::Config sub_map_config;
  sub_map_config.splitting_distance = 1.0F; // small so synthetic motion triggers splits fast
  sub_map_config.voxel_map = {.voxel_size = 0.5F, .max_points_per_voxel = 20};
  rko_slam::core::SubMapBuilder builder(sub_map_config);
  const rko_slam::core::SLAM::Config slam_config;
  rko_slam::core::SLAM slam(slam_config, sub_map_config.voxel_map);

  const auto cloud = cube_cloud(0.4F, 6);

  std::atomic<bool> stop_reader{false};
  std::atomic<bool> monotonic{true};
  std::atomic<std::size_t> reads_with_keyposes{0};
  std::atomic<std::size_t> max_keypose_count_seen{0};

  // Catch2 assertions are not thread-safe, so the verdict is an atomic flag checked after join().
  std::thread reader([&] {
    while (!stop_reader.load(std::memory_order_acquire)) {
      const auto keyposes = slam.latest_keyposes();
      if (keyposes) {
        ++reads_with_keyposes;
        const auto keypose_count = keyposes->map_T_keypose.size();
        const std::size_t prev_max = max_keypose_count_seen.load(std::memory_order_relaxed);
        if (keypose_count < prev_max) {
          monotonic.store(false, std::memory_order_relaxed);
        } else if (keypose_count > prev_max) {
          max_keypose_count_seen.store(keypose_count, std::memory_order_relaxed);
        }
      }
      std::this_thread::sleep_for(50us);
    }
  });

  // 1.2 m per scan against splitting_distance 1.0, so every scan splits.
  constexpr std::size_t kScans = 50;
  constexpr float kStep = 1.2F;
  std::size_t splits_observed = 0;
  for (std::size_t i = 0; i < kScans; ++i) {
    Sophus::SE3f odom_T_base;
    odom_T_base.translation() = Eigen::Vector3f(static_cast<float>(i) * kStep, 0.0F, 0.0F);
    auto finished =
        builder.add_to_live_map(cloud, rko_slam::core::Nsec(static_cast<int64_t>(i) * 100'000'000), odom_T_base);
    if (finished) {
      slam.process_finished_sub_map(std::move(finished->sub_map), finished->points);
      ++splits_observed;
    }
  }
  if (auto trailing = builder.finalize()) {
    slam.process_finished_sub_map(std::move(trailing->sub_map), trailing->points);
  }
  stop_reader.store(true, std::memory_order_release);
  reader.join();

  CAPTURE(splits_observed, reads_with_keyposes.load(), max_keypose_count_seen.load());
  REQUIRE(monotonic.load());
  REQUIRE(splits_observed > 0);
  REQUIRE(reads_with_keyposes.load() > 0);
  REQUIRE(max_keypose_count_seen.load() >= 1);
  auto final_keyposes = slam.latest_keyposes();
  REQUIRE(final_keyposes);
  REQUIRE(final_keyposes->map_T_keypose.size() == slam.sub_maps.size());
  rko_slam::core::KeyposeId highest_keypose_id = 0;
  for (const auto& edge : slam.pose_graph.edges()) {
    highest_keypose_id = std::max({highest_keypose_id, edge.from_id, edge.to_id});
  }
  REQUIRE(highest_keypose_id + 1 == slam.sub_maps.size());
}
