#include <Eigen/Core>
#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <memory>
#include <optional>
#include <sophus/se3.hpp>
#include <vector>

#include "rko_slam/core/slam.hpp"

using rko_slam::core::Nsec;
using rko_slam::core::SLAM;
using rko_slam::core::SubMap;

namespace {

std::vector<Eigen::Vector3f> cube_cloud() {
  std::vector<Eigen::Vector3f> points;
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      for (int k = 0; k < 6; ++k) {
        points.emplace_back(0.1F * static_cast<float>(i), 0.1F * static_cast<float>(j), 0.1F * static_cast<float>(k));
      }
    }
  }
  return points;
}

std::unique_ptr<SubMap> sub_map_at(const std::uint64_t keypose_id, const std::optional<Eigen::Vector3f>& measured_up) {
  auto sub_map = std::make_unique<SubMap>();
  sub_map->id = keypose_id;
  sub_map->odom_T_keypose = Sophus::SE3f::transX(static_cast<float>(keypose_id));
  sub_map->odom_T_next_keypose = Sophus::SE3f::transX(static_cast<float>(keypose_id) + 1.0F);
  sub_map->local_trajectory = {Sophus::SE3f{}};
  sub_map->scan_times = {Nsec{static_cast<std::int64_t>(keypose_id)}};
  sub_map->measured_up = measured_up;
  return sub_map;
}

} // namespace

TEST_CASE("SLAM: a sub-map's measured up becomes a gravity edge on its own keypose", "[slam]") {
  const SLAM::Config slam_config;
  SLAM slam(slam_config, 0.5F);
  const std::vector<Eigen::Vector3f> cloud = cube_cloud();

  slam.process_finished_sub_map(sub_map_at(0, std::nullopt), cloud);
  CHECK(slam.pose_graph.gravity_edges.empty());

  slam.process_finished_sub_map(sub_map_at(1, Eigen::Vector3f(0.0F, 0.0F, 9.81F)), cloud);
  REQUIRE(slam.pose_graph.gravity_edges.size() == 1);
  CHECK(slam.pose_graph.gravity_edges.front().keypose_id == 1);
  CHECK(slam.pose_graph.anchor.keypose_id == 0);
  CHECK((slam.pose_graph.anchor.map_T_held.inverse() * slam.pose_graph.keyposes.front()).log().norm() < 1e-9);
}
