#include "rko_slam/core/sub_map_builder.hpp"

#include <UTL/profiler.hpp>
#include <cmath>
#include <format>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <sophus/se3.hpp>

namespace rko_slam::core {

namespace {

std::vector<Eigen::Vector3f>
range_filter_scan(const std::vector<Eigen::Vector3f>& base_points, const float min_range, const float max_range) {
  const float min_sq = min_range * min_range;
  const float max_sq = max_range * max_range;
  std::vector<Eigen::Vector3f> filtered;
  filtered.reserve(base_points.size());
  for (const auto& point : base_points) {
    const float range_sq = point.squaredNorm();
    if (!std::isfinite(range_sq) || range_sq < min_sq || range_sq > max_sq) {
      continue;
    }
    filtered.emplace_back(point);
  }
  return filtered;
}

// The carried points come back in the new keypose's frame.
std::vector<Eigen::Vector3f>
carryover_points(const VoxelHashMap& voxel_map, const Sophus::SE3f& new_T_old, const float radius) {
  std::vector<Eigen::Vector3f> carried;
  carried.reserve(voxel_map.voxels.size() * voxel_map.max_points_per_voxel);
  const float sq_radius = radius * radius;
  for (const auto& [voxel, block] : voxel_map.voxels) {
    if (block.empty() || (new_T_old * block.front()).squaredNorm() > sq_radius) {
      continue;
    }
    for (const auto& point : block) {
      carried.emplace_back(new_T_old * point);
    }
  }
  return carried;
}

} // namespace

std::string SubMapBuilder::Config::to_yaml() const {
  return std::format("voxel_size: {}\nsplitting_distance: {}\nmax_points_per_voxel: {}\nmin_range: {}\n"
                     "max_range: {}\n",
                     voxel_map.voxel_size, splitting_distance, voxel_map.max_points_per_voxel, min_range, max_range);
}

SubMapBuilder::SubMapBuilder(const Config sub_map_config)
    : config(sub_map_config), voxel_map(sub_map_config.voxel_map) {}

std::optional<FinishedSubMap> SubMapBuilder::add_to_live_map(const std::vector<Eigen::Vector3f>& base_points,
                                                             const Nsec end_time,
                                                             const Sophus::SE3f& odom_T_base) {
  UTL_PROFILER_SCOPE("SubMapBuilder::add_to_live_map");
  std::optional<FinishedSubMap> finished;
  std::vector<Eigen::Vector3f> carryover_in_new_frame;
  Sophus::SE3f live_T_base;

  if (live) {
    live_T_base = live->odom_T_keypose.inverse() * odom_T_base;
    if (live_T_base.translation().norm() >= config.splitting_distance) {
      carryover_in_new_frame = carryover_points(voxel_map, live_T_base.inverse(), config.splitting_distance);
      live->odom_T_next_keypose = odom_T_base;
      fill_sub_map(voxel_map, *live);
      finished = FinishedSubMap{.sub_map = std::make_unique<SubMap>(std::move(*live)), .points = voxel_map.points()};
      live.reset();
    }
  }

  if (!live) {
    live = SubMap{.id = next_id++, .odom_T_keypose = odom_T_base};
    voxel_map = VoxelHashMap(config.voxel_map);
    voxel_map.add_points(carryover_in_new_frame); // can be empty, on first sub_map
    live_T_base = Sophus::SE3f{};
  }

  if (!base_points.empty()) {
    std::vector<Eigen::Vector3f> filtered_points = range_filter_scan(base_points, config.min_range, config.max_range);
    for (Eigen::Vector3f& point : filtered_points) {
      point = live_T_base * point;
    }
    voxel_map.add_points(filtered_points);
  }
  live->local_trajectory.push_back(live_T_base);
  live->scan_times.push_back(end_time);
  return finished;
}

std::optional<FinishedSubMap> SubMapBuilder::finalize() {
  if (!live) {
    return std::nullopt;
  }
  fill_sub_map(voxel_map, *live);
  FinishedSubMap out{.sub_map = std::make_unique<SubMap>(std::move(*live)), .points = voxel_map.points()};
  live.reset();
  return out;
}

} // namespace rko_slam::core
