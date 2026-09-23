#pragma once

#include <algorithm>
#include <vector>

#include <Eigen/Core>
#include <tsl/robin_map.h>

#include <rko_lio/core/voxel_down_sample.hpp> // brings rko_lio::core::VoxelHash


namespace rko_slam::core {

struct VoxelHashMap {
  using Voxel = Eigen::Vector3i;
  using Point = Eigen::Vector3f;
  using VoxelBlock = std::vector<Point>;

  struct Config {
    float voxel_size = 0.5F;
    unsigned int max_points_per_voxel = 20;
    bool operator==(const Config&) const = default;
  };

  explicit VoxelHashMap(const Config& config)
      : max_points_per_voxel(config.max_points_per_voxel),
        inv_voxel_size(1.0F / config.voxel_size),
        map_resolution_sq(config.voxel_size * config.voxel_size / static_cast<float>(config.max_points_per_voxel)) {}

  void add_points(const std::vector<Eigen::Vector3f>& points) {
    for (const Point& point : points) {
      const Voxel voxel = rko_lio::core::point_to_voxel(point, inv_voxel_size);
      const auto [voxel_entry, inserted] = voxels.try_emplace(voxel);
      VoxelBlock& voxel_points = voxel_entry.value();
      if (inserted) {
        voxel_points.reserve(max_points_per_voxel);
      }
      if (voxel_points.size() == max_points_per_voxel) {
        continue;
      }
      const bool too_close = std::any_of(voxel_points.cbegin(), voxel_points.cend(), [&](const Point& stored_point) {
        return (stored_point - point).squaredNorm() < map_resolution_sq;
      });
      if (too_close) {
        continue;
      }
      voxel_points.emplace_back(point);
    }
  }

  std::vector<Point> points() const {
    std::vector<Point> map_points;
    map_points.reserve(voxels.size() * max_points_per_voxel);
    for (const auto& [_, block] : voxels) {
      map_points.insert(map_points.end(), block.begin(), block.end());
    }
    return map_points;
  }

  tsl::robin_map<Voxel, VoxelBlock, rko_lio::core::VoxelHash> voxels;

  unsigned int max_points_per_voxel;

  float inv_voxel_size;
  float map_resolution_sq;
};

} // namespace rko_slam::core
