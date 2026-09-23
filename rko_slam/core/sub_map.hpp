#pragma once

#include <cstddef>
#include <rko_lio/core/util.hpp>
#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include "rko_slam/core/voxel_hash_map.hpp"

namespace rko_slam::core {

using rko_lio::core::Nsec;

// Point fields are in the keypose-relative frame; the map-frame keypose lives in the PoseGraph under `id`.
struct SubMap {
  std::size_t id = 0;
  Sophus::SE3f odom_T_keypose;
  // set only when a split sealed this one, so the trailing sub-map has none
  std::optional<Sophus::SE3f> odom_T_next_keypose;
  // Keypose-relative scan poses, index-aligned with `scan_times`; `front()` is identity.
  std::vector<Sophus::SE3f> local_trajectory;
  std::vector<Nsec> scan_times;
  // Per-voxel mean and normal, index-aligned with each other; the closure refinement's ICP target.
  std::vector<Eigen::Vector3f> centroids;
  std::vector<Eigen::Vector3f> normals;
  // Gravity's reaction in the keypose frame (m/s², expected length g).
  std::optional<Eigen::Vector3f> measured_up;
};

struct FinishedSubMap {
  std::unique_ptr<SubMap> sub_map;
  std::vector<Eigen::Vector3f> points;
};

// the normal is the smallest covariance eigenvector, per voxel with enough points
void fill_sub_map(const VoxelHashMap& map, SubMap& sub_map);

} // namespace rko_slam::core
