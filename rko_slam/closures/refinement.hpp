#pragma once

#include <sophus/se3.hpp>
#include <vector>

#include <Eigen/Core>

namespace rko_slam::closures {

struct ClosureRefinement {
  static constexpr float kDefaultOverlapThreshold = 0.4F;
  Sophus::SE3f refined_target_T_source;
  double overlap = 0.0;
};

// `voxel_size` must be the sub-map map resolution: one centroid per occupied voxel
ClosureRefinement refine_closure(const float voxel_size,
                                 const float max_correspondence_distance,
                                 const std::vector<Eigen::Vector3f>& source_centroids,
                                 const std::vector<Eigen::Vector3f>& target_centroids,
                                 const std::vector<Eigen::Vector3f>& target_normals,
                                 const Sophus::SE3f& target_T_source_init);

Sophus::SE3f icp_point_to_plane(const std::vector<Eigen::Vector3f>& source,
                                const std::vector<Eigen::Vector3f>& target,
                                const std::vector<Eigen::Vector3f>& target_normals,
                                const float max_correspondence_distance,
                                const Sophus::SE3f& initial_guess);

// Overlap coefficient of both clouds voxelised in the same frame: intersection / min(|first|, |second|).
double voxel_overlap_coefficient(const std::vector<Eigen::Vector3f>& first_pts,
                                 const std::vector<Eigen::Vector3f>& second_pts,
                                 const Sophus::SE3f& first_T_second,
                                 const float voxel_size);

} // namespace rko_slam::closures
