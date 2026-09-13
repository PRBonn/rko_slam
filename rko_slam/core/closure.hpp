#pragma once

#include <cstddef>
#include <memory>
#include <numbers>
#include <optional>
#include <sophus/se3.hpp>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "rko_slam/core/sub_map.hpp"
#include "rko_slam/core/types.hpp"

namespace map_closures {
class MapClosures;
} // namespace map_closures

namespace rko_slam::core {

struct ClosureCandidate {
  KeyposeId source_id = 0;
  KeyposeId target_id = 0;
  Sophus::SE3f target_T_source;
  std::size_t number_of_inliers = 0;
};

class ClosureDetector {
public:
  struct Config {
    float density_map_resolution = 0.5F;
    float density_threshold = 0.05F;
    int hamming_distance_threshold = 50;
    std::size_t inliers_threshold = 5;
    int no_of_sub_maps_to_skip = 3;

    // icp search distance fixed based on the map
    constexpr float correspondence_distance() const { return std::numbers::sqrt3_v<float> * density_map_resolution; }

    std::string to_yaml() const;
  };

  explicit ClosureDetector(const Config& detector_config);
  ~ClosureDetector();

  ClosureDetector(const ClosureDetector&) = delete;
  ClosureDetector& operator=(const ClosureDetector&) = delete;
  ClosureDetector(ClosureDetector&&) = delete;
  ClosureDetector& operator=(ClosureDetector&&) = delete;

  // The top-inlier candidate with at least `config.inliers_threshold` inliers.
  std::optional<ClosureCandidate> query(const KeyposeId keypose_id, const std::vector<Eigen::Vector3f>& points);

  // Every candidate with its raw inlier count, unfiltered by inliers_threshold.
  std::vector<ClosureCandidate> query_all(const KeyposeId keypose_id, const std::vector<Eigen::Vector3f>& points);

private:
  // the PIMPL
  std::unique_ptr<map_closures::MapClosures> detector;

  std::size_t inliers_threshold;
  // Upstream needs a dense, monotonic-from-0 id stream and caller keypose ids can have gaps, so it is fed a
  // private dense id and the returned candidate ids are translated back.
  std::vector<KeyposeId> local_to_global;
  int register_local_id(const KeyposeId global_id);
};

struct ClosureRefinement {
  static constexpr float kDefaultOverlapThreshold = 0.4F;
  Sophus::SE3f refined_target_T_source;
  double overlap = 0.0;
};

// `voxel_size` must be the sub-map map resolution: one centroid per occupied voxel
ClosureRefinement refine_closure(const float voxel_size,
                                 const float max_correspondence_distance,
                                 const SubMap& source,
                                 const SubMap& target,
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

} // namespace rko_slam::core
