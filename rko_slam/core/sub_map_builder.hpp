#pragma once

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include "rko_slam/core/sub_map.hpp"
#include "rko_slam/core/types.hpp"
#include "rko_slam/core/voxel_hash_map.hpp"

namespace rko_slam::core {

// In the base frame (rad/s, m/s²).
struct ImuSample {
  Nsec time{0};
  Eigen::Vector3f angular_velocity = Eigen::Vector3f::Zero();
  Eigen::Vector3f specific_force = Eigen::Vector3f::Zero();
};

// Owns one live sub-map, integrated in its keypose-relative frame and keyposed at its first-scan odom pose.
class SubMapBuilder {
public:
  struct Config {
    VoxelHashMap::Config voxel_map = {};
    // Translation in the keypose-relative frame at which the live map closes (m).
    float splitting_distance = 50.0F;
    // Per-scan range filter (m), applied in the base frame before insertion.
    float min_range = 1.0F;
    float max_range = 100.0F;

    std::string to_yaml() const;
  };

  explicit SubMapBuilder(const Config sub_map_config);

  // `base_points` is in the base frame at scan-end time (post-deskew, post-extrinsic); an empty cloud is
  // tolerated, the relative pose is still recorded. Returns the finished sub-map on a split.
  std::optional<FinishedSubMap> add_to_live_map(const std::vector<Eigen::Vector3f>& base_points,
                                                const Nsec end_time,
                                                const Sophus::SE3f& odom_T_base);

  // Kept until the live sub-map closes; dropped when none is open.
  void add_imu_sample(const ImuSample& sample) {
    if (live) {
      live->imu_samples.push_back(sample);
    }
  }

  // empty if no scan ever arrived
  std::optional<FinishedSubMap> finalize();

  Config config;
  KeyposeId next_id = 0;
  struct LiveSubMap {
    SubMap sub_map;
    std::vector<ImuSample> imu_samples;
  };
  std::optional<LiveSubMap> live;
  VoxelHashMap voxel_map;
};

} // namespace rko_slam::core
