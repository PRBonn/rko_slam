#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <optional>
#include <sophus/se3.hpp>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "rko_slam/core/closure.hpp"
#include "rko_slam/core/pose_graph.hpp"
#include "rko_slam/core/sub_map.hpp"
#include "rko_slam/core/types.hpp"
#include "rko_slam/core/voxel_hash_map.hpp"

namespace rko_slam::core {

// Index is the keypose id; `back()` is the live keypose.
struct Keyposes {
  std::vector<Sophus::SE3f> map_T_keypose;
  std::vector<Sophus::SE3f> odom_T_keypose;
};

class SLAM {
public:
  struct Config {
    PoseGraph::Config pose_graph = {};
    ClosureDetector::Config closure_detector = {};
    float closure_overlap_threshold = ClosureRefinement::kDefaultOverlapThreshold;
  };

  // `voxel_map_config` must be the sub-map builder's.
  SLAM(const Config slam_config, const VoxelHashMap::Config& voxel_map_config);

  SLAM(const SLAM&) = delete;
  SLAM& operator=(const SLAM&) = delete;
  SLAM(SLAM&&) = delete;
  SLAM& operator=(SLAM&&) = delete;
  ~SLAM() = default;

  std::optional<std::pair<KeyposeId, KeyposeId>> process_finished_sub_map(std::unique_ptr<SubMap> sub_map,
                                                                          const std::vector<Eigen::Vector3f>& points);

  // Safe from any thread. nullptr until the first process_finished_sub_map.
  std::shared_ptr<const Keyposes> latest_keyposes() const;

  // Not concurrent with process_finished_sub_map.
  void save_run_artifacts(const std::filesystem::path& dir, const std::string_view run_name) const;

  Config config;
  PoseGraph pose_graph;
  ClosureDetector closure_detector;
  // Finished sub-maps, indexed by id.
  std::vector<std::unique_ptr<SubMap>> sub_maps;

private:
  VoxelHashMap::Config voxel_map_config;

  void update_keyposes(const SubMap& just_finished);

  std::atomic<std::shared_ptr<const Keyposes>> keyposes{nullptr};
};

} // namespace rko_slam::core
