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

#include "rko_slam/closures/detector.hpp"
#include "rko_slam/closures/refinement.hpp"
#include "rko_slam/core/sub_map.hpp"
#include "rko_slam/core/voxel_hash_map.hpp"
#include "rko_slam/pgo/pose_graph.hpp"

namespace rko_slam::core {

// Index is the keypose id; `back()` is the live keypose, and map_T_odom is its map_T_keypose * keypose_T_odom.
struct Keyposes {
  std::vector<Sophus::SE3f> map_T_keypose;
  Sophus::SE3f map_T_odom;
};

class SLAM {
public:
  struct Config {
    pgo::PoseGraph::Config pose_graph = {};
    closures::ClosureDetector::Config closure_detector = {};
    float closure_overlap_threshold = closures::ClosureRefinement::kDefaultOverlapThreshold;
  };

  SLAM(const Config slam_config, float sub_map_voxel_size);

  SLAM(const SLAM&) = delete;
  SLAM& operator=(const SLAM&) = delete;
  SLAM(SLAM&&) = delete;
  SLAM& operator=(SLAM&&) = delete;
  ~SLAM() = default;

  std::optional<std::pair<std::size_t, std::size_t>>
  process_finished_sub_map(std::unique_ptr<SubMap> sub_map, const std::vector<Eigen::Vector3f>& points);

  // Safe from any thread. nullptr until the first process_finished_sub_map.
  std::shared_ptr<const Keyposes> latest_keyposes() const;

  // Not concurrent with process_finished_sub_map.
  void save_run_artifacts(const std::filesystem::path& dir, const std::string_view run_name) const;

  const Config config;
  pgo::PoseGraph pose_graph;
  closures::ClosureDetector closure_detector;
  // Finished sub-maps, indexed by id.
  std::vector<std::unique_ptr<SubMap>> sub_maps;

private:
  float sub_map_voxel_size;

  void update_keyposes(const SubMap& just_finished);

  std::atomic<std::shared_ptr<const Keyposes>> keyposes{nullptr};
};

} // namespace rko_slam::core
