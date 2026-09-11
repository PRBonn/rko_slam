#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "rko_slam/core/closure.hpp"
#include "rko_slam/core/pose_graph.hpp"
#include "rko_slam/core/voxel_hash_map.hpp"

namespace rko_slam::align_sessions {

struct AlignResult {
  std::size_t accepted_closures = 0;
  // Indices into the input run_dirs.
  std::size_t reference_session = 0;
  std::vector<std::size_t> dropped_sessions;
  std::filesystem::path out_run_dir;
  std::string out_run_name;
  core::VoxelHashMap::Config voxel_map_config;
};

// Inter-session closures only, one graph gauged on the reference session's first keypose. Sessions with no
// closure path to the reference are dropped. Sub-maps are read from each run_dir's own `sub_maps/`. Throws
// rko_lio::core::InputError on bad input and std::runtime_error if the joint optimization fails; nothing is
// written in either case.
std::optional<AlignResult> align(const core::ClosureDetector::Config& detector_config,
                                 const float overlap_threshold,
                                 const core::PoseGraph::Config& pose_graph_config,
                                 const std::vector<std::filesystem::path>& run_dirs,
                                 const std::filesystem::path& output_dir,
                                 const std::string_view run_name);

} // namespace rko_slam::align_sessions
