#pragma once

#include <cstdint>
#include <filesystem>
#include <sophus/se3.hpp>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "rko_slam/core/pose_graph.hpp"
#include "rko_slam/core/types.hpp"
#include "rko_slam/core/voxel_hash_map.hpp"

namespace rko_slam::core {

// Binary little-endian PLY, float32 xyz, in the cloud's own frame. Creates the parent directory.
bool write_ply_xyz(const std::filesystem::path& path, const std::vector<Eigen::Vector3f>& points);
// Throws rko_lio::core::InputError on a missing file or an unexpected header.
std::vector<Eigen::Vector3f> read_ply_xyz(const std::filesystem::path& path);

// Creates <parent>/<run_name>_<idx> at the smallest free idx; returns (resolved_run_name, resolved_path).
std::pair<std::string, std::filesystem::path> resolve_run_dir(const std::filesystem::path& parent,
                                                              const std::string_view run_name);

// top-down XY
bool write_trajectory_png(const std::filesystem::path& path,
                          const std::vector<TrajectorySample>& trajectory,
                          const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& closures);

// TUM trajectory text I/O (timestamp tx ty tz qx qy qz qw per row).
bool write_tum(const std::filesystem::path& path, const std::vector<TrajectorySample>& samples);
std::vector<TrajectorySample> read_tum(const std::filesystem::path& path);

} // namespace rko_slam::core
