#include "rko_slam/core/slam.hpp"

#include <UTL/profiler.hpp>
#include <cmath>
#include <filesystem>
#include <format>
#include <future>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>

#include "rko_slam/core/closure.hpp"
#include "rko_slam/core/run_artifacts.hpp"

namespace rko_slam::core {

SLAM::SLAM(const Config slam_config, const VoxelHashMap::Config& voxel_map_config)
    : config(slam_config),
      voxel_map_config(voxel_map_config),
      pose_graph(config.pose_graph),
      closure_detector(config.closure_detector) {
  if (config.closure_detector.no_of_sub_maps_to_skip < 1) {
    throw std::invalid_argument("no_of_sub_maps_to_skip must be >= 1: adjacent sub-maps are not loop closures");
  }
}

std::optional<std::pair<KeyposeId, KeyposeId>>
SLAM::process_finished_sub_map(std::unique_ptr<SubMap> sub_map, const std::vector<Eigen::Vector3f>& points) {
  UTL_PROFILER_SCOPE("SLAM::process_finished_sub_map");
  if (sub_maps.empty()) {
    pose_graph.add_keypose(sub_map->id, sub_map->odom_T_keypose.cast<double>());
    if (sub_map->measured_up) {
      pose_graph.add_gauge_edge(sub_map->id);
    } else {
      // nothing else anchors the graph yet, and the solve is singular without an anchor
      pose_graph.set_keypose_fixed(sub_map->id, true);
      spdlog::warn("no gravity measurement in the first sub-map: holding its keypose fixed until one arrives. "
                   "running rko_slam without an IMU is a suboptimal way to run it");
    }
  }

  sub_maps.push_back(std::move(sub_map));
  SubMap& just_finished = *sub_maps.back();

  if (just_finished.odom_T_next_keypose) {
    const Sophus::SE3d keypose_T_next_keypose =
        (just_finished.odom_T_keypose.inverse() * *just_finished.odom_T_next_keypose).cast<double>();
    pose_graph.add_keypose(just_finished.id + 1, pose_graph.get_keypose(just_finished.id) * keypose_T_next_keypose);
    pose_graph.add_odom_edge(just_finished.id, just_finished.id + 1, keypose_T_next_keypose);
  }
  if (just_finished.measured_up) {
    const KeyposeId first_id = sub_maps.front()->id;
    if (pose_graph.is_keypose_fixed(first_id)) {
      // gravity observes roll and pitch from here on, and a fully fixed keypose would fight those edges
      pose_graph.set_keypose_fixed(first_id, false);
      pose_graph.add_gauge_edge(first_id);
    }
    pose_graph.add_gravity_edge(just_finished.id, just_finished.measured_up->cast<double>());
  }

  std::optional<std::pair<KeyposeId, KeyposeId>> accepted;
  const std::optional<ClosureCandidate> candidate = closure_detector.query(just_finished.id, points);
  if (candidate) {
    const SubMap& source = *sub_maps.at(candidate->source_id);
    const SubMap& target = *sub_maps.at(candidate->target_id);
    if (!source.centroids.empty() && !target.centroids.empty()) {
      const ClosureRefinement refinement =
          refine_closure(voxel_map_config.voxel_size, config.closure_detector.correspondence_distance(), source, target,
                         candidate->target_T_source);
      if (refinement.overlap >= config.closure_overlap_threshold) {
        pose_graph.add_closure_edge(candidate->source_id, candidate->target_id,
                                    refinement.refined_target_T_source.cast<double>().inverse());
        accepted = {candidate->source_id, candidate->target_id};
      }
    }
  }

  if (!pose_graph.optimize() && accepted) {
    pose_graph.remove_closure_edge(accepted->first, accepted->second);
    accepted.reset();
  }
  update_keyposes(just_finished);
  return accepted;
}

std::shared_ptr<const Keyposes> SLAM::latest_keyposes() const { return keyposes.load(std::memory_order_acquire); }

void SLAM::save_run_artifacts(const std::filesystem::path& dir, const std::string_view run_name) const {
  // Output is time-ordered.
  std::vector<TrajectorySample> trajectory;
  for (const auto& sub_map : sub_maps) {
    const Sophus::SE3f keypose = pose_graph.get_keypose(sub_map->id).cast<float>();
    for (std::size_t i = 0; i < sub_map->local_trajectory.size(); ++i) {
      trajectory.push_back({.time = sub_map->scan_times.at(i), .pose = keypose * sub_map->local_trajectory.at(i)});
    }
  }
  const std::filesystem::path trajectory_path = dir / std::format("{}_tum.txt", run_name);
  if (!write_tum(trajectory_path, trajectory)) {
    spdlog::error("failed to write {}", trajectory_path.string());
  }
  const std::filesystem::path keypose_graph_path = dir / std::format("{}_keypose_graph.g2o", run_name);
  if (!pose_graph.save(keypose_graph_path)) {
    spdlog::error("failed to write {}", keypose_graph_path.string());
  }

  std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> closures;
  for (const auto& edge : pose_graph.se3_edges()) {
    if (is_closure_pair(edge.from_id, edge.to_id)) {
      closures.emplace_back(pose_graph.get_keypose(edge.from_id).translation().cast<float>(),
                            pose_graph.get_keypose(edge.to_id).translation().cast<float>());
    }
  }
  if (!trajectory.empty()) {
    const std::filesystem::path trajectory_png = dir / std::format("{}_trajectory.png", run_name);
    if (!write_trajectory_png(trajectory_png, trajectory, closures)) {
      spdlog::error("failed to write {}", trajectory_png.string());
    }
  }
}

void SLAM::update_keyposes(const SubMap& just_finished) {
  auto updated = std::make_shared<Keyposes>();
  const std::size_t n_keyposes = sub_maps.size() + 1;
  updated->map_T_keypose.reserve(n_keyposes);
  updated->odom_T_keypose.reserve(n_keyposes);
  for (const auto& sub_map : sub_maps) {
    updated->map_T_keypose.push_back(pose_graph.get_keypose(sub_map->id).cast<float>());
    updated->odom_T_keypose.push_back(sub_map->odom_T_keypose);
  }
  if (just_finished.odom_T_next_keypose) {
    const KeyposeId live = just_finished.id + 1;
    updated->map_T_keypose.push_back(pose_graph.get_keypose(live).cast<float>());
    updated->odom_T_keypose.push_back(*just_finished.odom_T_next_keypose);
  }
  keyposes.store(std::move(updated), std::memory_order_release);
}

} // namespace rko_slam::core
