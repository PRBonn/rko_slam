#include "rko_slam/core/slam.hpp"

#include "rko_slam/pgo/io.hpp"

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

SLAM::SLAM(const Config slam_config, const float sub_map_voxel_size)
    : config(slam_config),
      pose_graph(config.pose_graph),
      closure_detector(config.closure_detector),
      sub_map_voxel_size(sub_map_voxel_size) {
  if (config.closure_detector.no_of_sub_maps_to_skip < 1) {
    throw std::invalid_argument("no_of_sub_maps_to_skip must be >= 1: adjacent sub-maps are not loop closures");
  }
}

std::optional<std::pair<std::size_t, std::size_t>>
SLAM::process_finished_sub_map(std::unique_ptr<SubMap> sub_map, const std::vector<Eigen::Vector3f>& points) {
  UTL_PROFILER_SCOPE("SLAM::process_finished_sub_map");
  if (sub_maps.empty()) {
    pose_graph.add_keypose(sub_map->odom_T_keypose.cast<double>());
    pose_graph.anchor_at(sub_map->id);
    if (!sub_map->measured_up) {
      spdlog::warn("no gravity measurement in the first sub-map. running rko_slam without an IMU is a suboptimal "
                   "way to run it");
    }
  }

  sub_maps.push_back(std::move(sub_map));
  SubMap& just_finished = *sub_maps.back();

  if (just_finished.odom_T_next_keypose) {
    const Sophus::SE3d keypose_T_next_keypose =
        (just_finished.odom_T_keypose.inverse() * *just_finished.odom_T_next_keypose).cast<double>();
    const std::size_t next_id =
        pose_graph.add_keypose(pose_graph.keyposes.at(just_finished.id) * keypose_T_next_keypose);
    pose_graph.add_odometry_edge(just_finished.id, next_id, keypose_T_next_keypose);
  }
  if (just_finished.measured_up) {
    pose_graph.add_gravity_edge(just_finished.id, just_finished.measured_up->cast<double>());
  }

  std::optional<std::pair<std::size_t, std::size_t>> accepted;
  const std::optional<ClosureCandidate> candidate = closure_detector.query(just_finished.id, points);
  if (candidate) {
    const SubMap& source = *sub_maps.at(candidate->source_id);
    const SubMap& target = *sub_maps.at(candidate->target_id);
    if (!source.centroids.empty() && !target.centroids.empty()) {
      const ClosureRefinement refinement =
          refine_closure(sub_map_voxel_size, config.closure_detector.correspondence_distance(), source, target,
                         candidate->target_T_source);
      if (refinement.overlap >= config.closure_overlap_threshold) {
        pose_graph.add_closure_edge(candidate->source_id, candidate->target_id,
                                    refinement.refined_target_T_source.cast<double>().inverse());
        accepted = {candidate->source_id, candidate->target_id};
      }
    }
  }

  const pgo::PoseGraph::Outcome outcome = pose_graph.optimize();
  spdlog::debug("pose graph optimization: {}", pgo::to_string(outcome));
  if (outcome == pgo::PoseGraph::Outcome::failed && accepted) {
    spdlog::warn("pose graph optimization failed; dropping closure {} -> {}", accepted->first, accepted->second);
    pose_graph.remove_last_pose_edge();
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
    const Sophus::SE3f keypose = pose_graph.keyposes.at(sub_map->id).cast<float>();
    for (std::size_t i = 0; i < sub_map->local_trajectory.size(); ++i) {
      trajectory.push_back({.time = sub_map->scan_times.at(i), .pose = keypose * sub_map->local_trajectory.at(i)});
    }
  }
  const std::filesystem::path trajectory_path = dir / std::format("{}_tum.txt", run_name);
  if (!write_tum(trajectory_path, trajectory)) {
    spdlog::error("failed to write {}", trajectory_path.string());
  }
  const std::filesystem::path keypose_graph_path = dir / std::format("{}_keypose_graph.g2o", run_name);
  if (!pgo::save(pose_graph, keypose_graph_path)) {
    spdlog::error("failed to write {}", keypose_graph_path.string());
  }

  std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> closures;
  for (const pgo::PoseEdge& edge : pose_graph.pose_edges) {
    if (edge.kind == pgo::PoseEdge::Kind::closure) {
      closures.emplace_back(pose_graph.keyposes.at(edge.from_id).translation().cast<float>(),
                            pose_graph.keyposes.at(edge.to_id).translation().cast<float>());
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
  updated->map_T_keypose.reserve(sub_maps.size() + 1);
  for (const auto& sub_map : sub_maps) {
    updated->map_T_keypose.push_back(pose_graph.keyposes.at(sub_map->id).cast<float>());
  }
  Sophus::SE3f odom_T_live = just_finished.odom_T_keypose;
  if (just_finished.odom_T_next_keypose) {
    updated->map_T_keypose.push_back(pose_graph.keyposes.at(just_finished.id + 1).cast<float>());
    odom_T_live = *just_finished.odom_T_next_keypose;
  }
  updated->map_T_odom = updated->map_T_keypose.back() * odom_T_live.inverse();
  keyposes.store(std::move(updated), std::memory_order_release);
}

} // namespace rko_slam::core
