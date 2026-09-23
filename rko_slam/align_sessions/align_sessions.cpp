#include "rko_slam/align_sessions/align_sessions.hpp"

#include "rko_slam/pgo/io.hpp"

#include <algorithm>
#include <charconv>
#include <cstdint>
#include <format>
#include <optional>
#include <ranges>
#include <rko_lio/core/error.hpp>
#include <stdexcept>
#include <system_error>
#include <unordered_map>
#include <utility>

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include "rko_slam/core/closure.hpp"
#include "rko_slam/core/run_artifacts.hpp"
#include "rko_slam/core/sub_map.hpp"
#include "rko_slam/core/voxel_hash_map.hpp"
#include "rko_slam/pgo/pose_graph.hpp"

namespace rko_slam::align_sessions {

namespace {

namespace fs = std::filesystem;

struct SessionSubMap {
  fs::path ply;
  core::Nsec keypose_time{0};
  Sophus::SE3d map_T_keypose;
};

struct Session {
  fs::path run_dir;
  std::vector<SessionSubMap> sub_maps;
  std::vector<pgo::PoseEdge> pose_edges;
  std::vector<pgo::GravityEdge> gravity_edges;
  std::vector<core::TrajectorySample> tum;
  core::VoxelHashMap::Config voxel_map_config;
};

struct SubMapIndex {
  std::size_t session = 0;
  std::size_t sub_map = 0; // session-local
};

struct InterSessionClosure {
  SubMapIndex source;
  SubMapIndex target;
  std::size_t inliers = 0;
  Sophus::SE3d refined_source_T_target;
};

std::vector<std::pair<core::Nsec, fs::path>> sub_map_plys(const fs::path& run_dir) {
  const fs::path dir = run_dir / "sub_maps";
  if (!fs::is_directory(dir)) {
    throw rko_lio::core::InputError(run_dir.string() + ": no sub_maps/ - rerun this session with dump_sub_maps:=true");
  }
  std::vector<std::pair<core::Nsec, fs::path>> plys;
  for (const fs::directory_entry& entry : fs::directory_iterator(dir)) {
    const std::string stem = entry.path().stem().string();
    if (entry.path().extension() != ".ply" || !stem.starts_with("sub_map_")) {
      continue;
    }
    const std::size_t split = stem.rfind('_');
    if (split == std::string::npos) {
      throw rko_lio::core::InputError(entry.path().string() + ": no keypose timestamp in the name");
    }
    const std::string_view stamp = std::string_view(stem).substr(split + 1);
    std::int64_t nanoseconds = 0;
    if (std::from_chars(stamp.data(), stamp.data() + stamp.size(), nanoseconds).ec != std::errc{}) {
      throw rko_lio::core::InputError(entry.path().string() + ": no keypose timestamp in the name");
    }
    plys.emplace_back(core::Nsec{nanoseconds}, entry.path());
  }
  std::ranges::sort(plys);
  return plys;
}

Session load_session(const fs::path& run_dir) {
  Session session;
  session.run_dir = run_dir;
  const std::string stem = run_dir.filename().string();
  const std::vector<std::pair<core::Nsec, fs::path>> plys = sub_map_plys(run_dir);
  const YAML::Node config = YAML::LoadFile((run_dir / (stem + "_config.yaml")).string());
  session.voxel_map_config = {
      .voxel_size = config["voxel_size"].as<float>(),
      .max_points_per_voxel = config["max_points_per_voxel"].as<unsigned int>(),
  };

  pgo::PoseGraph keypose_graph = pgo::load(run_dir / (stem + "_keypose_graph.g2o"));
  if (keypose_graph.keyposes.size() != plys.size()) {
    throw rko_lio::core::InputError(std::format("{}: keypose graph has {} vertices but sub_maps/ has {} sub-maps",
                                                run_dir.string(), keypose_graph.keyposes.size(), plys.size()));
  }

  session.pose_edges = std::move(keypose_graph.pose_edges);
  session.gravity_edges = std::move(keypose_graph.gravity_edges);
  session.tum = core::read_tum(run_dir / (stem + "_tum.txt"));

  session.sub_maps.reserve(plys.size());
  for (std::size_t id = 0; id < plys.size(); ++id) {
    session.sub_maps.push_back({
        .ply = plys.at(id).second,
        .keypose_time = plys.at(id).first,
        .map_T_keypose = keypose_graph.keyposes.at(id),
    });
  }
  return session;
}

std::unique_ptr<core::SubMap> rebuild_sub_map(const core::VoxelHashMap::Config& voxel_map_config,
                                              const std::vector<Eigen::Vector3f>& cloud,
                                              const std::size_t local_id) {
  core::VoxelHashMap voxel_map(voxel_map_config);
  voxel_map.add_points(cloud);
  auto sub_map = std::make_unique<core::SubMap>();
  sub_map->id = local_id;
  core::fill_sub_map(voxel_map, *sub_map);
  return sub_map;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity) c++23 views flatten the loop nest
std::vector<InterSessionClosure> find_inter_session_closures(const std::vector<Session>& sessions,
                                                             const core::ClosureDetector::Config& detector_config,
                                                             const core::VoxelHashMap::Config& voxel_map_config,
                                                             const float overlap_threshold) {
  const float max_correspondence_distance = detector_config.correspondence_distance();

  core::ClosureDetector detector(detector_config);
  // The detector numbers sub-maps in the order it is given them, so its id indexes this.
  std::vector<SubMapIndex> registered;
  std::unordered_map<std::size_t, std::unique_ptr<core::SubMap>> cache;
  const auto sub_map_for = [&](const std::size_t detector_id) -> const core::SubMap& {
    auto cached = cache.find(detector_id);
    if (cached == cache.end()) {
      const SubMapIndex index = registered.at(detector_id);
      const fs::path& ply = sessions.at(index.session).sub_maps.at(index.sub_map).ply;
      cached =
          cache.emplace(detector_id, rebuild_sub_map(voxel_map_config, core::read_ply_xyz(ply), index.sub_map)).first;
    }
    return *cached->second;
  };

  std::vector<InterSessionClosure> accepted_closures;
  std::size_t candidates_considered = 0;
  std::size_t rejected_below_inliers = 0;
  std::size_t rejected_by_overlap = 0;
  for (std::size_t session_index = 0; session_index < sessions.size(); ++session_index) {
    const Session& session = sessions.at(session_index);
    for (std::size_t id = 0; id < session.sub_maps.size(); ++id) {
      const std::size_t detector_id = registered.size();
      registered.push_back({.session = session_index, .sub_map = id});
      const std::vector<Eigen::Vector3f> cloud = core::read_ply_xyz(session.sub_maps.at(id).ply);
      const std::vector<core::ClosureCandidate> candidates = detector.query_all(detector_id, cloud);
      for (const auto& candidate : candidates) {
        const SubMapIndex source_index = registered.at(candidate.source_id);
        const SubMapIndex target_index = registered.at(candidate.target_id);
        if (source_index.session == target_index.session) {
          continue; // intra-session: already an edge in that session's graph
        }
        ++candidates_considered;
        if (std::cmp_less(candidate.number_of_inliers, detector_config.inliers_threshold)) {
          ++rejected_below_inliers;
          continue;
        }
        if (!cache.contains(detector_id)) {
          cache.emplace(detector_id, rebuild_sub_map(voxel_map_config, cloud, id));
        }
        const core::SubMap& source = sub_map_for(candidate.source_id);
        const core::SubMap& target = sub_map_for(candidate.target_id);
        if (source.centroids.empty() || target.centroids.empty()) {
          throw rko_lio::core::InputError(
              std::format("sub-map {} of session {} or sub-map {} of session {} has no points; "
                          "the run dir they were read from is incomplete",
                          source_index.sub_map, source_index.session, target_index.sub_map, target_index.session));
        }
        const core::ClosureRefinement refinement = core::refine_closure(
            voxel_map_config.voxel_size, max_correspondence_distance, source, target, candidate.target_T_source);
        if (refinement.overlap < overlap_threshold) {
          ++rejected_by_overlap;
          continue;
        }
        accepted_closures.push_back({
            .source = source_index,
            .target = target_index,
            .inliers = candidate.number_of_inliers,
            .refined_source_T_target = refinement.refined_target_T_source.cast<double>().inverse(),
        });
      }
    }
  }

  if (accepted_closures.empty()) {
    spdlog::info("no inter-session closures validated: 0 of {} candidates across {} sessions ({} below "
                 "inliers_threshold, {} failed overlap validation). Tune closure detection "
                 "(density_map_resolution / hamming_distance_threshold / inliers_threshold / overlap_threshold) "
                 "if these sessions should overlap.",
                 candidates_considered, sessions.size(), rejected_below_inliers, rejected_by_overlap);
  }
  return accepted_closures;
}

std::size_t session_reaching_most_others(const std::size_t n_sessions,
                                         const std::vector<InterSessionClosure>& accepted_closures) {
  std::vector<std::vector<std::size_t>> neighbours(n_sessions);
  for (const InterSessionClosure& closure : accepted_closures) {
    neighbours.at(closure.source.session).push_back(closure.target.session);
    neighbours.at(closure.target.session).push_back(closure.source.session);
  }
  std::vector<bool> reached(n_sessions, false);
  std::size_t best_start = 0;
  std::size_t most_reached = 0;
  for (std::size_t start = 0; start < n_sessions; ++start) {
    if (reached.at(start)) {
      continue;
    }
    std::size_t n_reached = 0;
    std::vector<std::size_t> to_visit{start};
    reached.at(start) = true;
    while (!to_visit.empty()) {
      const std::size_t current = to_visit.back();
      to_visit.pop_back();
      ++n_reached;
      for (const std::size_t neighbour : neighbours.at(current)) {
        if (!reached.at(neighbour)) {
          reached.at(neighbour) = true;
          to_visit.push_back(neighbour);
        }
      }
    }
    if (n_reached > most_reached) {
      most_reached = n_reached;
      best_start = start;
    }
  }
  return best_start;
}

// `reference` is the world frame; every other session is placed through its strongest closure to a placed one
std::vector<std::optional<Sophus::SE3d>> anchor_sessions(const std::vector<Session>& sessions,
                                                         const std::vector<InterSessionClosure>& accepted_closures,
                                                         const std::size_t reference) {
  struct Placed {
    std::size_t session_index;
    Sophus::SE3d world_T_map;
  };
  std::vector<std::optional<Sophus::SE3d>> world_T_session(sessions.size());
  world_T_session.at(reference) = Sophus::SE3d{};
  std::vector<Placed> to_visit{{.session_index = reference, .world_T_map = Sophus::SE3d{}}};
  while (!to_visit.empty()) {
    const Placed placed = to_visit.back();
    to_visit.pop_back();
    for (std::size_t next_session = 0; next_session < sessions.size(); ++next_session) {
      if (world_T_session.at(next_session)) {
        continue;
      }
      auto connecting =
          accepted_closures | std::views::filter([&](const InterSessionClosure& closure) {
            return (closure.source.session == placed.session_index && closure.target.session == next_session) ||
                   (closure.source.session == next_session && closure.target.session == placed.session_index);
          });
      const auto best = std::ranges::max_element(connecting, {}, &InterSessionClosure::inliers);
      if (best == std::ranges::end(connecting)) {
        continue;
      }
      // stored source-to-target, but either end may be the already-placed session
      const bool source_is_placed = best->source.session == placed.session_index;
      const SubMapIndex& placed_index = source_is_placed ? best->source : best->target;
      const SubMapIndex& new_index = source_is_placed ? best->target : best->source;
      const Sophus::SE3d placed_keypose_T_new_keypose =
          source_is_placed ? best->refined_source_T_target : best->refined_source_T_target.inverse();

      const Sophus::SE3d world_T_placed_keypose =
          placed.world_T_map * sessions.at(placed_index.session).sub_maps.at(placed_index.sub_map).map_T_keypose;
      const Sophus::SE3d world_T_new_keypose = world_T_placed_keypose * placed_keypose_T_new_keypose;
      const Sophus::SE3d world_T_next_map =
          world_T_new_keypose * sessions.at(new_index.session).sub_maps.at(new_index.sub_map).map_T_keypose.inverse();
      world_T_session.at(next_session) = world_T_next_map;
      to_visit.push_back({.session_index = next_session, .world_T_map = world_T_next_map});
    }
  }
  return world_T_session;
}

struct JointGraph {
  pgo::PoseGraph pose_graph;
  // Indexed by session and then by that session's own sub-map id, giving its keypose in the joint graph. The inner
  // vector is empty for a session that was dropped.
  std::vector<std::vector<std::size_t>> keypose_ids;
};

JointGraph build_joint_graph(const std::vector<Session>& sessions,
                             const std::vector<std::optional<Sophus::SE3d>>& world_T_session,
                             const std::vector<InterSessionClosure>& accepted_closures,
                             const std::size_t reference,
                             const pgo::PoseGraph::Config& pose_graph_config) {
  JointGraph joint{.pose_graph = pgo::PoseGraph(pose_graph_config)};
  joint.keypose_ids.resize(sessions.size());
  for (std::size_t session_index = 0; session_index < sessions.size(); ++session_index) {
    const std::optional<Sophus::SE3d>& world_T_map = world_T_session.at(session_index);
    if (!world_T_map) {
      continue;
    }
    const Session& session = sessions.at(session_index);
    std::vector<std::size_t>& keypose_ids = joint.keypose_ids.at(session_index);
    for (const SessionSubMap& sub_map : session.sub_maps) {
      keypose_ids.push_back(joint.pose_graph.add_keypose(world_T_map.value() * sub_map.map_T_keypose));
    }
    if (session_index == reference) {
      joint.pose_graph.anchor_at(keypose_ids.front());
    }
    for (const pgo::PoseEdge& edge : session.pose_edges) {
      const std::size_t from_id = keypose_ids.at(edge.from_id);
      const std::size_t to_id = keypose_ids.at(edge.to_id);
      if (edge.kind == pgo::PoseEdge::Kind::closure) {
        joint.pose_graph.add_closure_edge(from_id, to_id, edge.from_T_to);
      } else {
        joint.pose_graph.add_odometry_edge(from_id, to_id, edge.from_T_to);
      }
    }
    for (const pgo::GravityEdge& gravity : session.gravity_edges) {
      joint.pose_graph.add_gravity_edge(keypose_ids.at(gravity.keypose_id), gravity.measured_up);
    }
  }
  for (const InterSessionClosure& closure : accepted_closures) {
    const std::vector<std::size_t>& source_ids = joint.keypose_ids.at(closure.source.session);
    const std::vector<std::size_t>& target_ids = joint.keypose_ids.at(closure.target.session);
    if (source_ids.empty() || target_ids.empty()) {
      continue;
    }
    joint.pose_graph.add_closure_edge(source_ids.at(closure.source.sub_map), target_ids.at(closure.target.sub_map),
                                      closure.refined_source_T_target);
  }
  if (joint.pose_graph.gravity_edges.empty()) {
    spdlog::warn("no gravity edges in any aligned session. running rko_slam without an IMU is a suboptimal way to "
                 "run it");
  }
  return joint;
}

std::vector<core::TrajectorySample>
deform_trajectory(const Session& session, const std::vector<std::size_t>& keypose_ids, const pgo::PoseGraph& joint) {
  std::vector<core::TrajectorySample> world_trajectory;
  world_trajectory.reserve(session.tum.size());
  std::size_t row = 0;
  for (std::size_t id = 0; id < session.sub_maps.size(); ++id) {
    const SessionSubMap& sub_map = session.sub_maps.at(id);
    const Sophus::SE3f world_T_map =
        (joint.keyposes.at(keypose_ids.at(id)) * sub_map.map_T_keypose.inverse()).cast<float>();
    const bool last = id + 1 == session.sub_maps.size();
    while (row < session.tum.size() && (last || session.tum.at(row).time < session.sub_maps.at(id + 1).keypose_time)) {
      world_trajectory.push_back({.time = session.tum.at(row).time, .pose = world_T_map * session.tum.at(row).pose});
      ++row;
    }
  }
  return world_trajectory;
}

} // namespace

std::optional<AlignResult> align(const core::ClosureDetector::Config& detector_config,
                                 const float overlap_threshold,
                                 const pgo::PoseGraph::Config& pose_graph_config,
                                 const std::vector<std::filesystem::path>& run_dirs,
                                 const std::filesystem::path& output_dir,
                                 const std::string_view run_name) {
  AlignResult result;
  if (run_dirs.size() < 2) {
    throw rko_lio::core::InputError("align_sessions needs at least 2 run_dirs");
  }

  std::vector<Session> sessions;
  sessions.reserve(run_dirs.size());
  for (const auto& dir : run_dirs) {
    sessions.push_back(load_session(dir));
  }

  const core::VoxelHashMap::Config voxel_map_config = sessions.front().voxel_map_config;
  for (const Session& session : sessions) {
    if (session.voxel_map_config != voxel_map_config) {
      throw rko_lio::core::InputError(
          std::format("sub_map voxelisation differs across sessions: {} ran with voxel_size={} "
                      "max_points_per_voxel={}, {} with voxel_size={} max_points_per_voxel={}",
                      sessions.front().run_dir.string(), voxel_map_config.voxel_size,
                      voxel_map_config.max_points_per_voxel, session.run_dir.string(),
                      session.voxel_map_config.voxel_size, session.voxel_map_config.max_points_per_voxel));
    }
  }
  result.voxel_map_config = voxel_map_config;

  const std::vector<InterSessionClosure> accepted_closures =
      find_inter_session_closures(sessions, detector_config, voxel_map_config, overlap_threshold);
  if (accepted_closures.empty()) {
    return std::nullopt;
  }

  const std::size_t reference = session_reaching_most_others(sessions.size(), accepted_closures);
  result.reference_session = reference;
  const std::vector<std::optional<Sophus::SE3d>> world_T_session =
      anchor_sessions(sessions, accepted_closures, reference);

  for (std::size_t session_index = 0; session_index < sessions.size(); ++session_index) {
    if (!world_T_session.at(session_index)) {
      result.dropped_sessions.push_back(session_index);
    }
  }
  if (sessions.size() - result.dropped_sessions.size() < 2) {
    spdlog::info("no alignment: no session has a validated closure into any other; reference session {} ({}) is "
                 "alone and the remaining {} session(s) are unreachable from it. Tune closure detection.",
                 reference, sessions.at(reference).run_dir.string(), result.dropped_sessions.size());
    return std::nullopt;
  }

  JointGraph joint = build_joint_graph(sessions, world_T_session, accepted_closures, reference, pose_graph_config);
  const pgo::PoseGraph::Outcome outcome = joint.pose_graph.optimize();
  if (outcome == pgo::PoseGraph::Outcome::failed) {
    throw std::runtime_error(std::format(
        "joint pose-graph optimization failed ({} inter-session closures); nothing written", accepted_closures.size()));
  }
  spdlog::info("joint pose-graph optimization: {}", pgo::to_string(outcome));

  auto [resolved_name, out_dir] = core::resolve_run_dir(output_dir, run_name);
  result.out_run_name = resolved_name;
  result.out_run_dir = out_dir;
  const fs::path joint_path = out_dir / (resolved_name + "_joint_keypose_graph.g2o");
  if (!pgo::save(joint.pose_graph, joint_path)) {
    spdlog::error("failed to write {}", joint_path.string());
  }

  for (std::size_t session_index = 0; session_index < sessions.size(); ++session_index) {
    if (!world_T_session.at(session_index)) {
      continue;
    }
    const fs::path tum_path = out_dir / std::format("{}_session_{}_tum.txt", resolved_name, session_index);
    if (!core::write_tum(tum_path, deform_trajectory(sessions.at(session_index), joint.keypose_ids.at(session_index),
                                                     joint.pose_graph))) {
      spdlog::error("failed to write {}", tum_path.string());
    }
  }

  result.accepted_closures = accepted_closures.size();
  return result;
}

} // namespace rko_slam::align_sessions
