#include "rko_slam/align_sessions/align_sessions.hpp"

#include <algorithm>
#include <charconv>
#include <cstdint>
#include <format>
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

namespace rko_slam::align_sessions {

namespace {

namespace fs = std::filesystem;

struct SessionSubMap {
  fs::path ply;
  std::size_t n_scans = 0;
  Sophus::SE3d map_T_keypose;
};

struct Session {
  fs::path run_dir;
  std::vector<SessionSubMap> sub_maps;
  std::vector<core::PoseGraph::EdgeView> edges;
  std::vector<core::TrajectorySample> tum;
  core::VoxelHashMap::Config voxel_map_config;
  core::KeyposeId global_id_offset = 0; // global id of this session's keypose 0
};

struct InterSessionClosure {
  std::size_t source_session_index = 0;
  std::size_t target_session_index = 0;
  core::KeyposeId source_sub_map_id = 0; // session-local sub_map ids
  core::KeyposeId target_sub_map_id = 0;
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

  core::PoseGraph keypose_graph{core::PoseGraph::Config{}};
  const fs::path graph_path = run_dir / (stem + "_keypose_graph.g2o");
  if (!keypose_graph.load(graph_path)) {
    throw rko_lio::core::InputError(run_dir.string() + ": cannot read " + graph_path.filename().string());
  }
  if (keypose_graph.num_keyposes() != plys.size()) {
    throw rko_lio::core::InputError(std::format("{}: keypose graph has {} vertices but sub_maps/ has {} sub-maps",
                                                run_dir.string(), keypose_graph.num_keyposes(), plys.size()));
  }

  session.edges = keypose_graph.edges();
  session.tum = core::read_tum(run_dir / (stem + "_tum.txt"));

  std::vector<std::size_t> keypose_rows;
  keypose_rows.reserve(plys.size());
  std::size_t row = 0;
  for (std::size_t id = 0; id < plys.size(); ++id) {
    const core::Nsec keypose_time = plys.at(id).first;
    while (row < session.tum.size() && session.tum.at(row).time < keypose_time) {
      ++row;
    }
    if (row == session.tum.size() || session.tum.at(row).time != keypose_time) {
      throw rko_lio::core::InputError(std::format("{}: no TUM row at sub-map {}'s keypose time {} - artifacts are from "
                                                  "different runs?",
                                                  run_dir.string(), id, keypose_time.count()));
    }
    keypose_rows.push_back(row);
  }
  if (keypose_rows.front() != 0) {
    throw rko_lio::core::InputError(
        std::format("{}: {} TUM rows precede the first keypose", run_dir.string(), keypose_rows.front()));
  }

  session.sub_maps.reserve(plys.size());
  for (std::size_t id = 0; id < plys.size(); ++id) {
    const std::size_t end = (id + 1 < keypose_rows.size()) ? keypose_rows.at(id + 1) : session.tum.size();
    session.sub_maps.push_back({
        .ply = plys.at(id).second,
        .n_scans = end - keypose_rows.at(id),
        .map_T_keypose = keypose_graph.get_keypose(id),
    });
  }
  return session;
}

std::unique_ptr<core::SubMap> rebuild_sub_map(const core::VoxelHashMap::Config& voxel_map_config,
                                              const std::vector<Eigen::Vector3f>& cloud,
                                              const core::KeyposeId local_id) {
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

  const auto session_of_global_id = [&](const core::KeyposeId global_id) -> std::size_t {
    const auto first_starting_after = std::ranges::upper_bound(sessions, global_id, {}, &Session::global_id_offset);
    return static_cast<std::size_t>(first_starting_after - sessions.begin()) - 1;
  };

  core::ClosureDetector detector(detector_config);
  std::unordered_map<core::KeyposeId, std::unique_ptr<core::SubMap>> cache;
  const auto sub_map_for = [&](const std::size_t session_index, const core::KeyposeId local_id) -> const core::SubMap& {
    const core::KeyposeId global_id = sessions.at(session_index).global_id_offset + local_id;
    auto cached = cache.find(global_id);
    if (cached == cache.end()) {
      const fs::path& ply = sessions.at(session_index).sub_maps.at(local_id).ply;
      cached = cache.emplace(global_id, rebuild_sub_map(voxel_map_config, core::read_ply_xyz(ply), local_id)).first;
    }
    return *cached->second;
  };

  std::vector<InterSessionClosure> accepted_closures;
  std::size_t candidates_considered = 0;
  std::size_t rejected_below_inliers = 0;
  std::size_t rejected_by_overlap = 0;
  for (const Session& session : sessions) {
    for (std::size_t id = 0; id < session.sub_maps.size(); ++id) {
      const core::KeyposeId global_id = session.global_id_offset + id;
      const std::vector<Eigen::Vector3f> cloud = core::read_ply_xyz(session.sub_maps.at(id).ply);
      const std::vector<core::ClosureCandidate> candidates = detector.query_all(global_id, cloud);
      for (const auto& candidate : candidates) {
        const std::size_t source_session_index = session_of_global_id(candidate.source_id);
        const std::size_t target_session_index = session_of_global_id(candidate.target_id);
        if (source_session_index == target_session_index) {
          continue; // intra-session: already an edge in that session's graph
        }
        ++candidates_considered;
        if (candidate.number_of_inliers < detector_config.inliers_threshold) {
          ++rejected_below_inliers;
          continue;
        }
        const core::KeyposeId source_sub_map_id =
            candidate.source_id - sessions.at(source_session_index).global_id_offset;
        const core::KeyposeId target_sub_map_id =
            candidate.target_id - sessions.at(target_session_index).global_id_offset;
        if (!cache.contains(global_id)) {
          cache.emplace(global_id, rebuild_sub_map(voxel_map_config, cloud, id));
        }
        const core::SubMap& source = sub_map_for(source_session_index, source_sub_map_id);
        const core::SubMap& target = sub_map_for(target_session_index, target_sub_map_id);
        if (source.centroids.empty() || target.centroids.empty()) {
          throw rko_lio::core::InputError(
              std::format("sub-map {} of session {} or sub-map {} of session {} has no points; "
                          "the run dir they were read from is incomplete",
                          source_sub_map_id, source_session_index, target_sub_map_id, target_session_index));
        }
        const core::ClosureRefinement refinement = core::refine_closure(
            voxel_map_config.voxel_size, max_correspondence_distance, source, target, candidate.target_T_source);
        if (refinement.overlap < overlap_threshold) {
          ++rejected_by_overlap;
          continue;
        }
        accepted_closures.push_back({
            .source_session_index = source_session_index,
            .target_session_index = target_session_index,
            .source_sub_map_id = source_sub_map_id,
            .target_sub_map_id = target_sub_map_id,
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
    neighbours.at(closure.source_session_index).push_back(closure.target_session_index);
    neighbours.at(closure.target_session_index).push_back(closure.source_session_index);
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
      auto connecting = accepted_closures | std::views::filter([&](const InterSessionClosure& closure) {
                          return (closure.source_session_index == placed.session_index &&
                                  closure.target_session_index == next_session) ||
                                 (closure.source_session_index == next_session &&
                                  closure.target_session_index == placed.session_index);
                        });
      const auto best = std::ranges::max_element(connecting, {}, &InterSessionClosure::inliers);
      if (best == std::ranges::end(connecting)) {
        continue;
      }
      // stored source-to-target, but either end may be the already-placed session
      const bool source_is_placed = best->source_session_index == placed.session_index;
      const core::KeyposeId placed_sub_map_id = source_is_placed ? best->source_sub_map_id : best->target_sub_map_id;
      const core::KeyposeId new_sub_map_id = source_is_placed ? best->target_sub_map_id : best->source_sub_map_id;
      const Sophus::SE3d placed_keypose_T_new_keypose =
          source_is_placed ? best->refined_source_T_target : best->refined_source_T_target.inverse();

      const Sophus::SE3d world_T_placed_keypose =
          placed.world_T_map * sessions.at(placed.session_index).sub_maps.at(placed_sub_map_id).map_T_keypose;
      const Sophus::SE3d world_T_new_keypose = world_T_placed_keypose * placed_keypose_T_new_keypose;
      const Sophus::SE3d world_T_next_map =
          world_T_new_keypose * sessions.at(next_session).sub_maps.at(new_sub_map_id).map_T_keypose.inverse();
      world_T_session.at(next_session) = world_T_next_map;
      to_visit.push_back({.session_index = next_session, .world_T_map = world_T_next_map});
    }
  }
  return world_T_session;
}

std::unique_ptr<core::PoseGraph> build_joint_graph(const std::vector<Session>& sessions,
                                                   const std::vector<std::optional<Sophus::SE3d>>& world_T_session,
                                                   const std::vector<InterSessionClosure>& accepted_closures,
                                                   const std::size_t reference,
                                                   const core::PoseGraph::Config& pose_graph_config) {
  auto joint = std::make_unique<core::PoseGraph>(pose_graph_config);
  for (std::size_t session_index = 0; session_index < sessions.size(); ++session_index) {
    const std::optional<Sophus::SE3d>& world_T_map = world_T_session.at(session_index);
    if (!world_T_map) {
      continue;
    }
    const Session& session = sessions.at(session_index);
    for (std::size_t id = 0; id < session.sub_maps.size(); ++id) {
      joint->add_keypose(session.global_id_offset + id, world_T_map.value() * session.sub_maps.at(id).map_T_keypose);
    }
    for (const core::PoseGraph::EdgeView& edge : session.edges) {
      const core::KeyposeId from_id = session.global_id_offset + edge.from_id;
      const core::KeyposeId to_id = session.global_id_offset + edge.to_id;
      if (core::is_closure_pair(edge.from_id, edge.to_id)) {
        joint->add_closure_edge(from_id, to_id, edge.from_T_to);
      } else {
        joint->add_odom_edge(from_id, to_id, edge.from_T_to);
      }
    }
  }
  for (const InterSessionClosure& closure : accepted_closures) {
    if (!world_T_session.at(closure.source_session_index) || !world_T_session.at(closure.target_session_index)) {
      continue;
    }
    joint->add_closure_edge(sessions.at(closure.source_session_index).global_id_offset + closure.source_sub_map_id,
                            sessions.at(closure.target_session_index).global_id_offset + closure.target_sub_map_id,
                            closure.refined_source_T_target);
  }
  joint->set_keypose_fixed(sessions.at(reference).global_id_offset, true);
  return joint;
}

std::vector<core::TrajectorySample> deform_trajectory(const Session& session, const core::PoseGraph& joint) {
  std::vector<core::TrajectorySample> world_trajectory;
  world_trajectory.reserve(session.tum.size());
  std::size_t row = 0;
  for (std::size_t id = 0; id < session.sub_maps.size(); ++id) {
    const SessionSubMap& sub_map = session.sub_maps.at(id);
    const Sophus::SE3d world_T_keypose = joint.get_keypose(session.global_id_offset + id);
    const Sophus::SE3f world_T_map = (world_T_keypose * sub_map.map_T_keypose.inverse()).cast<float>();
    for (std::size_t k = 0; k < sub_map.n_scans; ++k, ++row) {
      const Sophus::SE3f& map_T_base = session.tum.at(row).pose;
      world_trajectory.push_back({.time = session.tum.at(row).time, .pose = world_T_map * map_T_base});
    }
  }
  return world_trajectory;
}

} // namespace

std::optional<AlignResult> align(const core::ClosureDetector::Config& detector_config,
                                 const float overlap_threshold,
                                 const core::PoseGraph::Config& pose_graph_config,
                                 const std::vector<std::filesystem::path>& run_dirs,
                                 const std::filesystem::path& output_dir,
                                 const std::string_view run_name) {
  AlignResult result;
  if (run_dirs.size() < 2) {
    throw rko_lio::core::InputError("align_sessions needs at least 2 run_dirs");
  }

  std::vector<Session> sessions;
  sessions.reserve(run_dirs.size());
  core::KeyposeId next_global_id_offset = 0;
  for (const auto& dir : run_dirs) {
    Session session = load_session(dir);
    session.global_id_offset = next_global_id_offset;
    next_global_id_offset += session.sub_maps.size();
    sessions.push_back(std::move(session));
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

  const std::unique_ptr<core::PoseGraph> joint =
      build_joint_graph(sessions, world_T_session, accepted_closures, reference, pose_graph_config);
  if (!joint->optimize()) {
    throw std::runtime_error(std::format(
        "joint pose-graph optimization failed ({} inter-session closures); nothing written", accepted_closures.size()));
  }

  auto [resolved_name, out_dir] = core::resolve_run_dir(output_dir, run_name);
  result.out_run_name = resolved_name;
  result.out_run_dir = out_dir;
  const fs::path joint_path = out_dir / (resolved_name + "_joint_keypose_graph.g2o");
  if (!joint->save(joint_path)) {
    spdlog::error("failed to write {}", joint_path.string());
  }

  for (std::size_t session_index = 0; session_index < sessions.size(); ++session_index) {
    if (!world_T_session.at(session_index)) {
      continue;
    }
    const fs::path tum_path = out_dir / std::format("{}_session_{}_tum.txt", resolved_name, session_index);
    if (!core::write_tum(tum_path, deform_trajectory(sessions.at(session_index), *joint))) {
      spdlog::error("failed to write {}", tum_path.string());
    }
  }

  result.accepted_closures = accepted_closures.size();
  return result;
}

} // namespace rko_slam::align_sessions
