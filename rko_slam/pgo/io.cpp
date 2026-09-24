#include "rko_slam/pgo/io.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <format>
#include <fstream>
#include <istream>
#include <optional>
#include <rko_lio/core/error.hpp>
#include <sophus/se3.hpp>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

namespace rko_slam::pgo {

namespace {

// g2o ignores comment lines, so what it has no field for rides in one and a strip is `grep -v '^#'`.
// Marks which edges are closures, by index: "# rko_slam closures: 3 7".
constexpr std::string_view kClosurePrefix = "# rko_slam closures:";

std::string closure_text(const std::vector<PoseEdge>& pose_edges) {
  std::string text{kClosurePrefix};
  for (std::size_t index = 0; index < pose_edges.size(); ++index) {
    if (pose_edges.at(index).kind == PoseEdge::Kind::closure) {
      text += std::format(" {}", index);
    }
  }
  return text + "\n";
}

// As SparseOptimizer::save writes them.
std::string pose_text(const Sophus::SE3d& pose) {
  const Eigen::Vector3d& translation = pose.translation();
  const Eigen::Quaterniond quaternion = Eigen::Quaterniond(pose.rotationMatrix()).normalized();
  return std::format("{:g} {:g} {:g} {:g} {:g} {:g} {:g} ", translation.x(), translation.y(), translation.z(),
                     quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
}

std::unordered_set<std::size_t> read_closures(const std::string_view text) {
  std::istringstream fields{std::string(text)};
  std::unordered_set<std::size_t> closures;
  std::size_t index = 0;
  while (fields >> index) {
    closures.insert(index);
  }
  if (!fields.eof()) {
    throw rko_lio::core::InputError("malformed closure list");
  }
  return closures;
}

inline bool at_end_of_line(std::istream& fields) { return (fields >> std::ws).eof(); }

// Consuming them checks the line's shape.
bool skip_information(std::istream& fields, const int count) {
  double ignored = 0.0;
  for (int entry = 0; entry < count; ++entry) {
    if (!(fields >> ignored)) {
      return false;
    }
  }
  return true;
}

std::optional<Sophus::SE3d> read_pose(std::istream& fields) {
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Eigen::Quaterniond quaternion = Eigen::Quaterniond::Identity();
  if (!(fields >> translation.x() >> translation.y() >> translation.z() >> quaternion.x() >> quaternion.y() >>
        quaternion.z() >> quaternion.w()) ||
      quaternion.norm() < Sophus::Constants<double>::epsilon()) {
    return std::nullopt;
  }
  return Sophus::SE3d(quaternion, translation);
}

struct Vertex {
  std::size_t id;
  Sophus::SE3d pose;
};

std::optional<Vertex> read_vertex(std::istream& fields) {
  std::size_t keypose_id = 0;
  if (!(fields >> keypose_id)) {
    return std::nullopt;
  }
  const std::optional<Sophus::SE3d> pose = read_pose(fields);
  if (!pose.has_value() || !at_end_of_line(fields)) {
    return std::nullopt;
  }
  return Vertex{.id = keypose_id, .pose = *pose};
}

Sophus::SE3d next_keypose(std::istream& fields, const std::size_t expected_id) {
  const std::optional<Vertex> vertex = read_vertex(fields);
  if (!vertex.has_value()) {
    throw rko_lio::core::InputError("malformed line");
  }
  if (vertex->id != expected_id) {
    throw rko_lio::core::InputError(std::format("vertex id {}, expected {}", vertex->id, expected_id));
  }
  return vertex->pose;
}

struct ParsedEdge {
  std::size_t from_id;
  std::size_t to_id;
  Sophus::SE3d from_T_to;
};

std::optional<ParsedEdge> read_edge(std::istream& fields) {
  std::size_t from_id = 0;
  std::size_t to_id = 0;
  if (!(fields >> from_id >> to_id)) {
    return std::nullopt;
  }
  const std::optional<Sophus::SE3d> from_T_to = read_pose(fields);
  if (!from_T_to.has_value()) {
    return std::nullopt;
  }
  if (!skip_information(fields, Sophus::SE3d::DoF * (Sophus::SE3d::DoF + 1) / 2) || !at_end_of_line(fields)) {
    return std::nullopt;
  }
  return ParsedEdge{.from_id = from_id, .to_id = to_id, .from_T_to = *from_T_to};
}

std::optional<GravityEdge> read_gravity_edge(std::istream& fields) {
  std::size_t keypose_id = 0;
  Eigen::Vector3d measured_up = Eigen::Vector3d::Zero();
  if (!(fields >> keypose_id >> measured_up.x() >> measured_up.y() >> measured_up.z())) {
    return std::nullopt;
  }
  if (!skip_information(fields, 3) || !at_end_of_line(fields)) {
    return std::nullopt;
  }
  return GravityEdge{.keypose_id = keypose_id, .measured_up = measured_up};
}

std::optional<std::size_t> read_fix(std::istream& fields) {
  std::size_t keypose_id = 0;
  if (!(fields >> keypose_id) || !at_end_of_line(fields)) {
    return std::nullopt;
  }
  return keypose_id;
}

struct Loading {
  PoseGraph pose_graph;
  std::optional<std::size_t> anchor;
  std::unordered_set<std::size_t> closures;
};

void read_line(const std::string& line, Loading& loading) {
  PoseGraph& pose_graph = loading.pose_graph;
  if (line.starts_with(kClosurePrefix)) {
    loading.closures = read_closures(std::string_view(line).substr(kClosurePrefix.size()));
    return;
  }
  std::istringstream fields(line);
  std::string tag;
  fields >> tag;
  const std::size_t num_keyposes = pose_graph.keyposes.size();
  if (tag == "VERTEX_SE3:QUAT") {
    pose_graph.add_keypose(next_keypose(fields, num_keyposes));
  } else if (tag == "EDGE_SE3:QUAT") {
    const std::optional<ParsedEdge> edge = read_edge(fields);
    if (!edge.has_value()) {
      throw rko_lio::core::InputError("malformed line");
    }
    if (std::max(edge->from_id, edge->to_id) >= num_keyposes) {
      throw rko_lio::core::InputError("edge to a missing vertex");
    }
    if (loading.closures.contains(pose_graph.pose_edges.size())) {
      pose_graph.add_closure_edge(edge->from_id, edge->to_id, edge->from_T_to);
    } else {
      pose_graph.add_odometry_edge(edge->from_id, edge->to_id, edge->from_T_to);
    }
  } else if (tag == "EDGE_GRAVITY") {
    const std::optional<GravityEdge> gravity = read_gravity_edge(fields);
    if (!gravity.has_value()) {
      throw rko_lio::core::InputError("malformed line");
    }
    if (gravity->keypose_id >= num_keyposes) {
      throw rko_lio::core::InputError("gravity edge on a missing vertex");
    }
    pose_graph.add_gravity_edge(gravity->keypose_id, gravity->measured_up);
  } else if (tag == "FIX") {
    const std::optional<std::size_t> fixed = read_fix(fields);
    if (!fixed.has_value()) {
      throw rko_lio::core::InputError("malformed line");
    }
    if (*fixed >= num_keyposes) {
      throw rko_lio::core::InputError("FIX of a missing vertex");
    }
    loading.anchor = fixed;
  } else if (!tag.empty() && !tag.starts_with('#')) {
    throw rko_lio::core::InputError("unknown tag " + tag);
  }
}
} // namespace

bool save(const PoseGraph& pose_graph, const std::filesystem::path& path) {
  // Upper triangle of a 6x6 identity: the weights are in the run's config.yaml.
  constexpr std::string_view kEdgeInformation = "1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1 ";

  std::ofstream out(path);
  out << closure_text(pose_graph.pose_edges);
  for (std::size_t keypose_id = 0; keypose_id < pose_graph.keyposes.size(); ++keypose_id) {
    out << std::format("VERTEX_SE3:QUAT {} {}\n", keypose_id, pose_text(pose_graph.keyposes.at(keypose_id)));
    if (keypose_id == pose_graph.anchor.keypose_id) {
      out << std::format("FIX {}\n", keypose_id);
    }
  }

  for (const PoseEdge& edge : pose_graph.pose_edges) {
    out << std::format("EDGE_SE3:QUAT {} {} {}{}\n", edge.from_id, edge.to_id, pose_text(edge.from_T_to),
                       kEdgeInformation);
  }

  for (const GravityEdge& gravity : pose_graph.gravity_edges) {
    out << std::format("EDGE_GRAVITY {} {:g} {:g} {:g} 1 0 1\n", gravity.keypose_id, gravity.measured_up.x(),
                       gravity.measured_up.y(), gravity.measured_up.z());
  }

  out.close();
  return !out.fail();
}

PoseGraph load(const std::filesystem::path& path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    throw rko_lio::core::InputError("pgo::load: cannot open " + path.string());
  }

  Loading loading;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(file, line)) {
    ++line_number;
    try {
      read_line(line, loading);
    } catch (const rko_lio::core::InputError& bad) {
      throw rko_lio::core::InputError(std::format("pgo::load: {} at {}:{}", bad.what(), path.string(), line_number));
    }
  }

  if (!loading.anchor.has_value()) {
    throw rko_lio::core::InputError(std::format("pgo::load: no FIX in {}", path.string()));
  }
  loading.pose_graph.anchor_at(*loading.anchor);
  return std::move(loading.pose_graph);
}

} // namespace rko_slam::pgo
