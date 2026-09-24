#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_exception.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <filesystem>
#include <fstream>
#include <random>
#include <rko_lio/core/error.hpp>
#include <sophus/se3.hpp>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "rko_slam/pgo/io.hpp"
#include "rko_slam/pgo/pose_graph.hpp"

namespace fs = std::filesystem;

using Config = rko_slam::pgo::PoseGraph::Config;
using rko_slam::pgo::GravityEdge;
using rko_slam::pgo::load;
using rko_slam::pgo::PoseEdge;
using rko_slam::pgo::PoseGraph;
using rko_slam::pgo::save;

namespace {

// Lines g2o's SparseOptimizer::save wrote for a real run: its first three keyposes and the odometry between them.
constexpr std::string_view kG2oDump =
    "VERTEX_SE3:QUAT 0 0.0245461 0.0174529 0.00601598 0.001039 -0.001136 -0.004459 0.999989 \n"
    "FIX 0\n"
    "VERTEX_SE3:QUAT 1 -49.6873 -3.92524 4.46639 0.000683369 0.0192119 0.127291 0.991679 \n"
    "VERTEX_SE3:QUAT 2 -56.7502 -53.4797 4.69044 0.0265304 -0.00157755 0.777667 0.628115 \n"
    "EDGE_SE3:QUAT 0 1 -49.665 -4.37603 4.58204 -0.000394284 0.0204662 0.131773 0.991069 "
    "1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 100 0 0 100 0 100 \n"
    "EDGE_SE3:QUAT 1 2 -19.3492 -46.1644 -0.221463 0.01054 -0.0165622 0.691733 0.721886 "
    "1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 100 0 0 100 0 100 \n";

// What we write back for it: the weights are not in the graph file.
constexpr std::string_view kResaved =
    "VERTEX_SE3:QUAT 0 0.0245461 0.0174529 0.00601598 0.001039 -0.001136 -0.004459 0.999989 \n"
    "FIX 0\n"
    "VERTEX_SE3:QUAT 1 -49.6873 -3.92524 4.46639 0.000683369 0.0192119 0.127291 0.991679 \n"
    "VERTEX_SE3:QUAT 2 -56.7502 -53.4797 4.69044 0.0265304 -0.00157755 0.777667 0.628115 \n"
    "EDGE_SE3:QUAT 0 1 -49.665 -4.37603 4.58204 -0.000394284 0.0204662 0.131773 0.991069 "
    "1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1 \n"
    "EDGE_SE3:QUAT 1 2 -19.3492 -46.1644 -0.221463 0.01054 -0.0165622 0.691733 0.721886 "
    "1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1 \n";

fs::path temp_dir() {
  auto dir = fs::temp_directory_path() / ("rko_slam_test_pgo_" + std::to_string(std::random_device{}()));
  fs::create_directories(dir);
  return dir;
}

void write_text(const fs::path& path, const std::string_view text) {
  std::ofstream out(path);
  out << text;
}

std::string read_text(const fs::path& path) {
  const std::ifstream file(path);
  std::ostringstream text;
  text << file.rdbuf();
  return text.str();
}

// What g2o itself would read: our own comment lines are not part of the format.
std::string read_g2o_text(const fs::path& path) {
  std::ifstream file(path);
  std::ostringstream text;
  for (std::string line; std::getline(file, line);) {
    if (!line.starts_with('#')) {
      text << line << "\n";
    }
  }
  return text.str();
}

} // namespace

TEST_CASE("pgo: a graph g2o wrote loads and saves back without its weights", "[pgo]") {
  const fs::path dir = temp_dir();
  write_text(dir / "g2o.g2o", kG2oDump);
  REQUIRE(save(load(dir / "g2o.g2o"), dir / "pgo.g2o"));
  REQUIRE(read_g2o_text(dir / "pgo.g2o") == kResaved);

  write_text(dir / "annotated.g2o", "# blank and comment lines are skipped\n\n" + std::string(kG2oDump));
  REQUIRE(save(load(dir / "annotated.g2o"), dir / "pgo.g2o"));
  REQUIRE(read_g2o_text(dir / "pgo.g2o") == kResaved);
  fs::remove_all(dir);
}

TEST_CASE("pgo: a graph of one keypose saves it and loads back", "[pgo]") {
  const fs::path dir = temp_dir();
  const std::string_view lone_keypose =
      "VERTEX_SE3:QUAT 0 0.0245461 0.0174529 0.00601598 0.001039 -0.001136 -0.004459 0.999989 \n"
      "FIX 0\n";
  write_text(dir / "lone.g2o", lone_keypose);
  REQUIRE(save(load(dir / "lone.g2o"), dir / "pgo.g2o"));
  REQUIRE(read_g2o_text(dir / "pgo.g2o") == lone_keypose);
  fs::remove_all(dir);
}

TEST_CASE("pgo: a saved graph loads back with its poses and edge kinds", "[pgo]") {
  const fs::path dir = temp_dir();
  const Sophus::SE3d pose0;
  const Sophus::SE3d pose1 = Sophus::SE3d::exp(Sophus::Vector6d{1.0, 0.0, 0.0, 0.0, 0.0, 0.1});
  const Sophus::SE3d pose2 = Sophus::SE3d::exp(Sophus::Vector6d{2.0, 0.5, 0.0, 0.0, 0.0, 0.2});
  PoseGraph pose_graph;
  for (const Sophus::SE3d& pose : {pose0, pose1, pose2}) {
    pose_graph.add_keypose(pose);
  }
  pose_graph.add_odometry_edge(0, 1, pose0.inverse() * pose1);
  pose_graph.add_odometry_edge(1, 2, pose1.inverse() * pose2);
  pose_graph.add_closure_edge(0, 2, pose0.inverse() * pose2);
  pose_graph.anchor_at(0);
  REQUIRE(save(pose_graph, dir / "g.g2o"));

  const PoseGraph loaded = load(dir / "g.g2o");
  REQUIRE(loaded.keyposes.size() == 3);
  const auto& pose_edges = loaded.pose_edges;
  REQUIRE(pose_edges.size() == 3);
  REQUIRE(std::ranges::count(pose_edges, PoseEdge::Kind::closure, &PoseEdge::kind) == 1);
  // The text holds six significant digits, so a pose comes back agreeing to ~1e-6, not to double epsilon.
  const auto odom = std::find_if(pose_edges.begin(), pose_edges.end(),
                                 [](const auto& edge) { return edge.from_id == 0 && edge.to_id == 1; });
  REQUIRE(odom != pose_edges.end());
  REQUIRE(((pose0.inverse() * pose1).inverse() * odom->from_T_to).log().norm() < 1e-5);
  fs::remove_all(dir);
}

TEST_CASE("pgo: malformed g2o text throws InputError saying what is wrong", "[pgo]") {
  using Catch::Matchers::ContainsSubstring;
  using Catch::Matchers::MessageMatches;
  const fs::path dir = temp_dir();
  const std::string vertex = "VERTEX_SE3:QUAT 0 0 0 0 0 0 0 1 \n";
  const std::string edge = "EDGE_SE3:QUAT 0 1 0 0 0 0 0 0 1 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1 \n";
  const std::vector<std::pair<std::string, std::string>> cases{
      {"VERTEX_SE3:QUAT 0 0 0 0 0 0 1 \nFIX 0\n", "malformed line at"},
      {vertex + "FIX 0\nVERTEX_SE3:QUAT 2 0 0 0 0 0 0 1 \n", "vertex id 2, expected 1 at"},
      {"VERTEX_SE3:QUAT 0 0 0 0 0 0 0 0 \nFIX 0\n", "malformed line at"},
      {vertex + "FIX 0\nPARAMS_SE3OFFSET 0 0 0 0 0 0 0 1 \n", "unknown tag PARAMS_SE3OFFSET at"},
      {vertex, "no FIX in"},
      {vertex + "FIX 1\n", "FIX of a missing vertex at"},
      {vertex + "FIX 0\n" + edge, "edge to a missing vertex at"},
      {vertex + "FIX 0\nEDGE_GRAVITY 1 0 0 9.81 100 0 100 \n", "gravity edge on a missing vertex at"},
      {vertex + "FIX 0\nEDGE_GRAVITY 0 0 0 9.81 100 0 \n", "malformed line at"},
  };
  for (const auto& [text, message] : cases) {
    CAPTURE(text);
    write_text(dir / "malformed.g2o", text);
    REQUIRE_THROWS_MATCHES(load(dir / "malformed.g2o"), rko_lio::core::InputError,
                           MessageMatches(ContainsSubstring(message)));
  }
  REQUIRE_THROWS_AS(load(dir / "absent.g2o"), rko_lio::core::InputError);
  fs::remove_all(dir);
}

TEST_CASE("pgo: a saved graph carries its measured up directions and its anchor back", "[pgo]") {
  const fs::path dir = temp_dir();
  // the text holds six significant digits, so a measurement comes back agreeing to ~1e-5 of its length
  const std::vector<Eigen::Vector3d> ups{
      {0.3141592653, -0.1234567891, 9.8066499123},
      {0.0, 0.0, 9.81},
      {-0.4567891234, 0.2718281828, 9.7912345678},
  };
  PoseGraph written;
  written.add_keypose(Sophus::SE3d{});
  for (std::size_t keypose_id = 0; keypose_id < ups.size(); ++keypose_id) {
    written.add_gravity_edge(keypose_id, ups.at(keypose_id));
    if (keypose_id + 1 < ups.size()) {
      const Sophus::SE3d step(Sophus::SO3d::rotZ(0.1), Eigen::Vector3d(5.0, 0.0, 0.0));
      const std::size_t next_id = written.add_keypose(written.keyposes.back() * step);
      written.add_odometry_edge(keypose_id, next_id, step);
    }
  }
  written.anchor_at(2);
  REQUIRE(save(written, dir / "g.g2o"));

  const PoseGraph loaded = load(dir / "g.g2o");
  REQUIRE(loaded.anchor.keypose_id == 2);
  REQUIRE(loaded.gravity_edges.size() == ups.size());
  for (const GravityEdge& gravity : loaded.gravity_edges) {
    CAPTURE(gravity.keypose_id);
    CHECK((gravity.measured_up - ups.at(gravity.keypose_id)).norm() < 1e-4);
  }
  fs::remove_all(dir);
}
