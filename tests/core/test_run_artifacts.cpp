#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <numbers>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "rko_slam/core/pose_graph.hpp"
#include "rko_slam/core/run_artifacts.hpp"

namespace fs = std::filesystem;
using namespace rko_slam::core;

namespace {

fs::path temp_dir() {
  auto dir = fs::temp_directory_path() / ("rko_slam_test_artifacts_" + std::to_string(std::random_device{}()));
  fs::create_directories(dir);
  return dir;
}

Sophus::SE3d se3(const double rot_x,
                 const double rot_y,
                 const double rot_z,
                 const double trans_x,
                 const double trans_y,
                 const double trans_z) {
  return Sophus::SE3d::exp(Sophus::Vector6d{trans_x, trans_y, trans_z, rot_x, rot_y, rot_z});
}

} // namespace

TEST_CASE("run_artifacts: ply xyz f32 round-trip is exact at f32", "[run_artifacts]") {
  const auto dir = temp_dir();
  std::vector<Eigen::Vector3f> cloud;
  cloud.reserve(1000);
  for (int i = 0; i < 1000; ++i) {
    // Full f32 mantissas, so a write that loses precision shows up; a fixed sequence keeps a failure reproducible.
    const float spread = (static_cast<float>(i) * 0.7312345F) - 100.0F;
    cloud.emplace_back(spread, spread * 1.3172F, spread * -0.4271F);
  }
  REQUIRE(write_ply_xyz(dir / "a.ply", cloud));
  const std::vector<Eigen::Vector3f> back = read_ply_xyz(dir / "a.ply");
  REQUIRE(back.size() == cloud.size());
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    CAPTURE(i);
    // The cloud is already f32, and the ply body is f32: bit-exact both ways.
    REQUIRE(back.at(i).x() == cloud.at(i).x());
    REQUIRE(back.at(i).y() == cloud.at(i).y());
    REQUIRE(back.at(i).z() == cloud.at(i).z());
  }
  fs::remove_all(dir);
}

TEST_CASE("run_artifacts: a saved pose graph loads back with its edges", "[run_artifacts]") {
  const auto dir = temp_dir();
  const PoseGraph::Config config;
  PoseGraph pose_graph(config);
  const Sophus::SE3d pose0;
  const Sophus::SE3d pose1 = se3(0, 0, 0.1, 1.0, 0.0, 0.0);
  const Sophus::SE3d pose2 = se3(0, 0, 0.2, 2.0, 0.5, 0.0);
  pose_graph.add_keypose(0, pose0);
  pose_graph.set_keypose_fixed(0, true);
  pose_graph.add_keypose(1, pose1);
  pose_graph.add_keypose(2, pose2);
  pose_graph.add_odom_edge(0, 1, pose0.inverse() * pose1);
  pose_graph.add_odom_edge(1, 2, pose1.inverse() * pose2);
  pose_graph.add_closure_edge(0, 2, pose0.inverse() * pose2);
  REQUIRE(pose_graph.save(dir / "g.g2o"));

  PoseGraph loaded(config);
  REQUIRE(loaded.load(dir / "g.g2o"));
  REQUIRE(loaded.num_keyposes() == 3);
  const auto edges = loaded.se3_edges();
  REQUIRE(edges.size() == 3);
  std::size_t n_closure = 0;
  for (const auto& edge : edges) {
    n_closure += (edge.from_id + 1 != edge.to_id && edge.to_id + 1 != edge.from_id) ? 1 : 0;
  }
  REQUIRE(n_closure == 1);
  // g2o writes its text at fixed precision, so a double comes back agreeing to ~1e-7, not to its own epsilon.
  const auto odom =
      std::find_if(edges.begin(), edges.end(), [](const auto& edge) { return edge.from_id == 0 && edge.to_id == 1; });
  REQUIRE(odom != edges.end());
  REQUIRE_THAT(((pose0.inverse() * pose1).inverse() * odom->from_T_to).log().norm(),
               Catch::Matchers::WithinAbs(0.0, 1e-6));
  fs::remove_all(dir);
}

TEST_CASE("run_artifacts: a saved pose graph holds every edge it solved", "[run_artifacts]") {
  const auto dir = temp_dir();
  PoseGraph pose_graph{PoseGraph::Config{}};
  const Sophus::SE3d pose1 = se3(0, 0, 0.1, 1.0, 0.0, 0.0);
  pose_graph.add_keypose(0, Sophus::SE3d{});
  pose_graph.add_keypose(1, pose1);
  pose_graph.add_gauge_edge(0);
  pose_graph.add_gravity_edge(0, Eigen::Vector3d::UnitZ());
  pose_graph.add_gravity_edge(1, Eigen::Vector3d::UnitZ());
  pose_graph.add_odom_edge(0, 1, pose1);
  REQUIRE(pose_graph.save(dir / "g.g2o"));

  std::ifstream file(dir / "g.g2o");
  std::vector<std::string> tags_and_ids;
  for (std::string line; std::getline(file, line);) {
    tags_and_ids.push_back(line.substr(0, line.find(' ', line.find(' ') + 1)));
  }
  CHECK(tags_and_ids == std::vector<std::string>{"PARAMS_SE3OFFSET 0", "VERTEX_SE3:QUAT 0", "VERTEX_SE3:QUAT 1",
                                                 "EDGE_SE3_PRIOR 0", "EDGE_GRAVITY 0", "EDGE_GRAVITY 1",
                                                 "EDGE_SE3:QUAT 0"});
  PoseGraph loaded{PoseGraph::Config{}};
  REQUIRE(loaded.load(dir / "g.g2o"));
  CHECK(loaded.se3_edges().size() == 1);
  CHECK(loaded.gravity_edges().size() == 2);
  fs::remove_all(dir);
}

TEST_CASE("run_artifacts: a keypose id with no vertex throws rather than dereferencing null", "[run_artifacts]") {
  // align_sessions reaches these with ids taken from a file, so under NDEBUG an assert would be a null deref.
  const PoseGraph::Config config;
  PoseGraph pose_graph(config);
  pose_graph.add_keypose(0, Sophus::SE3d{});
  REQUIRE_NOTHROW(pose_graph.get_keypose(0));
  REQUIRE_THROWS_AS(pose_graph.get_keypose(1), std::runtime_error);
  REQUIRE_THROWS_AS(pose_graph.set_keypose_fixed(1, true), std::runtime_error);
}

TEST_CASE("run_artifacts: the trajectory png draws the run and its closures", "[run_artifacts]") {
  const auto dir = temp_dir();
  std::vector<TrajectorySample> trajectory;
  // A square loop, so the start and the end meet and a closure chord is a real diagonal.
  for (int step = 0; step <= 40; ++step) {
    const double angle = 2.0 * std::numbers::pi * step / 40.0;
    trajectory.push_back({
        .time = rko_slam::core::Nsec{step},
        .pose = Sophus::SE3f::trans(10.0F * static_cast<float>(std::cos(angle)),
                                    10.0F * static_cast<float>(std::sin(angle)), 0.0F),
    });
  }
  const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> closures{
      {Eigen::Vector3f{-10.0F, 0.0F, 0.0F}, Eigen::Vector3f{10.0F, 0.0F, 0.0F}},
  };
  const auto path = dir / "trajectory.png";
  REQUIRE(write_trajectory_png(path, trajectory, closures));
  REQUIRE(fs::file_size(path) > 0);

  const cv::Mat image = cv::imread(path.string(), cv::IMREAD_COLOR);
  REQUIRE_FALSE(image.empty());
  // Isotropic scale on a circle: the image is square bar the equal margins.
  REQUIRE(image.cols == image.rows);
  // The closure chord is the only red thing drawn.
  std::size_t red_pixels = 0;
  for (int row = 0; row < image.rows; ++row) {
    for (int col = 0; col < image.cols; ++col) {
      const auto& pixel = image.at<cv::Vec3b>(row, col);
      red_pixels += (pixel.val[2] > 150 && pixel.val[0] < 100 && pixel.val[1] < 100) ? 1 : 0;
    }
  }
  REQUIRE(red_pixels > 100);

  REQUIRE_THROWS_AS(write_trajectory_png(dir / "empty.png", {}, {}), std::invalid_argument);

  // A run that never moves has no span to scale by, and a dead-straight one has none on one axis.
  const std::vector<TrajectorySample> stationary(4, {.time = rko_slam::core::Nsec{0}, .pose = Sophus::SE3f{}});
  REQUIRE(write_trajectory_png(dir / "stationary.png", stationary, {}));
  const cv::Mat still = cv::imread((dir / "stationary.png").string(), cv::IMREAD_COLOR);
  REQUIRE_FALSE(still.empty());
  REQUIRE(still.cols == still.rows);

  std::vector<TrajectorySample> straight;
  straight.reserve(50);
  for (int step = 0; step < 50; ++step) {
    straight.push_back(
        {.time = rko_slam::core::Nsec{step}, .pose = Sophus::SE3f::trans(static_cast<float>(step) * 2.0F, 0.0F, 0.0F)});
  }
  REQUIRE(write_trajectory_png(dir / "straight.png", straight, {}));
  const cv::Mat line = cv::imread((dir / "straight.png").string(), cv::IMREAD_COLOR);
  REQUIRE_FALSE(line.empty());
  // The floor on the short side, not a one-pixel sliver.
  REQUIRE(line.rows > line.cols / 8);
  fs::remove_all(dir);
}

TEST_CASE("run_artifacts: resolve_run_dir auto-increments", "[run_artifacts]") {
  const auto dir = temp_dir();
  const auto [name0, path0] = resolve_run_dir(dir, "run");
  REQUIRE(name0 == "run_0");
  REQUIRE(fs::exists(path0));
  const auto [name1, path1] = resolve_run_dir(dir, "run");
  REQUIRE(name1 == "run_1");
  fs::remove_all(dir);
}
