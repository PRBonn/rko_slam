#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <numbers>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <filesystem>
#include <random>

#include "rko_slam/core/run_artifacts.hpp"

namespace fs = std::filesystem;
using namespace rko_slam::core;

namespace {

fs::path temp_dir() {
  auto dir = fs::temp_directory_path() / ("rko_slam_test_artifacts_" + std::to_string(std::random_device{}()));
  fs::create_directories(dir);
  return dir;
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
