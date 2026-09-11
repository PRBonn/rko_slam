#include <Eigen/Core>
#include <Eigen/Geometry>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sophus/se3.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "rko_slam/core/run_artifacts.hpp"

using rko_slam::core::read_tum;
using rko_slam::core::TrajectorySample;
using rko_slam::core::write_tum;

namespace {
// Poses are stored in float and printed/parsed as double, so a round-trip
// agrees at float epsilon, not double.
constexpr double kPoseTol = 1e-6;
// radians equivalent of kPoseTol, since |dot| ~ 1 - angle^2 / 8 for small angles
constexpr double kRotTol = 2.83e-3;

std::filesystem::path write_raw(std::string_view filename, std::string_view body) {
  const auto path = std::filesystem::temp_directory_path() / filename;
  std::ofstream out(path);
  out << body;
  return path;
}

std::vector<TrajectorySample> make_samples() {
  std::vector<TrajectorySample> out;
  // Five poses at 100 ms cadence; representative non-trivial timestamps + quaternions.
  std::int64_t base_ns = 1'700'000'000'123'456'789LL;
  for (int i = 0; i < 5; ++i) {
    TrajectorySample sample;
    sample.time = rko_slam::core::Nsec{base_ns + (static_cast<std::int64_t>(i) * 100'000'000LL)};
    Sophus::Vector6d delta;
    delta.head<3>() = Eigen::Vector3d{static_cast<double>(i) * 0.5, -0.25 * i, 0.1 * i};
    delta.tail<3>() = Eigen::Vector3d{0.0, 0.05 * i, 0.0};
    sample.pose = Sophus::SE3d::exp(delta).cast<float>();
    out.push_back(sample);
  }
  return out;
}
} // namespace

TEST_CASE("write_tum: round-trip preserves timestamps + poses", "[tum]") {
  const auto samples = make_samples();
  const auto path = std::filesystem::temp_directory_path() / "rko_slam_tum_writer_test.tum";
  REQUIRE(write_tum(path, samples));

  std::ifstream file(path);
  REQUIRE(file.is_open());

  std::vector<TrajectorySample> readback;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }
    long long sec_part = 0;
    long long frac_part = 0;
    double trans_x = 0;
    double trans_y = 0;
    double trans_z = 0;
    double quat_x = 0;
    double quat_y = 0;
    double quat_z = 0;
    double quat_w = 0;
    // Parse "sec.frac tx ty tz qx qy qz qw" - the "." sit in the same numeric token.
    char dot = 0;
    std::istringstream line_stream(line);
    line_stream >> sec_part >> dot >> frac_part >> trans_x >> trans_y >> trans_z >> quat_x >> quat_y >> quat_z >>
        quat_w;
    REQUIRE((line_stream.good() || line_stream.eof()));

    TrajectorySample sample;
    sample.time = rko_slam::core::Nsec{(sec_part * 1'000'000'000LL) + frac_part};
    const Eigen::Quaterniond quaternion(quat_w, quat_x, quat_y, quat_z); // (w, x, y, z) ctor
    sample.pose = Sophus::SE3d{quaternion, Eigen::Vector3d{trans_x, trans_y, trans_z}}.cast<float>();
    readback.push_back(sample);
  }

  REQUIRE(readback.size() == samples.size());
  for (std::size_t i = 0; i < samples.size(); ++i) {
    CAPTURE(i);
    CHECK(readback.at(i).time == samples.at(i).time);
    CHECK_THAT((readback.at(i).pose.translation() - samples.at(i).pose.translation()).norm(),
               Catch::Matchers::WithinAbs(0.0, kPoseTol));
    const Eigen::Quaterniond quaternion_in = samples.at(i).pose.unit_quaternion().cast<double>();
    const Eigen::Quaterniond quaternion_out = readback.at(i).pose.unit_quaternion().cast<double>();
    // angularDistance takes the sign flip a quaternion may carry without changing the rotation.
    CHECK_THAT(quaternion_in.angularDistance(quaternion_out), Catch::Matchers::WithinAbs(0.0, kRotTol));
  }

  std::filesystem::remove(path);
}

TEST_CASE("write_tum: identity pose writes 'sec.frac 0 0 0 0 0 0 1'", "[tum]") {
  // Pins the on-disk column order against an external oracle: a round-trip
  // would pass through a symmetric qx/qw swap without noticing.
  std::vector<TrajectorySample> samples;
  TrajectorySample sample;
  sample.time = rko_slam::core::Nsec{1'777'033'464'655'300'000LL};
  sample.pose = Sophus::SE3f{};
  samples.push_back(sample);

  const auto path = std::filesystem::temp_directory_path() / "rko_slam_tum_writer_columns_test.tum";
  REQUIRE(write_tum(path, samples));

  std::ifstream file(path);
  REQUIRE(file.is_open());
  std::string line;
  REQUIRE(std::getline(file, line));

  std::istringstream line_stream(line);
  long long sec_part = 0;
  long long frac_part = 0;
  char dot = 0;
  double trans_x = 0;
  double trans_y = 0;
  double trans_z = 0;
  double quat_x = 0;
  double quat_y = 0;
  double quat_z = 0;
  double quat_w = 0;
  line_stream >> sec_part >> dot >> frac_part >> trans_x >> trans_y >> trans_z >> quat_x >> quat_y >> quat_z >> quat_w;
  REQUIRE(sec_part == 1'777'033'464LL);
  REQUIRE(frac_part == 655'300'000LL);
  REQUIRE(trans_x == 0.0);
  REQUIRE(trans_y == 0.0);
  REQUIRE(trans_z == 0.0);
  REQUIRE(quat_x == 0.0);
  REQUIRE(quat_y == 0.0);
  REQUIRE(quat_z == 0.0);
  REQUIRE(quat_w == 1.0);

  std::filesystem::remove(path);
}

TEST_CASE("read_tum: parses rows, skips comments and blanks", "[tum]") {
  const auto path = write_raw("rko_slam_tum_reader_test.tum", "# comment\n"
                                                              "1.000 0.0 0.0 0.0 0.0 0.0 0.0 1.0\n"
                                                              "\n"
                                                              "2.000 1.0 0.0 0.0 0.0 0.0 0.0 1.0\n"
                                                              "3.000 2.0 0.0 0.0 0.0 0.0 0.0 1.0\n");
  const auto samples = read_tum(path);
  REQUIRE(samples.size() == 3);
  REQUIRE(samples.at(0).time.count() == 1'000'000'000);
  REQUIRE(samples.at(1).time.count() == 2'000'000'000);
  REQUIRE(samples.at(2).time.count() == 3'000'000'000);
  REQUIRE(samples.at(1).pose.translation().x() == 1.0);
  REQUIRE(samples.at(2).pose.translation().x() == 2.0);
  std::filesystem::remove(path);
}

TEST_CASE("read_tum: malformed row throws", "[tum]") {
  const auto path = write_raw("rko_slam_tum_reader_bad_test.tum", "1.000 0.0 0.0 0.0 not_a_number 0.0 0.0 1.0\n");
  REQUIRE_THROWS_AS(read_tum(path), std::runtime_error);
  std::filesystem::remove(path);
}

TEST_CASE("read_tum: missing file throws", "[tum]") {
  REQUIRE_THROWS_AS(read_tum(std::filesystem::temp_directory_path() / "rko_slam_tum_reader_absent.tum"),
                    std::runtime_error);
}
