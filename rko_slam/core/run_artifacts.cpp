#include "rko_slam/core/run_artifacts.hpp"

#include <Eigen/Geometry>
#include <array>
#include <bit>
#include <charconv>
#include <cmath>
#include <format>
#include <fstream>
#include <iomanip>
#include <ios>
#include <istream>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <ostream>
#include <rko_lio/core/error.hpp>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>

namespace {

constexpr int kLongSidePixels = 1600;
constexpr int kMarginPixels = 24;
// Floor on the short side, as a fraction of the long one, so a straight run is still a picture.
constexpr float kMinAspect = 0.2F;

std::int64_t nanoseconds_from_stamp(std::string_view text) {
  const std::size_t dot = text.find('.');
  const std::string_view seconds_text = text.substr(0, dot);
  std::int64_t seconds = 0;
  if (std::from_chars(seconds_text.data(), seconds_text.data() + seconds_text.size(), seconds).ec != std::errc{}) {
    return -1;
  }
  std::int64_t nanoseconds = 0;
  if (dot != std::string_view::npos) {
    std::string fraction(text.substr(dot + 1));
    fraction.resize(9, '0');
    if (std::from_chars(fraction.data(), fraction.data() + 9, nanoseconds).ec != std::errc{}) {
      return -1;
    }
  }
  return (seconds * 1'000'000'000LL) + nanoseconds;
}

bool read_pose_tq(std::istream& stream, Sophus::SE3d& pose) {
  // NOLINTBEGIN(readability-identifier-length) TUM's own column names
  double tx = 0;
  double ty = 0;
  double tz = 0;
  double qx = 0;
  double qy = 0;
  double qz = 0;
  double qw = 0;
  // NOLINTEND(readability-identifier-length)
  if (!(stream >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) {
    return false;
  }
  pose = Sophus::SE3d(Eigen::Quaterniond(qw, qx, qy, qz).normalized(), Eigen::Vector3d{tx, ty, tz});
  return true;
}

} // namespace

namespace rko_slam::core {

bool write_ply_xyz(const std::filesystem::path& path, const std::vector<Eigen::Vector3f>& points) {
  std::error_code dir_error;
  std::filesystem::create_directories(path.parent_path(), dir_error);
  if (dir_error) {
    return false;
  }
  std::ofstream ofs(path, std::ios::binary);
  if (!ofs) {
    return false;
  }
  ofs << "ply\nformat binary_little_endian 1.0\n"
      << "element vertex " << points.size() << '\n'
      << "property float x\nproperty float y\nproperty float z\n"
      << "end_header\n";
  for (const Eigen::Vector3f& point : points) {
    const auto bytes =
        std::bit_cast<std::array<char, 3 * sizeof(float)>>(std::array<float, 3>{point.x(), point.y(), point.z()});
    ofs.write(bytes.data(), bytes.size());
  }
  return ofs.good();
}

std::vector<Eigen::Vector3f> read_ply_xyz(const std::filesystem::path& path) {
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs) {
    throw rko_lio::core::InputError("read_ply_xyz: cannot open " + path.string());
  }
  constexpr std::string_view kVertexCountPrefix = "element vertex ";
  std::string line;
  std::size_t n_vertices = 0;
  bool header_ok = false;
  while (std::getline(ifs, line)) {
    if (line.starts_with(kVertexCountPrefix)) {
      const std::string_view count = std::string_view(line).substr(kVertexCountPrefix.size());
      if (std::from_chars(count.data(), count.data() + count.size(), n_vertices).ec != std::errc{}) {
        throw rko_lio::core::InputError("read_ply_xyz: bad vertex count in " + path.string());
      }
    } else if (line == "end_header") {
      header_ok = true;
      break;
    }
  }
  if (!header_ok) {
    throw rko_lio::core::InputError("read_ply_xyz: no end_header in " + path.string());
  }
  std::vector<Eigen::Vector3f> out;
  out.reserve(n_vertices);
  for (std::size_t i = 0; i < n_vertices; ++i) {
    std::array<char, 3 * sizeof(float)> bytes{};
    if (!ifs.read(bytes.data(), bytes.size())) {
      throw rko_lio::core::InputError("read_ply_xyz: truncated vertex data in " + path.string());
    }
    const auto xyz = std::bit_cast<std::array<float, 3>>(bytes);
    out.emplace_back(xyz.at(0), xyz.at(1), xyz.at(2));
  }
  return out;
}

std::pair<std::string, std::filesystem::path> resolve_run_dir(const std::filesystem::path& parent,
                                                              const std::string_view run_name) {
  for (int idx = 0;; ++idx) {
    std::string resolved = std::format("{}_{}", run_name, idx);
    std::filesystem::path candidate = parent / resolved;
    if (!std::filesystem::exists(candidate)) {
      std::filesystem::create_directories(candidate);
      return {std::move(resolved), std::move(candidate)};
    }
  }
}

bool write_trajectory_png(const std::filesystem::path& path,
                          const std::vector<TrajectorySample>& trajectory,
                          const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& closures) {
  Eigen::Vector2f lower = trajectory.front().pose.translation().head<2>();
  Eigen::Vector2f upper = lower;
  for (const auto& sample : trajectory) {
    lower = lower.cwiseMin(sample.pose.translation().head<2>());
    upper = upper.cwiseMax(sample.pose.translation().head<2>());
  }

  const Eigen::Vector2f measured = upper - lower;
  const float floor_span = std::max(measured.maxCoeff() * kMinAspect, 1.0F);
  const Eigen::Vector2f grow = (Eigen::Vector2f::Constant(floor_span) - measured).cwiseMax(0.0F) / 2.0F;
  lower -= grow;
  upper += grow;
  const Eigen::Vector2f span = upper - lower;

  const float scale = static_cast<float>(kLongSidePixels) / span.maxCoeff();
  const int width = static_cast<int>(std::lround(span.x() * scale)) + (2 * kMarginPixels);
  const int height = static_cast<int>(std::lround(span.y() * scale)) + (2 * kMarginPixels);
  cv::Mat image(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

  const auto to_pixel = [&](const Eigen::Vector3f& point) {
    return cv::Point(static_cast<int>(std::lround((point.x() - lower.x()) * scale)) + kMarginPixels,
                     height - kMarginPixels - static_cast<int>(std::lround((point.y() - lower.y()) * scale)));
  };

  for (std::size_t i = 1; i < trajectory.size(); ++i) {
    cv::line(image, to_pixel(trajectory.at(i - 1).pose.translation()), to_pixel(trajectory.at(i).pose.translation()),
             cv::Scalar(60, 60, 60), 2, cv::LINE_AA);
  }
  for (const auto& [from, into] : closures) {
    cv::line(image, to_pixel(from), to_pixel(into), cv::Scalar(0, 0, 220), 2, cv::LINE_AA);
  }
  cv::circle(image, to_pixel(trajectory.front().pose.translation()), 8, cv::Scalar(60, 170, 60), -1, cv::LINE_AA);
  cv::circle(image, to_pixel(trajectory.back().pose.translation()), 4, cv::Scalar(0, 140, 255), -1, cv::LINE_AA);

  return cv::imwrite(path.string(), image);
}

bool write_tum(const std::filesystem::path& path, const std::vector<TrajectorySample>& samples) {
  std::ofstream out(path);
  if (!out.is_open()) {
    return false;
  }
  // Max double precision, so the components round-trip.
  out << std::defaultfloat;
  out.precision(std::numeric_limits<double>::max_digits10);

  for (const auto& sample : samples) {
    // sec.fffffffff as two integer halves, so int64 nanoseconds round-trip exactly.
    const std::int64_t nanoseconds = sample.time.count();
    const std::int64_t sec = nanoseconds / 1'000'000'000LL;
    out << sec << '.' << std::setfill('0') << std::setw(9) << nanoseconds - (sec * 1'000'000'000LL) << ' ';
    const Eigen::Vector3f& translation = sample.pose.translation();
    const Eigen::Quaternionf& quaternion = sample.pose.unit_quaternion();
    out << translation.x() << ' ' << translation.y() << ' ' << translation.z() << ' ' << quaternion.x() << ' '
        << quaternion.y() << ' ' << quaternion.z() << ' ' << quaternion.w();
    out << '\n';
  }
  return out.good();
}

std::vector<TrajectorySample> read_tum(const std::filesystem::path& path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    throw rko_lio::core::InputError("read_tum: cannot open " + path.string());
  }
  std::vector<TrajectorySample> out;
  std::string line;
  std::size_t lineno = 0;
  while (std::getline(file, line)) {
    ++lineno;
    const auto first_non_ws = line.find_first_not_of(" \t\r\n");
    if (first_non_ws == std::string::npos || line.at(first_non_ws) == '#') {
      continue;
    }
    std::istringstream iss(line);
    std::string stamp;
    Sophus::SE3d pose;
    if (!(iss >> stamp) || !read_pose_tq(iss, pose)) {
      throw rko_lio::core::InputError(std::format("read_tum: malformed row at {}:{}", path.string(), lineno));
    }
    const std::int64_t nanoseconds = nanoseconds_from_stamp(stamp);
    if (nanoseconds < 0) {
      throw rko_lio::core::InputError(std::format("read_tum: malformed timestamp at {}:{}", path.string(), lineno));
    }
    out.push_back({.time = Nsec{nanoseconds}, .pose = pose.cast<float>()});
  }
  return out;
}

} // namespace rko_slam::core
