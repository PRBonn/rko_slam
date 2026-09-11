#include <Eigen/Geometry>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <random>
#include <rko_lio/core/error.hpp>
#include <sophus/se3.hpp>

#include "rko_slam/core/closure.hpp"

namespace {

// Tolerances tuned for the project's float points - float32 ICP can't
// hit 1e-3/1e-4 the way double can.
constexpr double kIcpTransTol = 5e-3;
constexpr double kIcpRotTol = 1e-3;

// Three mutually-perpendicular wall patches with their exact analytic normals, so all 6 DoF are
// observable; a single plane would be rank-3-deficient.
struct WallsWithNormals {
  std::vector<Eigen::Vector3f> points;
  std::vector<Eigen::Vector3f> normals;
};

WallsWithNormals make_three_walls(std::size_t n_per_wall, float extent, std::uint32_t seed = 31) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> dist(0, extent);
  WallsWithNormals out;
  out.points.reserve(3 * n_per_wall);
  out.normals.reserve(3 * n_per_wall);
  for (std::size_t i = 0; i < n_per_wall; ++i) {
    out.points.emplace_back(dist(rng), dist(rng), 0.0F);
    out.normals.emplace_back(Eigen::Vector3f::UnitZ());
    out.points.emplace_back(dist(rng), 0.0F, dist(rng));
    out.normals.emplace_back(Eigen::Vector3f::UnitY());
    out.points.emplace_back(0.0F, dist(rng), dist(rng));
    out.normals.emplace_back(Eigen::Vector3f::UnitX());
  }
  return out;
}

std::vector<Eigen::Vector3f> make_dense_3d(std::size_t num_points, float extent, std::uint32_t seed = 31) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> dist(-extent, extent);
  std::vector<Eigen::Vector3f> points;
  points.reserve(num_points);
  for (std::size_t i = 0; i < num_points; ++i) {
    points.emplace_back(dist(rng), dist(rng), dist(rng));
  }
  return points;
}

std::vector<Eigen::Vector3f> transform(const std::vector<Eigen::Vector3f>& points, const Sophus::SE3f& pose) {
  std::vector<Eigen::Vector3f> out;
  out.reserve(points.size());
  for (const auto& point : points) {
    out.push_back(pose * point);
  }
  return out;
}

std::vector<Eigen::Vector3f> rotate(const std::vector<Eigen::Vector3f>& normals, const Sophus::SO3f& rotation) {
  std::vector<Eigen::Vector3f> out;
  out.reserve(normals.size());
  for (const auto& normal : normals) {
    out.push_back(rotation * normal);
  }
  return out;
}

Sophus::SE3f small_perturbation(float t_max, float r_max_rad, std::uint32_t seed = 5) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> translation_dist(-t_max, t_max);
  std::uniform_real_distribution<float> rotation_dist(-r_max_rad, r_max_rad);
  const Eigen::Vector3f trans(translation_dist(rng), translation_dist(rng), translation_dist(rng));
  const Eigen::Vector3f rot_axis(rotation_dist(rng), rotation_dist(rng), rotation_dist(rng));
  return {Sophus::SO3f::exp(rot_axis), trans};
}

void check_pose_close(const Sophus::SE3f& actual, const Sophus::SE3f& expected, double t_tol, double r_tol_rad) {
  const Eigen::Vector3f translation_error = (actual.translation() - expected.translation());
  const float rotation_error = (actual.so3() * expected.so3().inverse()).log().norm();
  REQUIRE_THAT(static_cast<double>(translation_error.norm()), Catch::Matchers::WithinAbs(0.0, t_tol));
  REQUIRE_THAT(static_cast<double>(rotation_error), Catch::Matchers::WithinAbs(0.0, r_tol_rad));
}

} // namespace

TEST_CASE("icp: point-to-plane converges on three-walls input", "[icp]") {
  const auto src = make_three_walls(700, 5.0F);
  const Eigen::Vector3f gt_translation(0.2, -0.1, 0.05);
  const Eigen::Vector3f gt_rotation(0.04, -0.03, 0.06);
  const Sophus::SE3f T_gt(Sophus::SO3f::exp(gt_rotation), gt_translation);
  const auto tgt = transform(src.points, T_gt);
  const auto tgt_normals = rotate(src.normals, T_gt.so3());
  const Sophus::SE3f init = small_perturbation(0.05F, 0.02F, 11);
  const auto result = rko_slam::core::icp_point_to_plane(src.points, tgt, tgt_normals, 2.0F, init);

  check_pose_close(result, T_gt, kIcpTransTol, kIcpRotTol);
}

TEST_CASE("icp: non-converging input returns a well-formed pose", "[icp]") {
  // Random source vs random target, no geometric correspondence: the iteration cap is what ends it.
  const auto src = make_dense_3d(500, 5.0F, 7);
  const auto tgt = make_dense_3d(500, 5.0F, 19);
  const std::vector<Eigen::Vector3f> tgt_normals(tgt.size(), Eigen::Vector3f::UnitZ());

  const auto result = rko_slam::core::icp_point_to_plane(src, tgt, tgt_normals, 1.0F, Sophus::SE3f{});
  // The result should at least be a valid SE(3). Sophus enforces this on construction.
  const Eigen::Vector3f& trans = result.translation();
  REQUIRE(std::isfinite(trans.x()));
  REQUIRE(std::isfinite(trans.y()));
  REQUIRE(std::isfinite(trans.z()));
}

TEST_CASE("icp: an empty target is an input error", "[icp][degenerate]") {
  const Sophus::SE3f guess = Sophus::SE3f::trans(1.0F, 2.0F, 3.0F);
  REQUIRE_THROWS_AS(rko_slam::core::icp_point_to_plane({{1.0F, 0.0F, 0.0F}}, {}, {}, 1.0F, guess),
                    rko_lio::core::InputError);
}
