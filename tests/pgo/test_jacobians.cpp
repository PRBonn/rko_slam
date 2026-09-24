#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <concepts>
#include <sophus/se3.hpp>

#include "rko_slam/pgo/pose_graph.hpp"

using rko_slam::pgo::Anchor;
using rko_slam::pgo::GravityEdge;
using rko_slam::pgo::PoseEdge;

namespace {

Sophus::SE3d pose_at(const double phase, const double max_angle, const double max_translation) {
  const Eigen::Vector3d axis =
      Eigen::Vector3d(std::sin(phase), std::cos(1.7 * phase), std::sin(2.3 * phase) + 0.1).normalized();
  const Eigen::Vector3d translation(std::cos(0.9 * phase), std::sin(1.3 * phase), std::cos(3.1 * phase));
  return {Sophus::SO3d::exp(max_angle * std::sin(0.7 * phase) * axis), max_translation * translation};
}

template <int Rows>
double worst_scaled_gap(const Eigen::Matrix<double, Rows, Sophus::SE3d::DoF>& jacobian,
                        const std::invocable<const Sophus::Vector6d&> auto& error_at) {
  constexpr double kStep = 1e-6;
  using Error = Eigen::Matrix<double, Rows, 1>;
  double worst = 0.0;
  for (Eigen::Index k = 0; k < Sophus::SE3d::DoF; ++k) {
    const Sophus::Vector6d delta = kStep * Sophus::Vector6d::Unit(k);
    const Error numeric = (error_at(delta) - error_at(-delta)) / (2.0 * kStep);
    const Error gap = (numeric - jacobian.col(k)).cwiseAbs().cwiseQuotient(Error::Ones() + jacobian.col(k).cwiseAbs());
    worst = std::max(worst, gap.maxCoeff());
  }
  return worst;
}

} // namespace

TEST_CASE("pgo: edge Jacobians match central differences", "[pgo]") {
  for (const double scale : {1.0, 100.0, 5000.0}) {
    for (int sample = 0; sample < 50; ++sample) {
      CAPTURE(scale, sample);
      const auto phase = static_cast<double>(sample);
      const Sophus::SE3d from_keypose = pose_at(phase, 3.0, scale);
      const Sophus::SE3d to_keypose = pose_at(phase + 11.0, 3.0, scale);
      // An exact measurement, as a converged odometry edge has, and one up to 1.2 rad off, as an outlier closure can
      // be.
      for (const Sophus::SE3d& measurement_error : {Sophus::SE3d{}, pose_at(phase + 23.0, 1.2, 0.1 * scale)}) {
        const PoseEdge edge{
            .from_id = 0,
            .to_id = 1,
            .from_T_to = from_keypose.inverse() * to_keypose * measurement_error,
            .kind = PoseEdge::Kind::odometry,
        };
        const PoseEdge::Linearization linearization = edge.linearize({from_keypose, to_keypose});
        CHECK(worst_scaled_gap(linearization.from_jacobian, [&](const Sophus::Vector6d& delta) {
                return edge.error({from_keypose * Sophus::SE3d::exp(delta), to_keypose});
              }) < 1e-5);
        CHECK(worst_scaled_gap(linearization.to_jacobian, [&](const Sophus::Vector6d& delta) {
                return edge.error({from_keypose, to_keypose * Sophus::SE3d::exp(delta)});
              }) < 1e-5);
      }
    }
  }
}

TEST_CASE("pgo: gravity Jacobians match central differences", "[pgo]") {
  for (int sample = 0; sample < 50; ++sample) {
    CAPTURE(sample);
    const auto phase = static_cast<double>(sample);
    const Sophus::SE3d keypose = pose_at(phase, 3.0, 100.0);
    // any length, and up to 1.2 rad off the keypose's own up, as a sub-map that drove a slope measures
    const Eigen::Vector3d measured_up = 9.81 * (pose_at(phase + 7.0, 1.2, 1.0).so3() * Eigen::Vector3d::UnitZ());
    const GravityEdge gravity{.keypose_id = 0, .measured_up = measured_up};
    CHECK(worst_scaled_gap<2>(gravity.linearize({keypose}).jacobian, [&](const Sophus::Vector6d& delta) {
            return gravity.error({keypose * Sophus::SE3d::exp(delta)});
          }) < 1e-5);
  }
}

TEST_CASE("pgo: anchor Jacobians match central differences", "[pgo]") {
  for (int sample = 0; sample < 50; ++sample) {
    CAPTURE(sample);
    const auto phase = static_cast<double>(sample);
    const Sophus::SE3d keypose = pose_at(phase, 3.0, 100.0);
    // where it is held, and 1.2 rad off it, which is further than an anchor ever drifts
    for (const Sophus::SE3d& map_T_held : {keypose, keypose * pose_at(phase + 23.0, 1.2, 10.0)}) {
      const Anchor anchor{.keypose_id = 0, .map_T_held = map_T_held};
      CHECK(
          worst_scaled_gap<Sophus::SE3d::DoF>(anchor.linearize({keypose}).jacobian, [&](const Sophus::Vector6d& delta) {
            return anchor.error({keypose * Sophus::SE3d::exp(delta)});
          }) < 1e-5);
    }
  }
}

TEST_CASE("pgo: the gravity error is zero for the keypose's own up axis, whatever its length and heading", "[pgo]") {
  const Sophus::SE3d pose = pose_at(3.0, 3.0, 100.0);
  const Eigen::Vector3d up_in_keypose = 4.2 * (pose.so3().inverse() * Eigen::Vector3d::UnitZ());
  for (const double heading : {0.0, 0.9, -2.5}) {
    CAPTURE(heading);
    const Sophus::SE3d turned = Sophus::SE3d(Sophus::SO3d::rotZ(heading), Eigen::Vector3d::Zero()) * pose;
    CHECK(GravityEdge{.keypose_id = 0, .measured_up = up_in_keypose}.error({turned}).norm() < 1e-12);
  }
  const Sophus::SE3d tilted = Sophus::SE3d(Sophus::SO3d::rotX(0.2), Eigen::Vector3d::Zero()) * pose;
  CHECK(std::abs(GravityEdge{.keypose_id = 0, .measured_up = up_in_keypose}.error({tilted}).norm() - std::sin(0.2)) <
        1e-9);
}
