#pragma once

#include <cstddef>
#include <array>
#include <cstdint>
#include <sophus/se3.hpp>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Core>

namespace rko_slam::pgo {

struct PoseEdge {
  enum class Kind : std::uint8_t { odometry, closure };

  struct Linearization {
    Sophus::Vector6d error;
    // For map_T_from * exp(delta).
    Sophus::Matrix6d from_jacobian;
    // For map_T_to * exp(delta).
    Sophus::Matrix6d to_jacobian;
  };

  std::size_t from_id;
  std::size_t to_id;
  Sophus::SE3d from_T_to;
  Kind kind;

  Sophus::Vector6d error(const std::vector<Sophus::SE3d>& keyposes) const {
    const Sophus::SE3d& map_T_from = keyposes.at(from_id);
    const Sophus::SE3d& map_T_to = keyposes.at(to_id);
    return (from_T_to.inverse() * map_T_from.inverse() * map_T_to).log();
  }

  Linearization linearize(const std::vector<Sophus::SE3d>& keyposes) const {
    const Sophus::SE3d& map_T_from = keyposes.at(from_id);
    const Sophus::SE3d& map_T_to = keyposes.at(to_id);
    const Sophus::Vector6d residual = error(keyposes);
    const Sophus::Matrix6d to_jacobian = Sophus::SE3d::leftJacobianInverse(-residual);
    return {
        .error = residual,
        .from_jacobian = -to_jacobian * (map_T_to.inverse() * map_T_from).Adj(),
        .to_jacobian = to_jacobian,
    };
  }
};

struct GravityEdge {
  using JacobianMatrix = Eigen::Matrix<double, 2, Sophus::SE3d::DoF>;

  struct Linearization {
    Eigen::Vector2d error;
    // For map_T_keypose * exp(delta).
    JacobianMatrix jacobian;
  };

  std::size_t keypose_id;
  // In the keypose frame, of any length.
  Eigen::Vector3d measured_up;

  // The error lives on the plane normal to measured_up: two DoF, the tilt to up_in_map in the keypose frame. On the
  // measurement and not the prediction, so a keypose update needs no re-estimation of it.
  Eigen::Matrix<double, 2, 3> tangent_basis() const {
    const Eigen::Vector3d direction = measured_up.normalized();
    const Eigen::Vector3d first = direction.unitOrthogonal();
    Eigen::Matrix<double, 2, 3> basis;
    basis.row(0) = first.transpose();
    basis.row(1) = direction.cross(first).transpose();
    return basis;
  }

  Eigen::Vector2d error(const std::vector<Sophus::SE3d>& keyposes) const {
    return tangent_basis() * (keyposes.at(keypose_id).so3().inverse() * Eigen::Vector3d::UnitZ());
  }

  Linearization linearize(const std::vector<Sophus::SE3d>& keyposes) const {
    // error() inlined: the Jacobian needs both halves of it.
    const Eigen::Vector3d up_in_keypose = keyposes.at(keypose_id).so3().inverse() * Eigen::Vector3d::UnitZ();
    const Eigen::Matrix<double, 2, 3> basis = tangent_basis();
    Linearization linearization{.error = basis * up_in_keypose, .jacobian = JacobianMatrix::Zero()};
    linearization.jacobian.rightCols<3>() = basis * Sophus::SO3d::hat(up_in_keypose);
    return linearization;
  }
};

// The gauge fix.
struct Anchor {
  struct Linearization {
    Sophus::Vector6d error;
    // For map_T_keypose * exp(delta).
    Sophus::Matrix6d jacobian;
  };

  std::size_t keypose_id = 0;
  Sophus::SE3d map_T_held;

  Sophus::Vector6d error(const std::vector<Sophus::SE3d>& keyposes) const {
    return (map_T_held.inverse() * keyposes.at(keypose_id)).log();
  }

  Linearization linearize(const std::vector<Sophus::SE3d>& keyposes) const {
    const Sophus::Vector6d residual = error(keyposes);
    return {.error = residual, .jacobian = Sophus::SE3d::leftJacobianInverse(-residual)};
  }
};

struct PoseGraph {
  struct Config {
    int max_iterations = 10;
    // Closure edge type-scale; odom's is 1.
    double closure_info_scale = 1.0;
    // A squared range: at 5 m a rotation error costs the same as the translation it produces there.
    double rotation_info_scale = 25.0;
    // Cauchy delta on closure edges, in metres; odom edges stay quadratic.
    double closure_kernel_delta = 1.0;
    // Weight on a measured up, per rad^2 of tilt.
    double gravity_info_scale = 100.0;
  
    std::string to_yaml() const;
  };

  PoseGraph() = default;
  explicit PoseGraph(const Config& config) : config(config) {}

  Config config;
  std::vector<Sophus::SE3d> keyposes;
  std::vector<PoseEdge> pose_edges;
  std::vector<GravityEdge> gravity_edges;
  Anchor anchor;

  // Holds this keypose at its current pose. The graph has one anchor, so a later call replaces it.
  void anchor_at(const std::size_t keypose_id) {
    anchor = {.keypose_id = keypose_id, .map_T_held = keyposes.at(keypose_id)};
  }

  // Returns the new keypose's id.
  std::size_t add_keypose(const Sophus::SE3d& map_T_keypose) {
    keyposes.push_back(map_T_keypose);
    return keyposes.size() - 1;
  }

  void add_odometry_edge(const std::size_t from_id, const std::size_t to_id, const Sophus::SE3d& from_T_to) {
    pose_edges.push_back({.from_id = from_id, .to_id = to_id, .from_T_to = from_T_to, .kind = PoseEdge::Kind::odometry});
  }

  void add_closure_edge(const std::size_t from_id, const std::size_t to_id, const Sophus::SE3d& from_T_to) {
    pose_edges.push_back({.from_id = from_id, .to_id = to_id, .from_T_to = from_T_to, .kind = PoseEdge::Kind::closure});
  }

  void add_gravity_edge(const std::size_t keypose_id, const Eigen::Vector3d& measured_up) {
    gravity_edges.push_back({.keypose_id = keypose_id, .measured_up = measured_up});
  }

  // Takes back the edge just added, after an optimize that failed on it.
  void remove_last_pose_edge() { pose_edges.pop_back(); }

  // Why optimize stopped. Only failed leaves the keyposes untouched by this call; the others leave them at the
  // last state that lowered the cost.
  enum class Outcome : std::uint8_t { converged, stalled, max_iterations, failed };

  // failed is a cost that is not finite, or a Hessian no damping we allow can factor.
  Outcome optimize();
};

inline std::string_view to_string(const PoseGraph::Outcome outcome) {
  constexpr std::array<std::string_view, 4> kNames{"converged", "stalled", "max iterations", "failed"};
  return kNames.at(static_cast<std::size_t>(outcome));
}

} // namespace rko_slam::pgo
