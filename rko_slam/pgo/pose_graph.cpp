// The dogleg defaults and the stopping criteria are g2o's OptimizationAlgorithmDogleg's and Ceres'.
#include "rko_slam/pgo/pose_graph.hpp"

#include <Eigen/Core>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>
#include <UTL/profiler.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <format>
#include <sophus/se3.hpp>
#include <string>
#include <utility>
#include <vector>

namespace rko_slam::pgo {

namespace {

// In Sophus's [translation, rotation] tangent order.
Sophus::Vector6d information_diagonal(const PoseGraph::Config& config, const PoseEdge::Kind kind) {
  const double type_scale = kind == PoseEdge::Kind::closure ? config.closure_info_scale : 1.0;
  const double rotation = type_scale * config.rotation_info_scale;
  return (Sophus::Vector6d() << type_scale, type_scale, type_scale, rotation, rotation, rotation).finished();
}

// rho(s) = d^2 * log(1 + s/d^2)
inline double cauchy_cost(const double chi2, const double delta_squared) {
  return delta_squared * std::log1p(chi2 / delta_squared);
}

// rho'(s) = 1 / (1 + s/d^2)
inline double cauchy_weight(const double chi2, const double delta_squared) {
  return 1.0 / (1.0 + (chi2 / delta_squared));
}

// Holds heading only when gravity levels the graph, all three rotations otherwise.
Sophus::Matrix6d anchor_information(const PoseGraph& pose_graph) {
  // A large prior on the anchor pose.
  constexpr double kAnchorInformation = 1e8;

  const Eigen::Vector3d up_in_held = pose_graph.anchor.map_T_held.so3().inverse() * Eigen::Vector3d::UnitZ();
  Sophus::Matrix6d information = Sophus::Matrix6d::Zero();
  information.topLeftCorner<3, 3>() = kAnchorInformation * Eigen::Matrix3d::Identity();
  information.bottomRightCorner<3, 3>() =
      kAnchorInformation * (pose_graph.gravity_edges.empty()
                                ? Eigen::Matrix3d::Identity()
                                : Eigen::Matrix3d(up_in_held * up_in_held.transpose()));
  return information;
}

// The graph's cost at these keyposes, closure edges through the kernel and everything else quadratic.
double robust_cost(const PoseGraph& pose_graph, const std::vector<Sophus::SE3d>& keyposes) {
  const PoseGraph::Config& config = pose_graph.config;
  const double delta_squared = config.closure_kernel_delta * config.closure_kernel_delta;
  const Sophus::Vector6d odometry_information = information_diagonal(config, PoseEdge::Kind::odometry);
  const Sophus::Vector6d closure_information = information_diagonal(config, PoseEdge::Kind::closure);

  double cost = 0.0;
  for (const PoseEdge& edge : pose_graph.pose_edges) {
    const bool is_closure = edge.kind == PoseEdge::Kind::closure;
    const Sophus::Vector6d error = edge.error(keyposes);
    const double chi2 = error.dot((is_closure ? closure_information : odometry_information).cwiseProduct(error));
    cost += is_closure ? cauchy_cost(chi2, delta_squared) : chi2;
  }
  for (const GravityEdge& gravity : pose_graph.gravity_edges) {
    cost += config.gravity_info_scale * gravity.error(keyposes).squaredNorm();
  }

  const Sophus::Vector6d anchor_error = pose_graph.anchor.error(keyposes);
  return cost + anchor_error.dot(anchor_information(pose_graph) * anchor_error);
}

// Row of this keypose's first tangent coordinate in H and b.
inline Eigen::Index block_start(const std::size_t keypose_id) {
  return Sophus::SE3d::DoF * static_cast<Eigen::Index>(keypose_id);
}

// The Gauss-Newton system H dx = -b, summed over the edges as H = J^T W J and b = J^T W e.
struct LinearSystem {
  Eigen::SparseMatrix<double> H;
  Eigen::VectorXd b;
};

LinearSystem linearize(const PoseGraph& pose_graph) {
  const PoseGraph::Config& config = pose_graph.config;
  const Eigen::Index size = Sophus::SE3d::DoF * static_cast<Eigen::Index>(pose_graph.keyposes.size());
  LinearSystem system{.H = {size, size}, .b = Eigen::VectorXd::Zero(size)};

  constexpr auto kScalarsPerBlock = static_cast<std::size_t>(Sophus::SE3d::DoF) * Sophus::SE3d::DoF;
  using TripletVector = std::vector<Eigen::Triplet<double, Eigen::Index>>;
  TripletVector triplets;
  // 4 blocks per pose edge + 1 per gravity edge + 1 anchor.
  triplets.reserve(kScalarsPerBlock * ((4 * pose_graph.pose_edges.size()) + pose_graph.gravity_edges.size() + 1));

  const auto add_block = [&triplets](const Sophus::Matrix6d& block, const std::size_t row_id,
                                     const std::size_t col_id) {
    const Eigen::Index row_start = block_start(row_id);
    const Eigen::Index col_start = block_start(col_id);
    for (Eigen::Index col = 0; col < Sophus::SE3d::DoF; ++col) {
      for (Eigen::Index row = 0; row < Sophus::SE3d::DoF; ++row) {
        triplets.emplace_back(row_start + row, col_start + col, block(row, col));
      }
    }
  };

  // An edge on a single keypose lands in one diagonal block of H and one segment of b.
  const auto add_unary = [&]<int Rows>(const Eigen::Matrix<double, Rows, 1>& error,
                                       const Eigen::Matrix<double, Rows, Sophus::SE3d::DoF>& jacobian,
                                       const Eigen::Matrix<double, Rows, Rows>& information,
                                       const std::size_t keypose_id) {
    const Eigen::Matrix<double, Sophus::SE3d::DoF, Rows> weighted = jacobian.transpose() * information;
    system.b.segment<Sophus::SE3d::DoF>(block_start(keypose_id)) += weighted * error;
    add_block(weighted * jacobian, keypose_id, keypose_id);
  };

  const double delta_squared = config.closure_kernel_delta * config.closure_kernel_delta;
  const Sophus::Vector6d odometry_information = information_diagonal(config, PoseEdge::Kind::odometry);
  const Sophus::Vector6d closure_information = information_diagonal(config, PoseEdge::Kind::closure);
  for (const PoseEdge& edge : pose_graph.pose_edges) {
    const bool is_closure = edge.kind == PoseEdge::Kind::closure;
    const auto [error, from_jacobian, to_jacobian] = edge.linearize(pose_graph.keyposes);
    Sophus::Vector6d information = is_closure ? closure_information : odometry_information;
    if (is_closure) {
      information *= cauchy_weight(error.dot(information.cwiseProduct(error)), delta_squared);
    }
    // A pose edge is on two keyposes, so it lands in all four blocks between them.
    const std::array<std::size_t, 2> ids{edge.from_id, edge.to_id};
    const std::array<Sophus::Matrix6d, 2> jacobians{from_jacobian, to_jacobian};
    for (std::size_t row = 0; row < ids.size(); ++row) {
      const Sophus::Matrix6d weighted = jacobians.at(row).transpose() * information.asDiagonal();
      system.b.segment<Sophus::SE3d::DoF>(block_start(ids.at(row))) += weighted * error;
      for (std::size_t col = 0; col < ids.size(); ++col) {
        add_block(weighted * jacobians.at(col), ids.at(row), ids.at(col));
      }
    }
  }

  const Eigen::Matrix2d gravity_information = config.gravity_info_scale * Eigen::Matrix2d::Identity();
  for (const GravityEdge& gravity : pose_graph.gravity_edges) {
    const auto [error, jacobian] = gravity.linearize(pose_graph.keyposes);
    add_unary(error, jacobian, gravity_information, gravity.keypose_id);
  }

  const auto [error, jacobian] = pose_graph.anchor.linearize(pose_graph.keyposes);
  add_unary(error, jacobian, anchor_information(pose_graph), pose_graph.anchor.keypose_id);

  // setFromTriplets sums the entries that share a coordinate.
  system.H.setFromTriplets(triplets.begin(), triplets.end());
  return system;
}

// Powell's dogleg: the Gauss-Newton step while it fits the trust region, the steepest descent step cut to the
// boundary when even that leaves it, and where the leg between the two crosses the boundary in between.
Eigen::VectorXd
dogleg_step(const Eigen::VectorXd& gauss_newton, const Eigen::VectorXd& steepest_descent, const double trust_region) {
  if (gauss_newton.norm() < trust_region) {
    return gauss_newton;
  }
  const double steepest_descent_norm = steepest_descent.norm();
  if (steepest_descent_norm > trust_region) {
    return (trust_region / steepest_descent_norm) * steepest_descent;
  }

  // Crossing of the leg between the two: ||steepest_descent + beta * leg||^2 = trust_region^2, so
  // a * beta^2 + b * beta + c = 0.
  // NOLINTBEGIN(readability-identifier-length) standard quadratic notation
  const Eigen::VectorXd leg = gauss_newton - steepest_descent;
  const double a = leg.squaredNorm();
  const double b = 2.0 * steepest_descent.dot(leg);
  // At most zero: the steepest descent step ends inside the region.
  const double c = steepest_descent.squaredNorm() - (trust_region * trust_region);
  const double discriminant_root = std::sqrt((b * b) - (4.0 * a * c));
  // The positive root, in whichever of its two equivalent forms avoids subtracting near-equal numbers.
  const double beta = b <= 0.0 ? (discriminant_root - b) / (2.0 * a) : (-2.0 * c) / (b + discriminant_root);
  // NOLINTEND(readability-identifier-length)
  return steepest_descent + (beta * leg);
}

// The keyposes moved by their own segments of the step.
std::vector<Sophus::SE3d> stepped_keyposes(const std::vector<Sophus::SE3d>& keyposes,
                                                  const Eigen::VectorXd& step) {
  std::vector<Sophus::SE3d> moved;
  moved.reserve(keyposes.size());
  for (std::size_t keypose_id = 0; keypose_id < keyposes.size(); ++keypose_id) {
    moved.push_back(keyposes.at(keypose_id) *
                    Sophus::SE3d::exp(step.segment<Sophus::SE3d::DoF>(block_start(keypose_id))));
  }
  return moved;
}

// Ceres' parameter_tolerance test, on the [quaternion, translation] parameters.
bool parameters_settled(const std::vector<Sophus::SE3d>& keyposes,
                               const std::vector<Sophus::SE3d>& candidate) {
  constexpr double kParameterTolerance = 1e-8;

  double parameters_squared = 0.0;
  double step_squared = 0.0;
  for (std::size_t keypose_id = 0; keypose_id < keyposes.size(); ++keypose_id) {
    const Eigen::Matrix<double, Sophus::SE3d::num_parameters, 1> parameters = keyposes.at(keypose_id).params();
    parameters_squared += parameters.squaredNorm();
    step_squared += (candidate.at(keypose_id).params() - parameters).squaredNorm();
  }

  return std::sqrt(step_squared) <= kParameterTolerance * (std::sqrt(parameters_squared) + kParameterTolerance);
}

// The drop the model promises for this step, -(2 b^T s + s^T H s), with a minimum value to avoid div by zero.
inline double predicted_gain(const LinearSystem& system, const Eigen::VectorXd& step) {
  constexpr double kMinPredictedGain = 1e-12;
  const double gain = -((2.0 * system.b.dot(step)) + step.dot(system.H * step));
  return std::abs(gain) < kMinPredictedGain ? kMinPredictedGain : gain;
}

// Grows past a step the model predicted well, halves on one it predicted badly.
double updated_trust_region(const double trust_region, const double gain_ratio, const double step_norm) {
  if (gain_ratio > 0.75) {
    return std::max(trust_region, 3.0 * step_norm);
  }
  if (gain_ratio < 0.25) {
    return 0.5 * trust_region;
  }
  return trust_region;
}
} // namespace

std::string PoseGraph::Config::to_yaml() const {
  return std::format("max_iterations: {}\nclosure_info_scale: {}\nrotation_info_scale: {}\n"
                     "closure_kernel_delta: {}\ngravity_info_scale: {}\n",
                     max_iterations, closure_info_scale, rotation_info_scale, closure_kernel_delta, gravity_info_scale);
}

PoseGraph::Outcome PoseGraph::optimize() {
  UTL_PROFILER_SCOPE("pgo::optimize");
  if (pose_edges.empty() && gravity_edges.empty()) {
    return Outcome::converged;
  }

  constexpr double kInitialTrustRegion = 1e4;
  constexpr int kMaxStepTrials = 100;
  constexpr double kInitialDamping = 1e-7;
  constexpr double kDampingFactor = 10.0;
  constexpr double kMinDamping = 1e-12;
  constexpr double kMaxDamping = 1e3;
  constexpr double kFunctionTolerance = 1e-6;

  double cost = robust_cost(*this, keyposes);
  if (!std::isfinite(cost)) {
    return Outcome::failed;
  }

  double trust_region = kInitialTrustRegion;
  // Added to the Hessian diagonal, and stays zero until a factorization fails.
  double damping = 0.0;
  Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver;
  for (int iter = 0; iter < config.max_iterations; ++iter) {
    const LinearSystem system = linearize(*this);
    if (iter == 0) {
      // Every linearization of this graph has the same sparsity, so the symbolic factorization is done once.
      solver.analyzePattern(system.H);
    }

    const auto factorize = [&] {
      solver.setShift(damping);
      solver.factorize(system.H);
      return solver.info() == Eigen::Success;
    };
    // Raise the damping until it factors, and give up on a Hessian that needs more than kMaxDamping.
    while (!factorize()) {
      damping = std::max(damping, kInitialDamping) * kDampingFactor;
      if (damping > kMaxDamping) {
        return Outcome::failed;
      }
    }
    // Lowers the damping the next iteration starts from; this one already factored.
    if (damping > 0.0) {
      damping = std::max(kMinDamping, damping / (0.5 * kDampingFactor));
    }

    const Eigen::VectorXd gauss_newton = solver.solve(-system.b);
    // The Cauchy point: the model's minimum along -b, at t = ||b||^2 / (b^T H b).
    const Eigen::VectorXd steepest_descent =
        -(system.b.squaredNorm() / system.b.dot(system.H * system.b)) * system.b;

    bool accepted = false;
    for (int trial = 0; trial < kMaxStepTrials; ++trial) {
      const Eigen::VectorXd step = dogleg_step(gauss_newton, steepest_descent, trust_region);
      std::vector<Sophus::SE3d> candidate = stepped_keyposes(keyposes, step);
      const double candidate_cost = robust_cost(*this, candidate);
      if (parameters_settled(keyposes, candidate) ||
          std::abs(cost - candidate_cost) <= kFunctionTolerance * cost) {
        return Outcome::converged;
      }
      const double gain_ratio = (cost - candidate_cost) / predicted_gain(system, step);
      trust_region = updated_trust_region(trust_region, gain_ratio, step.norm());
      if (gain_ratio > 0.0 && candidate_cost < cost) {
        keyposes = std::move(candidate);
        cost = candidate_cost;
        accepted = true;
        break;
      }
    }

    // No trial lowered the cost, however far the trust region shrank.
    if (!accepted) {
      return Outcome::stalled;
    }
  }
  return Outcome::max_iterations;
}

} // namespace rko_slam::pgo
