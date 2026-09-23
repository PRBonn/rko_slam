#include <Eigen/Core>
#include <Eigen/Geometry>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <memory>
#include <numbers>
#include <sophus/se3.hpp>
#include <vector>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/sparse_optimizer_terminate_action.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include "rko_slam/pgo/pose_graph.hpp"

using Config = rko_slam::pgo::PoseGraph::Config;
using rko_slam::pgo::PoseEdge;
using rko_slam::pgo::PoseGraph;

namespace {

constexpr std::size_t kLoopSize = 12;

// A 20 m loop that climbs and rolls, so all six tangent directions carry error.
std::vector<Sophus::SE3d> loop_keyposes() {
  std::vector<Sophus::SE3d> keyposes;
  for (std::size_t keypose_id = 0; keypose_id < kLoopSize; ++keypose_id) {
    const double angle = 2.0 * std::numbers::pi * static_cast<double>(keypose_id) / static_cast<double>(kLoopSize);
    const Sophus::SO3d heading =
        Sophus::SO3d::rotZ(angle + (std::numbers::pi / 2.0)) * Sophus::SO3d::rotX(0.1 * std::sin(3.0 * angle));
    keyposes.emplace_back(heading,
                          Eigen::Vector3d(20.0 * std::cos(angle), 20.0 * std::sin(angle), std::sin(2.0 * angle)));
  }
  return keyposes;
}

PoseGraph noisy_loop() {
  const std::vector<Sophus::SE3d> truth = loop_keyposes();
  PoseGraph pose_graph;
  const auto measure = [&truth, &pose_graph](const std::size_t from_id, const std::size_t to_id, const PoseEdge::Kind kind) {
    const auto phase = static_cast<double>(pose_graph.pose_edges.size());
    const Sophus::Vector6d noise =
        (Sophus::Vector6d() << 0.05 * std::sin(phase), 0.05 * std::cos(1.3 * phase), 0.03 * std::sin(2.1 * phase),
         0.005 * std::cos(phase), 0.005 * std::sin(1.7 * phase), 0.008 * std::cos(2.3 * phase))
            .finished();
    const Sophus::SE3d from_T_to = truth.at(from_id).inverse() * truth.at(to_id) * Sophus::SE3d::exp(noise);
    if (kind == PoseEdge::Kind::closure) {
      pose_graph.add_closure_edge(from_id, to_id, from_T_to);
    } else {
      pose_graph.add_odometry_edge(from_id, to_id, from_T_to);
    }
  };
  for (std::size_t from_id = 0; from_id + 1 < kLoopSize; ++from_id) {
    measure(from_id, from_id + 1, PoseEdge::Kind::odometry);
  }
  measure(11, 0, PoseEdge::Kind::closure);
  measure(0, 4, PoseEdge::Kind::closure);
  measure(4, 8, PoseEdge::Kind::closure);
  measure(8, 0, PoseEdge::Kind::closure);
  measure(2, 7, PoseEdge::Kind::closure);
  pose_graph.pose_edges.back().from_T_to = pose_graph.pose_edges.back().from_T_to * Sophus::SE3d::trans(5.0, 0.0, 0.0);

  pose_graph.add_keypose(truth.front());
  for (const PoseEdge& edge : pose_graph.pose_edges) {
    if (edge.kind == PoseEdge::Kind::odometry) {
      pose_graph.add_keypose(pose_graph.keyposes.back() * edge.from_T_to);
    }
  }
  pose_graph.anchor_at(0);
  return pose_graph;
}

g2o::EdgeSE3::InformationType g2o_information(const Config& config, const PoseEdge& edge) {
  const double type_scale = edge.kind == PoseEdge::Kind::closure ? config.closure_info_scale : 1.0;
  g2o::EdgeSE3::InformationType information = g2o::EdgeSE3::InformationType::Identity();
  information.diagonal().tail<3>().setConstant(4.0 * config.rotation_info_scale);
  return type_scale * information;
}

std::vector<Sophus::SE3d> optimize_in_g2o(const Config& config, const PoseGraph& pose_graph) {
  using LinearSolver = g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>;
  g2o::SparseOptimizerTerminateAction terminate_action;
  terminate_action.setGainThreshold(1e-6);
  g2o::SparseOptimizer optimizer;
  auto algorithm = std::make_unique<g2o::OptimizationAlgorithmDogleg>(
      std::make_unique<g2o::BlockSolverX>(std::make_unique<LinearSolver>()));
  optimizer.setAlgorithm(algorithm.release());
  optimizer.addPostIterationAction(&terminate_action);

  std::vector<g2o::VertexSE3*> vertices;
  for (std::size_t keypose_id = 0; keypose_id < pose_graph.keyposes.size(); ++keypose_id) {
    auto vertex = std::make_unique<g2o::VertexSE3>();
    vertex->setId(static_cast<int>(keypose_id));
    vertex->setEstimate(Eigen::Isometry3d(pose_graph.keyposes.at(keypose_id).matrix()));
    vertex->setFixed(keypose_id == pose_graph.anchor.keypose_id);
    vertices.push_back(vertex.get());
    optimizer.addVertex(vertex.release());
  }
  for (const PoseEdge& edge : pose_graph.pose_edges) {
    auto g2o_edge = std::make_unique<g2o::EdgeSE3>();
    g2o_edge->setVertex(0, vertices.at(edge.from_id));
    g2o_edge->setVertex(1, vertices.at(edge.to_id));
    g2o_edge->setMeasurement(Eigen::Isometry3d(edge.from_T_to.matrix()));
    g2o_edge->setInformation(g2o_information(config, edge));
    if (edge.kind == PoseEdge::Kind::closure) {
      auto kernel = std::make_unique<g2o::RobustKernelCauchy>();
      kernel->setDelta(config.closure_kernel_delta);
      g2o_edge->setRobustKernel(kernel.release());
    }
    optimizer.addEdge(g2o_edge.release());
  }
  optimizer.initializeOptimization();
  optimizer.optimize(config.max_iterations);

  std::vector<Sophus::SE3d> keyposes;
  keyposes.reserve(vertices.size());
  for (const g2o::VertexSE3* vertex : vertices) {
    keyposes.emplace_back(Eigen::Quaterniond(vertex->estimate().linear()), vertex->estimate().translation());
  }
  return keyposes;
}

} // namespace

TEST_CASE("pgo g2o oracle: a noisy loop with an outlier closure optimizes to g2o's poses", "[pgo][g2o]") {
  const Config config;
  PoseGraph pose_graph = noisy_loop();
  const std::vector<Sophus::SE3d> g2o_keyposes = optimize_in_g2o(config, pose_graph);
  REQUIRE(pose_graph.optimize() != PoseGraph::Outcome::failed);
  for (std::size_t keypose_id = 0; keypose_id < pose_graph.keyposes.size(); ++keypose_id) {
    CAPTURE(keypose_id);
    const Sophus::SE3d difference = g2o_keyposes.at(keypose_id).inverse() * pose_graph.keyposes.at(keypose_id);
    REQUIRE(difference.translation().norm() < 1e-2);
    REQUIRE(difference.so3().log().norm() < 1e-3);
  }
}
