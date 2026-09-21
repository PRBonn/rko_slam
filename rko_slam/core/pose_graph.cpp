#include "rko_slam/core/pose_graph.hpp"

#include <UTL/profiler.hpp>
#include <algorithm>
#include <filesystem>
#include <format>
#include <memory>
#include <ranges>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/sparse_optimizer_terminate_action.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/parameter_se3_offset.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <spdlog/spdlog.h>

#include "rko_slam/core/edges.hpp"

// Forces the linker to keep g2o's slam3d type-registration TU. Dead-stripped without it, the g2o Factory has no
// slam3d tags and SparseOptimizer::save() silently writes 0-byte files.
G2O_USE_TYPE_GROUP(slam3d);

namespace rko_slam::core {

namespace {

// The identity offset EdgeSE3Prior resolves its cache through.
constexpr int kGaugeOffsetParameterId = 0;

g2o::EdgeSE3* find_se3_edge(const g2o::SparseOptimizer& optimizer, const KeyposeId from_id, const KeyposeId to_id) {
  const auto edges =
      optimizer.edges() | std::views::transform([](auto* edge) { return dynamic_cast<g2o::EdgeSE3*>(edge); });
  const auto match = std::ranges::find_if(edges, [&](const g2o::EdgeSE3* edge) {
    return edge != nullptr && std::cmp_equal(edge->vertex(0)->id(), from_id) &&
           std::cmp_equal(edge->vertex(1)->id(), to_id);
  });
  return match == edges.end() ? nullptr : *match;
}

} // namespace

std::string PoseGraph::Config::to_yaml() const {
  return std::format("max_iterations: {}\nclosure_info_scale: {}\nrotation_info_scale: {}\n"
                     "closure_kernel_delta: {}\ngravity_info_scale: {}\n",
                     max_iterations, closure_info_scale, rotation_info_scale, closure_kernel_delta, gravity_info_scale);
}

PoseGraph::PoseGraph(const Config pose_graph_config) : config(pose_graph_config) {
  using BlockSolverType = g2o::BlockSolverX;
  using LinearSolverType = g2o::LinearSolverCholmod<BlockSolverType::PoseMatrixType>;
  auto linear = std::make_unique<LinearSolverType>();
  auto block = std::make_unique<BlockSolverType>(std::move(linear));

  auto* algo = new g2o::OptimizationAlgorithmDogleg(std::move(block));

  optimizer = std::make_unique<g2o::SparseOptimizer>();
  optimizer->setAlgorithm(algo);
  terminate_action = std::make_unique<g2o::SparseOptimizerTerminateAction>();
  terminate_action->setGainThreshold(1e-6);
  optimizer->addPostIterationAction(terminate_action.get());
}

PoseGraph::~PoseGraph() = default;

void PoseGraph::add_keypose(const KeyposeId keypose_id, const Sophus::SE3d& pose) {
  // NOLINTNEXTLINE(cppcoreguidelines-owning-memory) g2o takes ownership of the raw pointer
  auto* vertex = new g2o::VertexSE3();
  vertex->setId(static_cast<int>(keypose_id));
  vertex->setEstimate(to_isometry(pose));
  optimizer->addVertex(vertex);
}

void PoseGraph::set_keypose_fixed(const KeyposeId keypose_id, const bool fixed) {
  auto* vertex = dynamic_cast<g2o::VertexSE3*>(optimizer->vertex(static_cast<int>(keypose_id)));
  if (vertex == nullptr) {
    throw std::runtime_error(std::format("PoseGraph::set_keypose_fixed: no VertexSE3 with id {}", keypose_id));
  }
  vertex->setFixed(fixed);
}

bool PoseGraph::is_keypose_fixed(const KeyposeId keypose_id) const {
  const auto* vertex = dynamic_cast<const g2o::VertexSE3*>(optimizer->vertex(static_cast<int>(keypose_id)));
  if (vertex == nullptr) {
    throw std::runtime_error(std::format("PoseGraph::is_keypose_fixed: no VertexSE3 with id {}", keypose_id));
  }
  return vertex->fixed();
}

Sophus::SE3d PoseGraph::get_keypose(const KeyposeId keypose_id) const {
  const auto* vertex = dynamic_cast<const g2o::VertexSE3*>(optimizer->vertex(static_cast<int>(keypose_id)));
  if (vertex == nullptr) {
    throw std::runtime_error(std::format("PoseGraph::get_keypose: no VertexSE3 with id {}", keypose_id));
  }
  return from_isometry(vertex->estimate());
}

void PoseGraph::add_odom_edge(const KeyposeId from_id, const KeyposeId to_id, const Sophus::SE3d& from_T_to) {
  std::unique_ptr<g2o::EdgeSE3> edge = make_odom_edge(from_T_to, config.rotation_info_scale);
  edge->setVertex(0, optimizer->vertex(static_cast<int>(from_id)));
  edge->setVertex(1, optimizer->vertex(static_cast<int>(to_id)));
  optimizer->addEdge(edge.release());
}

void PoseGraph::add_closure_edge(const KeyposeId from_id, const KeyposeId to_id, const Sophus::SE3d& from_T_to) {
  std::unique_ptr<g2o::EdgeSE3> edge =
      make_closure_edge(from_T_to, config.rotation_info_scale, config.closure_info_scale, config.closure_kernel_delta);
  edge->setVertex(0, optimizer->vertex(static_cast<int>(from_id)));
  edge->setVertex(1, optimizer->vertex(static_cast<int>(to_id)));
  optimizer->addEdge(edge.release());
}

void PoseGraph::remove_closure_edge(const KeyposeId from_id, const KeyposeId to_id) {
  if (!is_closure_pair(from_id, to_id)) {
    return;
  }
  if (auto* edge = find_se3_edge(*optimizer, from_id, to_id)) {
    optimizer->removeEdge(edge);
  }
}

void PoseGraph::add_gravity_edge(const KeyposeId keypose_id, const Eigen::Vector3d& measured_up) {
  std::unique_ptr<EdgeGravity> edge = make_gravity_edge(measured_up, config.gravity_info_scale);
  edge->setVertex(0, optimizer->vertex(static_cast<int>(keypose_id)));
  optimizer->addEdge(edge.release());
}

void PoseGraph::add_gauge_edge(const KeyposeId keypose_id) {
  // load() adds this parameter from the file it reads, and g2o logs an error if it is added twice.
  if (optimizer->parameter(kGaugeOffsetParameterId) == nullptr) {
    auto gauge_offset = std::make_unique<g2o::ParameterSE3Offset>();
    gauge_offset->setId(kGaugeOffsetParameterId);
    optimizer->addParameter(gauge_offset.release());
  }
  std::unique_ptr<g2o::EdgeSE3Prior> edge = make_gauge_edge(get_keypose(keypose_id), kGaugeOffsetParameterId);
  edge->setVertex(0, optimizer->vertex(static_cast<int>(keypose_id)));
  optimizer->addEdge(edge.release());
}

bool PoseGraph::optimize() {
  UTL_PROFILER_SCOPE("PoseGraph::optimize");
  if (optimizer->edges().empty()) {
    return false;
  }
  try {
    optimizer->initializeOptimization();
    optimizer->optimize(config.max_iterations);
    return true;
  } catch (const std::exception& error) {
    spdlog::error("PoseGraph::optimize: {}", error.what());
    return false;
  }
}

bool PoseGraph::save(const std::filesystem::path& path) const {
  return optimizer->save(path.c_str()) && std::filesystem::file_size(path) > 0;
}

bool PoseGraph::load(const std::filesystem::path& path) { return optimizer->load(path.c_str()); }

std::size_t PoseGraph::num_keyposes() const { return optimizer->vertices().size(); }

std::size_t PoseGraph::num_closure_edges() const {
  return static_cast<std::size_t>(std::ranges::count_if(
      se3_edges(), [](const Se3EdgeView& edge) { return is_closure_pair(edge.from_id, edge.to_id); }));
}

std::vector<PoseGraph::GravityEdgeView> PoseGraph::gravity_edges() const {
  std::vector<GravityEdgeView> out;
  for (const auto* edge : optimizer->edges()) {
    const auto* gravity = dynamic_cast<const EdgeGravity*>(edge);
    if (gravity == nullptr) {
      continue;
    }
    out.push_back({
        .keypose_id = static_cast<KeyposeId>(gravity->vertex(0)->id()),
        .measured_up = gravity->measurement(),
    });
  }
  return out;
}

std::vector<PoseGraph::Se3EdgeView> PoseGraph::se3_edges() const {
  std::vector<Se3EdgeView> out;
  out.reserve(optimizer->edges().size());
  for (const auto* edge : optimizer->edges()) {
    const auto* edge_se3 = dynamic_cast<const g2o::EdgeSE3*>(edge);
    if (edge_se3 == nullptr) {
      continue;
    }
    out.push_back({
        .from_id = static_cast<KeyposeId>(edge_se3->vertex(0)->id()),
        .to_id = static_cast<KeyposeId>(edge_se3->vertex(1)->id()),
        .from_T_to = from_isometry(edge_se3->measurement()),
    });
  }
  return out;
}

double PoseGraph::edge_chi2(const KeyposeId from_id, const KeyposeId to_id) {
  g2o::EdgeSE3* edge = find_se3_edge(*optimizer, from_id, to_id);
  if (edge == nullptr) {
    throw std::runtime_error(std::format("PoseGraph::edge_chi2: no edge from {} to {}", from_id, to_id));
  }
  edge->computeError();
  return edge->chi2();
}

} // namespace rko_slam::core
