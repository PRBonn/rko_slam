#include "rko_slam/core/pose_graph.hpp"

#include <UTL/profiler.hpp>
#include <algorithm>
#include <filesystem>
#include <format>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/sparse_optimizer_terminate_action.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <spdlog/spdlog.h>

// Forces the linker to keep g2o's slam3d type-registration TU. Dead-stripped without it, the g2o Factory has no
// slam3d tags and SparseOptimizer::save() silently writes 0-byte files.
G2O_USE_TYPE_GROUP(slam3d);

namespace rko_slam::core {

namespace {

g2o::EdgeSE3& as_edge_se3(g2o::HyperGraph::Edge& edge) {
  auto* edge_se3 = dynamic_cast<g2o::EdgeSE3*>(&edge);
  if (edge_se3 == nullptr) {
    throw std::runtime_error("PoseGraph: the graph holds an edge that is not an EdgeSE3");
  }
  return *edge_se3;
}

Eigen::Isometry3d to_isometry(const Sophus::SE3d& pose) { return Eigen::Isometry3d(pose.matrix()); }

Sophus::SE3d from_isometry(const Eigen::Isometry3d& iso) {
  return {Eigen::Quaterniond(iso.linear()), iso.translation()};
}

// g2o EdgeSE3 orders its error [translation(0:2), rotation(3:5)].
g2o::EdgeSE3::InformationType info_for(const PoseGraph::Config& config, const double type_scale) {
  g2o::EdgeSE3::InformationType info = g2o::EdgeSE3::InformationType::Identity();
  info(3, 3) = info(4, 4) = info(5, 5) = config.rotation_info_scale;
  return type_scale * info;
}

} // namespace

std::string PoseGraph::Config::to_yaml() const {
  return std::format("max_iterations: {}\nclosure_info_scale: {}\nrotation_info_scale: {}\n"
                     "closure_kernel_delta: {}\n",
                     max_iterations, closure_info_scale, rotation_info_scale, closure_kernel_delta);
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

Sophus::SE3d PoseGraph::get_keypose(const KeyposeId keypose_id) const {
  const auto* vertex = dynamic_cast<const g2o::VertexSE3*>(optimizer->vertex(static_cast<int>(keypose_id)));
  if (vertex == nullptr) {
    throw std::runtime_error(std::format("PoseGraph::get_keypose: no VertexSE3 with id {}", keypose_id));
  }
  return from_isometry(vertex->estimate());
}

void PoseGraph::add_odom_edge(const KeyposeId from_id, const KeyposeId to_id, const Sophus::SE3d& from_T_to) {
  // NOLINTNEXTLINE(cppcoreguidelines-owning-memory) g2o takes ownership of the raw pointer
  auto* edge = new g2o::EdgeSE3();
  edge->setVertex(0, optimizer->vertex(static_cast<int>(from_id)));
  edge->setVertex(1, optimizer->vertex(static_cast<int>(to_id)));
  edge->setMeasurement(to_isometry(from_T_to));
  edge->setInformation(info_for(config, 1.0));
  optimizer->addEdge(edge);
}

void PoseGraph::add_closure_edge(const KeyposeId from_id, const KeyposeId to_id, const Sophus::SE3d& from_T_to) {
  // NOLINTNEXTLINE(cppcoreguidelines-owning-memory) g2o takes ownership of the raw pointer
  auto* edge = new g2o::EdgeSE3();
  edge->setVertex(0, optimizer->vertex(static_cast<int>(from_id)));
  edge->setVertex(1, optimizer->vertex(static_cast<int>(to_id)));
  edge->setMeasurement(to_isometry(from_T_to));
  edge->setInformation(info_for(config, config.closure_info_scale));

  // NOLINTNEXTLINE(cppcoreguidelines-owning-memory) g2o takes ownership of the raw pointer
  auto* kernel = new g2o::RobustKernelCauchy();
  kernel->setDelta(config.closure_kernel_delta);
  edge->setRobustKernel(kernel);

  optimizer->addEdge(edge);
}

void PoseGraph::remove_closure_edge(const KeyposeId from_id, const KeyposeId to_id) {
  if (!is_closure_pair(from_id, to_id)) {
    return;
  }
  const auto match = std::ranges::find_if(optimizer->edges(), [&](const auto* edge) {
    return std::cmp_equal(edge->vertex(0)->id(), from_id) && std::cmp_equal(edge->vertex(1)->id(), to_id);
  });
  if (match != optimizer->edges().end()) {
    optimizer->removeEdge(*match);
  }
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
  return static_cast<std::size_t>(
      std::ranges::count_if(edges(), [](const EdgeView& edge) { return is_closure_pair(edge.from_id, edge.to_id); }));
}

std::vector<PoseGraph::EdgeView> PoseGraph::edges() const {
  std::vector<EdgeView> out;
  out.reserve(optimizer->edges().size());
  for (auto* edge : optimizer->edges()) {
    const g2o::EdgeSE3& edge_se3 = as_edge_se3(*edge);
    out.push_back({
        .from_id = static_cast<KeyposeId>(edge_se3.vertex(0)->id()),
        .to_id = static_cast<KeyposeId>(edge_se3.vertex(1)->id()),
        .from_T_to = from_isometry(edge_se3.measurement()),
    });
  }
  return out;
}

double PoseGraph::edge_chi2(const KeyposeId from_id, const KeyposeId to_id) {
  const auto match = std::ranges::find_if(optimizer->edges(), [&](const auto* edge) {
    return std::cmp_equal(edge->vertex(0)->id(), from_id) && std::cmp_equal(edge->vertex(1)->id(), to_id);
  });
  if (match == optimizer->edges().end()) {
    throw std::runtime_error(std::format("PoseGraph::edge_chi2: no edge from {} to {}", from_id, to_id));
  }
  g2o::EdgeSE3& edge_se3 = as_edge_se3(**match);
  edge_se3.computeError();
  return edge_se3.chi2();
}

} // namespace rko_slam::core
