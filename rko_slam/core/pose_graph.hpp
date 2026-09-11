#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <sophus/se3.hpp>
#include <string>
#include <vector>

#include "rko_slam/core/types.hpp"

namespace g2o {
class SparseOptimizer;
class SparseOptimizerTerminateAction;
} // namespace g2o

namespace rko_slam::core {

constexpr bool is_closure_pair(const KeyposeId from_id, const KeyposeId to_id) {
  return from_id + 1 != to_id && to_id + 1 != from_id;
}

class PoseGraph {
public:
  struct Config {
    int max_iterations = 10;
    // Closure edge type-scale; odom's is 1.
    double closure_info_scale = 1.0;
    // Rotation weight against translation 1: s_r = R^2, R the range where both errors cost the same (100 -> 10 m).
    double rotation_info_scale = 100.0;
    // Cauchy delta on closure edges, in metres; odom edges stay quadratic.
    double closure_kernel_delta = 1.0;

    std::string to_yaml() const;
  };

  explicit PoseGraph(const Config pose_graph_config);
  ~PoseGraph();

  PoseGraph(const PoseGraph&) = delete;
  PoseGraph& operator=(const PoseGraph&) = delete;
  PoseGraph(PoseGraph&&) = delete;
  PoseGraph& operator=(PoseGraph&&) = delete;

  void add_keypose(const KeyposeId keypose_id, const Sophus::SE3d& pose);

  void set_keypose_fixed(const KeyposeId keypose_id, const bool fixed);

  Sophus::SE3d get_keypose(const KeyposeId keypose_id) const;

  // Information = diag(1,1,1,s_r,s_r,s_r)
  void add_odom_edge(const KeyposeId from_id, const KeyposeId to_id, const Sophus::SE3d& from_T_to);

  // Information = closure_info_scale * diag(1,1,1,s_r,s_r,s_r).
  void add_closure_edge(const KeyposeId from_id, const KeyposeId to_id, const Sophus::SE3d& from_T_to);

  void remove_closure_edge(const KeyposeId from_id, const KeyposeId to_id);

  // Returns false if g2o throws or the graph has no edges.
  bool optimize();

  // g2o text format: VERTEX_SE3:QUAT + EDGE_SE3:QUAT + FIX 0.
  bool save(const std::filesystem::path& path) const;

  bool load(const std::filesystem::path& path);

  std::size_t num_keyposes() const;
  std::size_t num_closure_edges() const;

  struct EdgeView {
    KeyposeId from_id;
    KeyposeId to_id;
    Sophus::SE3d from_T_to;
  };
  std::vector<EdgeView> edges() const;

  // Chi2 at the current vertex configuration.
  double edge_chi2(const KeyposeId from_id, const KeyposeId to_id);

private:
  Config config;
  std::unique_ptr<g2o::SparseOptimizerTerminateAction> terminate_action;
  std::unique_ptr<g2o::SparseOptimizer> optimizer;
};

} // namespace rko_slam::core
