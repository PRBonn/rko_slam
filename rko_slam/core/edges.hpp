#pragma once

#include <iosfwd>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <g2o/core/base_unary_edge.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/edge_se3_prior.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <sophus/se3.hpp>

namespace rko_slam::core {

inline Sophus::SE3d from_isometry(const Eigen::Isometry3d& iso) {
  return {Eigen::Quaterniond(iso.linear()), iso.translation()};
}

inline Eigen::Isometry3d to_isometry(const Sophus::SE3d& pose) { return Eigen::Isometry3d(pose.matrix()); }

class EdgeGravity : public g2o::BaseUnaryEdge<2, Eigen::Vector3d, g2o::VertexSE3> {
public:
  EdgeGravity() : EdgeGravity(Eigen::Vector3d::UnitZ()) {}
  explicit EdgeGravity(const Eigen::Vector3d& measured_up);

  void setMeasurement(const Eigen::Vector3d& measured_up) override;
  void computeError() override;
  void linearizeOplus() override;
  bool read(std::istream& input) override;
  bool write(std::ostream& output) const override;

private:
  Eigen::Matrix<double, 2, 3> tangent_basis_transpose;
};

std::unique_ptr<g2o::EdgeSE3> make_odom_edge(const Sophus::SE3d& from_T_to, const double rotation_info_scale);
std::unique_ptr<g2o::EdgeSE3> make_closure_edge(const Sophus::SE3d& from_T_to,
                                                const double rotation_info_scale,
                                                const double closure_info_scale,
                                                const double kernel_delta);
std::unique_ptr<EdgeGravity> make_gravity_edge(const Eigen::Vector3d& measured_up, const double gravity_info_scale);
// Pins a keypose to `anchor` in position and yaw only, leaving roll and pitch to the gravity edges.
// `offset_parameter_id` is the ParameterSE3Offset the optimizer must carry for this edge to resolve its cache.
std::unique_ptr<g2o::EdgeSE3Prior> make_gauge_edge(const Sophus::SE3d& anchor, const int offset_parameter_id);

} // namespace rko_slam::core
