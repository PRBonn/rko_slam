#include "rko_slam/core/edges.hpp"

#include <istream>
#include <memory>
#include <ostream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <g2o/core/factory.h>
#include <g2o/core/io_helper.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

namespace rko_slam::core {

namespace {

Eigen::Vector3d up_in_keypose(const g2o::VertexSE3& vertex) { return vertex.estimate().linear().row(2).transpose(); }

// g2o increments a vertex by the vector part of a unit quaternion, half the rotation vector the Jacobian below is
// derived against.
constexpr double kRotationIncrementScale = 2.0;

constexpr double kGaugeInformation = 1e8;

// g2o EdgeSE3 orders its error [translation(0:2), rotation(3:5)].
g2o::EdgeSE3::InformationType se3_information(const double rotation_info_scale, const double type_scale) {
  g2o::EdgeSE3::InformationType info = g2o::EdgeSE3::InformationType::Identity();
  info(3, 3) = info(4, 4) = info(5, 5) = rotation_info_scale;
  return type_scale * info;
}

} // namespace

EdgeGravity::EdgeGravity(const Eigen::Vector3d& measured_up) { EdgeGravity::setMeasurement(measured_up); }

void EdgeGravity::setMeasurement(const Eigen::Vector3d& measured_up) {
  BaseUnaryEdge::setMeasurement(measured_up);
  const Eigen::Vector3d direction = measured_up.normalized();
  const Eigen::Vector3d first = direction.unitOrthogonal();
  tangent_basis_transpose.row(0) = first.transpose();
  tangent_basis_transpose.row(1) = direction.cross(first).transpose();
}

void EdgeGravity::computeError() { _error = tangent_basis_transpose * up_in_keypose(*vertexXn<0>()); }

void EdgeGravity::linearizeOplus() {
  _jacobianOplusXi.leftCols<3>().setZero();
  _jacobianOplusXi.rightCols<3>() =
      kRotationIncrementScale * tangent_basis_transpose * Sophus::SO3d::hat(up_in_keypose(*vertexXn<0>()));
}

bool EdgeGravity::read(std::istream& input) {
  Eigen::Vector3d measured_up;
  if (!g2o::internal::readVector(input, measured_up)) {
    return false;
  }
  setMeasurement(measured_up);
  return readInformationMatrix(input);
}

bool EdgeGravity::write(std::ostream& output) const {
  g2o::internal::writeVector(output, measurement());
  return writeInformationMatrix(output);
}

std::unique_ptr<g2o::EdgeSE3> make_odom_edge(const Sophus::SE3d& from_T_to, const double rotation_info_scale) {
  auto edge = std::make_unique<g2o::EdgeSE3>();
  edge->setMeasurement(to_isometry(from_T_to));
  edge->setInformation(se3_information(rotation_info_scale, 1.0));
  return edge;
}

std::unique_ptr<g2o::EdgeSE3> make_closure_edge(const Sophus::SE3d& from_T_to,
                                                const double rotation_info_scale,
                                                const double closure_info_scale,
                                                const double kernel_delta) {
  auto edge = std::make_unique<g2o::EdgeSE3>();
  edge->setMeasurement(to_isometry(from_T_to));
  edge->setInformation(se3_information(rotation_info_scale, closure_info_scale));

  auto kernel = std::make_unique<g2o::RobustKernelCauchy>();
  kernel->setDelta(kernel_delta);
  edge->setRobustKernel(kernel.release());
  return edge;
}

std::unique_ptr<EdgeGravity> make_gravity_edge(const Eigen::Vector3d& measured_up, const double gravity_info_scale) {
  auto edge = std::make_unique<EdgeGravity>(measured_up);
  edge->setInformation(gravity_info_scale * Eigen::Matrix2d::Identity());
  return edge;
}

std::unique_ptr<g2o::EdgeSE3Prior> make_gauge_edge(const Sophus::SE3d& anchor, const int offset_parameter_id) {
  const Eigen::Vector3d up_in_anchor = anchor.so3().inverse() * Eigen::Vector3d::UnitZ();
  Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
  information.topLeftCorner<3, 3>() = kGaugeInformation * Eigen::Matrix3d::Identity();
  // The rotation error is in the anchor frame, where the turn about `up_in_anchor` is yaw: weighting that one
  // direction holds yaw and leaves roll and pitch free for the gravity edges.
  information.bottomRightCorner<3, 3>() = kGaugeInformation * up_in_anchor * up_in_anchor.transpose();

  auto edge = std::make_unique<g2o::EdgeSE3Prior>();
  edge->setMeasurement(to_isometry(anchor));
  edge->setInformation(information);
  edge->setParameterId(0, offset_parameter_id);
  return edge;
}

G2O_REGISTER_TYPE(EDGE_GRAVITY, EdgeGravity)

} // namespace rko_slam::core
