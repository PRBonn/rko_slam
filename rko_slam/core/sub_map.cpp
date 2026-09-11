#include "rko_slam/core/sub_map.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include "rko_slam/core/voxel_hash_map.hpp"

namespace rko_slam::core {

void fill_sub_map(const VoxelHashMap& map, SubMap& sub_map) {
  constexpr unsigned int kMinPointsForCovariance = 3;
  sub_map.centroids.reserve(map.voxels.size());
  sub_map.normals.reserve(map.voxels.size());

  for (const auto& [_, block] : map.voxels) {
    if (block.size() < kMinPointsForCovariance) {
      continue;
    }
    const auto point_count = static_cast<float>(block.size());
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (const auto& point : block) {
      mean += point;
    }
    mean /= point_count;
    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    for (const auto& point : block) {
      const Eigen::Vector3f deviation = point - mean;
      covariance += deviation * deviation.transpose();
    }
    covariance /= (point_count - 1.0F);
    sub_map.centroids.emplace_back(mean);
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance, Eigen::ComputeEigenvectors);
    sub_map.normals.emplace_back(solver.eigenvectors().col(0));
  }
}

} // namespace rko_slam::core
