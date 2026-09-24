#include <Eigen/Core>
#include <catch2/catch_test_macros.hpp>
#include <sophus/se3.hpp>

#include "rko_slam/closures/refinement.hpp"

namespace {

std::vector<Eigen::Vector3f>
grid_block(int count_x, int count_y, int count_z, const Eigen::Vector3f& origin, float step) {
  std::vector<Eigen::Vector3f> points;
  for (int i = 0; i < count_x; ++i) {
    for (int j = 0; j < count_y; ++j) {
      for (int k = 0; k < count_z; ++k) {
        points.emplace_back(origin + Eigen::Vector3f(static_cast<float>(i) * step, static_cast<float>(j) * step,
                                                     static_cast<float>(k) * step));
      }
    }
  }
  return points;
}

} // namespace

TEST_CASE("overlap coefficient: identical clouds -> 1", "[overlap_coefficient]") {
  const std::vector<Eigen::Vector3f> first_cloud = grid_block(8, 8, 8, Eigen::Vector3f::Zero(), 0.5F);
  const auto overlap = rko_slam::closures::voxel_overlap_coefficient(first_cloud, first_cloud, Sophus::SE3f{}, 0.5F);
  REQUIRE(overlap == 1.0);
}

TEST_CASE("overlap coefficient: disjoint clouds -> 0", "[overlap_coefficient]") {
  const std::vector<Eigen::Vector3f> first_cloud = grid_block(4, 4, 4, Eigen::Vector3f::Zero(), 0.5F);
  const std::vector<Eigen::Vector3f> second_cloud = grid_block(4, 4, 4, Eigen::Vector3f(100, 0, 0), 0.5F);
  const auto overlap = rko_slam::closures::voxel_overlap_coefficient(first_cloud, second_cloud, Sophus::SE3f{}, 0.5F);
  REQUIRE(overlap == 0.0);
}

TEST_CASE("overlap coefficient: raw inputs at finer-than-voxel spacing stay <= 1", "[overlap_coefficient]") {
  // A `|second| := second_pts.size()` denominator would give 1.5 here.
  const std::vector<Eigen::Vector3f> first_cloud = grid_block(4, 1, 1, Eigen::Vector3f(0.0F, 0.5F, 0.5F), 0.5F);
  const std::vector<Eigen::Vector3f> second_cloud = grid_block(4, 1, 1, Eigen::Vector3f(1.0F, 0.5F, 0.5F), 0.5F);
  const auto overlap = rko_slam::closures::voxel_overlap_coefficient(first_cloud, second_cloud, Sophus::SE3f{}, 1.0F);
  REQUIRE(overlap == 0.5);
}

TEST_CASE("overlap coefficient: half-overlapping clouds -> in [0.5, 0.55]", "[overlap_coefficient]") {
  // A spans voxels [0..3], B spans voxels [2..5]: intersection 2, min 4.
  const std::vector<Eigen::Vector3f> first_cloud = grid_block(4, 1, 1, Eigen::Vector3f(0.5F, 0.5F, 0.5F), 1.0F);
  const std::vector<Eigen::Vector3f> second_cloud = grid_block(4, 1, 1, Eigen::Vector3f(2.5F, 0.5F, 0.5F), 1.0F);
  const auto overlap = rko_slam::closures::voxel_overlap_coefficient(first_cloud, second_cloud, Sophus::SE3f{}, 1.0F);
  REQUIRE(overlap >= 0.5);
  REQUIRE(overlap <= 0.55);
}

TEST_CASE("overlap coefficient: first_T_second transforms second_pts into first's frame", "[overlap_coefficient]") {
  const std::vector<Eigen::Vector3f> first_cloud = grid_block(4, 4, 1, Eigen::Vector3f(0.5F, 0.5F, 0.5F), 1.0F);
  // 180 deg around z + integer translation - both exactly representable in float.
  const Sophus::SO3f rot(Eigen::Quaternionf(0.0F, 0.0F, 0.0F, 1.0F));
  const Sophus::SE3f first_T_second(rot, Eigen::Vector3f{10.0F, 20.0F, 0.0F});
  const Sophus::SE3f second_T_first = first_T_second.inverse();
  std::vector<Eigen::Vector3f> second_cloud;
  second_cloud.reserve(first_cloud.size());
  for (const auto& point : first_cloud) {
    second_cloud.emplace_back(second_T_first * point);
  }
  REQUIRE(rko_slam::closures::voxel_overlap_coefficient(first_cloud, second_cloud, first_T_second, 1.0F) == 1.0);
}

TEST_CASE("overlap coefficient: asymmetric sizes - denom is min(|F|, |S|)", "[overlap_coefficient]") {
  const std::vector<Eigen::Vector3f> first_cloud = grid_block(4, 1, 1, Eigen::Vector3f(0.5F, 0.5F, 0.5F), 1.0F);
  const std::vector<Eigen::Vector3f> second_cloud = grid_block(3, 1, 1, Eigen::Vector3f(2.5F, 0.5F, 0.5F), 1.0F);
  // |a| = 4, |b| = 3, intersection = {2, 3} -> 2 / min(4, 3) = 2/3.
  const auto overlap = rko_slam::closures::voxel_overlap_coefficient(first_cloud, second_cloud, Sophus::SE3f{}, 1.0F);
  REQUIRE(overlap > 0.66);
  REQUIRE(overlap < 0.67);
}
