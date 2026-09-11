#include <catch2/catch_test_macros.hpp>
#include <vector>

#include "rko_slam/core/voxel_hash_map.hpp"

using rko_slam::core::VoxelHashMap;

TEST_CASE("voxel_hash_map: add_points near-duplicate dedup within a voxel", "[voxel_hash_map]") {
  // map_resolution_sq = voxel_size^2 / cap = 0.25: two points 0.1 m apart share
  // a voxel and sit inside the dedup radius, so the second is rejected.
  VoxelHashMap map({.voxel_size = 1.0F, .max_points_per_voxel = 4});
  map.add_points(std::vector<Eigen::Vector3f>{{0.10F, 0.10F, 0.10F}, {0.20F, 0.10F, 0.10F}});
  REQUIRE(map.points().size() == 1);

  // A point 0.6 m away (sq 0.36 > 0.25) but still in the SAME voxel survives.
  map.add_points(std::vector<Eigen::Vector3f>{{0.70F, 0.10F, 0.10F}});
  REQUIRE(map.points().size() == 2);
  REQUIRE(map.voxels.size() == 1);
}

TEST_CASE("voxel_hash_map: max_points_per_voxel cap enforced", "[voxel_hash_map]") {
  // The 8 corners of a 5 m cube share one 10 m voxel and are pairwise beyond
  // the 4.47 m dedup radius, so all 8 are distinct candidates - cap admits 5.
  const unsigned int cap = 5;
  VoxelHashMap map({.voxel_size = 10.0F, .max_points_per_voxel = cap});
  std::vector<Eigen::Vector3f> points;
  for (int grid_x = 0; grid_x <= 5; grid_x += 5) {
    for (int grid_y = 0; grid_y <= 5; grid_y += 5) {
      for (int grid_z = 0; grid_z <= 5; grid_z += 5) {
        points.emplace_back(static_cast<float>(grid_x), static_cast<float>(grid_y), static_cast<float>(grid_z));
      }
    }
  }
  REQUIRE(points.size() == 8);
  map.add_points(points);
  REQUIRE(map.voxels.size() == 1);
  REQUIRE(map.points().size() == cap);
  REQUIRE(map.voxels.begin()->second.size() == cap);

  // Adding another distinct point after the cap is reached is a no-op.
  map.add_points(std::vector<Eigen::Vector3f>{{1.0F, 9.0F, 1.0F}});
  REQUIRE(map.points().size() == cap);
}
