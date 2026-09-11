#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "rko_slam/core/pose_graph.hpp"

using namespace rko_slam::core;
using Catch::Matchers::WithinRel;

namespace {

// chi2 of a single closure whose measurement disagrees with the (identity)
// vertex placement by `err`, under a given rotation_info_scale.
double closure_chi2(const Sophus::SE3d& err, double s_r) {
  PoseGraph::Config config;
  config.rotation_info_scale = s_r;
  PoseGraph pose_graph(config);
  // Ids 0 and 2, never 0 and 1: adjacent keyposes are the odom chain, which is why the detector is
  // required to skip at least one sub-map, and what tells a closure from odom in a saved graph.
  pose_graph.add_keypose(0, Sophus::SE3d{});
  pose_graph.add_keypose(1, Sophus::SE3d{});
  pose_graph.add_keypose(2, Sophus::SE3d{});
  pose_graph.set_keypose_fixed(0, true);
  pose_graph.add_closure_edge(0, 2, err);
  return pose_graph.edge_chi2(0, 2);
}
} // namespace

TEST_CASE("info_scale: rotation_info_scale=1 reproduces identity weighting", "[info_scale]") {
  // Pure translation error of 1 m -> chi2 = 1 regardless of s_r.
  for (const double s_r : {1.0, 10.0, 100.0}) {
    REQUIRE_THAT(closure_chi2(Sophus::SE3d::trans(1.0, 0, 0), s_r), WithinRel(1.0, 1e-9));
  }
}

TEST_CASE("info_scale: rotation-residual chi2 scales linearly with s_r", "[info_scale]") {
  // Linear scaling proves s_r weights the rotation block (diagonal 3-5) and only that block.
  const double base = closure_chi2(Sophus::SE3d::rotZ(0.1), 1.0);
  REQUIRE(base > 0);
  REQUIRE_THAT(closure_chi2(Sophus::SE3d::rotZ(0.1), 10.0), WithinRel(10.0 * base, 1e-6));
  REQUIRE_THAT(closure_chi2(Sophus::SE3d::rotZ(0.1), 100.0), WithinRel(100.0 * base, 1e-6));
}
