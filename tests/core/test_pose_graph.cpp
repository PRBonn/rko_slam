#include <catch2/catch_test_macros.hpp>

#include <sophus/se3.hpp>

#include "rko_slam/core/pose_graph.hpp"

using rko_slam::core::KeyposeId;
using rko_slam::core::PoseGraph;

TEST_CASE("pose_graph: a closure edge is removed by its id pair, the odom chain is not", "[pose_graph]") {
  PoseGraph pose_graph{PoseGraph::Config{}};
  for (KeyposeId keypose_id = 0; keypose_id < 3; ++keypose_id) {
    pose_graph.add_keypose(keypose_id, Sophus::SE3d{});
  }
  pose_graph.add_odom_edge(0, 1, Sophus::SE3d{});
  pose_graph.add_odom_edge(1, 2, Sophus::SE3d{});
  pose_graph.add_closure_edge(0, 2, Sophus::SE3d{});
  REQUIRE(pose_graph.edges().size() == 3);
  REQUIRE(pose_graph.num_closure_edges() == 1);

  pose_graph.remove_closure_edge(0, 1);
  pose_graph.remove_closure_edge(1, 2);
  pose_graph.remove_closure_edge(2, 0);
  REQUIRE(pose_graph.edges().size() == 3);

  pose_graph.remove_closure_edge(0, 2);
  REQUIRE(pose_graph.edges().size() == 2);
  REQUIRE(pose_graph.num_closure_edges() == 0);
}
