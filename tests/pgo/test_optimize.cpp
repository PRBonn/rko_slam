#include <Eigen/Core>
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <limits>
#include <numbers>
#include <sophus/se3.hpp>
#include <vector>

#include "rko_slam/pgo/pose_graph.hpp"

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using Config = rko_slam::pgo::PoseGraph::Config;
using rko_slam::pgo::PoseEdge;
using rko_slam::pgo::PoseGraph;

namespace {

constexpr double kGravity = 9.81;

constexpr std::size_t kLoopSize = 12;

// A 20 m loop that climbs and rolls, so all six tangent directions carry error.
std::vector<Sophus::SE3d> loop_keyposes() {
  std::vector<Sophus::SE3d> keyposes;
  for (std::size_t keypose_id = 0; keypose_id < kLoopSize; ++keypose_id) {
    const double angle = 2.0 * std::numbers::pi * static_cast<double>(keypose_id) / static_cast<double>(kLoopSize);
    const Sophus::SO3d heading =
        Sophus::SO3d::rotZ(angle + (std::numbers::pi / 2.0)) * Sophus::SO3d::rotX(0.1 * std::sin(3.0 * angle));
    keyposes.emplace_back(heading,
                          Eigen::Vector3d(20.0 * std::cos(angle), 20.0 * std::sin(angle), std::sin(2.0 * angle)));
  }
  return keyposes;
}

void add_keyposes(PoseGraph& pose_graph, const std::vector<Sophus::SE3d>& keyposes) {
  for (const Sophus::SE3d& keypose : keyposes) {
    pose_graph.add_keypose(keypose);
  }
}

void add_loop_edges(PoseGraph& pose_graph, const std::vector<Sophus::SE3d>& keyposes) {
  const auto measure = [&keyposes, &pose_graph](const std::size_t from_id, const std::size_t to_id,
                                                const PoseEdge::Kind kind) {
    const Sophus::SE3d from_T_to = keyposes.at(from_id).inverse() * keyposes.at(to_id);
    if (kind == PoseEdge::Kind::closure) {
      pose_graph.add_closure_edge(from_id, to_id, from_T_to);
    } else {
      pose_graph.add_odometry_edge(from_id, to_id, from_T_to);
    }
  };
  for (std::size_t from_id = 0; from_id + 1 < kLoopSize; ++from_id) {
    measure(from_id, from_id + 1, PoseEdge::Kind::odometry);
  }
  measure(11, 0, PoseEdge::Kind::closure);
  measure(0, 4, PoseEdge::Kind::closure);
  measure(4, 8, PoseEdge::Kind::closure);
  measure(8, 0, PoseEdge::Kind::closure);
}

std::vector<Sophus::SE3d> perturbed(const std::vector<Sophus::SE3d>& keyposes) {
  std::vector<Sophus::SE3d> moved;
  for (std::size_t keypose_id = 0; keypose_id < keyposes.size(); ++keypose_id) {
    const auto phase = static_cast<double>(keypose_id);
    const Sophus::Vector6d nudge =
        (Sophus::Vector6d() << 0.5 * std::sin(phase), 0.4 * std::cos(phase), 0.3 * std::sin(2.0 * phase),
         0.1 * std::cos(3.0 * phase), 0.1 * std::sin(5.0 * phase), 0.15 * std::cos(phase))
            .finished();
    moved.push_back(keyposes.at(keypose_id) * Sophus::SE3d::exp(nudge));
  }
  return moved;
}

struct PoseError {
  double translation = 0.0;
  double rotation = 0.0;
};

PoseError worst_error(const PoseGraph& pose_graph, const std::vector<Sophus::SE3d>& truth) {
  const std::size_t anchor = pose_graph.anchor.keypose_id;
  const Sophus::SE3d graph_T_truth = pose_graph.keyposes.at(anchor) * truth.at(anchor).inverse();
  PoseError worst;
  for (std::size_t keypose_id = 0; keypose_id < truth.size(); ++keypose_id) {
    const Sophus::SE3d difference =
        (graph_T_truth * truth.at(keypose_id)).inverse() * pose_graph.keyposes.at(keypose_id);
    worst.translation = std::max(worst.translation, difference.translation().norm());
    worst.rotation = std::max(worst.rotation, difference.so3().log().norm());
  }
  return worst;
}

} // namespace

TEST_CASE("pgo: a perturbed loop converges to its consistent solution", "[pgo]") {
  const std::vector<Sophus::SE3d> truth = loop_keyposes();
  PoseGraph pose_graph;
  add_keyposes(pose_graph, perturbed(truth));
  add_loop_edges(pose_graph, truth);
  pose_graph.anchor_at(0);
  REQUIRE(pose_graph.optimize() != PoseGraph::Outcome::failed);
  const PoseError worst = worst_error(pose_graph, truth);
  REQUIRE(worst.translation < 1e-4);
  REQUIRE(worst.rotation < 1e-5);
}

TEST_CASE("pgo: the anchored keypose stays where it was", "[pgo]") {
  const std::vector<Sophus::SE3d> truth = loop_keyposes();
  PoseGraph pose_graph;
  add_keyposes(pose_graph, perturbed(truth));
  add_loop_edges(pose_graph, truth);
  pose_graph.anchor_at(5);
  const Sophus::SE3d anchored = pose_graph.keyposes.at(5);
  REQUIRE(pose_graph.optimize() != PoseGraph::Outcome::failed);
  const Sophus::SE3d moved = anchored.inverse() * pose_graph.keyposes.at(5);
  REQUIRE(moved.translation().norm() < 1e-6);
  REQUIRE(moved.so3().log().norm() < 1e-6);
  const PoseError worst = worst_error(pose_graph, truth);
  REQUIRE(worst.translation < 1e-4);
  REQUIRE(worst.rotation < 1e-5);
}

TEST_CASE("pgo: the Cauchy kernel down-weights one inconsistent closure", "[pgo]") {
  const std::vector<Sophus::SE3d> truth = loop_keyposes();
  PoseGraph pose_graph;
  add_keyposes(pose_graph, truth);
  add_loop_edges(pose_graph, truth);
  pose_graph.anchor_at(0);
  pose_graph.add_closure_edge(2, 7, truth.at(2).inverse() * truth.at(7) * Sophus::SE3d::trans(10.0, 0.0, 0.0));

  PoseGraph robust = pose_graph;
  REQUIRE(robust.optimize() != PoseGraph::Outcome::failed);
  PoseGraph almost_quadratic = pose_graph;
  almost_quadratic.config = Config{.closure_kernel_delta = 1e4};
  REQUIRE(almost_quadratic.optimize() != PoseGraph::Outcome::failed);

  REQUIRE(worst_error(robust, truth).translation < 0.5);
  REQUIRE(worst_error(almost_quadratic, truth).translation > 1.0);
}

TEST_CASE("pgo: a 20 km loop started 600 m off by heading drift converges to its consistent solution", "[pgo]") {
  constexpr std::size_t kLinks = 400;
  std::vector<Sophus::SE3d> truth{Sophus::SE3d{}};
  for (std::size_t link = 0; link < kLinks; ++link) {
    const double phase = 2.0 * std::numbers::pi * static_cast<double>(link) / static_cast<double>(kLinks);
    const Eigen::Vector3d turn(0.0, 0.01 * std::sin(3.0 * phase), 2.0 * std::numbers::pi / static_cast<double>(kLinks));
    truth.push_back(truth.back() *
                    Sophus::SE3d(Sophus::SO3d::exp(turn), Eigen::Vector3d(50.0, 0.0, 0.5 * std::cos(phase))));
  }
  PoseGraph pose_graph(Config{.max_iterations = 100});
  pose_graph.add_keypose(Sophus::SE3d{});
  pose_graph.anchor_at(0);
  const auto measure = [&truth, &pose_graph](const std::size_t from_id, const std::size_t to_id,
                                             const PoseEdge::Kind kind) {
    const Sophus::SE3d from_T_to = truth.at(from_id).inverse() * truth.at(to_id);
    if (kind == PoseEdge::Kind::closure) {
      pose_graph.add_closure_edge(from_id, to_id, from_T_to);
    } else {
      pose_graph.add_odometry_edge(from_id, to_id, from_T_to);
    }
  };
  const Sophus::SE3d heading_drift = Sophus::SE3d::rotZ(5e-4);
  for (std::size_t link = 0; link < kLinks; ++link) {
    measure(link, link + 1, PoseEdge::Kind::odometry);
    pose_graph.add_keypose(pose_graph.keyposes.back() * pose_graph.pose_edges.back().from_T_to * heading_drift);
  }
  measure(0, kLinks, PoseEdge::Kind::closure);

  REQUIRE(pose_graph.optimize() != PoseGraph::Outcome::failed);
  for (std::size_t keypose_id = 0; keypose_id <= kLinks; ++keypose_id) {
    CAPTURE(keypose_id);
    const Sophus::SE3d difference = truth.at(keypose_id).inverse() * pose_graph.keyposes.at(keypose_id);
    REQUIRE(difference.translation().norm() < 1e-2);
    REQUIRE(difference.so3().log().norm() < 1e-4);
  }
}

TEST_CASE("pgo: rotation_info_scale weights rotation against translation", "[pgo]") {
  // A closure 0.2 m to the side of a straight chain: the middle keypose turns less, in proportion, as rotation
  // stiffens.
  const auto optimized_heading = [](const double rotation_info_scale) {
    PoseGraph pose_graph(
        Config{.max_iterations = 100, .rotation_info_scale = rotation_info_scale, .closure_kernel_delta = 1e4});
    add_keyposes(pose_graph, {Sophus::SE3d{}, Sophus::SE3d::trans(1.0, 0.0, 0.0), Sophus::SE3d::trans(2.0, 0.0, 0.0)});
    pose_graph.add_odometry_edge(0, 1, Sophus::SE3d::trans(1.0, 0.0, 0.0));
    pose_graph.add_odometry_edge(1, 2, Sophus::SE3d::trans(1.0, 0.0, 0.0));
    pose_graph.add_closure_edge(0, 2, Sophus::SE3d::trans(2.0, 0.2, 0.0));
    pose_graph.anchor_at(0);
    REQUIRE(pose_graph.optimize() != PoseGraph::Outcome::failed);
    return pose_graph.keyposes.at(1).so3().log().z();
  };
  const double heading = optimized_heading(100.0);
  REQUIRE(heading > 0.0);
  REQUIRE_THAT(optimized_heading(400.0), WithinRel(heading / 4.0, 0.01));
}

TEST_CASE("pgo: a graph whose cost is not finite is left as it was", "[pgo]") {
  const std::vector<Sophus::SE3d> truth = loop_keyposes();
  PoseGraph pose_graph;
  add_keyposes(pose_graph, perturbed(truth));
  add_loop_edges(pose_graph, truth);
  pose_graph.anchor_at(0);
  pose_graph.pose_edges.back().from_T_to.translation().x() = std::numeric_limits<double>::quiet_NaN();
  const std::vector<Sophus::SE3d> before = pose_graph.keyposes;
  REQUIRE(pose_graph.optimize() == PoseGraph::Outcome::failed);
  for (std::size_t keypose_id = 0; keypose_id < before.size(); ++keypose_id) {
    CAPTURE(keypose_id);
    REQUIRE(pose_graph.keyposes.at(keypose_id).params() == before.at(keypose_id).params());
  }
}

TEST_CASE("pgo: a keypose placed by its odometry leaves a converged graph where it was", "[pgo]") {
  const std::vector<Sophus::SE3d> truth = loop_keyposes();
  PoseGraph pose_graph(Config{.max_iterations = 100});
  add_keyposes(pose_graph, perturbed(truth));
  add_loop_edges(pose_graph, truth);
  pose_graph.anchor_at(0);
  REQUIRE(pose_graph.optimize() != PoseGraph::Outcome::failed);
  const std::vector<Sophus::SE3d> converged = pose_graph.keyposes;

  const std::size_t last = pose_graph.keyposes.size() - 1;
  const Sophus::SE3d odometry = Sophus::SE3d::trans(5.0, 0.5, 0.0) * Sophus::SE3d::rotZ(0.1);
  const std::size_t placed = pose_graph.add_keypose(pose_graph.keyposes.back() * odometry);
  pose_graph.add_odometry_edge(last, placed, odometry);
  REQUIRE(pose_graph.optimize() != PoseGraph::Outcome::failed);
  for (std::size_t keypose_id = 0; keypose_id <= last; ++keypose_id) {
    CAPTURE(keypose_id);
    REQUIRE((converged.at(keypose_id).translation() - pose_graph.keyposes.at(keypose_id).translation()).norm() < 1e-5);
  }
}

TEST_CASE("pgo: gravity levels a map started tilted, keeping the anchor's position and heading", "[pgo]") {
  constexpr std::size_t kKeyposes = 5;
  constexpr double kHeading = 0.3;
  const Sophus::SE3d start(Sophus::SO3d::rotZ(kHeading) * Sophus::SO3d::rotX(0.09) * Sophus::SO3d::rotY(-0.05),
                           Eigen::Vector3d(4.0, -2.0, 1.0));
  const auto level_step = [](const std::size_t keypose_id) {
    return Sophus::SE3d(Sophus::SO3d::rotZ(0.1 * static_cast<double>(keypose_id)), Eigen::Vector3d(10.0, 0.0, 0.0));
  };

  PoseGraph pose_graph(Config{.max_iterations = 50});
  pose_graph.add_keypose(start);
  pose_graph.anchor_at(0);
  for (std::size_t keypose_id = 0; keypose_id < kKeyposes; ++keypose_id) {
    pose_graph.add_gravity_edge(keypose_id, kGravity * Eigen::Vector3d::UnitZ());
    if (keypose_id + 1 < kKeyposes) {
      const std::size_t next_id = pose_graph.add_keypose(pose_graph.keyposes.back() * level_step(keypose_id));
      pose_graph.add_odometry_edge(keypose_id, next_id, level_step(keypose_id));
    }
  }
  REQUIRE(pose_graph.optimize() != PoseGraph::Outcome::failed);

  const Sophus::SE3d& first = pose_graph.keyposes.front();
  CHECK((first.translation() - start.translation()).norm() < 1e-6);
  // a heading under two tilts is defined up to their product
  const Eigen::Vector3d forward = first.so3() * Eigen::Vector3d::UnitX();
  CHECK_THAT(std::atan2(forward.y(), forward.x()), WithinAbs(kHeading, 0.09 * 0.05));
  Sophus::SE3d expected = first;
  for (std::size_t keypose_id = 0; keypose_id < kKeyposes; ++keypose_id) {
    const Sophus::SE3d& keypose = pose_graph.keyposes.at(keypose_id);
    CAPTURE(keypose_id);
    CHECK((keypose.so3() * Eigen::Vector3d::UnitZ()).z() > std::cos(1e-6));
    CHECK((keypose.inverse() * expected).log().norm() < 1e-6);
    expected = expected * level_step(keypose_id);
  }
}

TEST_CASE("pgo: the anchor holds position and heading, and leaves roll and pitch to gravity", "[pgo]") {
  const Eigen::Vector3d translation(4.0, -2.0, 1.0);
  const auto heading = [](const Sophus::SE3d& pose) {
    const Eigen::Vector3d forward = pose.so3() * Eigen::Vector3d::UnitX();
    return std::atan2(forward.y(), forward.x());
  };

  // an anchor with no tilt is the case where its yaw axis coincides with the map's and a wrong projection still looks
  // right; the tilted one separates them
  const Sophus::SO3d level = Sophus::SO3d::rotZ(0.3);
  const Sophus::SO3d tilted = level * Sophus::SO3d::rotX(0.09) * Sophus::SO3d::rotY(-0.05);
  for (const Sophus::SO3d& rotation : {level, tilted}) {
    const Sophus::SE3d start(rotation, translation);
    const Eigen::Vector3d measured_up =
        kGravity * (Sophus::SO3d::rotX(0.05) * Sophus::SO3d::rotY(-0.04) * Eigen::Vector3d::UnitZ());
    PoseGraph pose_graph(Config{.max_iterations = 50});
    pose_graph.add_keypose(start);
    pose_graph.add_gravity_edge(0, measured_up);
    pose_graph.anchor_at(0);
    REQUIRE(pose_graph.optimize() != PoseGraph::Outcome::failed);

    const Sophus::SE3d& solved = pose_graph.keyposes.front();
    CAPTURE(heading(start));
    CHECK((solved.translation() - translation).norm() < 1e-6);
    CHECK((solved.so3().inverse() * Eigen::Vector3d::UnitZ() - measured_up.normalized()).norm() < 1e-4);
    CHECK_THAT(heading(solved), WithinAbs(heading(start), 0.09 * 0.05));
  }
}

TEST_CASE("pgo: with no gravity the anchor holds roll and pitch too", "[pgo]") {
  const std::vector<Sophus::SE3d> truth = loop_keyposes();
  PoseGraph tilted(Config{.max_iterations = 50});
  add_keyposes(tilted, perturbed(truth));
  add_loop_edges(tilted, truth);
  tilted.anchor_at(0);
  const Sophus::SE3d start = tilted.keyposes.front();
  REQUIRE(tilted.optimize() != PoseGraph::Outcome::failed);
  CHECK((start.inverse() * tilted.keyposes.front()).log().norm() < 1e-6);
}

TEST_CASE("pgo: rotation_info_scale leaves a pure translation error alone", "[pgo]") {
  // the chain is straight and the closure says the same, 1 m short: rotation carries none of that error, so no
  // weighting of it changes where the keyposes land
  const auto optimized_middle = [](const double rotation_info_scale) {
    PoseGraph pose_graph(Config{.max_iterations = 100, .rotation_info_scale = rotation_info_scale});
    add_keyposes(pose_graph, {Sophus::SE3d{}, Sophus::SE3d::trans(1.0, 0.0, 0.0), Sophus::SE3d::trans(2.0, 0.0, 0.0)});
    pose_graph.add_odometry_edge(0, 1, Sophus::SE3d::trans(1.0, 0.0, 0.0));
    pose_graph.add_odometry_edge(1, 2, Sophus::SE3d::trans(1.0, 0.0, 0.0));
    pose_graph.add_closure_edge(0, 2, Sophus::SE3d::trans(1.0, 0.0, 0.0));
    pose_graph.anchor_at(0);
    REQUIRE(pose_graph.optimize() != PoseGraph::Outcome::failed);
    return pose_graph.keyposes.at(1).translation().x();
  };
  const double middle = optimized_middle(1.0);
  CHECK_THAT(optimized_middle(25.0), WithinRel(middle, 1e-9));
  CHECK_THAT(optimized_middle(400.0), WithinRel(middle, 1e-9));
}
