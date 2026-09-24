#include <cmath>
#include <cstddef>
#include <memory>
#include <numbers>
#include <optional>
#include <random>
#include <vector>

#include <Eigen/Core>
#include <catch2/catch_test_macros.hpp>
#include <sophus/se3.hpp>

#include "rko_slam/closures/detector.hpp"
#include "rko_slam/core/sub_map.hpp"
#include "rko_slam/core/voxel_hash_map.hpp"

using rko_slam::closures::ClosureCandidate;
using rko_slam::closures::ClosureDetector;
using rko_slam::closures::SubMapPoints;
using rko_slam::core::FinishedSubMap;

namespace {

constexpr float kSpacing = 0.2F;
// Dense enough for every ground voxel to clear fill_sub_map's per-voxel point minimum.
constexpr float kGroundSpacing = 0.125F;

std::vector<Eigen::Vector3f> synthetic_scene(const unsigned int seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> position(-30.0F, 30.0F);
  std::uniform_real_distribution<float> side(3.0F, 10.0F);
  std::uniform_real_distribution<float> height(2.0F, 6.0F);
  std::uniform_real_distribution<float> unit(0.0F, 1.0F);
  std::vector<Eigen::Vector3f> scene;
  for (int step_x = 0; step_x < 720; ++step_x) {
    for (int step_y = 0; step_y < 720; ++step_y) {
      scene.emplace_back(-45.0F + (static_cast<float>(step_x) * kGroundSpacing),
                         -45.0F + (static_cast<float>(step_y) * kGroundSpacing), 0.0F);
    }
  }
  for (int building = 0; building < 15; ++building) {
    const float corner_x = position(rng);
    const float corner_y = position(rng);
    const float width = side(rng);
    const float depth = side(rng);
    const auto n_levels = static_cast<int>(height(rng) / kSpacing);
    for (int level = 1; level <= n_levels; ++level) {
      const float wall_z = static_cast<float>(level) * kSpacing;
      for (int step = 0; step < static_cast<int>(width / kSpacing); ++step) {
        const float along = static_cast<float>(step) * kSpacing;
        scene.emplace_back(corner_x + along, corner_y, wall_z);
        scene.emplace_back(corner_x + along, corner_y + depth, wall_z);
      }
      for (int step = 0; step < static_cast<int>(depth / kSpacing); ++step) {
        const float along = static_cast<float>(step) * kSpacing;
        scene.emplace_back(corner_x, corner_y + along, wall_z);
        scene.emplace_back(corner_x + width, corner_y + along, wall_z);
      }
    }
  }
  for (int pole = 0; pole < 30; ++pole) {
    const float centre_x = position(rng);
    const float centre_y = position(rng);
    for (int level = 1; level <= 20; ++level) {
      for (int around = 0; around < 12; ++around) {
        const float angle = static_cast<float>(around) * std::numbers::pi_v<float> / 6.0F;
        scene.emplace_back(centre_x + (0.25F * std::cos(angle)), centre_y + (0.25F * std::sin(angle)),
                           static_cast<float>(level) * kSpacing);
      }
    }
  }
  for (int clutter = 0; clutter < 60; ++clutter) {
    const Eigen::Vector3f corner(position(rng), position(rng), 0.2F);
    const Eigen::Vector3f size(0.5F + unit(rng), 0.5F + unit(rng), 0.5F + (2.0F * unit(rng)));
    const auto n_points = static_cast<int>(50.0F + (300.0F * unit(rng)));
    for (int count = 0; count < n_points; ++count) {
      scene.emplace_back(corner + Eigen::Vector3f(unit(rng), unit(rng), unit(rng)).cwiseProduct(size));
    }
  }
  return scene;
}

std::vector<Eigen::Vector3f> with_ramp(std::vector<Eigen::Vector3f> scene) {
  const float slope = std::tan(10.0F * std::numbers::pi_v<float> / 180.0F);
  for (Eigen::Vector3f& point : scene) {
    const bool ground_under_ramp =
        point.z() == 0.0F && point.x() >= 5.0F && point.x() < 35.0F && std::abs(point.y()) < 8.0F;
    if (ground_under_ramp) {
      point.z() = (point.x() - 5.0F) * slope;
    }
  }
  return scene;
}

FinishedSubMap finished_sub_map(const std::size_t sub_map_id,
                                const std::vector<Eigen::Vector3f>& cloud,
                                const std::optional<Eigen::Vector3f>& measured_up = std::nullopt) {
  rko_slam::core::VoxelHashMap voxel_map(rko_slam::core::VoxelHashMap::Config{});
  voxel_map.add_points(cloud);
  auto sub_map = std::make_unique<rko_slam::core::SubMap>();
  sub_map->id = sub_map_id;
  sub_map->measured_up = measured_up;
  rko_slam::core::fill_sub_map(voxel_map, *sub_map);
  return {.sub_map = std::move(sub_map), .points = voxel_map.points()};
}

SubMapPoints points_of(const FinishedSubMap& finished) {
  return {.centroids = finished.sub_map->centroids, .normals = finished.sub_map->normals, .points = finished.points};
}

Eigen::Vector3f measured_up_in(const Sophus::SO3f& frame_R_world) {
  constexpr float kGravity = 9.81F;
  return kGravity * (frame_R_world * Eigen::Vector3f::UnitZ());
}

std::vector<Eigen::Vector3f> transformed(const Sophus::SE3f& new_T_old, std::vector<Eigen::Vector3f> points) {
  for (Eigen::Vector3f& point : points) {
    point = new_T_old * point;
  }
  return points;
}

std::vector<Eigen::Vector3f>
revisited(const std::vector<Eigen::Vector3f>& scene, const Sophus::SE3f& revisit_T_scene, const unsigned int seed) {
  std::mt19937 rng(seed);
  std::bernoulli_distribution seen(0.5);
  std::normal_distribution<float> noise(0.0F, 0.05F);
  std::vector<Eigen::Vector3f> points;
  for (const Eigen::Vector3f& point : scene) {
    if (seen(rng)) {
      points.emplace_back(revisit_T_scene * (point + Eigen::Vector3f(noise(rng), noise(rng), noise(rng))));
    }
  }
  return points;
}

Sophus::SE3f moved_T_scene() {
  return {Sophus::SO3f::rotZ(0.6F) * Sophus::SO3f::rotX(0.035F) * Sophus::SO3f::rotY(-0.026F),
          Eigen::Vector3f(6.0F, -4.0F, 0.3F)};
}

} // namespace

TEST_CASE("ClosureDetector: a moved copy of a sub-map closes with the motion as its pose", "[closures]") {
  // Only the copy has the ramp: a ground bias both maps shared would cancel in their relative pose.
  const std::vector<Eigen::Vector3f> scene = synthetic_scene(1);
  const FinishedSubMap original = finished_sub_map(7, scene);
  const FinishedSubMap unrelated = finished_sub_map(8, synthetic_scene(2));
  const FinishedSubMap moved = finished_sub_map(9, transformed(moved_T_scene(), with_ramp(scene)));

  ClosureDetector detector({.no_of_sub_maps_to_skip = 0});
  detector.query({.sub_map = original.sub_map->id}, std::nullopt, points_of(original));
  detector.query({.sub_map = unrelated.sub_map->id}, std::nullopt, points_of(unrelated));
  const std::optional<ClosureCandidate> closure =
      detector.query({.sub_map = moved.sub_map->id}, std::nullopt, points_of(moved));

  REQUIRE(closure);
  CHECK(closure->source.sub_map == 7);
  CHECK(closure->target.sub_map == 9);
  const Sophus::SE3f error = closure->target_T_source * moved_T_scene().inverse();
  const float rotation_error = error.so3().log().norm();
  const float translation_error = error.translation().norm();
  const float tilt_error = error.rotationMatrix().col(2).head<2>().norm();
  const float height_error = std::abs(error.translation().z());
  CAPTURE(closure->number_of_inliers, rotation_error, translation_error, tilt_error, height_error);
  CHECK(rotation_error < 0.01F);
  CHECK(translation_error < 0.1F);
  CHECK(tilt_error < 1e-3F);
  CHECK(height_error < 0.02F);
}

TEST_CASE("ClosureDetector: no_of_sub_maps_to_skip keeps the most recent sub-maps out of the search", "[closures]") {
  const std::vector<Eigen::Vector3f> scene = synthetic_scene(1);
  const FinishedSubMap original = finished_sub_map(0, scene);
  const FinishedSubMap moved = finished_sub_map(1, transformed(moved_T_scene(), with_ramp(scene)));

  ClosureDetector skipping({.no_of_sub_maps_to_skip = 1});
  skipping.query({.sub_map = 0}, std::nullopt, points_of(original));
  CHECK_FALSE(skipping.query({.sub_map = 1}, std::nullopt, points_of(moved)));

  ClosureDetector searching({.no_of_sub_maps_to_skip = 0});
  searching.query({.sub_map = 0}, std::nullopt, points_of(original));
  CHECK(searching.query({.sub_map = 1}, std::nullopt, points_of(moved)));
}

TEST_CASE("ClosureDetector: identical query sequences give identical candidates", "[closures]") {
  const std::vector<Eigen::Vector3f> scene = synthetic_scene(1);
  std::vector<FinishedSubMap> sub_maps;
  sub_maps.push_back(finished_sub_map(0, scene));
  sub_maps.push_back(finished_sub_map(1, revisited(scene, moved_T_scene(), 11)));
  sub_maps.push_back(finished_sub_map(2, synthetic_scene(2)));
  sub_maps.push_back(finished_sub_map(3, revisited(scene, moved_T_scene(), 12)));

  const auto detect_all = [&sub_maps] {
    ClosureDetector detector({
        .hamming_distance_threshold = 256, // distance filter off: only RANSAC's draws decide
        .no_of_sub_maps_to_skip = 0,
    });
    std::vector<ClosureCandidate> candidates;
    for (const FinishedSubMap& sub_map : sub_maps) {
      const std::vector<ClosureCandidate> found =
          detector.query_all({.sub_map = sub_map.sub_map->id}, std::nullopt, points_of(sub_map));
      candidates.insert(candidates.end(), found.begin(), found.end());
    }
    return candidates;
  };
  const std::vector<ClosureCandidate> first = detect_all();
  const std::vector<ClosureCandidate> second = detect_all();

  REQUIRE(!first.empty());
  REQUIRE(first.size() == second.size());
  for (std::size_t index = 0; index < first.size(); ++index) {
    CHECK(first[index].source == second[index].source);
    CHECK(first[index].target == second[index].target);
    CHECK(first[index].number_of_inliers == second[index].number_of_inliers);
    const bool same_pose = first[index].target_T_source.matrix() == second[index].target_T_source.matrix();
    CHECK(same_pose);
  }
}

TEST_CASE("ClosureDetector: the up direction levels a steeply tilted revisit", "[closures]") {
  const std::vector<Eigen::Vector3f> scene = synthetic_scene(1);
  const Sophus::SE3f tilted_T_scene(Sophus::SO3f::rotZ(-0.4F) * Sophus::SO3f::rotX(0.14F),
                                    Eigen::Vector3f(3.0F, 5.0F, -0.2F));
  const FinishedSubMap original = finished_sub_map(0, scene, measured_up_in({}));
  const FinishedSubMap tilted =
      finished_sub_map(1, transformed(tilted_T_scene, scene), measured_up_in(tilted_T_scene.so3()));

  const auto tilt_error_with = [&](const std::optional<Eigen::Vector3f>& tilted_up) -> std::optional<float> {
    ClosureDetector detector({.no_of_sub_maps_to_skip = 0});
    detector.query({.sub_map = 0}, original.sub_map->measured_up, points_of(original));
    const std::optional<ClosureCandidate> closure = detector.query({.sub_map = 1}, tilted_up, points_of(tilted));
    if (!closure) {
      return std::nullopt;
    }
    return (closure->target_T_source * tilted_T_scene.inverse()).rotationMatrix().col(2).head<2>().norm();
  };
  const std::optional<float> measured_up_tilt = tilt_error_with(tilted.sub_map->measured_up);
  REQUIRE(measured_up_tilt);
  CHECK(*measured_up_tilt < 1e-3F);
  const std::optional<float> own_axis_tilt = tilt_error_with(Eigen::Vector3f::UnitZ());
  CAPTURE(own_axis_tilt.value_or(-1.0F));
  CHECK((!own_axis_tilt || *own_axis_tilt > 0.1F));
}
