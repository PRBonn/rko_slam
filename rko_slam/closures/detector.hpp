#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <sophus/se3.hpp>

namespace rko_slam::closures {

// An ORB descriptor of a density image cell.
using Descriptor = std::array<std::uint64_t, 4>;

struct Feature {
  // density image cell (x, y)
  Eigen::Vector2f cell = Eigen::Vector2f::Zero();
  Descriptor descriptor{};
};

struct SubMapIndex {
  std::size_t session = 0;
  std::size_t sub_map = 0;
  bool operator==(const SubMapIndex&) const = default;
};

struct SubMapFeatures {
  SubMapIndex index;
  Sophus::SE3f ground_T_sub_map;
  std::vector<Feature> features;
};

struct ClosureCandidate {
  SubMapIndex source;
  SubMapIndex target;
  Sophus::SE3f target_T_source;
  std::size_t number_of_inliers = 0;
};

// In the sub-map frame.
struct SubMapPoints {
  std::span<const Eigen::Vector3f> centroids;
  std::span<const Eigen::Vector3f> normals;
  std::span<const Eigen::Vector3f> points;
};

struct ClosureDetector {
  struct Config {
    float density_map_resolution = 0.5F;
    float density_threshold = 0.05F;
    int hamming_distance_threshold = 50;
    std::size_t inliers_threshold = 5;
    int no_of_sub_maps_to_skip = 3;

    // icp search distance fixed based on the map
    constexpr float correspondence_distance() const { return std::numbers::sqrt3_v<float> * density_map_resolution; }

    std::string to_yaml() const;
  };

  ClosureDetector() = default;
  explicit ClosureDetector(const Config& config) : config(config) {}

  Config config;
  // Every sub-map queried so far, in query order.
  std::vector<SubMapFeatures> database;

  // The top-inlier candidate with at least `config.inliers_threshold` inliers. Without a `measured_up`, the ground
  // under the sub-map sets the up direction.
  std::optional<ClosureCandidate>
  query(const SubMapIndex target, const std::optional<Eigen::Vector3f>& measured_up, const SubMapPoints sub_map);

  // Every candidate with its raw inlier count, unfiltered by inliers_threshold.
  std::vector<ClosureCandidate>
  query_all(const SubMapIndex target, const std::optional<Eigen::Vector3f>& measured_up, const SubMapPoints sub_map);
};

} // namespace rko_slam::closures
