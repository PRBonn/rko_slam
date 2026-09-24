#include "rko_slam/closures/detector.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <format>
#include <functional>
#include <iterator>
#include <limits>
#include <numeric>
#include <optional>
#include <random>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <UTL/profiler.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <rko_lio/core/voxel_down_sample.hpp> // brings rko_lio::core::VoxelHash
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>
#include <tsl/robin_map.h>

namespace rko_slam::closures {

namespace {

constexpr float kGroundNormalAgreement = 0.95F; // |n·dominant| above this is ground, so within 18 degrees of it
constexpr int kMaxGroundIterations = 10;
constexpr float kGroundConvergedStep = 1e-3F; // metres of height, radians of tilt
constexpr double kRansacInlierRatio = 0.1;
constexpr double kRansacSuccessProbability = 0.999;
// const because compilers
const int kRansacTrials = static_cast<int>(
    std::ceil(std::log(1.0 - kRansacSuccessProbability) / std::log(1.0 - (kRansacInlierRatio * kRansacInlierRatio))));

// The lowest point in each 1 m column, which is where the ground shows through.
tsl::robin_map<Eigen::Vector3i, std::size_t, rko_lio::core::VoxelHash>
lowest_per_column(const std::span<const Eigen::Vector3f> points) {
  tsl::robin_map<Eigen::Vector3i, std::size_t, rko_lio::core::VoxelHash> lowest;
  lowest.reserve(points.size());
  for (std::size_t index = 0; index < points.size(); ++index) {
    const Eigen::Vector3f& point = points[index];
    const Eigen::Vector3i column = rko_lio::core::point_to_voxel(Eigen::Vector3f(point.x(), point.y(), 0.0F), 1.0F);
    const auto [entry, inserted] = lowest.try_emplace(column, index);
    if (!inserted && point.z() < points[entry->second].z()) {
      entry.value() = index;
    }
  }

  return lowest;
}

// Levels on the ground the sub-map sits on: the dominant normal of the lowest voxel per column, then its plane.
Sophus::SE3f level_by_ground(const std::span<const Eigen::Vector3f> centroids,
                             const std::span<const Eigen::Vector3f> normals) {
  const auto lowest_voxel_per_column = lowest_per_column(centroids);
  if (lowest_voxel_per_column.size() < 2) {
    return {};
  }

  Eigen::Matrix3f normal_outer_products = Eigen::Matrix3f::Zero();
  for (const auto& [_, index] : lowest_voxel_per_column) {
    normal_outer_products += normals[index] * normals[index].transpose();
  }
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(normal_outer_products, Eigen::ComputeEigenvectors);
  Eigen::Vector3f ground_normal = solver.eigenvectors().col(2);
  if (ground_normal.z() < 0.0F) {
    ground_normal = -ground_normal;
  }

  std::vector<Eigen::Vector3f> ground;
  Eigen::Vector3f ground_mean = Eigen::Vector3f::Zero();
  for (const auto& [_, index] : lowest_voxel_per_column) {
    if (std::abs(normals[index].dot(ground_normal)) > kGroundNormalAgreement) {
      ground.push_back(centroids[index]);
      ground_mean += centroids[index];
    }
  }
  if (ground.empty()) {
    return {};
  }
  ground_mean /= static_cast<float>(ground.size());

  // A correction on the sub-map frame, not a pose in the map; the closure composes it back out at both ends.
  const Eigen::Quaternionf level = Eigen::Quaternionf::FromTwoVectors(ground_normal, Eigen::Vector3f::UnitZ());
  Sophus::SE3f ground_T_sub_map(level, Eigen::Vector3f(0.0F, 0.0F, -(level * ground_mean).z()));
  for (Eigen::Vector3f& point : ground) {
    point = ground_T_sub_map * point;
  }

  // Least squares on each ground point's z over height, roll and pitch; the closure pose carries only yaw and xy.
  for (int iteration = 0; iteration < kMaxGroundIterations; ++iteration) {
    Eigen::Matrix3f hessian = Eigen::Matrix3f::Zero();
    Eigen::Vector3f gradient = Eigen::Vector3f::Zero();
    for (const Eigen::Vector3f& point : ground) {
      const Eigen::Vector3f jacobian(1.0F, point.y(), -point.x());
      const float weight = std::exp(-point.z() * point.z());
      hessian.noalias() += weight * jacobian * jacobian.transpose();
      gradient += weight * point.z() * jacobian;
    }
    const Eigen::Vector3f step = hessian.ldlt().solve(-gradient);
    Sophus::Vector6f tangent;
    tangent << 0.0F, 0.0F, step.x(), step.y(), step.z(), 0.0F;
    const Sophus::SE3f update = Sophus::SE3f::exp(tangent);
    for (Eigen::Vector3f& point : ground) {
      point = update * point;
    }
    ground_T_sub_map = update * ground_T_sub_map;
    if (step.norm() < kGroundConvergedStep) {
      break;
    }
  }

  return ground_T_sub_map;
}

// Levels on a measured up: the rotation is fixed by it, the ground under the sub-map gives only the height.
Sophus::SE3f level_by_up(const Eigen::Vector3f& measured_up,
                         const std::span<const Eigen::Vector3f> centroids,
                         const std::span<const Eigen::Vector3f> normals) {
  const Eigen::Vector3f unit_up = measured_up.normalized();
  const Sophus::SO3f level(Eigen::Quaternionf::FromTwoVectors(unit_up, Eigen::Vector3f::UnitZ()));
  std::vector<Eigen::Vector3f> levelled(centroids.size());
  std::ranges::transform(centroids, levelled.begin(),
                         [&level](const Eigen::Vector3f& centroid) { return Eigen::Vector3f(level * centroid); });

  const auto lowest_voxel_per_column = lowest_per_column(levelled);
  std::vector<float> ground_heights;
  for (const auto& [_, index] : lowest_voxel_per_column) {
    if (std::abs(normals[index].dot(unit_up)) > kGroundNormalAgreement) {
      ground_heights.push_back(levelled[index].z());
    }
  }
  if (ground_heights.empty()) {
    return {level, Eigen::Vector3f::Zero()};
  }
  float height = std::reduce(ground_heights.begin(), ground_heights.end()) / static_cast<float>(ground_heights.size());

  for (int iteration = 0; iteration < kMaxGroundIterations; ++iteration) {
    float weight_sum = 0.0F;
    float weighted_offset = 0.0F;
    for (const float ground_height : ground_heights) {
      const float offset = ground_height - height;
      const float weight = std::exp(-offset * offset);
      weight_sum += weight;
      weighted_offset += weight * offset;
    }
    if (weight_sum == 0.0F) {
      break; // every column is further than exp(-d^2) can weigh in float, so the mean stands
    }
    const float step = weighted_offset / weight_sum;
    height += step;
    if (std::abs(step) < kGroundConvergedStep) {
      break;
    }
  }

  return {level, Eigen::Vector3f(0.0F, 0.0F, -height)};
}

struct DensityImage {
  cv::Mat1b image;
  // cell (x, y) of pixel (row 0, column 0): rows run along x, columns along y
  Eigen::Vector2i origin = Eigen::Vector2i::Zero();
};

DensityImage density_image(const std::span<const Eigen::Vector3f> points,
                           const Sophus::SE3f& ground_T_sub_map,
                           const float resolution,
                           const float threshold) {
  if (points.empty()) {
    return {};
  }
  const float inv_resolution = 1.0F / resolution;
  std::vector<Eigen::Vector2i> cells;
  cells.reserve(points.size());
  Eigen::Vector2i lower = Eigen::Vector2i::Constant(std::numeric_limits<int>::max());
  Eigen::Vector2i upper = Eigen::Vector2i::Constant(std::numeric_limits<int>::min());
  for (const Eigen::Vector3f& point : points) {
    const Eigen::Vector2i cell = ((ground_T_sub_map * point).head<2>() * inv_resolution).array().floor().cast<int>();
    lower = lower.cwiseMin(cell);
    upper = upper.cwiseMax(cell);
    cells.push_back(cell);
  }

  const Eigen::Vector2i extent = upper - lower + Eigen::Vector2i::Ones();
  cv::Mat1i counts(extent.x(), extent.y(), 0);
  int max_count = 0;
  for (const Eigen::Vector2i& cell : cells) {
    int& count = counts(cell.x() - lower.x(), cell.y() - lower.y());
    ++count;
    max_count = std::max(max_count, count);
  }

  DensityImage density{.image = cv::Mat1b(extent.x(), extent.y(), uchar{0}), .origin = lower};
  if (max_count == 1) {
    return density;
  }
  for (int row = 0; row < counts.rows; ++row) {
    for (int col = 0; col < counts.cols; ++col) {
      const float normalised = static_cast<float>(counts(row, col) - 1) / static_cast<float>(max_count - 1);
      if (normalised > threshold) {
        density.image(row, col) = static_cast<uchar>(255.0F * normalised);
      }
    }
  }

  return density;
}

inline int hamming_distance(const Descriptor& first, const Descriptor& second) {
  return std::transform_reduce(
      first.begin(), first.end(), second.begin(), 0, std::plus<>(),
      [](const std::uint64_t lhs, const std::uint64_t rhs) { return std::popcount(lhs ^ rhs); });
}

std::vector<Feature> distinctive_orb_features(const DensityImage& density) {
  const cv::Ptr<cv::ORB> orb = cv::ORB::create();
  orb->setScaleFactor(1.0);
  orb->setNLevels(1);
  orb->setFastThreshold(35);
  std::vector<cv::KeyPoint> orb_keypoints;
  cv::Mat orb_descriptors;
  orb->detectAndCompute(density.image, cv::noArray(), orb_keypoints, orb_descriptors);

  std::vector<Feature> features(orb_keypoints.size());
  for (std::size_t row = 0; row < features.size(); ++row) {
    const cv::Point2f& pixel = orb_keypoints[row].pt;
    features[row].cell = Eigen::Vector2f(pixel.y + 0.5F + static_cast<float>(density.origin.x()), // cell centre
                                         pixel.x + 0.5F + static_cast<float>(density.origin.y()));
    std::memcpy(features[row].descriptor.data(), orb_descriptors.ptr<std::uint8_t>(static_cast<int>(row)),
                sizeof(features[row].descriptor));
  }

  // Two unrelated descriptors differ in about 128 of their 256 bits, so anything this close repeats within the sub-map.
  constexpr int kSelfSimilarityDistanceBits = 35;
  std::vector<Feature> distinctive;
  std::copy_if(features.begin(), features.end(), std::back_inserter(distinctive), [&features](const Feature& feature) {
    int nearest_other = std::numeric_limits<int>::max();
    for (const Feature& other : features) {
      if (&other != &feature) {
        nearest_other = std::min(nearest_other, hamming_distance(feature.descriptor, other.descriptor));
      }
    }
    return nearest_other > kSelfSimilarityDistanceBits;
  });

  return distinctive;
}

struct MatchedCells {
  Eigen::Matrix2Xf reference;
  Eigen::Matrix2Xf query;
};

MatchedCells match_features(const std::vector<Feature>& reference_features,
                            const std::vector<Feature>& query_features,
                            const int hamming_distance_threshold) {
  std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f>> nearest;
  for (const Feature& query_feature : query_features) {
    Eigen::Vector2f nearest_cell = Eigen::Vector2f::Zero();
    int nearest_distance = std::numeric_limits<int>::max();
    for (const Feature& reference_feature : reference_features) {
      const int distance = hamming_distance(query_feature.descriptor, reference_feature.descriptor);
      if (distance < nearest_distance) {
        nearest_cell = reference_feature.cell;
        nearest_distance = distance;
      }
    }
    if (nearest_distance < hamming_distance_threshold) {
      nearest.emplace_back(nearest_cell, query_feature.cell);
    }
  }

  const Eigen::Index n_matches = std::ssize(nearest);
  MatchedCells matched{.reference = Eigen::Matrix2Xf(2, n_matches), .query = Eigen::Matrix2Xf(2, n_matches)};
  for (Eigen::Index column = 0; column < n_matches; ++column) {
    const auto& [reference_cell, query_cell] = nearest[static_cast<std::size_t>(column)];
    matched.reference.col(column) = reference_cell;
    matched.query.col(column) = query_cell;
  }

  return matched;
}

std::vector<Eigen::Index> ransac_inliers(const MatchedCells& matched, std::mt19937& rng) {
  // Cells are what the matched features are measured in, so this radius is 1.5 m at the default resolution.
  constexpr float kInlierRadiusCells = 3.0F;
  const Eigen::Index n_matches = matched.reference.cols();
  std::vector<Eigen::Index> indices(static_cast<std::size_t>(n_matches));
  std::iota(indices.begin(), indices.end(), Eigen::Index{0});

  std::array<Eigen::Index, 2> sample{};
  Eigen::Array<bool, 1, Eigen::Dynamic> best = Eigen::Array<bool, 1, Eigen::Dynamic>::Zero(n_matches);
  for (int trial = 0; trial < kRansacTrials; ++trial) {
    std::sample(indices.begin(), indices.end(), sample.begin(), sample.size(), rng);
    const Eigen::Matrix3f query_T_reference = Eigen::umeyama(matched.reference(Eigen::indexing::all, sample),
                                                             matched.query(Eigen::indexing::all, sample), false);

    const Eigen::Array<bool, 1, Eigen::Dynamic> is_inlier =
        (((query_T_reference.topLeftCorner<2, 2>() * matched.reference).colwise() +
          query_T_reference.topRightCorner<2, 1>()) -
         matched.query)
            .colwise()
            .squaredNorm()
            .array() < kInlierRadiusCells * kInlierRadiusCells;

    if (is_inlier.count() > best.count()) {
      best = is_inlier;
    }
  }

  std::erase_if(indices, [&](const Eigen::Index index) { return !best(index); });
  return indices;
}

} // namespace

std::string ClosureDetector::Config::to_yaml() const {
  return std::format("density_map_resolution: {}\ndensity_threshold: {}\nhamming_distance_threshold: {}\n"
                     "inliers_threshold: {}\nno_of_sub_maps_to_skip: {}\n",
                     density_map_resolution, density_threshold, hamming_distance_threshold, inliers_threshold,
                     no_of_sub_maps_to_skip);
}

std::optional<ClosureCandidate> ClosureDetector::query(const SubMapIndex target,
                                                       const std::optional<Eigen::Vector3f>& measured_up,
                                                       const SubMapPoints sub_map) {
  UTL_PROFILER_SCOPE("ClosureDetector::query");
  const std::vector<ClosureCandidate> candidates = query_all(target, measured_up, sub_map);
  const auto best = std::ranges::max_element(candidates, {}, &ClosureCandidate::number_of_inliers);
  if (best == candidates.end() || best->number_of_inliers < config.inliers_threshold) {
    return std::nullopt;
  }
  return *best;
}

std::vector<ClosureCandidate> ClosureDetector::query_all(const SubMapIndex target,
                                                         const std::optional<Eigen::Vector3f>& measured_up,
                                                         const SubMapPoints sub_map) {
  UTL_PROFILER_SCOPE("ClosureDetector::query_all");
  const Sophus::SE3f ground_T_sub_map = measured_up.has_value()
                                            ? level_by_up(*measured_up, sub_map.centroids, sub_map.normals)
                                            : level_by_ground(sub_map.centroids, sub_map.normals);
  SubMapFeatures query{
      .index = target,
      .ground_T_sub_map = ground_T_sub_map,
      .features = distinctive_orb_features(
          density_image(sub_map.points, ground_T_sub_map, config.density_map_resolution, config.density_threshold)),
  };

  // The model is fitted through two matches, so those two agree by construction; evidence starts at the third.
  constexpr std::size_t kTooFewMatches = 2;
  std::seed_seq seed{target.session, target.sub_map};
  std::mt19937 rng(seed);
  const std::ptrdiff_t n_references =
      std::clamp<std::ptrdiff_t>(std::ssize(database) - config.no_of_sub_maps_to_skip, 0, std::ssize(database));
  std::vector<ClosureCandidate> candidates;
  for (const SubMapFeatures& reference : std::span(database).first(static_cast<std::size_t>(n_references))) {
    const MatchedCells matched = match_features(reference.features, query.features, config.hamming_distance_threshold);
    if (std::cmp_less_equal(matched.reference.cols(), kTooFewMatches)) {
      continue;
    }

    const std::vector<Eigen::Index> inliers = ransac_inliers(matched, rng);
    if (inliers.size() <= kTooFewMatches) {
      continue;
    }

    const Sophus::SE2f query_cells_T_reference_cells(Eigen::umeyama(
        matched.reference(Eigen::indexing::all, inliers), matched.query(Eigen::indexing::all, inliers), false));
    const float yaw = query_cells_T_reference_cells.so2().log();
    const Eigen::Vector2f translation = query_cells_T_reference_cells.translation() * config.density_map_resolution;
    // no height in a bird's-eye fit
    const Sophus::SE3f query_ground_T_reference_ground(Sophus::SO3f::rotZ(yaw),
                                                       Eigen::Vector3f(translation.x(), translation.y(), 0.0F));

    candidates.push_back({
        .source = reference.index,
        .target = query.index,
        .target_T_source =
            query.ground_T_sub_map.inverse() * query_ground_T_reference_ground * reference.ground_T_sub_map,
        .number_of_inliers = inliers.size(),
    });
  }

  database.push_back(std::move(query));

  return candidates;
}

} // namespace rko_slam::closures
