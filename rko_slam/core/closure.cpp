#include "rko_slam/core/closure.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <functional>
#include <memory>
#include <optional>
#include <rko_lio/core/error.hpp>
#include <rko_lio/core/voxel_down_sample.hpp> // brings rko_lio::core::VoxelHash
#include <sophus/se3.hpp>
#include <spdlog/spdlog.h>
#include <stdexcept>
#include <string>
#include <tsl/robin_map.h>
#include <utility>
#include <vector>

#include <UTL/profiler.hpp>
#include <map_closures/MapClosures.hpp>
#include <nanoflann.hpp>

namespace rko_slam::core {

namespace {

map_closures::Config to_upstream(const ClosureDetector::Config& config) {
  return {
      .density_map_resolution = config.density_map_resolution,
      .density_threshold = config.density_threshold,
      .hamming_distance_threshold = config.hamming_distance_threshold,
      .no_of_local_maps_to_skip = config.no_of_sub_maps_to_skip,
  };
}

// Upstream's cloud API is `std::vector<Eigen::Vector3d>`
std::vector<Eigen::Vector3d> to_upstream_cloud(const std::vector<Eigen::Vector3f>& points) {
  std::vector<Eigen::Vector3d> out;
  out.reserve(points.size());
  for (const auto& point : points) {
    out.emplace_back(point.cast<double>());
  }
  return out;
}

ClosureCandidate from_upstream(const map_closures::ClosureCandidate& upstream,
                               const std::vector<KeyposeId>& local_to_global) {
  return {
      .source_id = local_to_global.at(static_cast<std::size_t>(upstream.source_id)),
      .target_id = local_to_global.at(static_cast<std::size_t>(upstream.target_id)),
      .target_T_source = Sophus::SE3d(Eigen::Quaterniond(upstream.pose.block<3, 3>(0, 0)).normalized(),
                                      upstream.pose.block<3, 1>(0, 3))
                             .cast<float>(),
      .number_of_inliers = upstream.number_of_inliers,
  };
}

struct LinearSystem {
  Sophus::Matrix6f H = Sophus::Matrix6f::Zero();
  Sophus::Vector6f b = Sophus::Vector6f::Zero();
  float chi = 0;
  std::size_t count = 0;
};

struct PointCloudAdaptor {
  std::reference_wrapper<const std::vector<Eigen::Vector3f>> points;
  std::size_t kdtree_get_point_count() const { return points.get().size(); }
  // nanoflann only asks for indices below kdtree_get_point_count and dims below the 3 it was built with
  float kdtree_get_pt(std::size_t idx, std::size_t dim) const { return points.get()[idx](static_cast<int>(dim)); }
  template <class BBOX>
  bool kdtree_get_bbox(BBOX& /*bbox*/) const {
    return false;
  }
};

using NanoIndex = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, PointCloudAdaptor>,
                                                      PointCloudAdaptor,
                                                      3,
                                                      std::uint32_t>;
constexpr std::size_t kLeafMaxSize = 10;
constexpr std::size_t kMaxIters = 100;
constexpr double kRelativeFitness = 1e-6;
constexpr double kRelativeRmse = 1e-4;

struct VoxelTag {
  bool in_first = false;
  bool in_second = false;
};

} // namespace

std::string ClosureDetector::Config::to_yaml() const {
  return std::format("density_map_resolution: {}\ndensity_threshold: {}\nhamming_distance_threshold: {}\n"
                     "inliers_threshold: {}\nno_of_sub_maps_to_skip: {}\n",
                     density_map_resolution, density_threshold, hamming_distance_threshold, inliers_threshold,
                     no_of_sub_maps_to_skip);
}

ClosureDetector::ClosureDetector(const Config& detector_config)
    : config(detector_config), detector(std::make_unique<map_closures::MapClosures>(to_upstream(config))) {}
ClosureDetector::~ClosureDetector() = default;

std::optional<ClosureCandidate> ClosureDetector::query(const KeyposeId keypose_id,
                                                       const std::vector<Eigen::Vector3f>& points) {
  UTL_PROFILER_SCOPE("ClosureDetector::query");
  const int local_id = register_local_id(keypose_id);
  const map_closures::ClosureCandidate raw = detector->GetBestClosure(local_id, to_upstream_cloud(points));
  if (std::cmp_less(raw.number_of_inliers, config.inliers_threshold)) {
    return std::nullopt;
  }
  return from_upstream(raw, local_to_global);
}

std::vector<ClosureCandidate> ClosureDetector::query_all(const KeyposeId keypose_id,
                                                         const std::vector<Eigen::Vector3f>& points) {
  UTL_PROFILER_SCOPE("ClosureDetector::query_all");
  const int local_id = register_local_id(keypose_id);
  const std::vector<map_closures::ClosureCandidate> raw = detector->GetClosures(local_id, to_upstream_cloud(points));
  std::vector<ClosureCandidate> out;
  out.reserve(raw.size());
  for (const auto& candidate : raw) {
    out.push_back(from_upstream(candidate, local_to_global));
  }
  return out;
}

int ClosureDetector::register_local_id(const KeyposeId global_id) {
  const auto local = static_cast<int>(local_to_global.size());
  local_to_global.push_back(global_id);
  return local;
}

ClosureRefinement refine_closure(const float voxel_size,
                                 const float max_correspondence_distance,
                                 const SubMap& source,
                                 const SubMap& target,
                                 const Sophus::SE3f& target_T_source_init) {
  UTL_PROFILER_SCOPE("refine_closure");
  if (source.centroids.empty() || target.centroids.empty()) {
    throw std::invalid_argument("refine_closure: a sub-map with no centroids cannot be refined against");
  }

  const Sophus::SE3f refined = icp_point_to_plane(source.centroids, target.centroids, target.normals,
                                                  max_correspondence_distance, target_T_source_init);

  const double overlap = voxel_overlap_coefficient(target.centroids, source.centroids, refined, voxel_size);

  return ClosureRefinement{.refined_target_T_source = refined, .overlap = overlap};
}

Sophus::SE3f icp_point_to_plane(const std::vector<Eigen::Vector3f>& source,
                                const std::vector<Eigen::Vector3f>& target,
                                const std::vector<Eigen::Vector3f>& target_normals,
                                const float max_correspondence_distance,
                                const Sophus::SE3f& initial_guess) {
  if (source.empty() || target.empty()) {
    throw rko_lio::core::InputError("icp_point_to_plane: source and target must both be non-empty");
  }
  Sophus::SE3f pose_estimate = initial_guess;
  const PointCloudAdaptor adaptor{target};
  const NanoIndex target_tree(3, adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(kLeafMaxSize));
  const float max_sq_dist = max_correspondence_distance * max_correspondence_distance;
  const auto source_size = static_cast<double>(source.size());
  double prev_fitness = 0.0;
  double prev_rmse = 0.0;

  for (std::size_t iter = 0; iter < kMaxIters; ++iter) {
    // NOLINTBEGIN(readability-identifier-length) standard point-to-plane notation
    const Sophus::SE3f pose = pose_estimate;
    const Eigen::Matrix3f rotation = pose.rotationMatrix();
    LinearSystem linear_system;
    for (const Eigen::Vector3f& s : source) {
      const Eigen::Vector3f s_t = pose * s;
      std::uint32_t target_index = 0;
      float sq_distance = 0;
      if (target_tree.rknnSearch(s_t.data(), 1, &target_index, &sq_distance, max_sq_dist) == 0) {
        continue;
      }
      const Eigen::Vector3f& n = target_normals[target_index];
      const Eigen::Vector3f error_r3 = s_t - target[target_index];
      const float error = n.dot(error_r3);
      // e = nᵀ(T·s - t), right perturbation
      const Eigen::Vector3f n_tilde = rotation.transpose() * n; // ñ = Rᵀn
      Eigen::Matrix<float, 1, 6> J;
      J.head<3>() = n_tilde.transpose();
      J.tail<3>() = s.cross(n_tilde).transpose();
      linear_system.H.noalias() += J.transpose() * J;
      linear_system.b += J.transpose() * error;
      linear_system.chi += error * error;
      ++linear_system.count;
    }

    if (linear_system.count == 0) {
      spdlog::warn("icp_point_to_plane: no correspondence within {} m, returning the current estimate",
                   max_correspondence_distance);
      break;
    }

    const Sophus::Vector6f dx = linear_system.H.ldlt().solve(-linear_system.b);
    // NOLINTEND(readability-identifier-length)
    pose_estimate = pose_estimate * Sophus::SE3f::exp(dx);
    const double fitness = static_cast<double>(linear_system.count) / source_size;
    const double rmse = std::sqrt(static_cast<double>(linear_system.chi) / static_cast<double>(linear_system.count));
    if (std::abs(prev_fitness - fitness) < kRelativeFitness && std::abs(prev_rmse - rmse) < kRelativeRmse) {
      break;
    }
    prev_fitness = fitness;
    prev_rmse = rmse;
  }
  return pose_estimate;
}

double voxel_overlap_coefficient(const std::vector<Eigen::Vector3f>& first_pts,
                                 const std::vector<Eigen::Vector3f>& second_pts,
                                 const Sophus::SE3f& first_T_second,
                                 const float voxel_size) {
  if (first_pts.empty() || second_pts.empty() || voxel_size <= 0.0F) {
    throw std::invalid_argument("voxel_overlap_coefficient: needs two non-empty clouds and a positive voxel size");
  }
  const float inv_voxel_size = 1.0F / voxel_size;

  tsl::robin_map<Eigen::Vector3i, VoxelTag, rko_lio::core::VoxelHash> seen;
  seen.reserve(first_pts.size() + second_pts.size());

  std::size_t first_count = 0;
  for (const auto& point : first_pts) {
    auto& tag = seen.try_emplace(rko_lio::core::point_to_voxel(point, inv_voxel_size)).first.value();
    if (!tag.in_first) {
      tag.in_first = true;
      ++first_count;
    }
  }

  std::size_t second_count = 0;
  std::size_t intersection = 0;
  for (const auto& point : second_pts) {
    auto& tag = seen.try_emplace(rko_lio::core::point_to_voxel(first_T_second * point, inv_voxel_size)).first.value();
    if (!tag.in_second) {
      tag.in_second = true;
      ++second_count;
      if (tag.in_first) {
        ++intersection;
      }
    }
  }

  const std::size_t denom = std::min(first_count, second_count);
  return static_cast<double>(intersection) / static_cast<double>(denom);
}

} // namespace rko_slam::core
