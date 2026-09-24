#include "rko_slam/core/sub_map_builder.hpp"

#include <UTL/profiler.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <format>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <rko_lio/core/util.hpp>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

namespace rko_slam::core {

namespace {

using rko_lio::core::to_seconds;

std::vector<Eigen::Vector3f>
range_filter_scan(const std::vector<Eigen::Vector3f>& base_points, const float min_range, const float max_range) {
  const float min_sq = min_range * min_range;
  const float max_sq = max_range * max_range;
  std::vector<Eigen::Vector3f> filtered;
  filtered.reserve(base_points.size());
  for (const auto& point : base_points) {
    const float range_sq = point.squaredNorm();
    if (!std::isfinite(range_sq) || range_sq < min_sq || range_sq > max_sq) {
      continue;
    }
    filtered.emplace_back(point);
  }
  return filtered;
}

// The carried points come back in the new keypose's frame.
std::vector<Eigen::Vector3f>
carryover_points(const VoxelHashMap& voxel_map, const Sophus::SE3f& new_T_old, const float radius) {
  std::vector<Eigen::Vector3f> carried;
  carried.reserve(voxel_map.voxels.size() * voxel_map.max_points_per_voxel);
  const float sq_radius = radius * radius;
  for (const auto& [voxel, block] : voxel_map.voxels) {
    if ((new_T_old * block.front()).squaredNorm() > sq_radius) {
      continue;
    }
    for (const auto& point : block) {
      carried.emplace_back(new_T_old * point);
    }
  }
  return carried;
}

struct BracketingScans {
  Nsec before_time;
  Sophus::SE3f before;
  Nsec after_time;
  Sophus::SE3f after;
};

// `time` must lie after the sub-map's first scan.
BracketingScans
bracketing_scans(const SubMap& sub_map, const Nsec time, const Nsec end_time, const Sophus::SE3f& live_T_base) {
  const auto after =
      static_cast<std::size_t>(std::ranges::lower_bound(sub_map.scan_times, time) - sub_map.scan_times.begin());
  const bool is_last_scan = after == sub_map.scan_times.size();
  return {
      .before_time = sub_map.scan_times.at(after - 1),
      .before = sub_map.local_trajectory.at(after - 1),
      .after_time = is_last_scan ? end_time : sub_map.scan_times[after],
      .after = is_last_scan ? live_T_base : sub_map.local_trajectory[after],
  };
}

Sophus::SO3f rotation_at(const BracketingScans& scans, const Nsec time) {
  const float fraction =
      to_seconds<float>(time - scans.before_time) / to_seconds<float>(scans.after_time - scans.before_time);
  return scans.before.so3() * Sophus::SO3f::exp(fraction * (scans.before.so3().inverse() * scans.after.so3()).log());
}

struct IntervalVelocity {
  Eigen::Vector3f velocity;
  // the time the mean velocity is taken to hold
  Nsec middle;
};

std::optional<Eigen::Vector3f> measured_up(const std::vector<ImuSample>& imu_samples,
                                           const SubMap& sub_map,
                                           const Nsec end_time,
                                           const Sophus::SE3f& live_T_base) {
  const auto velocity_at = [&](const Nsec time) {
    const BracketingScans scans = bracketing_scans(sub_map, time, end_time, live_T_base);
    const Nsec interval = scans.after_time - scans.before_time;
    return IntervalVelocity{
        .velocity = (scans.after.translation() - scans.before.translation()) / to_seconds<float>(interval),
        .middle = scans.before_time + (interval / 2),
    };
  };

  Eigen::Vector3f sum = Eigen::Vector3f::Zero();
  std::size_t count = 0;
  Nsec first{0};
  Nsec last{0};
  for (const ImuSample& sample : imu_samples) {
    if (sample.time <= sub_map.scan_times.front() || sample.time > end_time) {
      continue;
    }
    sum +=
        rotation_at(bracketing_scans(sub_map, sample.time, end_time, live_T_base), sample.time) * sample.specific_force;
    first = count == 0 ? sample.time : std::min(first, sample.time);
    last = count == 0 ? sample.time : std::max(last, sample.time);
    ++count;
  }
  if (count == 0) {
    return std::nullopt;
  }

  const IntervalVelocity first_velocity = velocity_at(first);
  const IntervalVelocity last_velocity = velocity_at(last);
  // over a shorter span the velocity difference is scan-to-scan odometry noise, not the platform's acceleration
  constexpr Nsec kMinAccelerationSpan = std::chrono::seconds{1};
  if (last_velocity.middle - first_velocity.middle < kMinAccelerationSpan) {
    return std::nullopt;
  }
  // R(t)f(t) = a(t) - g, so mean(R f) - mean(a) = -g, the measured up.
  // we need the mean acceleration over the samples. a = dv/dt, so that mean is the endpoint difference, the interior
  // velocities cancel. we assume even IMU spacing, which makes the sample mean the time mean.
  const Eigen::Vector3f mean_acceleration = (last_velocity.velocity - first_velocity.velocity) /
                                            to_seconds<float>(last_velocity.middle - first_velocity.middle);
  return sum / static_cast<float>(count) - mean_acceleration;
}

} // namespace

std::string SubMapBuilder::Config::to_yaml() const {
  return std::format("voxel_size: {}\nsplitting_distance: {}\nmax_points_per_voxel: {}\nmin_range: {}\n"
                     "max_range: {}\n",
                     voxel_map.voxel_size, splitting_distance, voxel_map.max_points_per_voxel, min_range, max_range);
}

SubMapBuilder::SubMapBuilder(const Config sub_map_config)
    : config(sub_map_config), voxel_map(sub_map_config.voxel_map) {}

std::optional<FinishedSubMap> SubMapBuilder::add_to_live_map(const std::vector<Eigen::Vector3f>& base_points,
                                                             const Nsec end_time,
                                                             const Sophus::SE3f& odom_T_base) {
  UTL_PROFILER_SCOPE("SubMapBuilder::add_to_live_map");
  std::optional<FinishedSubMap> finished;
  std::vector<Eigen::Vector3f> carryover_in_new_frame;
  Sophus::SE3f live_T_base;

  if (live) {
    SubMap& sub_map = live->sub_map;
    live_T_base = sub_map.odom_T_keypose.inverse() * odom_T_base;
    if (live_T_base.translation().norm() >= config.splitting_distance) {
      carryover_in_new_frame = carryover_points(voxel_map, live_T_base.inverse(), config.splitting_distance);
      sub_map.odom_T_next_keypose = odom_T_base;
      fill_sub_map(voxel_map, sub_map);
      sub_map.measured_up = measured_up(live->imu_samples, sub_map, end_time, live_T_base);
      finished = FinishedSubMap{.sub_map = std::make_unique<SubMap>(std::move(sub_map)), .points = voxel_map.points()};
      live.reset();
    }
  }

  if (!live) {
    live = LiveSubMap{.sub_map = {.id = next_id++, .odom_T_keypose = odom_T_base}};
    voxel_map = VoxelHashMap(config.voxel_map);
    voxel_map.add_points(carryover_in_new_frame); // can be empty, on first sub_map
    live_T_base = Sophus::SE3f{};
  }

  if (!base_points.empty()) {
    std::vector<Eigen::Vector3f> filtered_points = range_filter_scan(base_points, config.min_range, config.max_range);
    for (Eigen::Vector3f& point : filtered_points) {
      point = live_T_base * point;
    }
    voxel_map.add_points(filtered_points);
  }
  live->sub_map.local_trajectory.push_back(live_T_base);
  live->sub_map.scan_times.push_back(end_time);
  return finished;
}

std::optional<FinishedSubMap> SubMapBuilder::finalize() {
  if (!live) {
    return std::nullopt;
  }
  SubMap& sub_map = live->sub_map;
  fill_sub_map(voxel_map, sub_map);
  // imu samples between the last scan added and now are dropped
  sub_map.measured_up =
      measured_up(live->imu_samples, sub_map, sub_map.scan_times.back(), sub_map.local_trajectory.back());
  FinishedSubMap out{.sub_map = std::make_unique<SubMap>(std::move(sub_map)), .points = voxel_map.points()};
  live.reset();
  return out;
}

} // namespace rko_slam::core
