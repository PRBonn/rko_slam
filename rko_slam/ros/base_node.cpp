#include "rko_slam/ros/base_node.hpp"

#include <UTL/profiler.hpp>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <format>
#include <fstream>
#include <future>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rko_lio/core/deskew.hpp>
#include <rko_lio/core/error.hpp>
#include <rko_lio/core/process_timestamps.hpp>
#include <rko_lio/ros/utils/spdlog_sink.hpp>
#include <rko_lio/ros/utils/utils.hpp>
#include <spdlog/spdlog.h>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "rko_slam/core/run_artifacts.hpp"

namespace {
using rko_lio::ros::utils::to_ns;
using OptionalPose = std::optional<Sophus::SE3f>;

struct Scan {
  std::vector<Eigen::Vector3f> points;
  Sophus::SE3f odom_T_base;
  rko_slam::core::Nsec end_time{0};
};

void transform_points(const Sophus::SE3f& pose, std::vector<Eigen::Vector3f>& points) {
  for (Eigen::Vector3f& point : points) {
    point = pose * point;
  }
}

geometry_msgs::msg::Point pose_to_point(const Sophus::SE3f& pose) {
  geometry_msgs::msg::Point point;
  rko_lio::ros::utils::eigen_vector_to_ros_xyz(pose.translation(), point);
  return point;
}

// The `map <- sub_map_0 <- sub_map_1 <- ...` TF chain; every link after the first is a relative pose.
std::vector<geometry_msgs::msg::TransformStamped>
build_sub_map_chain(const rko_slam::core::Keyposes& keyposes, const std::string& map_frame, const rclcpp::Time& stamp) {
  std::vector<geometry_msgs::msg::TransformStamped> chain;
  chain.reserve(keyposes.map_T_keypose.size());
  for (std::size_t i = 0; i < keyposes.map_T_keypose.size(); ++i) {
    geometry_msgs::msg::TransformStamped link;
    link.header.stamp = stamp;
    link.header.frame_id = i == 0 ? map_frame : std::format("sub_map_{}", i - 1);
    link.child_frame_id = std::format("sub_map_{}", i);
    link.transform = rko_lio::ros::utils::sophus_to_transform(i == 0 ? keyposes.map_T_keypose.at(0)
                                                                     : keyposes.map_T_keypose.at(i - 1).inverse() *
                                                                           keyposes.map_T_keypose.at(i));
    chain.push_back(std::move(link));
  }
  return chain;
}

inline visualization_msgs::msg::Marker make_marker(const std_msgs::msg::Header& header,
                                                   const std::string& marker_namespace,
                                                   const std::int32_t marker_id,
                                                   const std::int32_t type,
                                                   const double scale,
                                                   const float red,
                                                   const float green,
                                                   const float blue) {
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = marker_namespace;
  marker.id = marker_id;
  marker.type = type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = marker.scale.y = marker.scale.z = scale;
  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.color.a = 1.0F;
  marker.pose.orientation.w = 1.0;
  return marker;
}

// Both sides of a closure as one map-frame cloud, told apart by the intensity channel (source 0, target 1).
sensor_msgs::msg::PointCloud2::UniquePtr make_closure_pair_point_cloud2(const std::vector<Eigen::Vector3f>& source_pts,
                                                                        const Sophus::SE3f& map_T_source,
                                                                        const std::vector<Eigen::Vector3f>& target_pts,
                                                                        const Sophus::SE3f& map_T_target,
                                                                        const std_msgs::msg::Header& header) {
  std::vector<Eigen::Vector3f> points;
  std::vector<float> intensities;
  points.reserve(source_pts.size() + target_pts.size());
  intensities.reserve(source_pts.size() + target_pts.size());
  const auto append = [&points, &intensities](const std::vector<Eigen::Vector3f>& cloud,
                                              const Sophus::SE3f& map_T_keypose, const float intensity) {
    for (const Eigen::Vector3f& point : cloud) {
      points.push_back(map_T_keypose * point);
      intensities.push_back(intensity);
    }
  };
  append(source_pts, map_T_source, 0.0F);
  append(target_pts, map_T_target, 1.0F);
  return rko_lio::ros::utils::eigen_to_point_cloud2(points, intensities, header);
}
} // namespace

namespace rko_slam::ros {

BaseNode::BaseNode(const std::string& name, const rclcpp::NodeOptions& options) {
  node = rclcpp::Node::make_shared(name, options);
  auto logger = std::make_shared<spdlog::logger>("rko_slam",
                                                 std::make_shared<rko_lio::ros::utils::RclcppSink>(node->get_logger()));
  logger->set_level(spdlog::level::trace);
  spdlog::set_default_logger(std::move(logger));

#ifndef UTL_PROFILER_DISABLE
  utl::profiler::profiler.print_at_exit(false);
#endif

  lidar_topic = node->declare_parameter<std::string>("lidar_topic"); // required
  base_frame = node->declare_parameter<std::string>("base_frame", base_frame);
  odom_frame = node->declare_parameter<std::string>("odom_frame", odom_frame);
  map_frame = node->declare_parameter<std::string>("map_frame", map_frame);
  invert_map_tf = node->declare_parameter<bool>("invert_map_tf", invert_map_tf);

  deskew = node->declare_parameter<bool>("deskew", deskew);
  timestamps_config.multiplier_to_seconds = node->declare_parameter<double>("lidar_timestamps.multiplier_to_seconds",
                                                                            timestamps_config.multiplier_to_seconds);
  timestamps_config.force_absolute =
      node->declare_parameter<bool>("lidar_timestamps.force_absolute", timestamps_config.force_absolute);
  timestamps_config.force_relative =
      node->declare_parameter<bool>("lidar_timestamps.force_relative", timestamps_config.force_relative);

  core::SubMapBuilder::Config sub_map_config;
  sub_map_config.voxel_map.voxel_size =
      static_cast<float>(node->declare_parameter<double>("voxel_size", sub_map_config.voxel_map.voxel_size));
  sub_map_config.splitting_distance =
      static_cast<float>(node->declare_parameter<double>("splitting_distance", sub_map_config.splitting_distance));
  sub_map_config.voxel_map.max_points_per_voxel = static_cast<unsigned>(node->declare_parameter<std::int64_t>(
      "max_points_per_voxel", static_cast<std::int64_t>(sub_map_config.voxel_map.max_points_per_voxel)));
  sub_map_config.min_range = static_cast<float>(node->declare_parameter<double>("min_range", sub_map_config.min_range));
  sub_map_config.max_range = static_cast<float>(node->declare_parameter<double>("max_range", sub_map_config.max_range));
  sub_map_builder = std::make_unique<core::SubMapBuilder>(sub_map_config);

  tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock(), tf2::durationFromSec(50.0));
  tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*node);

  publish_sub_maps = node->declare_parameter<bool>("publish_sub_maps", publish_sub_maps);
  publish_closure_maps = node->declare_parameter<bool>("publish_closure_maps", publish_closure_maps);
  publish_keypose_graph = node->declare_parameter<bool>("publish_keypose_graph", publish_keypose_graph);

  const rclcpp::QoS qos(10);
  if (publish_sub_maps) {
    sub_maps_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("rko_slam/sub_maps", qos);
  }
  if (publish_closure_maps) {
    closure_maps_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("rko_slam/closure_maps", qos);
  }
  if (publish_keypose_graph) {
    keypose_graph_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("rko_slam/keypose_graph", qos);
  }

  core::SLAM::Config slam_config;
  slam_config.closure_detector.density_map_resolution = static_cast<float>(
      node->declare_parameter<double>("density_map_resolution", slam_config.closure_detector.density_map_resolution));
  slam_config.closure_detector.density_threshold = static_cast<float>(
      node->declare_parameter<double>("density_threshold", slam_config.closure_detector.density_threshold));
  slam_config.closure_detector.hamming_distance_threshold = static_cast<int>(node->declare_parameter<std::int64_t>(
      "hamming_distance_threshold", slam_config.closure_detector.hamming_distance_threshold));
  slam_config.closure_detector.inliers_threshold = static_cast<std::size_t>(node->declare_parameter<std::int64_t>(
      "inliers_threshold", static_cast<std::int64_t>(slam_config.closure_detector.inliers_threshold)));
  slam_config.closure_detector.no_of_sub_maps_to_skip = static_cast<int>(node->declare_parameter<std::int64_t>(
      "no_of_sub_maps_to_skip", slam_config.closure_detector.no_of_sub_maps_to_skip));

  slam_config.closure_overlap_threshold =
      static_cast<float>(node->declare_parameter<double>("overlap_threshold", slam_config.closure_overlap_threshold));

  slam_config.pose_graph.max_iterations =
      static_cast<int>(node->declare_parameter<std::int64_t>("max_iterations", slam_config.pose_graph.max_iterations));
  slam_config.pose_graph.rotation_info_scale =
      node->declare_parameter<double>("rotation_info_scale", slam_config.pose_graph.rotation_info_scale);
  slam_config.pose_graph.closure_info_scale =
      node->declare_parameter<double>("closure_info_scale", slam_config.pose_graph.closure_info_scale);
  slam_config.pose_graph.closure_kernel_delta =
      node->declare_parameter<double>("closure_kernel_delta", slam_config.pose_graph.closure_kernel_delta);

  slam = std::make_unique<core::SLAM>(slam_config, sub_map_builder->config.voxel_map);
}

OptionalPose BaseNode::resolve_base_T_lidar(const std_msgs::msg::Header& scan_header) {
  if (base_T_lidar) {
    return base_T_lidar;
  }
  if (base_frame == scan_header.frame_id) {
    base_T_lidar = Sophus::SE3f{};
    return base_T_lidar;
  }
  base_T_lidar = rko_lio::ros::utils::get_transform(tf_buffer, scan_header.frame_id, base_frame,
                                                    to_ns(scan_header.stamp), tf_lookup_timeout);
  if (base_T_lidar) {
    RCLCPP_INFO_STREAM(node->get_logger(), "Resolved " << base_frame << " <- " << scan_header.frame_id);
  } else {
    RCLCPP_WARN_STREAM_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                                "Waiting for extrinsic " << base_frame << " <- " << scan_header.frame_id);
  }
  return base_T_lidar;
}

void BaseNode::lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  UTL_PROFILER_SCOPE("BaseNode::lidar_callback");
  using rko_lio::ros::utils::get_transform;
  if (msg->header.frame_id.empty()) {
    RCLCPP_WARN_STREAM(node->get_logger(), "dropping scan: header.frame_id is empty, cannot look up its odometry");
    ++scans_dropped;
    return;
  }
  if (base_frame.empty()) {
    base_frame = msg->header.frame_id;
  }
  const OptionalPose extrinsic = resolve_base_T_lidar(msg->header);
  if (!extrinsic) {
    ++scans_dropped;
    return;
  }

  Scan scan;
  if (!deskew) {
    const OptionalPose odom_T_base =
        get_transform(tf_buffer, base_frame, odom_frame, to_ns(msg->header.stamp), tf_lookup_timeout);
    if (!odom_T_base) {
      RCLCPP_WARN_STREAM(node->get_logger(),
                         "dropping scan: no " << odom_frame << " <- " << base_frame << " at the scan stamp");
      ++scans_dropped;
      return;
    }
    scan.points = rko_lio::ros::utils::point_cloud2_to_eigen(msg);
    transform_points(*extrinsic, scan.points);
    scan.odom_T_base = *odom_T_base;
    scan.end_time = to_ns(msg->header.stamp);
  } else {
    std::vector<Eigen::Vector3f> points_lidar;
    rko_lio::core::Timestamps timestamps{};
    try {
      auto raw = rko_lio::ros::utils::point_cloud2_to_eigen_with_timestamps(msg);
      points_lidar = std::move(raw.points);
      timestamps = rko_lio::core::process_timestamps(raw.timestamps, to_ns(msg->header.stamp), timestamps_config);
    } catch (const rko_lio::core::InputError& error) {
      RCLCPP_WARN_STREAM(node->get_logger(), "dropping scan: " << error.what());
      ++scans_dropped;
      return;
    }

    const OptionalPose odom_T_base_at_start =
        get_transform(tf_buffer, base_frame, odom_frame, timestamps.min, tf_lookup_timeout);
    const OptionalPose odom_T_base_at_end =
        get_transform(tf_buffer, base_frame, odom_frame, timestamps.max, tf_lookup_timeout);
    if (!odom_T_base_at_start || !odom_T_base_at_end) {
      RCLCPP_WARN_STREAM(node->get_logger(), "dropping scan: no " << odom_frame << " <- " << base_frame
                                                                  << " at one or both scan endpoints, cannot deskew");
      ++scans_dropped;
      return;
    }

    transform_points(*extrinsic, points_lidar);

    if (timestamps.max > timestamps.min) {
      // Constant-velocity deskewing to scan-end.
      const Sophus::Vector6f tau = (odom_T_base_at_start->inverse() * *odom_T_base_at_end).log();
      const auto scan_duration = rko_lio::core::to_seconds<float>(timestamps.max - timestamps.min);
      rko_lio::core::deskew_scan(points_lidar, timestamps,
                                 {
                                     .start_time = timestamps.min,
                                     .linear_velocity = tau.head<3>() / scan_duration,
                                     .acceleration = Eigen::Vector3f::Zero(),
                                     .angular_velocity = tau.tail<3>() / scan_duration,
                                 });
    }
    scan.points = std::move(points_lidar);
    scan.odom_T_base = *odom_T_base_at_end;
    scan.end_time = timestamps.max;
  }

  latest_scan_time.store(scan.end_time, std::memory_order_relaxed);
  broadcast_map_tf(rko_lio::ros::utils::to_ros_time(scan.end_time));
  auto finished = sub_map_builder->add_to_live_map(scan.points, scan.end_time, scan.odom_T_base);
  ++scans_processed;
  if (finished) {
    if (closure_task.valid()) {
      closure_task.get();
    }
    closure_task = std::async(std::launch::async, &BaseNode::process_closure, this, std::move(*finished));
  }
}

void BaseNode::process_closure(core::FinishedSubMap finished) {
  UTL_PROFILER_SCOPE("BaseNode::process_closure");

  const core::KeyposeId keypose_id = finished.sub_map->id;
  const core::Nsec keypose_time = finished.sub_map->scan_times.front();
  const auto accepted_closure = slam->process_finished_sub_map(std::move(finished.sub_map), finished.points);
  write_sub_map(keypose_id, keypose_time, std::move(finished.points));
  const auto stamp = rko_lio::ros::utils::to_ros_time(latest_scan_time.load(std::memory_order_relaxed));
  const auto keyposes = slam->latest_keyposes();

  if (publish_sub_maps) {
    publish_sub_map_outputs(keyposes, stamp);
  }

  if (publish_closure_maps && accepted_closure) {
    const auto [source_id, target_id] = *accepted_closure;
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = map_frame;
    closure_maps_pub->publish(make_closure_pair_point_cloud2(
        slam->sub_maps.at(source_id)->centroids, keyposes->map_T_keypose.at(source_id),
        slam->sub_maps.at(target_id)->centroids, keyposes->map_T_keypose.at(target_id), header));
  }

  if (publish_keypose_graph) {
    publish_keypose_graph_markers(keyposes, stamp);
  }

  if (accepted_closure) {
    const auto [source_id, target_id] = *accepted_closure;
    RCLCPP_INFO_STREAM(node->get_logger(), "closure accepted " << source_id << " -> " << target_id << " (total="
                                                               << slam->pose_graph.num_closure_edges() << ")");
  }
  RCLCPP_DEBUG_STREAM(node->get_logger(), "closure cycle done; sub_maps=" << slam->sub_maps.size() << " closures="
                                                                          << slam->pose_graph.num_closure_edges());
}

void BaseNode::write_sub_map(const core::KeyposeId keypose_id,
                             const core::Nsec keypose_time,
                             std::vector<Eigen::Vector3f> points) {
  if (!run_output || !run_output->dump_sub_maps) {
    return;
  }
  sub_map_writes.push_back(std::async(
      std::launch::async,
      [path = run_output->dir / "sub_maps" / std::format("sub_map_{:06}_{}.ply", keypose_id, keypose_time.count()),
       points = std::move(points)] { return core::write_ply_xyz(path, points); }));
}

void BaseNode::broadcast_map_tf(const builtin_interfaces::msg::Time& stamp) const {
  const auto keyposes = slam->latest_keyposes();
  Sophus::SE3f map_T_odom;
  if (keyposes && !keyposes->map_T_keypose.empty()) {
    map_T_odom = keyposes->map_T_keypose.back() * keyposes->odom_T_keypose.back().inverse();
  }
  geometry_msgs::msg::TransformStamped out;
  out.header.stamp = stamp;
  if (invert_map_tf) {
    out.header.frame_id = odom_frame;
    out.child_frame_id = map_frame;
    out.transform = rko_lio::ros::utils::sophus_to_transform(map_T_odom.inverse());
  } else {
    out.header.frame_id = map_frame;
    out.child_frame_id = odom_frame;
    out.transform = rko_lio::ros::utils::sophus_to_transform(map_T_odom);
  }
  tf_broadcaster->sendTransform(out);
}

std::optional<BaseNode::RunOutput> BaseNode::declare_run_output(const std::string& default_run_name) {
  const std::filesystem::path parent = node->declare_parameter<std::string>("results_dir", "results");
  const std::string name = node->declare_parameter<std::string>("run_name", default_run_name);
  const bool dump_results = node->declare_parameter<bool>("dump_results", false);
  const bool dump_sub_maps = node->declare_parameter<bool>("dump_sub_maps", true);
  if (!dump_results) {
    return std::nullopt;
  }
  auto [resolved_name, dir] = core::resolve_run_dir(parent, name);
  return RunOutput{.dir = std::move(dir), .name = std::move(resolved_name), .dump_sub_maps = dump_sub_maps};
}

void BaseNode::write_run_config(const std::string_view extra) const {
  if (!run_output) {
    return;
  }
  const std::filesystem::path path = run_output->dir / (run_output->name + "_config.yaml");
  std::ofstream out(path);
  if (!out) {
    RCLCPP_WARN_STREAM(node->get_logger(), "cannot open " << path.string() << "; run config not written");
    return;
  }
  std::string resolved_extrinsic = "none";
  if (base_T_lidar) {
    const Eigen::Quaternionf& quaternion = base_T_lidar->unit_quaternion();
    const Eigen::Vector3f& translation = base_T_lidar->translation();
    resolved_extrinsic = std::format("[{}, {}, {}, {}, {}, {}, {}]", quaternion.x(), quaternion.y(), quaternion.z(),
                                     quaternion.w(), translation.x(), translation.y(), translation.z());
  }
  out << std::boolalpha;
  out << "lidar_topic: " << lidar_topic << '\n'
      << "base_frame: " << (base_frame.empty() ? "\"\"" : base_frame) << '\n'
      << "# resolved base_frame <- scan extrinsic: " << resolved_extrinsic << '\n'
      << "odom_frame: " << odom_frame << '\n'
      << "map_frame: " << map_frame << '\n'
      << "invert_map_tf: " << invert_map_tf << '\n'
      << "dump_results: " << true << '\n'
      << "dump_sub_maps: " << run_output->dump_sub_maps << '\n'
      << "deskew: " << deskew << '\n'
      << std::format("lidar_timestamps.multiplier_to_seconds: {}\n", timestamps_config.multiplier_to_seconds)
      << "lidar_timestamps.force_absolute: " << timestamps_config.force_absolute << '\n'
      << "lidar_timestamps.force_relative: " << timestamps_config.force_relative << '\n'
      << sub_map_builder->config.to_yaml() << slam->config.closure_detector.to_yaml()
      << std::format("overlap_threshold: {}\n", slam->config.closure_overlap_threshold)
      << slam->config.pose_graph.to_yaml() << "publish_sub_maps: " << publish_sub_maps << '\n'
      << "publish_closure_maps: " << publish_closure_maps << '\n'
      << "publish_keypose_graph: " << publish_keypose_graph << '\n'
      << extra;
  RCLCPP_INFO_STREAM(node->get_logger(), "wrote " << path.string());
}

void BaseNode::publish_sub_map_outputs(const std::shared_ptr<const core::Keyposes>& keyposes,
                                       const builtin_interfaces::msg::Time& stamp) {
  const std::size_t total = slam->sub_maps.size();
  for (std::size_t i = sub_maps_published; i < total; ++i) {
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = std::format("sub_map_{}", i);
    sub_maps_pub->publish(rko_lio::ros::utils::eigen_to_point_cloud2(slam->sub_maps.at(i)->centroids, header));
  }
  sub_maps_published = total;
  const auto chain = build_sub_map_chain(*keyposes, map_frame, stamp);
  if (!chain.empty()) {
    tf_broadcaster->sendTransform(chain);
  }
}

void BaseNode::publish_keypose_graph_markers(const std::shared_ptr<const core::Keyposes>& keyposes,
                                             const builtin_interfaces::msg::Time& stamp) {
  if (!keyposes || keyposes->map_T_keypose.empty()) {
    return;
  }
  using visualization_msgs::msg::Marker;
  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = map_frame;

  visualization_msgs::msg::MarkerArray markers;

  auto keypose_spheres = make_marker(header, "keyposes", 0, Marker::SPHERE_LIST, 1.5, 0.2F, 0.9F, 0.2F);
  for (const Sophus::SE3f& keypose : keyposes->map_T_keypose) {
    keypose_spheres.points.push_back(pose_to_point(keypose));
  }
  markers.markers.push_back(std::move(keypose_spheres));

  auto odom_edges = make_marker(header, "pose_graph_edges_odom", 1, Marker::LINE_LIST, 0.3, 0.95F, 0.5F, 0.1F);
  auto closure_edges = make_marker(header, "pose_graph_edges_closure", 2, Marker::LINE_LIST, 0.9, 0.95F, 0.1F, 0.1F);
  for (const auto& edge : slam->pose_graph.edges()) {
    auto& bucket = core::is_closure_pair(edge.from_id, edge.to_id) ? closure_edges : odom_edges;
    bucket.points.push_back(pose_to_point(keyposes->map_T_keypose.at(edge.from_id)));
    bucket.points.push_back(pose_to_point(keyposes->map_T_keypose.at(edge.to_id)));
  }
  markers.markers.push_back(std::move(odom_edges));
  markers.markers.push_back(std::move(closure_edges));

  keypose_graph_pub->publish(markers);
}

void BaseNode::dump_results_to_disk() {
  if (!run_output) {
    return;
  }
  const RunOutput& output = *run_output;
  if (closure_task.valid()) {
    closure_task.get();
  }
  if (std::optional<core::FinishedSubMap> trailing = sub_map_builder->finalize()) {
    const core::KeyposeId keypose_id = trailing->sub_map->id;
    const core::Nsec keypose_time = trailing->sub_map->scan_times.front();
    slam->process_finished_sub_map(std::move(trailing->sub_map), trailing->points);
    write_sub_map(keypose_id, keypose_time, std::move(trailing->points));
  }
  for (std::future<bool>& write : sub_map_writes) {
    if (!write.get()) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "a sub-map ply write failed");
    }
  }
  if (scans_processed == 0) {
    RCLCPP_INFO_STREAM(node->get_logger(), "no scans processed - skipping trajectory + pose-graph dump");
    return;
  }

  slam->save_run_artifacts(output.dir, output.name);

#ifndef UTL_PROFILER_DISABLE
  utl::profiler::Style style;
  style.color = false;
  const std::filesystem::path profile_path = output.dir / (output.name + "_profile.txt");
  std::ofstream ofs(profile_path);
  ofs << utl::profiler::profiler.format_results(style);
  RCLCPP_INFO_STREAM(node->get_logger(), "wrote " << profile_path.string());
#endif

  RCLCPP_INFO_STREAM(node->get_logger(),
                     "sub_maps=" << slam->sub_maps.size() << " closures=" << slam->pose_graph.num_closure_edges()
                                 << " scans_processed=" << scans_processed << " scans_dropped=" << scans_dropped);
}

} // namespace rko_slam::ros
