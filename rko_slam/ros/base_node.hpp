#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <filesystem>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/context.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rko_lio/core/process_timestamps.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rko_slam/core/slam.hpp"
#include "rko_slam/core/sub_map_builder.hpp"

namespace rko_slam::ros {

class BaseNode {
public:
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<core::SubMapBuilder> sub_map_builder;
  std::unique_ptr<core::SLAM> slam;

  std::string lidar_topic;
  std::string base_frame;

  std::string odom_frame{"odom"};
  std::string map_frame{"map"};

  std::chrono::milliseconds tf_lookup_timeout{80};

  rko_lio::core::TimestampProcessingConfig timestamps_config;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sub_maps_pub;
  std::size_t sub_maps_published = 0;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr closure_maps_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr keypose_graph_pub;

  std::optional<Sophus::SE3f> base_T_lidar;
  std::atomic<core::Nsec> latest_scan_time{core::Nsec{0}};
  std::size_t scans_processed = 0;
  std::size_t scans_dropped = 0;

  // A run's artifacts are <dir>/<name>_*.
  struct RunOutput {
    std::filesystem::path dir;
    std::string name;
    bool dump_sub_maps = false;
  };
  std::optional<RunOutput> run_output;
  bool publish_sub_maps = false;
  bool deskew = false;
  bool publish_closure_maps = false;
  bool publish_keypose_graph = false;

  // for last sealed sub-map. closure searches are one at a time, the wait at the next split is free in practice,
  // and otherwise degrades to running the closure inline.
  std::vector<std::future<bool>> sub_map_writes;
  std::future<void> closure_task; // declared after sub_map_writes, which its thread pushes into

  BaseNode() = delete;
  BaseNode(const std::string& name, const rclcpp::NodeOptions& options);
  ~BaseNode() = default;

  BaseNode(const BaseNode&) = delete;
  BaseNode(BaseNode&&) = delete;
  BaseNode& operator=(const BaseNode&) = delete;
  BaseNode& operator=(BaseNode&&) = delete;

  // sub-map integration; a sealed sub-map is handed to `closure_task`.
  void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

  // Runs on `closure_task`: the closure search, the pose-graph update and everything published off the result.
  void process_closure(core::FinishedSubMap finished);

  void
  write_sub_map(const core::KeyposeId keypose_id, const core::Nsec keypose_time, std::vector<Eigen::Vector3f> points);

  // A closure's correction reaches tf on the scan after it.
  void broadcast_map_T_odom(const builtin_interfaces::msg::Time& stamp) const;
  std::optional<RunOutput> declare_run_output(const std::string& default_run_name);
  // `extra` is appended: the keys the calling node owns and this class does not.
  void write_run_config(const std::string_view extra = {}) const;

  void publish_sub_map_outputs(const std::shared_ptr<const core::Keyposes>& keyposes,
                               const builtin_interfaces::msg::Time& stamp);

  void publish_keypose_graph_markers(const std::shared_ptr<const core::Keyposes>& keyposes,
                                     const builtin_interfaces::msg::Time& stamp);

  void dump_results_to_disk();
};

} // namespace rko_slam::ros
