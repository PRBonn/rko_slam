#include <UTL/profiler.hpp>
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <exception>
#include <filesystem>
#include <format>
#include <memory>
#include <optional>
#include <ranges>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/utilities.hpp>
#include <rko_lio/ros/utils/rosbag.hpp>
#include <rko_lio/ros/utils/time.hpp>
#include <rko_lio/ros/utils/transforms.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <spdlog/spdlog.h>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2/time.hpp>

#include "rko_slam/core/run_artifacts.hpp"
#include "rko_slam/core/types.hpp"
#include "rko_slam/ros/base_node.hpp"

namespace rko_slam::ros {

using rko_lio::ros::utils::to_ns;
using rko_lio::ros::utils::to_ros_time;

namespace {

class OfflineNode : public BaseNode {
public:
  std::string bag_path;
  std::optional<std::filesystem::path> odom_tum_path;
  std::unique_ptr<rko_lio::ros::utils::BufferableBag> bag;
  std::vector<core::TrajectorySample> odom_trajectory;
  static constexpr auto trajectory_inject_lookahead = std::chrono::seconds(1);

  std::size_t total_bag_msgs = 0;
  std::size_t processed_bag_msgs = 0;
  std::chrono::steady_clock::time_point bag_start_time;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr bag_progress_pub;

  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> lidar_serializer;
  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serializer;
  std::size_t odom_trajectory_cursor = 0;
  std::size_t scans_skipped_out_of_trajectory = 0;

  OfflineNode(OfflineNode&&) = delete;
  OfflineNode(const OfflineNode&) = delete;
  OfflineNode& operator=(OfflineNode&&) = delete;
  OfflineNode& operator=(const OfflineNode&) = delete;

  explicit OfflineNode(const rclcpp::NodeOptions& options) : BaseNode("rko_slam_offline", options) {
    bag_path = node->declare_parameter<std::string>("bag_path");
    if (const auto tum = node->declare_parameter<std::string>("odom_tum_path", std::string{}); !tum.empty()) {
      odom_tum_path = tum;
    }
    run_output = declare_run_output(
        std::filesystem::path(bag_path.substr(0, bag_path.find_last_not_of('/') + 1)).filename().string());

    if (odom_tum_path) {
      RCLCPP_INFO_STREAM(node->get_logger(), "odometry from " << *odom_tum_path << "; the bag's /tf is ignored");
      odom_trajectory = core::read_tum(*odom_tum_path);
      std::ranges::sort(odom_trajectory, {}, &core::TrajectorySample::time);
      RCLCPP_INFO_STREAM(node->get_logger(), "rko_slam_offline odom TUM mode: "
                                                 << odom_trajectory.size() << " samples from " << *odom_tum_path
                                                 << "; bracket = [" << odom_trajectory.front().time.count() << "ns, "
                                                 << odom_trajectory.back().time.count() << "ns]");
    }

    std::vector<std::string> topics{lidar_topic};
    if (!imu_topic.empty()) {
      topics.push_back(imu_topic);
    }
    bag = std::make_unique<rko_lio::ros::utils::BufferableBag>(bag_path, topics, tf_buffer, tf2::durationFromSec(0.0),
                                                               std::chrono::seconds(1), !odom_tum_path);
    RCLCPP_INFO_STREAM(node->get_logger(),
                       "rko_slam_offline opened " << bag_path << " (" << bag->message_count() << " msgs)");

    total_bag_msgs = bag->message_count();
    bag_start_time = std::chrono::steady_clock::now();
    bag_progress_pub = node->create_publisher<std_msgs::msg::Float32MultiArray>("rko_slam/bag_progress", 10);

    RCLCPP_INFO_STREAM(
        node->get_logger(),
        node->get_name() << " up. lidar=" << lidar_topic
                         << " imu=" << (imu_topic.empty() ? std::string{"<unset>"} : imu_topic)
                         << " base=" << base_frame << " odom=" << odom_frame << " map=" << map_frame << " run_dir="
                         << (run_output ? run_output->dir.string() : std::string{"<dump_results is false>"}));
  }

  ~OfflineNode() {
    try {
      write_run_config(
          std::format("bag_path: {}\nodom_tum_path: {}\n", bag_path, odom_tum_path ? odom_tum_path->string() : "\"\""));
      dump_results_to_disk();
    } catch (const std::exception& error) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "run dump failed: " << error.what());
    }
  }

  void dispatch_lidar_message(const rosbag2_storage::SerializedBagMessage& bag_msg) {
    UTL_PROFILER_SCOPE("OfflineNode::dispatch_lidar_message");
    const auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    const rclcpp::SerializedMessage serialized(*bag_msg.serialized_data);
    lidar_serializer.deserialize_message(&serialized, cloud_msg.get());

    if (!odom_trajectory.empty()) {
      const core::Nsec scan_stamp = to_ns(cloud_msg->header.stamp);
      if (scan_stamp < odom_trajectory.front().time || scan_stamp > odom_trajectory.back().time) {
        ++scans_skipped_out_of_trajectory;
        return;
      }
      if (base_frame.empty()) {
        base_frame = cloud_msg->header.frame_id;
      }
      if (!base_frame.empty()) {
        inject_trajectory_up_to(scan_stamp + trajectory_inject_lookahead);
      }
    }

    lidar_callback(cloud_msg);
  }

  void dispatch_imu_message(const rosbag2_storage::SerializedBagMessage& bag_msg) {
    auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
    const rclcpp::SerializedMessage serialized(*bag_msg.serialized_data);
    imu_serializer.deserialize_message(&serialized, imu_msg.get());
    imu_callback(imu_msg);
  }

  void publish_bag_progress() const {
    const auto now = std::chrono::steady_clock::now();
    const float elapsed_seconds = std::chrono::duration<float>(now - bag_start_time).count();
    const auto processed = static_cast<float>(processed_bag_msgs);
    const float percent_complete = 100.0F * processed / static_cast<float>(total_bag_msgs);
    const float avg_time_per_msg = (processed_bag_msgs > 0) ? elapsed_seconds / processed : 0.0F;
    const float seconds_remaining = avg_time_per_msg * static_cast<float>(total_bag_msgs - processed_bag_msgs);

    std_msgs::msg::Float32MultiArray progress_msg;
    progress_msg.layout.dim.resize(1);
    progress_msg.layout.dim.at(0).label = "percent_complete,seconds_remaining";
    progress_msg.layout.dim.at(0).size = 2;
    progress_msg.layout.dim.at(0).stride = 2;
    progress_msg.data = {percent_complete, seconds_remaining};
    bag_progress_pub->publish(progress_msg);
  }

  void inject_trajectory_up_to(const core::Nsec until) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.frame_id = odom_frame;
    tf_msg.child_frame_id = base_frame;
    while (odom_trajectory_cursor < odom_trajectory.size() &&
           odom_trajectory.at(odom_trajectory_cursor).time <= until) {
      const auto& sample = odom_trajectory.at(odom_trajectory_cursor);
      tf_msg.header.stamp = to_ros_time(sample.time);
      tf_msg.transform = rko_lio::ros::utils::sophus_to_transform(sample.pose);
      tf_buffer->setTransform(tf_msg, "tum_file", /*is_static=*/false);
      ++odom_trajectory_cursor;
    }
  }

  void run() {
    while (rclcpp::ok() && !bag->finished()) {
      UTL_PROFILER_SCOPE("OfflineNode::run::per_bag_msg");
      const rosbag2_storage::SerializedBagMessage bag_msg = bag->PopNextMessage();
      ++processed_bag_msgs;
      publish_bag_progress();
      if (bag_msg.topic_name == lidar_topic) {
        dispatch_lidar_message(bag_msg);
      } else if (bag_msg.topic_name == imu_topic) {
        dispatch_imu_message(bag_msg);
      }
    }

    // The last closure publishes, and dump_results_to_disk's drain runs after shutdown has begun.
    if (closure_task.valid()) {
      closure_task.get();
    }

    RCLCPP_INFO_STREAM(node->get_logger(), "rko_slam_offline drained; bag_messages="
                                               << processed_bag_msgs << " scans_skipped_out_of_trajectory="
                                               << scans_skipped_out_of_trajectory);
  }
};

} // namespace

} // namespace rko_slam::ros

int main(int argc, char* const* argv) {
  rclcpp::init(argc, argv);
  {
    rko_slam::ros::OfflineNode node{rclcpp::NodeOptions{}};
    node.run();
  }
  rclcpp::shutdown();
}
