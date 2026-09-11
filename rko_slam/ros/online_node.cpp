#include <exception>
#include <memory>
#include <string>

#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/version.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "rko_slam/ros/base_node.hpp"

namespace rko_slam::ros {

namespace {

class OnlineNode : public BaseNode {
public:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  OnlineNode(OnlineNode&&) = delete;
  OnlineNode(const OnlineNode&) = delete;
  OnlineNode& operator=(OnlineNode&&) = delete;
  OnlineNode& operator=(const OnlineNode&) = delete;

  explicit OnlineNode(const rclcpp::NodeOptions& options) : BaseNode("rko_slam_online", options) {
    run_output = declare_run_output("rko_slam");

#if RCLCPP_VERSION_MAJOR >= 30
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, *node);
#else
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node);
#endif
    const auto qos_lidar = rclcpp::SensorDataQoS().keep_last(50);
    scan_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic, qos_lidar,
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) { lidar_callback(msg); });

    RCLCPP_INFO_STREAM(
        node->get_logger(), node->get_name()
                                << " up. lidar=" << lidar_topic << " base=" << base_frame << " odom=" << odom_frame
                                << " map=" << map_frame << " run_dir="
                                << (run_output ? run_output->dir.string() : std::string{"<dump_results is false>"}));
  }

  ~OnlineNode() {
    try {
      write_run_config();
      dump_results_to_disk();
    } catch (const std::exception& error) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "run dump failed: " << error.what());
    }
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() {
    return node->get_node_base_interface();
  }
};

} // namespace

} // namespace rko_slam::ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rko_slam::ros::OnlineNode)
