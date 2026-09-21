#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <format>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <rko_lio/ros/utils/spdlog_sink.hpp>
#include <spdlog/spdlog.h>

#include "rko_slam/align_sessions/align_sessions.hpp"
#include "rko_slam/core/closure.hpp"

int main(int argc, char* const* argv) {
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>("rko_slam_align_sessions");
  auto logger = std::make_shared<spdlog::logger>("rko_slam",
                                                 std::make_shared<rko_lio::ros::utils::RclcppSink>(node->get_logger()));
  logger->set_level(spdlog::level::trace);
  spdlog::set_default_logger(std::move(logger));

  rko_slam::core::PoseGraph::Config pose_graph_config{.max_iterations = 100};
  rko_slam::core::ClosureDetector::Config detector_config{.no_of_sub_maps_to_skip = 0};

  const std::vector<std::string> run_dirs = node->declare_parameter<std::vector<std::string>>("run_dirs"); // required
  const std::filesystem::path results_dir = node->declare_parameter<std::string>("results_dir", "results");
  const auto run_name = node->declare_parameter<std::string>("run_name", "aligned");

  detector_config.density_map_resolution = static_cast<float>(
      node->declare_parameter<double>("density_map_resolution", detector_config.density_map_resolution));
  detector_config.density_threshold =
      static_cast<float>(node->declare_parameter<double>("density_threshold", detector_config.density_threshold));
  detector_config.hamming_distance_threshold = static_cast<int>(
      node->declare_parameter<std::int64_t>("hamming_distance_threshold", detector_config.hamming_distance_threshold));
  detector_config.inliers_threshold =
      static_cast<int>(node->declare_parameter<std::int64_t>("inliers_threshold", detector_config.inliers_threshold));
  detector_config.no_of_sub_maps_to_skip = static_cast<int>(
      node->declare_parameter<std::int64_t>("no_of_sub_maps_to_skip", detector_config.no_of_sub_maps_to_skip));
  const auto overlap_threshold = static_cast<float>(node->declare_parameter<double>(
      "overlap_threshold", rko_slam::core::ClosureRefinement::kDefaultOverlapThreshold));
  pose_graph_config.rotation_info_scale =
      node->declare_parameter<double>("rotation_info_scale", pose_graph_config.rotation_info_scale);
  pose_graph_config.closure_info_scale =
      node->declare_parameter<double>("closure_info_scale", pose_graph_config.closure_info_scale);
  pose_graph_config.max_iterations =
      static_cast<int>(node->declare_parameter<std::int64_t>("max_iterations", pose_graph_config.max_iterations));
  pose_graph_config.closure_kernel_delta =
      node->declare_parameter<double>("closure_kernel_delta", pose_graph_config.closure_kernel_delta);
  pose_graph_config.gravity_info_scale =
      node->declare_parameter<double>("gravity_info_scale", pose_graph_config.gravity_info_scale);

  const std::vector<std::filesystem::path> dirs(run_dirs.cbegin(), run_dirs.cend());

  const std::optional<rko_slam::align_sessions::AlignResult> aligned = rko_slam::align_sessions::align(
      detector_config, overlap_threshold, pose_graph_config, dirs, results_dir, run_name);
  if (!aligned) {
    RCLCPP_WARN_STREAM(node->get_logger(), "no alignment produced; nothing written");
    rclcpp::shutdown();
    return 0;
  }
  const rko_slam::align_sessions::AlignResult& result = *aligned;

  for (const std::size_t dropped_index : result.dropped_sessions) {
    const std::string& dropped_dir = run_dirs.at(dropped_index);
    RCLCPP_WARN_STREAM(node->get_logger(),
                       "session " << dropped_index << " (" << dropped_dir << ") dropped from the joint problem");
  }

  const std::filesystem::path config_path = result.out_run_dir / (result.out_run_name + "_config.yaml");
  std::ofstream config(config_path);
  if (!config) {
    spdlog::error("cannot open {}", config_path.string());
    return 0;
  }
  config << "run_dirs:\n";
  for (std::size_t i = 0; i < run_dirs.size(); ++i) {
    config << "  - " << run_dirs.at(i) << (i == result.reference_session ? "  # reference session" : "") << '\n';
  }
  config << std::format("voxel_size: {}\nmax_points_per_voxel: {}\n", result.voxel_map_config.voxel_size,
                        result.voxel_map_config.max_points_per_voxel)
         << detector_config.to_yaml() << std::format("overlap_threshold: {}\n", overlap_threshold)
         << pose_graph_config.to_yaml();
  RCLCPP_INFO_STREAM(node->get_logger(),
                     std::format("{} inter-session closures accepted across {} sessions; outputs in {}",
                                 result.accepted_closures, run_dirs.size() - result.dropped_sessions.size(),
                                 result.out_run_dir.string()));
  rclcpp::shutdown();
}
