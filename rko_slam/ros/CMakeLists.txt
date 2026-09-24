add_library(rko_slam.ros STATIC base_node.cpp)
add_library(rko_slam::ros ALIAS rko_slam.ros)

target_sources(
  rko_slam.ros
  PUBLIC FILE_SET
         HEADERS
         FILES
         base_node.hpp
         BASE_DIRS
         ../..)

target_link_libraries(
  rko_slam.ros
  PUBLIC rko_slam::core
         rko_lio::ros::utils
         tf2_ros::tf2_ros
         tf2::tf2
         rclcpp::rclcpp
         ${visualization_msgs_TARGETS}
         ${sensor_msgs_TARGETS}
         ${geometry_msgs_TARGETS}
         ${std_msgs_TARGETS}
         ${builtin_interfaces_TARGETS})

set_target_properties(rko_slam.ros PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(offline_node offline_node.cpp)
target_link_libraries(offline_node PRIVATE rko_slam::ros rosbag2_storage::rosbag2_storage)

add_library(rko_slam_online_node_component SHARED online_node.cpp)
target_link_libraries(rko_slam_online_node_component PRIVATE rko_slam::ros rclcpp_components::component
                                                             rclcpp_components::component_manager)
add_executable(align_sessions align_sessions_node.cpp)
target_link_libraries(align_sessions PRIVATE rko_slam::align_sessions rko_slam::closures rko_lio::ros::utils
                                             rclcpp::rclcpp)
