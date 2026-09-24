cmake_minimum_required(VERSION 3.28.0)
project(rko_slam LANGUAGES C CXX)

option(RKO_SLAM_BUILD_TESTS "Build rko_slam tests" OFF)
option(RKO_SLAM_FETCH_CONTENT_DEPS "Fetch nanoflann and Catch2 instead of finding them" OFF)
option(UTL_PROFILER_DISABLE "Compile out UTL_PROFILER_SCOPE and friends" OFF)

find_package(ament_cmake REQUIRED)
find_package(rko_lio REQUIRED CONFIG)
find_package(rclcpp REQUIRED CONFIG)
find_package(rclcpp_components REQUIRED CONFIG)
find_package(geometry_msgs REQUIRED CONFIG)
find_package(sensor_msgs REQUIRED CONFIG)
find_package(std_msgs REQUIRED CONFIG)
find_package(visualization_msgs REQUIRED CONFIG)
find_package(tf2 REQUIRED CONFIG)
find_package(tf2_ros REQUIRED CONFIG)
find_package(builtin_interfaces REQUIRED CONFIG)
find_package(rosbag2_storage REQUIRED CONFIG)

include(cmake/dependencies.cmake)

add_subdirectory(rko_slam/pgo)
add_subdirectory(rko_slam/core)
add_subdirectory(rko_slam/align_sessions)
add_subdirectory(rko_slam/ros)

rclcpp_components_register_node(
  rko_slam_online_node_component
  PLUGIN
  "rko_slam::ros::OnlineNode"
  EXECUTABLE
  online_node)

install(
  TARGETS rko_slam_online_node_component offline_node align_sessions
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()

if(RKO_SLAM_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
