cmake_minimum_required(VERSION 3.28.0)
project(rko_slam LANGUAGES C CXX)

option(RKO_SLAM_BUILD_TESTS "Build rko_slam tests" OFF)
option(RKO_SLAM_ENABLE_PROFILING "Fetch UTL and link it in, for the scoped profiler" OFF)

include(cmake/utl.cmake)

find_package(ament_cmake REQUIRED)
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
find_package(rko_lio 0.4.1 REQUIRED CONFIG)
find_package(Eigen3 3.4 REQUIRED CONFIG)
find_package(Sophus REQUIRED CONFIG)
find_package(spdlog REQUIRED CONFIG)
find_package(tsl-robin-map REQUIRED CONFIG)
find_package(yaml-cpp 0.8 REQUIRED CONFIG)
find_package(
  OpenCV REQUIRED
  COMPONENTS core
             features2d
             imgcodecs
             imgproc
             CONFIG)
find_package(nanoflann 1.5.1 REQUIRED CONFIG)

add_subdirectory(rko_slam/closures)
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
