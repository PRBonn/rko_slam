add_library(rko_slam.pgo STATIC)
add_library(rko_slam::pgo ALIAS rko_slam.pgo)

target_sources(
  rko_slam.pgo
  PRIVATE pose_graph.cpp io.cpp
  PUBLIC FILE_SET
         HEADERS
         FILES
         pose_graph.hpp
         io.hpp
         BASE_DIRS
         ../..)

target_link_libraries(
  rko_slam.pgo
  PUBLIC Sophus::Sophus Eigen3::Eigen rko_lio::core
  PRIVATE UTL::include)

target_compile_features(rko_slam.pgo PUBLIC cxx_std_20)
set_target_properties(rko_slam.pgo PROPERTIES POSITION_INDEPENDENT_CODE ON)
