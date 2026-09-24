add_library(rko_slam.core STATIC)
add_library(rko_slam::core ALIAS rko_slam.core)

target_sources(
  rko_slam.core
  PRIVATE run_artifacts.cpp slam.cpp sub_map.cpp sub_map_builder.cpp
  PUBLIC FILE_SET
         HEADERS
         FILES
         run_artifacts.hpp
         slam.hpp
         sub_map.hpp
         sub_map_builder.hpp
         voxel_hash_map.hpp
         BASE_DIRS
         ../..)

target_link_libraries(
  rko_slam.core
  PUBLIC rko_slam::closures
         rko_slam::pgo
         rko_lio::core
         Sophus::Sophus
         Eigen3::Eigen
         spdlog::spdlog
         tsl::robin_map
         UTL::include
  PRIVATE opencv_imgcodecs opencv_imgproc opencv_core)

target_compile_features(rko_slam.core PUBLIC cxx_std_20)
set_target_properties(rko_slam.core PROPERTIES POSITION_INDEPENDENT_CODE ON)
