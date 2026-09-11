add_library(rko_slam.core STATIC)
add_library(rko_slam::core ALIAS rko_slam.core)

target_sources(
  rko_slam.core
  PRIVATE closure.cpp
          pose_graph.cpp
          run_artifacts.cpp
          slam.cpp
          sub_map.cpp
          sub_map_builder.cpp
  PUBLIC FILE_SET
         HEADERS
         FILES
         closure.hpp
         pose_graph.hpp
         run_artifacts.hpp
         slam.hpp
         sub_map.hpp
         sub_map_builder.hpp
         types.hpp
         voxel_hash_map.hpp
         BASE_DIRS
         ../..)

target_link_libraries(
  rko_slam.core
  PUBLIC rko_lio::core
         Sophus::Sophus
         spdlog::spdlog
         Eigen3::Eigen
         tsl::robin_map
         nanoflann::nanoflann
         g2o::core
         g2o::types_slam3d
         g2o::solver_cholmod
         map_closures
         UTL::include)

target_compile_features(rko_slam.core PUBLIC cxx_std_20)
set_target_properties(rko_slam.core PROPERTIES POSITION_INDEPENDENT_CODE ON)
