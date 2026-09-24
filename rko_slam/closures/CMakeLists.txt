add_library(rko_slam.closures STATIC)
add_library(rko_slam::closures ALIAS rko_slam.closures)

target_sources(
  rko_slam.closures
  PRIVATE detector.cpp refinement.cpp
  PUBLIC FILE_SET
         HEADERS
         FILES
         detector.hpp
         refinement.hpp
         BASE_DIRS
         ../..)

target_link_libraries(
  rko_slam.closures
  PUBLIC Sophus::Sophus Eigen3::Eigen
  PRIVATE rko_lio::core
          opencv_features2d
          opencv_core
          spdlog::spdlog
          nanoflann::nanoflann
          tsl::robin_map
          UTL::include)

target_compile_features(rko_slam.closures PUBLIC cxx_std_20)
set_target_properties(rko_slam.closures PROPERTIES POSITION_INDEPENDENT_CODE ON)
