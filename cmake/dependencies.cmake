include(FetchContent)

set(RKO_SLAM_FETCH_CONTENT_FLAGS OVERRIDE_FIND_PACKAGE SYSTEM EXCLUDE_FROM_ALL)

# Nothing fetched here is ever installed, so none of it is built shared. Catch2 appends its own module directory, which
# the scope would otherwise discard along with the flag.
block(PROPAGATE CMAKE_MODULE_PATH)
set(BUILD_SHARED_LIBS OFF)

# Eigen, Sophus, spdlog and tsl-robin-map arrive with rko_lio, which is found before this file. yaml-cpp arrives with
# rosbag2_storage, which exports it, so it is found and never fetched
find_package(yaml-cpp 0.8 REQUIRED CONFIG)

if(RKO_SLAM_FETCH_CONTENT_DEPS)
  include(${CMAKE_CURRENT_LIST_DIR}/dependencies/nanoflann/nanoflann.cmake)
  if(RKO_SLAM_BUILD_TESTS)
    include(${CMAKE_CURRENT_LIST_DIR}/dependencies/catch2/catch2.cmake)
  endif()
else()
  find_package(nanoflann 1.5.1 REQUIRED CONFIG)
  if(RKO_SLAM_BUILD_TESTS)
    find_package(Catch2 3 REQUIRED CONFIG)
  endif()
endif()

# No usable system package exists for these, so they are fetched either way.
include(${CMAKE_CURRENT_LIST_DIR}/dependencies/g2o/g2o.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/dependencies/map_closures/map_closures.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/dependencies/utl/utl.cmake)
endblock()
