if(RKO_SLAM_ENABLE_PROFILING)
  include(FetchContent)
  FetchContent_Declare(
    utl
    GIT_REPOSITORY https://github.com/DmitriBogdanov/UTL.git
    GIT_TAG v9.0.0
    SYSTEM EXCLUDE_FROM_ALL)
  FetchContent_MakeAvailable(utl)
else()
  # the header UTL would have given us, with the profiler compiled out
  file(WRITE ${CMAKE_BINARY_DIR}/utl_disabled/UTL/profiler.hpp
       "#pragma once\n#define UTL_PROFILER_DISABLE\n#define UTL_PROFILER_SCOPE(label)\n")
  add_library(UTL::include INTERFACE IMPORTED)
  target_include_directories(UTL::include INTERFACE ${CMAKE_BINARY_DIR}/utl_disabled)
endif()
