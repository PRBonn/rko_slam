FetchContent_Declare(
  utl
  GIT_REPOSITORY https://github.com/DmitriBogdanov/UTL.git
  GIT_TAG v9.0.0
  ${RKO_SLAM_FETCH_CONTENT_FLAGS})
FetchContent_MakeAvailable(utl)

if(UTL_PROFILER_DISABLE)
  target_compile_definitions(utl_lib_include INTERFACE UTL_PROFILER_DISABLE)
endif()
