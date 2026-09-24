add_executable(
  rko_slam_core_tests
  test_closure_thread_synthetic.cpp
  test_run_artifacts.cpp
  test_slam.cpp
  test_sub_map.cpp
  test_tum_writer.cpp
  test_voxel_hash_map.cpp)

target_link_libraries(rko_slam_core_tests PRIVATE rko_slam::core opencv_imgcodecs opencv_core Catch2::Catch2WithMain)

catch_discover_tests(rko_slam_core_tests)
