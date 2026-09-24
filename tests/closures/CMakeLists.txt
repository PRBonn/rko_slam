add_executable(rko_slam_closures_tests test_detector.cpp test_icp.cpp test_voxel_overlap_coefficient.cpp)

target_link_libraries(rko_slam_closures_tests PRIVATE rko_slam::closures rko_slam::core rko_lio::core
                                                      Catch2::Catch2WithMain)

catch_discover_tests(rko_slam_closures_tests)
