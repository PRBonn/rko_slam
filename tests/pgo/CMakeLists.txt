add_executable(rko_slam_pgo_tests test_g2o_text.cpp test_jacobians.cpp test_optimize.cpp)

target_link_libraries(rko_slam_pgo_tests PRIVATE rko_slam::pgo rko_lio::core Catch2::Catch2WithMain)

find_package(g2o QUIET CONFIG)
if(g2o_FOUND)
  target_sources(rko_slam_pgo_tests PRIVATE test_g2o_oracle.cpp)
  target_link_libraries(rko_slam_pgo_tests PRIVATE g2o::core g2o::types_slam3d g2o::solver_eigen)
else()
  message(STATUS "rko_slam: no g2o, so the pgo oracle test is not built")
endif()

catch_discover_tests(rko_slam_pgo_tests)
