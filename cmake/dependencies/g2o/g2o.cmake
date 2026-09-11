FetchContent_Declare(
  g2o
  GIT_REPOSITORY https://github.com/RainerKuemmerle/g2o.git
  GIT_TAG ed2c608283ce44b29a68ccd75e1cdc10f7e1b52e
  ${RKO_SLAM_FETCH_CONTENT_FLAGS})
# g2o-specific options, scoped to just that: no parallelism.
block()
set(G2O_BUILD_APPS OFF)
set(G2O_BUILD_EXAMPLES OFF)
set(G2O_USE_OPENGL OFF)
set(G2O_USE_OPENMP OFF)
FetchContent_MakeAvailable(g2o)
endblock()
