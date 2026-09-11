# Fork of PRBonn/MapClosures, two commits ahead: `no_of_local_maps_to_skip` promoted to a Config field, and the vendored
# OpenCV builds imgcodecs so a fetch-only build can write the run's trajectory png.
FetchContent_Declare(
  map_closures
  GIT_REPOSITORY https://github.com/mehermvr/MapClosures.git
  GIT_TAG 02392b6d9f51bb30e69957e71e83b1fee3f4cbf5
  SOURCE_SUBDIR cpp ${RKO_SLAM_FETCH_CONTENT_FLAGS})
FetchContent_MakeAvailable(map_closures)
