# ament_cmake declares `uninstall` too, and cmake allows one name.
FetchContent_Declare(
  nanoflann
  GIT_REPOSITORY https://github.com/jlblancoc/nanoflann.git
  GIT_TAG v1.10.0
  PATCH_COMMAND sed -i "s/^  add_custom_target(uninstall/  add_custom_target(nanoflann_uninstall_all/" CMakeLists.txt
                ${RKO_SLAM_FETCH_CONTENT_FLAGS})
FetchContent_MakeAvailable(nanoflann)
