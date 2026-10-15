^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rko_slam
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.7 (2026-10-15)
------------------
* support compressed cloud input (`#8 <https://github.com/PRBonn/rko_slam/issues/8>`_)
* build cleanup: rosdep everything, profiling behind a flag (`#7 <https://github.com/PRBonn/rko_slam/issues/7>`_)
* reimplement closure detection, drop map closures from the package (`#6 <https://github.com/PRBonn/rko_slam/issues/6>`_)
* solve the pose graph ourselves, drop g2o from the package (`#5 <https://github.com/PRBonn/rko_slam/issues/5>`_)
* use the imu to level the map, gravity edges in the pose graph, correct height (z) drift (`#4 <https://github.com/PRBonn/rko_slam/issues/4>`_)
* launch cleanup, no more autodetect in slam, docs cleaned up (`#3 <https://github.com/PRBonn/rko_slam/issues/3>`_)
* always work in the base frame, drop the lidar extrinsic parameter (`#2 <https://github.com/PRBonn/rko_slam/issues/2>`_)
* require rko_lio 0.4.2
* odom launch file, further autodetect and launch cleanup, ux cleanup of the launch files and base node
* change default submap split distance to 50m
* docs: overhaul (`#1 <https://github.com/PRBonn/rko_slam/issues/1>`_) and again, published and subscribed topics cleanup, readme and index edits
* Contributors: Meher Malladi

0.0.1 (2026-09-11)
------------------
* map closures can return a degenerate pose, need the quaternion form to
  normalize
* cleanup workflows a bit
* clarify rko_lio's dependency situation in the readme
* cleanup readme slightly
* initial commit, port over the entire codebase
* Contributors: Meher Malladi
