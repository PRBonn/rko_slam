<h1 align="center">rko_slam</h1>

<div align="center">

[![Jazzy](https://github.com/PRBonn/rko_slam/actions/workflows/ros_jazzy.yaml/badge.svg)](https://github.com/PRBonn/rko_slam/actions/workflows/ros_jazzy.yaml) [![Kilted](https://github.com/PRBonn/rko_slam/actions/workflows/ros_kilted.yaml/badge.svg)](https://github.com/PRBonn/rko_slam/actions/workflows/ros_kilted.yaml) [![Lyrical](https://github.com/PRBonn/rko_slam/actions/workflows/ros_lyrical.yaml/badge.svg)](https://github.com/PRBonn/rko_slam/actions/workflows/ros_lyrical.yaml) [![Rolling](https://github.com/PRBonn/rko_slam/actions/workflows/ros_rolling.yaml/badge.svg)](https://github.com/PRBonn/rko_slam/actions/workflows/ros_rolling.yaml)
[![GitHub License](https://img.shields.io/github/license/PRBonn/rko_slam)](/LICENSE) [![GitHub last commit](https://img.shields.io/github/last-commit/PRBonn/rko_slam)](/)

</div>

ROS2 LiDAR-inertial SLAM system built on [rko_lio](https://github.com/PRBonn/rko_lio): sub-map-based loop closing, pose-graph optimization and multi-session alignment.

## SLAM - tl;dr

If you already run rko_lio, you have an odometry estimate and no way to deal with odometry drift when you come back to
somewhere you have been before. Drive a long loop and the two ends of it will not meet. rko_slam sits next to
the odometry, recognizes the revisit, and corrects the whole trajectory behind you. You keep the odometry you
had, and you additionally get a `map -> odom` correction, a pose graph, and the sub-maps the system built.

## Multi-Session Alignment - tl;dr

The same revisit machinery works across runs or sessions, not just within one. Give it the run directories of several
sessions of the same place - different days, different directions, whatever - and it finds where they overlap
and solves all of them into one frame. No bags and no live topics, it only reads what the runs already dumped.

## Build

Supported distros: Jazzy, Kilted, Lyrical, Rolling.

For now, I require `rko_lio` to be built in the same workspace. In the near future, `apt` installs will be
supported, same as with rko_lio.

```bash
cd <ws>/src
git clone https://github.com/PRBonn/rko_lio # (required dependency)
git clone https://github.com/PRBonn/rko_slam
cd <ws> && rosdep install --from-paths src --ignore-src -y
colcon build --packages-select rko_lio rko_slam --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Please note, as of right now using `rko_lio` via `sudo apt install ros-<distro>-rko-lio` is not supported. Please clone master into your workspace as shown above.

Tests are off by default; add `-DRKO_SLAM_BUILD_TESTS=ON` to the cmake args and run `colcon test --packages-select rko_slam`.

### Dependencies and quirks

Three dependencies - g2o, MapClosures and the UTL profiler - are fetched and pinned by CMake, always;
nanoflann and Catch2 are fetched too under `-DRKO_SLAM_FETCH_CONTENT_DEPS=ON` and found on the system otherwise.
Sourcing Eigen, Sophus, spdlog and tsl-robin-map relies on rko_lio, however you configure that (check rko_lio's
[ROS docs](https://prbonn.github.io/rko_lio/pages/ros.html)). Everything else resolves via rosdep.

Please note: rko_slam is MIT, but the g2o Cholmod solver links SuiteSparse's CHOLMOD, whose Ubuntu/Debian build
carries GPL-2+ modules.

Once again, steps are planned to cleanup the dependency requirements and support pure rosdep installs. Stay tuned.

## Usage

Two entrypoints: `slam.launch.py` (`mode:=online|offline`) and `align.launch.py`. `-s` lists every parameter
with its documentation, and anything you leave unset keeps the node's own default.

### SLAM

```bash
ros2 launch rko_slam slam.launch.py -s
```

Online, next to a running rko_lio odometry node:

```bash
ros2 launch rko_slam slam.launch.py \
  lidar_topic:=/rko_lio/deskewed_scan base_frame:=base_link rviz:=true
```

Online, spawning the rko_lio front-end too (`publish_deskewed_scan` for rko lio is forced on;
the slam side defaults to consuming `/rko_lio/deskewed_scan`):

```bash
ros2 launch rko_slam slam.launch.py odometry:=true \
  rko_lio_config_file:=my_rko_lio.yaml \
  rko_lio_lidar_topic:=/os_cloud_node/points rko_lio_imu_topic:=/os_cloud_node/imu \
  base_frame:=base_link
```

rko_lio is the default, not a requirement. Anything that publishes `odom -> base_frame` on TF works - point
`lidar_topic` at your scans and `base_frame` at the body frame your odometry estimates in. If that odometry does not
hand you deskewed scans, set `deskew:=true` (motion compensation) and rko_slam will deskew them itself using the same TF:

```bash
ros2 launch rko_slam slam.launch.py \
  lidar_topic:=/os_cloud_node/points base_frame:=base_link \
  odom_frame:=odom deskew:=true
```

Offline, self-draining a bag (every scan is processed faster than frame-rate if possible). The odometry comes
from the bag's own `/tf`:

```bash
ros2 launch rko_slam slam.launch.py mode:=offline \
  bag_path:=/data/my_bag \
  results_dir:=results run_name:=my_run dump_results:=true
```

If the odometry you want is not in the bag, `odom_tum_path` takes it from a TUM trajectory file instead, and
the bag's `/tf` is then ignored:

```bash
ros2 launch rko_slam slam.launch.py mode:=offline \
  bag_path:=/data/my_bag odom_tum_path:=/data/my_odometry_tum.txt \
  results_dir:=results run_name:=my_run dump_results:=true
```

`config_file:=<path>` takes a YAML file with any of these parameters.
`config/ros_default.yaml` lists every parameter and its default as an example.

### Multi-session alignment

`align.launch.py` merges the run directories of several sessions into one pose graph, using the same closure detector and refinement across sessions.
It reads only the dumped artifacts (no bags, no topics); the session reaching the most others defines the joint frame, and every run dir must carry its sub-maps, i.e. have been run with `dump_results:=true dump_sub_maps:=true`:

```bash
ros2 launch rko_slam align.launch.py \
  run_dirs:="[results/run_1, results/run_2]" \
  results_dir:=results run_name:=merged
```

## Outputs

Nothing is written unless you ask for it. With `dump_results:=true`, a run writes `<results_dir>/<run_name>_<n>/`:

| File | What |
|---|---|
| `*_tum.txt` | the trajectory |
| `*_keypose_graph.g2o` | the keypose pose graph |
| `*_config.yaml` | the config dumped for reproducibility |
| `*_profile.txt` | Profiling logs |
| `*_trajectory.png` | the trajectory as an image, loop closures in red |
| `sub_maps/sub_map_*.ply` | sub-maps by the system, can be used as a map for a localization system for example. The input to `align.launch.py` (`dump_sub_maps`, on by default) |

## The system

### TL;DR
rko_slam consumes the deskewed scan stream and odometry TF of a running odometry (rko_lio, or anything publishing `odom -> base` and optionally deskewed scans) and publishes a `map -> odom` correction. The pipeline is essentially a reimplementation of [KISS-SLAM](https://github.com/PRBonn/kiss-slam), uses [MapClosures](https://github.com/PRBonn/MapClosures) for detecting revisits, and a g2o pose graph.

Multi-session alignment runs the same detector and the same refinement, except the two sub-maps being compared
come from different runs. It reads the sub-maps and keypose graphs that those runs dumped, puts them all through
one detector and keeps the matches that cross sessions, picks the session that reaches the most others as the
gauge/reference, and solves one joint graph over all of them. A session nothing reaches is dropped, with a warning. The output is one pose graph and one frame that every session is expressed in.

### The details

Scans come in, get placed using the `odom -> base_frame` TF at their timestamp, and are integrated into a
voxel-hashed sub-map held in the frame of the keypose that started it. Points closer than `min_range` or
further than `max_range` never enter. The voxel grid is what keeps the sub-map bounded: `voxel_size` sets the
cell, `max_points_per_voxel` caps how many points a cell keeps.

A sub-map closes when the sensor is `splitting_distance` away from its keypose, and the next one opens where
it closed. This is straight-line displacement from the keypose, not distance travelled - a 700 m loop that
never gets more than 80 m from where it started is one sub-map, not seven.

Every closed sub-map becomes a vertex in the pose graph, with its odometry pose as the estimate, and an edge to
the previous keypose carrying the relative motion the odometry measured. That chain is the odometry, just
sampled at keyposes instead of at scans.

The closed sub-map also gets processed by a closure thread, which is where the actual SLAM happens, and which runs
separately so the scan stream keeps moving while it works. MapClosures renders the sub-map as a top-down density image
(`density_map_resolution` is its cell size, `density_threshold` decides when a cell counts as occupied),
describes it using ORB features, and looks for past sub-maps that look the same. `hamming_distance_threshold` is how different two
descriptors may be and still be called a match, and `no_of_sub_maps_to_skip` keeps the most recent sub-maps out
of the search so a sub-map does not match its own neighbours. What comes back is a first guess at the transform
between the two, and `inliers_threshold` is how much agreement that guess needs before it is worth checking
properly.

Checking it properly means point-to-plane ICP between the two sub-maps' voxel centroids, followed by measuring
how much of the two overlaps once aligned. That fraction is what `overlap_threshold` gates.

An accepted closure becomes an edge with a Cauchy robust kernel on it, and the graph is re-optimized (Dogleg, Cholmod) right there. The keyposes
move, the trajectory is rebuilt from them, and the difference between where the odometry thinks you are and
where the graph now says you are is published as `map -> odom`.

### Configuration

Everything below is a launch argument and a node parameter by the same name. `-s` prints this list too.

**Sub-maps**

- `splitting_distance` (100 m) is the one to change first: it decides how many sub-maps a run has, and a run
  with one sub-map can never close anything. Smaller means more keyposes, so more chances to catch a revisit,
  at the cost of a closure search per sub-map and a bigger graph to solve.
- `voxel_size` (0.5 m) and `max_points_per_voxel` (20) set the resolution the sub-map is kept at, and that
  resolution is what ICP and the overlap check see.
- `min_range` (1 m) and `max_range` (100 m) cut the scan before any of it.

**Finding a revisit**

- `density_map_resolution` (0.5 m) and `density_threshold` (0.05) shape the top-down image places are
  recognized in.
- `hamming_distance_threshold` (50) is how close two places have to look to be matched at all - lower is
  stricter.
- `inliers_threshold` (5) is how much of that match has to agree before the expensive check runs on it.
- `no_of_sub_maps_to_skip` (3) must be at least 1; adjacent sub-maps are not loop closures.

**Accepting a closure**

- `overlap_threshold` (0.4) is the one knob that decides what is a real closure. Raise it where many places
  look alike, lower it if you know revisits are being missed.

**The graph**

- `rotation_info_scale` (100) is how much more the optimizer trusts rotation than translation.
- `closure_info_scale` (1.0) trades closures against odometry - above 1 believes closures more.
- `closure_kernel_delta` (1.0 m) is where the robust kernel starts discounting a closure instead of believing
  it.
- `max_iterations` (10) caps the optimization done at each split.

**Visualization**

Off by default.

- `publish_sub_maps:=true` publishes each closed sub-map on `rko_slam/sub_maps`, along with a `sub_map_<i>`
  TF chain.
- `publish_keypose_graph:=true` publishes the keyposes and both kinds of edge as markers.
- `publish_closure_maps:=true` publishes each accepted closure pair as a two-tone cloud, so you can see what
  got matched to what.
- `rviz:=true` turns all three on and opens a view that already displays them.

## Acknowledgments and Citation

This work is essentially a reimplementation of [KISS-SLAM](https://github.com/PRBonn/kiss-slam) but for ROS2.
And relies on my lidar inertial odometry package [rko_lio](https://github.com/PRBonn/rko_lio) for much of the
internals. This was developed as part of my thesis work.

If you find this package useful, consider leaving a star ⭐ on KISS-SLAM and citing the original publication:
```bib
@INPROCEEDINGS{kiss2025iros,
  author    = {Guadagnino, Tiziano and Mersch, Benedikt and Gupta, Saurabh and Vizzo, Ignacio and Grisetti, Giorgio and Stachniss, Cyrill},
  booktitle = {2025 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  title     = {{KISS-SLAM: A Simple, Robust, and Accurate 3D LiDAR SLAM System With Enhanced Generalization Capabilities}},
  year      = {2025},
  pages     = {5363-5370},
  doi       = {10.1109/IROS60139.2025.11246613}
}
```

If you found the default odometry, i.e., rko_lio useful, consider leaving a star ⭐ there and citing the corresponding publication:
```bib
@article{malladi2026ral,
  author      = {M.V.R. Malladi and T. Guadagnino and L. Lobefaro and C. Stachniss},
  title       = {A Robust Approach for LiDAR-Inertial Odometry Without Sensor-Specific Modeling},
  journal     = {IEEE Robotics and Automation Letters},
  year        = {2026},
  volume      = {11},
  number      = {6},
  pages       = {7420--7427},
  doi         = {10.1109/LRA.2026.3685966},
}
```
