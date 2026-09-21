# Build and run

Supported distros: Jazzy, Kilted, Lyrical, Rolling.

## Build

rko_slam depends on [rko_lio](https://github.com/PRBonn/rko_lio) at build time, and for now both have to be built in the
same workspace.

```bash
cd <ws>/src
git clone https://github.com/PRBonn/rko_lio
git clone https://github.com/PRBonn/rko_slam
cd <ws> && rosdep install --from-paths src --ignore-src -y
colcon build --packages-select rko_lio rko_slam
```

Please note, as of right now using `rko_lio` via `sudo apt install ros-<distro>-rko-lio` is not supported. Please clone
master into your workspace as shown above. `apt` installs of both will be supported, same as with rko_lio today.

Tests are off by default; add `-DRKO_SLAM_BUILD_TESTS=ON` to the cmake args and run
`colcon test --packages-select rko_slam`.

### Dependencies and quirks

Three dependencies - g2o, MapClosures and the UTL profiler - are fetched and pinned by CMake, always; nanoflann and
Catch2 are fetched too under `-DRKO_SLAM_FETCH_CONTENT_DEPS=ON` and found on the system otherwise. Sourcing Eigen,
Sophus, spdlog and tsl-robin-map relies on rko_lio, however you configure that (check rko_lio's
[ROS docs](https://prbonn.github.io/rko_lio/pages/ros.html)). Everything else resolves via rosdep.

Please note: rko_slam is MIT, but the g2o Cholmod solver links SuiteSparse's CHOLMOD, whose Ubuntu/Debian build carries
GPL-2+ modules.

Steps are planned to clean up the dependency requirements and support pure rosdep installs.

## Run

Three entrypoints: `slam.launch.py` (`mode:=online|offline`), `odometry_and_slam.launch.py`, and `align.launch.py`. `-s`
lists every parameter with its documentation, and anything you leave unset keeps the node's own default. What each
parameter does is on the {doc}`Configuration <configuration>` page.

```bash
ros2 launch rko_slam slam.launch.py -s
```

### Online

Next to a running rko_lio, give rko_slam rko_lio's deskewed scan and the IMU rko_lio reads:

```bash
ros2 launch rko_slam slam.launch.py lidar_topic:=/rko_lio/deskewed_scan imu_topic:=/your/imu
```

The IMU levels the map: each sub-map gets a measured up direction and the pose graph pulls it onto gravity, see
{doc}`How it works <how_it_works>`. The IMU's frame has to be connected to `base_frame` by a static transform on TF.
Leaving `imu_topic` unset still runs, without the gravity edges, which is a suboptimal way to run rko_slam.

Add `rviz:=true` to open RViz with the default view: sub-maps, keypose graph and closures.

To run the odometry too, use the other entrypoint, see [Running rko_lio with it](#running-rko_lio-with-it):

```bash
ros2 launch rko_slam odometry_and_slam.launch.py
```

### The odometry

rko_lio is the default, not a requirement. rko_slam asks two things of the odometry: that its pose is on TF, as
described under [Frames](#frames), and that it is locally consistent, meaning the motion between two nearby scans is
right even if the whole trajectory drifts. The loop closing corrects the drift; it does not repair a jump. Any LiDAR
odometry qualifies. In my thesis the same back-end ran unchanged on top of
[Kinematic-ICP](https://github.com/PRBonn/kinematic-icp), which fuses a LiDAR with wheel odometry on a wheeled robot.

With another odometry the scans are usually raw, so set `deskew:=true` and rko_slam deskews them itself with the same
TF:

```bash
ros2 launch rko_slam slam.launch.py lidar_topic:=/points base_frame:=base_link deskew:=true
```

If its odometry frame is not `odom`, give `odom_frame` as well. If it publishes `odom_frame` as the TF child of
`base_frame`, set `invert_map_tf:=true`, and rko_slam publishes `map_frame` as the child of `odom_frame`.

### Offline

Offline, the node drains the bag as fast as it can process it, never waiting on message timestamps. The odometry comes
from the bag's own `/tf`:

```bash
ros2 launch rko_slam slam.launch.py mode:=offline bag_path:=/data/my_bag \
  lidar_topic:=/rko_lio/deskewed_scan
```

If the odometry you want is not in the bag, `odom_tum_path` takes it from a TUM trajectory file instead, see
[Frames](#frames):

```bash
ros2 launch rko_slam slam.launch.py mode:=offline bag_path:=/data/my_bag \
  lidar_topic:=/points base_frame:=base_link deskew:=true odom_tum_path:=/data/odometry_tum.txt
```

Nothing is written unless you ask for it, see [Outputs](#outputs):

```bash
ros2 launch rko_slam slam.launch.py mode:=offline bag_path:=/data/my_bag \
  lidar_topic:=/rko_lio/deskewed_scan \
  dump_results:=true results_dir:=results run_name:=my_run
```

### Multi-session alignment

`align.launch.py` is an offline step: it merges the run directories of several runs of the same place into one frame.
Every run directory must carry its sub-maps, i.e. have been run with `dump_results:=true` (sub-maps are dumped by
default):

```bash
ros2 launch rko_slam align.launch.py run_dirs:="[results/run_1, results/run_2]"
```

### Running rko_lio with it

`odometry_and_slam.launch.py` starts an rko_lio online node next to rko_slam and wires the two together. It is online
only.

It sets these, and passing them is an error:

| parameter                         | value                                                             |
| --------------------------------- | ----------------------------------------------------------------- |
| rko_lio's `publish_deskewed_scan` | true, it is what rko_slam subscribes to                           |
| rko_slam's `lidar_topic`          | rko_lio's deskewed scan topic                                     |
| rko_slam's `imu_topic`            | rko_lio's `imu_topic`, the IMU rko_lio reads                      |
| rko_slam's `deskew`               | false, that scan is already deskewed                              |
| rko_slam's `invert_map_tf`        | rko_lio's `invert_odom_tf`, so both publish in the same direction |

`base_frame`, `odom_frame` and `use_sim_time` are set once and passed to both nodes, so the two cannot disagree.
Everything else is rko_slam's, exactly as under `slam.launch.py`.

rko_lio's own settings, its raw `lidar_topic` and `imu_topic` among them, come from `rko_lio_config_file`. The file is
used as it is, except for the values in the table above and the three shared ones, which override it. A file that sets
`publish_deskewed_scan: false` is refused. Without the file, rko_lio configures itself with its
[autodetection](https://prbonn.github.io/rko_lio/pages/ros.html#launch-parameter-autodetection), waiting for the topics
and TF to show up.

For anything this does not allow, run rko_lio with its own launch file and rko_slam with `slam.launch.py`.

## Frames

rko_slam works in one frame, `base_frame`: the sub-maps, keyposes and trajectory are expressed in it. Left unset, it is
the scan's frame, the `frame_id` of the `PointCloud2`.

The odometry is the pose of `base_frame` in `odom_frame`, read from TF at each scan's timestamp. A static transform on
TF connects `base_frame` to the scan's frame, and rko_slam reads it once, on the first scan, to transform every scan
into `base_frame` before adding it to the sub-maps. With an IMU, another static transform connects `base_frame` to the
IMU's frame, read once on the first IMU message after the first scan.

Offline, `odom_tum_path` takes the odometry from a TUM file instead of the bag's `/tf`. The file holds the pose of
`base_frame` in `odom_frame`.

rko_slam publishes `map_frame <- odom_frame`, which makes `map_frame <- base_frame` the SLAM estimate.

## Topics and transforms

Subscribed:

| Topic / transform                         | What                                                                         |
| ----------------------------------------- | ---------------------------------------------------------------------------- |
| `lidar_topic` (`sensor_msgs/PointCloud2`) | the scans, taken as already deskewed unless you set `deskew:=true`           |
| `imu_topic` (`sensor_msgs/Imu`)           | the IMU whose accelerometer levels the map, the same one your odometry reads |
| `odom_frame <- base_frame` on TF          | the odometry, looked up at each scan's timestamp                             |
| `base_frame <- the scan's frame` on TF    | static, read once on the first scan to bring every scan into `base_frame`    |
| `base_frame <- the IMU's frame` on TF     | static, read once on the first IMU message                                   |

Published:

| Topic / transform                             | What                                                                                            |
| --------------------------------------------- | ----------------------------------------------------------------------------------------------- |
| `map_frame <- odom_frame` on TF               | the correction, flipped by `invert_map_tf`; `map_frame <- base_frame` is then the SLAM estimate |
| `rko_slam/sub_maps` (`PointCloud2`)           | each closed sub-map, with a `sub_map_<i>` TF chain (`publish_sub_maps:=true`)                   |
| `rko_slam/keypose_graph` (`MarkerArray`)      | the keyposes with their odometry and closure edges (`publish_keypose_graph:=true`)              |
| `rko_slam/closure_maps` (`PointCloud2`)       | each accepted closure pair as a two-tone cloud (`publish_closure_maps:=true`)                   |
| `rko_slam/bag_progress` (`Float32MultiArray`) | offline only, how far into the bag the node is                                                  |

## Visualization

Nothing is published for display unless you ask for it. Three publishers, each off by default:

- `publish_sub_maps:=true`, each closed sub-map as a point cloud on `rko_slam/sub_maps`, with a `sub_map_<i>` TF chain
  so RViz places them.
- `publish_keypose_graph:=true`, the keyposes as spheres with the odometry edges between consecutive keyposes and the
  closure edges, as markers on `rko_slam/keypose_graph`.
- `publish_closure_maps:=true`, each accepted closure as the two sub-maps it matched, one colour each, on
  `rko_slam/closure_maps`, so you can see what got matched to what.

`rviz:=true` turns all three on and opens RViz with the default view, `config/default.rviz`, set to your frames. Under
`odometry_and_slam.launch.py` it also shows rko_lio's deskewed scan and its local map; with your own
`rko_lio_config_file`, the local map shows only if that file publishes one. Pass your own config with
`rviz_config_file:=` and it is used unchanged, with nothing forced on.

<figure>
  <img src="../_static/img/rviz_default_view.png" alt="RViz with the default view on an Oxford Spires sequence: sub-maps, the keypose graph with odometry and closure edges, and a two-tone closure map">
  <figcaption class="pair-caption">The default view at the end of an Oxford Spires sequence: sub-maps, the keypose graph with its odometry and closure edges, and one closure map in two tones.</figcaption>
</figure>

## Outputs

Nothing is written unless you ask for it. With `dump_results:=true`, a SLAM run (`slam.launch.py`) writes
`<results_dir>/<run_name>_<n>/`:

| File                     | What                                                                                                                                                        |
| ------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `*_tum.txt`              | the trajectory                                                                                                                                              |
| `*_keypose_graph.g2o`    | the keypose pose graph                                                                                                                                      |
| `*_config.yaml`          | the config the run used                                                                                                                                     |
| `*_profile.txt`          | profiling logs                                                                                                                                              |
| `*_trajectory.png`       | the trajectory as an image, loop closures in red                                                                                                            |
| `sub_maps/sub_map_*.ply` | the sub-maps, in the frame of their keypose. Usable as a map for a localization system, and the input to `align.launch.py` (`dump_sub_maps`, on by default) |

`align.launch.py` always writes `<results_dir>/<run_name>_<n>/`, and needs the runs it merges to have been written with
`dump_results:=true`:

| File                        | What                                                                    |
| --------------------------- | ----------------------------------------------------------------------- |
| `*_joint_keypose_graph.g2o` | the joint pose graph over all sessions                                  |
| `*_session_<i>_tum.txt`     | each session's trajectory in the joint frame, `<i>` in `run_dirs` order |
| `*_config.yaml`             | the config the run used, with the run directories                       |
