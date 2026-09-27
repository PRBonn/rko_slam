# Configuration

Every parameter is a launch argument. Set it on the command line as `name:=value`, or put any number of them in a YAML
file and pass that with `config_file:=`. Command line values override the file, and anything you leave unset keeps the
default listed here. Launch-only arguments go on the command line, and a config file that sets them is refused:
`config_file` and `log_level` everywhere, `rviz` and `rviz_config_file` for the two SLAM launch files, `mode` for
`slam.launch.py`, and `rko_lio_config_file` and `autodetect_timeout` for `odometry_and_slam.launch.py`.

```bash
ros2 launch rko_slam slam.launch.py -s   # the same list, with defaults, from the launch file itself
```

[`config/ros_default.yaml`](https://github.com/PRBonn/rko_slam/blob/master/config/ros_default.yaml) has every parameter
a config file takes, with its default, commented out, as a file to start from.

Set `lidar_topic`, which is required, and `imu_topic`; `odometry_and_slam.launch.py` sets both itself, see
[Running rko_lio with it](build_and_run.md#running-rko_lio-with-it). Everything else has a default that works as is, and
the ones worth changing first are `splitting_distance` and `overlap_threshold`.

## Mode

- **mode** (default `online`)

  `online` subscribes to live topics. `offline` drains a rosbag at full speed and also needs `bag_path`.

- **bag_path** (offline only, required)

  The rosbag2 directory to process.

- **odom_tum_path** (offline only)

  A TUM trajectory file to take the odometry from instead of the bag's `/tf`, see [Frames](build_and_run.md#frames).

- **use_sim_time** (`bool`, default `false`)

  Use the `/clock` topic, for bag replay with `--clock`.

## Input

- **lidar_topic** (required)

  The `PointCloud2` topic with the scans, taken as already deskewed unless you set `deskew`.
  `odometry_and_slam.launch.py` sets it to rko_lio's deskewed scan and refuses it as an argument.

- **base_frame** (optional)

  The frame rko_slam works in, usually `base_link`, see [Frames](build_and_run.md#frames). Under
  `odometry_and_slam.launch.py` it is shared with rko_lio: set it and rko_lio is started with it, leave it and rko_lio's
  is used. Unset under `slam.launch.py`, it is the scan's own frame.

- **odom_frame** (default `odom`), **map_frame** (default `map`)

  The odometry's frame and the frame rko_slam publishes, see [Frames](build_and_run.md#frames): rko_slam broadcasts
  `map_frame <- odom_frame`, inverted with `invert_map_tf`. Under `odometry_and_slam.launch.py`, `odom_frame` is shared
  with rko_lio the same way `base_frame` is.

- **invert_map_tf** (`bool`, default `false`)

  Publish `map_frame` as the TF child of `odom_frame`, with the inverted transform. Set it when the odometry publishes
  `odom_frame` as the child of `base_frame`, as rko_lio does with `invert_odom_tf`. `odometry_and_slam.launch.py` sets
  it to rko_lio's `invert_odom_tf` and refuses it as an argument.

- **deskew** (`bool`, default `false`)

  Deskew the scan in rko_slam, from its per-point timestamps and the odometry TF. Set it when the scans are raw, and
  leave it off for an already deskewed cloud such as `/rko_lio/deskewed_scan`. `odometry_and_slam.launch.py` sets it
  false and refuses it as an argument.

- **imu_topic**

  The `sensor_msgs/Imu` topic, the same IMU your odometry reads. Its accelerometer adds gravity edges that improve the
  map, see [Keeping the map level](how_it_works.md#keeping-the-map-level). Its frame must be connected to `base_frame`
  by a static transform on TF. `odometry_and_slam.launch.py` sets it to rko_lio's `imu_topic` and refuses it as an
  argument.

- **tf_lookup_timeout_ms** (`int`, online only, default `80`)

  How long a TF lookup blocks before the scan is dropped, both the odometry and the static
  `base_frame <- the scan's frame`.

- **lidar_timestamps.multiplier_to_seconds** (`float`, default `0.0`), **lidar_timestamps.force_absolute** (`bool`,
  default `false`), **lidar_timestamps.force_relative** (`bool`, default `false`)

  Used with `deskew:=true`. The multiplier scales per-point timestamps to seconds, `0.0` auto-detects seconds versus
  nanoseconds. The two flags force the timestamps to be read as absolute times, or as relative to the message stamp,
  when the auto-detection gets it wrong.

## Sub-maps

- **splitting_distance** (`float`, default `50.0`)

  Straight-line distance in metres from the sub-map's keypose at which the sub-map is closed and the next one starts.
  This decides how many sub-maps a run has, and with `no_of_sub_maps_to_skip` at 3 the first loop can close at the fifth
  sub-map. Smaller means more chances to catch a revisit, at the cost of more closure searches and a bigger graph.
  Across the environments I evaluated on: 100 m on long car sequences, 50 m on compact campus-scale ones, 30 m on short
  forest plots.

- **voxel_size** (`float`, default `0.5`), **max_points_per_voxel** (`int`, default `20`)

  The resolution a sub-map is kept at: voxel size in metres, and how many points a voxel keeps. Closure refinement and
  the overlap check see the sub-map at this resolution.

- **min_range** (`float`, default `1.0`), **max_range** (`float`, default `100.0`)

  Points closer or further than this many metres never enter a sub-map.

## Finding a revisit

These shape how a candidate revisit is found, see [How it works](how_it_works.md#finding-a-revisit), and can be left at
their defaults.

- **density_map_resolution** (`float`, default `0.5`), **density_threshold** (`float`, default `0.05`)

  The cell size in metres of the top-down image a sub-map is matched in, and how full a cell must be, against the
  fullest cell of the same sub-map, to show up in that image.

- **hamming_distance_threshold** (`int`, default `50`)

  How different two places may look and still be called a match, on a scale of 0 to 256: 1 accepts only identical ones.
  Lower is stricter.

- **inliers_threshold** (`int`, default `5`)

  How many matched features between the two sub-maps must agree on the same alignment for the candidate to be refined.

- **no_of_sub_maps_to_skip** (`int`, default `3`)

  How many of the most recent sub-maps to leave out of the search, so a sub-map does not match its own neighbours. It
  must be at least 1 within a run. Multi-session alignment sets it to 0, see below.

## Accepting a closure

- **overlap_threshold** (`float`, default `0.4`)

  After refinement, how much of the two sub-maps must overlap for the closure to be accepted: the voxels they share once
  aligned, over the voxel count of the smaller one, so 0 to 1. Raise it where many places look alike, lower it where
  revisits are missed. I used 0.4 on car and forest data, 0.5 on the Oxford Spires campus sequences, where 0.4 accepted
  false closures, and 0.3 when merging sessions that barely overlap.

## Pose graph

These can be left at their defaults.

- **rotation_info_scale** (`float`, default `25.0`)

  How much the optimizer trusts the rotation of an odometry or closure edge against its translation. It is the square of
  the range at which a rotation error and a translation error cost the same: the default corresponds to 5 m.

- **closure_info_scale** (`float`, default `1.0`)

  How much the optimizer trusts closures against odometry. Above 1 trusts closures more.

- **closure_kernel_delta** (`float`, default `1.0`)

  Closures that disagree with the rest of the graph by more than this many metres are given less weight.

- **gravity_info_scale** (`float`, default `100.0`)

  How much the optimizer trusts each sub-map's measured up direction against the rest of the graph. Used with an
  `imu_topic`, and in multi-session alignment of runs that had one.

- **max_iterations** (`int`, default `10`)

  The optimizer runs at most this many iterations at each split.

## Output

- **dump_results** (`bool`, default `false`)

  Write the run directory: trajectory, pose graph, config and the sub-maps, see [Outputs](build_and_run.md#outputs).

- **dump_sub_maps** (`bool`, default `true`)

  Also write each sub-map under `<run>/sub_maps/`, which multi-session alignment needs. Ignored when `dump_results` is
  off. Turn it off to save disk space on long runs.

- **results_dir** (default `results`), **run_name** (default `rko_slam` online, the bag's name offline)

  Where the run directory goes, and its name. A number is appended so nothing is overwritten.

## Visualization

- **publish_sub_maps**, **publish_closure_maps**, **publish_keypose_graph** (`bool`, default `false`)

  Publish each closed sub-map on `rko_slam/sub_maps` with a `sub_map_<i>` TF chain, each accepted closure pair as a
  two-tone cloud on `rko_slam/closure_maps`, and the keyposes with their odometry and closure edges as markers on
  `rko_slam/keypose_graph`. For what that looks like, see [Visualization](build_and_run.md#visualization).

- **rviz** (`bool`, default `false`), **rviz_config_file** (default `config/default.rviz`)

  Launch RViz alongside. With the default config, the launch file sets this run's frames in it and turns the three
  publishers above on. Under `odometry_and_slam.launch.py` it also shows rko_lio's deskewed scan and its local map; with
  your own `rko_lio_config_file`, the local map shows only if that file publishes one. Any other RViz config is passed
  through unchanged, with nothing forced on.

## odometry_and_slam.launch.py only

- **rko_lio_config_file**

  The YAML config the rko_lio node runs on, in the flat format of rko_lio's own `config_file`; what goes in it is in
  rko_lio's [configuration](https://prbonn.github.io/rko_lio/pages/config.html). It replaces rko_lio's
  [autodetection](https://prbonn.github.io/rko_lio/pages/ros.html#launch-parameter-autodetection), so it must set
  `lidar_topic`, `imu_topic` and, unless given on this launch file, `base_frame`. `base_frame`, `odom_frame` and
  `use_sim_time` set on the launch file override what it defines, `publish_deskewed_scan: true` is forced, and a file
  setting it false is refused.

- **autodetect_timeout** (`float`, default `10.0`)

  Seconds rko_lio's autodetection waits for each of the topics, their messages and the TF tree. Raise it when the robot
  or the bag needs longer to come up.

## Other

- **config_file**

  A YAML file with parameters from this page. Command line values override it.

- **log_level** (default `info`)

  ROS log level: `DEBUG`, `INFO`, `WARN`, `ERROR` or `FATAL`.

## Multi-session alignment

`align.launch.py` works offline on run directories and takes only the parameters named here. The sub-map parameters come
from each run's dumped config, and all runs must agree on `voxel_size` and `max_points_per_voxel`. It takes the
[Finding a revisit](#finding-a-revisit), [Accepting a closure](#accepting-a-closure) and [Pose graph](#pose-graph)
parameters, with the defaults above except where listed below, plus `config_file`, `log_level` and:

- **run_dirs** (required)

  The run directories to merge, at least two, as a list: `run_dirs:="[results/run_1, results/run_2]"`.

- **results_dir** (default `results`), **run_name** (default `aligned`)

  Where the merged run goes, and its name. A number is appended so nothing is overwritten.

- **no_of_sub_maps_to_skip** (`int`, default `0`)

  Zero here, since neighbouring sub-maps from different sessions are real candidates.

- **max_iterations** (`int`, default `100`)

  The optimizer runs at most this many iterations over the merged graph.
