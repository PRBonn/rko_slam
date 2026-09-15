# Configuration

Every parameter is a launch argument and a node parameter by the same name. Set it on the command line as
`name:=value`, or put any number of them in a YAML file and pass that with `config_file:=`. Command line values
override the file, and anything you leave unset keeps the default listed here.

```bash
ros2 launch rko_slam slam.launch.py -s   # the same list, with defaults, from the launch file itself
```

[`config/ros_default.yaml`](https://github.com/PRBonn/rko_slam/blob/master/config/ros_default.yaml) has every
parameter with its default, commented out, as a file to start from.

Two of them are required and have no default, `lidar_topic` and `base_frame`, and both are
[autodetected](build_and_run.md#what-gets-autodetected) when you leave them unset. Everything else has a default
that works as is, and the ones worth changing first are `splitting_distance` and `overlap_threshold`.

## Mode

- **mode** (default `online`)

  `online` subscribes to live topics. `offline` drains a rosbag at full speed and also needs `bag_path`.

- **bag_path** (offline only, required)

  The rosbag2 directory to process.

- **odom_tum_path** (offline only)

  A TUM trajectory file to take the odometry from. When set, the bag's `/tf` is ignored and the file's poses are
  used as `odom -> base_frame` instead. The bag's `/tf_static` is still read, for the scan-to-`base_frame`
  extrinsic. If the file holds the LiDAR's own poses, set `base_frame` to the scan's frame.

- **use_sim_time** (`bool`, default `false`)

  Use the `/clock` topic, for bag replay with `--clock`.

## Input

- **lidar_topic** (required, autodetected)

  The `PointCloud2` topic with the scans. Deskewed scans unless you set `deskew`. With `odometry:=true` the launch
  file fills in `/rko_lio/deskewed_scan`.

- **base_frame** (required, autodetected)

  The body frame the odometry estimates and the scans are registered in, usually `base_link`.

- **odom_frame** (default `odom`), **map_frame** (default `map`)

  The frame the odometry publishes, as TF parent of `base_frame`, and the frame rko_slam publishes:
  rko_slam broadcasts `map_frame -> odom_frame`.

- **autodetect** (`bool`, default `true`), **autodetect_timeout** (`float`, default `10.0`)

  Fill in `lidar_topic` and `base_frame` when you did not give them, from the running graph or from the bag. The
  rules are under [What gets autodetected](build_and_run.md#what-gets-autodetected). The timeout is how long to
  wait for the topics and TF tree this needs, online only.

- **deskew** (`bool`, default `false`)

  Deskew the scan in rko_slam, from its per-point timestamps and the odometry TF. Leave it off when the odometry
  already publishes a deskewed cloud and you consume that, which is the case with `/rko_lio/deskewed_scan`.

- **base_T_lidar_qxyzw_xyz** (`[qx, qy, qz, qw, tx, ty, tz]`, optional)

  The LiDAR extrinsic. Left empty, it is resolved from TF on the first scan.

- **tf_lookup_timeout_ms** (`int`, online only, default `80`)

  How long an `odom -> base_frame` TF lookup blocks before the scan is dropped.

- **lidar_timestamps.multiplier_to_seconds** (`float`, default `0.0`), **lidar_timestamps.force_absolute**
  (`bool`, default `false`), **lidar_timestamps.force_relative** (`bool`, default `false`)

  Only relevant with `deskew:=true`. The multiplier scales per-point timestamps to seconds, `0.0` auto-detects
  seconds versus nanoseconds. The two flags force the timestamps to be read as absolute times, or as relative
  to the message stamp, when the auto-detection gets it wrong.

## Sub-maps

- **splitting_distance** (`float`, default `50.0`)

  Metres from the sub-map's keypose at which the sub-map is closed and the next one starts. Straight-line
  displacement, not distance travelled. This decides how many sub-maps, and so how many keyposes, a run has: a
  run with one sub-map can never close a loop. Smaller means more chances to catch a revisit, at the cost of a
  closure search per sub-map and a bigger graph. Across the environments I evaluated on: 100 m on long car
  sequences, 50 m on compact campus-scale ones, 30 m on short forest plots.

- **voxel_size** (`float`, default `0.5`), **max_points_per_voxel** (`int`, default `20`)

  The resolution a sub-map is kept at: voxel size in metres, and how many points a voxel keeps. Closure
  refinement and the overlap check see the sub-map at this resolution.

- **min_range** (`float`, default `1.0`), **max_range** (`float`, default `100.0`)

  Points closer or further than this many metres never enter a sub-map.

## Finding a revisit

These shape how a candidate revisit is found. They can be left at their defaults; the knob that decides what is
accepted is `overlap_threshold` below.

- **density_map_resolution** (`float`, default `0.5`), **density_threshold** (`float`, default `0.05`)

  Each sub-map is turned into a top-down density image, and places are recognized in that image. The cell size
  and how occupied a cell must be to count. Not the first knobs to reach for.

- **hamming_distance_threshold** (`int`, default `50`)

  How different two places may look and still be called a match. Lower is stricter. Not the first knob to reach
  for either.

- **inliers_threshold** (`int`, default `5`)

  How many matched points must agree before a candidate closure is checked properly, with the refinement below.

- **no_of_sub_maps_to_skip** (`int`, default `3`)

  How many of the most recent sub-maps to leave out of the search, so a sub-map does not match its own
  neighbours. Keep it at least 1 within a run. Multi-session alignment sets it to 0, see below.

## Accepting a closure

- **overlap_threshold** (`float`, default `0.4`)

  After refinement, how much of the two sub-maps must overlap for the closure to be accepted. The overlap
  coefficient is the voxels they share once aligned, over the voxel count of the smaller one, so 0 to 1. This is the one knob that decides what
  counts as a real closure. Raise it where many places look alike, lower it if you know revisits are being
  missed. I used 0.4 on car and forest data, 0.5 on the Oxford Spires campus sequences, where the detector produced
  false closures at 0.4, and 0.3 when merging sessions that barely overlap.

## Pose graph

These can be left at their defaults.

- **rotation_info_scale** (`float`, default `100.0`)

  Each pose-graph edge is weighted by a block-diagonal information matrix with isotropic translation and
  rotation blocks. This is the fixed multiple the rotation block is set to, accounting for the scale
  difference between the metre-scale translation and radian-scale rotation residuals.

- **closure_info_scale** (`float`, default `1.0`)

  How much the optimizer trusts closures against odometry. Above 1 trusts closures more.

- **closure_kernel_delta** (`float`, default `1.0`)

  Closures that disagree with the rest of the graph by more than this many metres are given less weight rather
  than believed.

- **max_iterations** (`int`, default `10`)

  Most optimizer iterations at each split.

## Output

- **dump_results** (`bool`, default `false`)

  Write the run directory: trajectory, pose graph, config, profile, and the sub-maps. See
  {doc}`Build and run, Outputs <build_and_run>` for what the files are.

- **dump_sub_maps** (`bool`, default `true`)

  Also write each sub-map under `<run>/sub_maps/`, which multi-session alignment needs. Ignored when
  `dump_results` is off. On a long run this adds up, at a rate the sub-map parameters set.

- **results_dir** (default `results`), **run_name** (default `rko_slam` online, the bag's name offline)

  Where the run directory goes, and its name. A number is appended so nothing is overwritten.

## Visualization

- **publish_sub_maps**, **publish_closure_maps**, **publish_keypose_graph** (`bool`, default `false`)

  Publish each closed sub-map on `rko_slam/sub_maps` with a `sub_map_<i>` TF chain, each accepted closure pair as a
  two-tone cloud on `rko_slam/closure_maps`, and the keyposes with their odometry and closure edges as markers
  on `rko_slam/keypose_graph`. For what that looks like, see
  [Visualization](build_and_run.md#visualization).

- **rviz** (`bool`, default `false`), **rviz_config_file** (default `config/default.rviz`)

  Launch RViz alongside. With the default config, the launch file patches in your frames, adds the odometry
  layers when `odometry:=true`, and turns the three publishers above on. Any other config is passed to RViz
  unchanged and you can configure the behaviour as desired.

## Spawning rko_lio

- **odometry** (`bool`, default `false`)

  Also start an rko_lio online node as the odometry. Its `publish_deskewed_scan` and `publish_local_map` are
  forced on and `lidar_topic` is pinned to its deskewed scan topic. Give `base_frame` as well, see
  [What gets autodetected](build_and_run.md#what-gets-autodetected).

- **rko_lio_config_file**, **rko_lio_lidar_topic**, **rko_lio_imu_topic**

  The YAML config, the raw LiDAR topic and the IMU topic for that rko_lio node. rko_lio autodetects its topics and
  frames as well when you leave them unset, see its
  [ROS docs](https://prbonn.github.io/rko_lio/pages/ros.html#launch-parameter-autodetection); what goes in the
  file is in its [configuration](https://prbonn.github.io/rko_lio/pages/config.html).

## Other

- **config_file**

  A YAML file with any of the parameters on this page. Command line values override it.

- **log_level** (default `info`)

  ROS log level: `DEBUG`, `INFO`, `WARN`, `ERROR` or `FATAL`.

## Multi-session alignment

`align.launch.py` reads the run directories and nothing live, so none of the input, mode, output or
visualization parameters above apply and there is nothing to autodetect. The sub-map parameters come from each
run's dumped config, and all runs must agree on `voxel_size` and `max_points_per_voxel`. What it takes is the
[Finding a revisit](#finding-a-revisit), [Accepting a closure](#accepting-a-closure) and
[Pose graph](#pose-graph) parameters with the same defaults, `config_file` and
`log_level`, and:

- **run_dirs** (required)

  The run directories to merge, at least two, as a list: `run_dirs:="[results/run_1, results/run_2]"`.

- **results_dir** (default `results`), **run_name** (default `aligned`)

  Where the merged run goes, and its name. A number is appended so nothing is overwritten.

- **no_of_sub_maps_to_skip** (`int`, default `0`)

  Zero here, since neighbouring sub-maps from different sessions are real candidates. Settable, like every
  parameter on this page.

- **max_iterations** (`int`, default `100`)

  Most optimizer iterations over the merged graph.
