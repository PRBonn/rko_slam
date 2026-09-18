# Configuration

Every parameter is a launch argument. Set it on the command line as `name:=value`, or put any number of them in
a YAML file and pass that with `config_file:=`. Command line values override the file, and anything you leave
unset keeps the default listed here.

```bash
ros2 launch rko_slam slam.launch.py -s   # the same list, with defaults, from the launch file itself
```

[`config/ros_default.yaml`](https://github.com/PRBonn/rko_slam/blob/master/config/ros_default.yaml) has every
parameter with its default, commented out, as a file to start from.

One of them is required and has no default, `lidar_topic`; with `odometry:=true` it and `base_frame` default
to rko_lio's, see [Starting rko_lio](build_and_run.md#starting-rko_lio). Everything else has a default that
works as is, and the ones worth changing first are `splitting_distance` and `overlap_threshold`.

## Mode

- **mode** (default `online`)

  `online` subscribes to live topics. `offline` drains a rosbag at full speed and also needs `bag_path`.

- **bag_path** (offline only, required)

  The rosbag2 directory to process.

- **odom_tum_path** (offline only)

  A TUM trajectory file to take the odometry from instead of the bag's `/tf`, see
  [Frames](build_and_run.md#frames).

- **use_sim_time** (`bool`, default `false`)

  Use the `/clock` topic, for bag replay with `--clock`.

## Input

- **lidar_topic** (required)

  The `PointCloud2` topic with the scans, taken as already deskewed unless you set `deskew`. With
  `odometry:=true` it defaults to rko_lio's deskewed scan.

- **base_frame** (optional)

  The frame rko_slam works in, usually `base_link`, see [Frames](build_and_run.md#frames). With
  `odometry:=true` it defaults to rko_lio's `base_frame`; left unset with no rko_lio to take it from, it is the
  scan's own frame.

- **odom_frame** (default `odom`), **map_frame** (default `map`)

  The odometry's frame and the frame rko_slam publishes, see [Frames](build_and_run.md#frames):
  rko_slam broadcasts `map_frame <- odom_frame`, inverted with `invert_map_tf`. With `odometry:=true`,
  `odom_frame` defaults to rko_lio's.

- **invert_map_tf** (`bool`, default `false`)

  Publish `map_frame` as the TF child of `odom_frame`, with the inverted transform. Set it when the odometry
  publishes `odom_frame` as the child of `base_frame`, as rko_lio does with `invert_odom_tf`. With `odometry:=true`
  it defaults to rko_lio's `invert_odom_tf`.

- **deskew** (`bool`, default `false`)

  Deskew the scan in rko_slam, from its per-point timestamps and the odometry TF. Leave it off when the odometry
  already publishes a deskewed cloud and you consume that, which is the case with `/rko_lio/deskewed_scan`.

- **tf_lookup_timeout_ms** (`int`, online only, default `80`)

  How long a TF lookup blocks before the scan is dropped, both the odometry and the static
  `base_frame <- the scan's frame`.

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

  Launch RViz alongside. With the default config, the launch file customises it for this run: your frames, plus
  rko_lio's deskewed scan when `odometry:=true` and its local map when rko_lio publishes one, and it turns the
  three publishers above on. Any other config is passed to RViz unchanged and you can configure the behaviour as
  desired.

## Spawning rko_lio

- **odometry** (`bool`, default `false`)

  Also start an rko_lio online node as the odometry, see [Starting rko_lio](build_and_run.md#starting-rko_lio).

- **rko_lio_config_file**

  The YAML config the rko_lio node runs on, as it is; what goes in it is in rko_lio's
  [configuration](https://prbonn.github.io/rko_lio/pages/config.html). It has to define
  `publish_deskewed_scan: true`, since that is the topic rko_slam subscribes to, and the launch stops if it does
  not. Without the file, rko_lio configures itself with its
  [autodetection](https://prbonn.github.io/rko_lio/pages/ros.html#launch-parameter-autodetection).

## Other

- **config_file**

  A YAML file with any of the parameters on this page. Command line values override it.

- **log_level** (default `info`)

  ROS log level: `DEBUG`, `INFO`, `WARN`, `ERROR` or `FATAL`.

## Multi-session alignment

`align.launch.py` reads the run directories and nothing live, so none of the input, mode, output or
visualization parameters above apply. The sub-map parameters come from each
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
