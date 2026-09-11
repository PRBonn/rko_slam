import importlib.util
import tempfile
from pathlib import Path

import launch_ros.actions
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration

from launch import LaunchDescription


def load_module(path: Path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


common = load_module(Path(__file__).parent / "rko_slam_launch_common.py")

MODES = ("online", "offline")

offline_only_parameters = [
    {
        "name": "bag_path",
        "default": "",
        "modes": ("offline",),
        "description": "[offline] rosbag2 directory to process (required)",
        "required": True,
    },
    {
        "name": "odom_tum_path",
        "default": "",
        "modes": ("offline",),
        "description": "[offline] TUM trajectory used as the odometry source; when set the bag's /tf is ignored and odom->base is synthesized from the file",
    },
]

# Everything the rko_slam nodes declare_parameter().
configurable_parameters = [
    {
        "name": "lidar_topic",
        "default": "",
        "description": "Deskewed (or raw, see deskew) PointCloud2 input topic (required, unless a single one can be autodetected). With odometry:=true the launch fills in /rko_lio/deskewed_scan",
        "required": True,
        "autodetectable": True,
    },
    {
        "name": "base_frame",
        "default": "",
        "description": "Robot body frame the scans are deskewed and registered in (required, unless it can be autodetected from the TF tree)",
        "required": True,
        "autodetectable": True,
    },
    {
        "name": "odom_frame",
        "default": "odom",
        "description": "Odometry frame published by the upstream odometry (TF parent of base_frame)",
    },
    {
        "name": "map_frame",
        "default": "map",
        "description": "Map frame; rko_slam broadcasts map->odom",
    },
    {
        "name": "results_dir",
        "default": "results",
        "description": "Parent directory for run outputs (TUM, g2o, config, profile)",
    },
    {
        "name": "run_name",
        "default": "",
        "description": "Run folder prefix under results_dir; auto-incremented suffix prevents overwrites. Online default: rko_slam. Offline default: bag basename",
    },
    {
        "name": "dump_results",
        "default": "false",
        "type": "bool",
        "description": "Write the run directory: trajectory, pose graphs, config, profile and sub-maps",
    },
    {
        "name": "dump_sub_maps",
        "default": "true",
        "type": "bool",
        "description": "Also write each sub-map under <run>/sub_maps/, which align.launch.py needs. Ignored when dump_results is false",
    },
    {
        "name": "deskew",
        "default": "false",
        "type": "bool",
        "description": "Deskew the input scan here, from its per-point timestamps and the odom TF. Leave false for /rko_lio/deskewed_scan, which is already deskewed to scan-end time",
    },
    {
        "name": "base_T_lidar_qxyzw_xyz",
        "default": "",
        "type": "float_array",
        "description": "Optional lidar extrinsic override [qx,qy,qz,qw,tx,ty,tz]; empty = resolve via TF on first scan",
    },
    {
        "modes": ("online",),
        "name": "tf_lookup_timeout_ms",
        "default": "80",
        "type": "int",
        "description": "Blocking timeout for odom->base TF lookups",
    },
    {
        "name": "lidar_timestamps.multiplier_to_seconds",
        "default": "0.0",
        "type": "float",
        "description": "Per-point timestamp scale; 0.0 auto-detects seconds vs nanoseconds",
    },
    {
        "name": "lidar_timestamps.force_absolute",
        "default": "false",
        "type": "bool",
        "description": "Force per-point timestamps to be treated as absolute times",
    },
    {
        "name": "lidar_timestamps.force_relative",
        "default": "false",
        "type": "bool",
        "description": "Force per-point timestamps to be treated as relative to the message stamp",
    },
    {
        "name": "voxel_size",
        "default": "0.5",
        "type": "float",
        "description": "Sub-map voxel size (m)",
    },
    {
        "name": "splitting_distance",
        "default": "100.0",
        "type": "float",
        "description": "Distance (m) travelled before the current sub-map is closed and a new one started",
    },
    {
        "name": "max_points_per_voxel",
        "default": "20",
        "type": "int",
        "description": "Most points kept per voxel in the sub-map",
    },
    {
        "name": "min_range",
        "default": "1.0",
        "type": "float",
        "description": "Points closer than this (m) are filtered out",
    },
    {
        "name": "max_range",
        "default": "100.0",
        "type": "float",
        "description": "Points further than this (m) are filtered out",
    },
    *common.CLOSURE_PARAMETERS,
    *common.POSE_GRAPH_PARAMETERS,
    {
        "name": "no_of_sub_maps_to_skip",
        "default": "3",
        "type": "int",
        "description": "How many of the most recent sub-maps to ignore when looking for a closure, so a sub-map does not match its own neighbours. Must be >= 1",
    },
    {
        "name": "max_iterations",
        "default": "10",
        "type": "int",
        "description": "Most optimizer iterations to run at each split",
    },
    {
        "name": "publish_sub_maps",
        "default": "false",
        "type": "bool",
        "description": "Publish each closed sub-map on rko_slam/sub_maps, with a sub_map_<i> TF chain",
    },
    {
        "name": "publish_closure_maps",
        "default": "false",
        "type": "bool",
        "description": "Publish accepted closure pairs as a two-tone cloud on rko_slam/closure_maps",
    },
    {
        "name": "publish_keypose_graph",
        "default": "false",
        "type": "bool",
        "description": "Publish keyposes + odom/closure edges as markers on rko_slam/keypose_graph",
    },
    {
        "name": "use_sim_time",
        "default": "false",
        "type": "bool",
        "description": "Use the /clock topic (bag replay with --clock)",
    },
    *common.LAUNCH_PARAMETERS,
    {
        "launch_only": True,
        "name": "mode",
        "default": "online",
        "description": "Launch mode: 'online' (live topics) or 'offline' (self-drain a rosbag)",
    },
    {
        "launch_only": True,
        "name": "autodetect",
        "default": "true",
        "type": "bool",
        "description": "[EXPERIMENTAL] Fill in lidar_topic and base_frame from the running graph, or from the bag offline, when you did not give them. Anything you did give is used as is",
    },
    {
        "launch_only": True,
        "name": "autodetect_timeout",
        "default": "10.0",
        "type": "float",
        "description": "[EXPERIMENTAL] Seconds to wait for the topics and TF tree autodetect needs (online only)",
    },
    {
        "launch_only": True,
        "name": "rviz",
        "default": "false",
        "type": "bool",
        "description": "Launch RViz with the rko_slam default view",
    },
    {
        "launch_only": True,
        "name": "rviz_config_file",
        "default": "config/default.rviz",
        "description": "Path to the RViz config. The default one is patched for this run - frames, plus the odometry layers when the sidecar is on - and forces the SLAM publish flags on. Any other path is passed to rviz unchanged",
    },
    {
        "launch_only": True,
        "name": "odometry",
        "default": "false",
        "type": "bool",
        "description": "Also spawn an rko_lio online node as the odometry front-end. Forces its publish_deskewed_scan and publish_local_map on, and pins both topics",
    },
    {
        "launch_only": True,
        "name": "rko_lio_config_file",
        "default": "",
        "description": "[odometry] YAML config for the rko_lio node",
    },
    {
        "launch_only": True,
        "name": "rko_lio_lidar_topic",
        "default": "",
        "description": "[odometry] raw lidar topic for rko_lio",
    },
    {
        "launch_only": True,
        "name": "rko_lio_imu_topic",
        "default": "",
        "description": "[odometry] IMU topic for rko_lio",
    },
    *offline_only_parameters,
]

executable_for_mode = {"online": "online_node", "offline": "offline_node"}


def param_modes(param):
    return param.get("modes", MODES)


def applicable_parameters(mode: str) -> list:
    return [param for param in configurable_parameters if mode in param_modes(param) or param.get("launch_only")]


def validate_parameters(merged: dict, mode: str, odometry: bool) -> None:
    if mode not in MODES:
        common.fail(f"[ERROR] unknown mode '{mode}'. Valid: {' | '.join(MODES)}.")
    common.check_required(applicable_parameters(mode), merged)
    if odometry and mode != "online":
        common.fail("[ERROR] odometry:=true only makes sense with mode:=online")


# rko_lio's own defaults, for when its config does not name them.
RKO_LIO_SCAN_TOPIC = "rko_lio/deskewed_scan"
RKO_LIO_MAP_TOPIC = "rko_lio/local_map"


def rko_lio_config(context) -> dict:
    path = LaunchConfiguration("rko_lio_config_file").perform(context)
    if not path:
        return {}
    # rko_lio configs use the flat launch-file format, not the ros__parameters wrapper.
    with open(path) as f:
        return yaml.safe_load(f) or {}


def prepare_rviz_config(rviz_config_file: Path, map_frame: str, base_frame: str, odometry_layers: list) -> Path:
    """The shipped view is generic; whatever depends on this run's frames or options is set here."""
    with open(Path(get_package_share_directory("rko_slam")) / rviz_config_file) as f:
        rviz_cfg = yaml.safe_load(f)
    rviz_cfg["Visualization Manager"]["Global Options"]["Fixed Frame"] = map_frame
    rviz_cfg["Visualization Manager"]["Views"]["Current"]["Target Frame"] = base_frame
    if odometry_layers:
        # Achromatic on purpose
        rviz_cfg["Visualization Manager"]["Displays"].extend(
            {
                "Class": "rviz_default_plugins/PointCloud2",
                "Enabled": True,
                "Name": name,
                # rko_lio publishes both at SystemDefaults/keep_last(1)/volatile.
                "Topic": {"Value": topic, "Durability Policy": "Volatile", "Reliability Policy": "Reliable"},
                "Color Transformer": "FlatColor",
                "Color": color,
                # Flat Squares as the shipped displays use: the Points style sizes in pixels.
                "Size (m)": size_m,
                "Style": "Flat Squares",
            }
            for name, topic, color, size_m in odometry_layers
        )

    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".rviz", delete=False)  # noqa: SIM115
    yaml.safe_dump(rviz_cfg, tmp)
    tmp.flush()
    return Path(tmp.name)


def odometry_sidecar_parameters(context, final_params: dict, config: dict) -> list:
    """Not an include of rko_lio's launch file: it sniffs argv for "explicitly set" args and the two
    packages share a dozen argument names, so an include would leak slam's values into this config.
    """
    config_file = LaunchConfiguration("rko_lio_config_file").perform(context)
    lidar_topic = LaunchConfiguration("rko_lio_lidar_topic").perform(context)
    imu_topic = LaunchConfiguration("rko_lio_imu_topic").perform(context)

    base_frame = final_params.get("base_frame")
    if config.get("base_frame") and config["base_frame"] != base_frame:
        common.fail(
            "[ERROR] the two nodes would estimate different bodies:",
            f"  base_frame:={base_frame}",
            f"  base_frame in {config_file}: {config['base_frame']}",
            "Set them to the same frame, or drop it from the rko_lio config and let this one through.",
        )

    parameters = [config] if config else []
    overrides = {"publish_deskewed_scan": True, "base_frame": base_frame}
    if lidar_topic:
        overrides["lidar_topic"] = lidar_topic
    if imu_topic:
        overrides["imu_topic"] = imu_topic
    if "use_sim_time" in final_params:
        overrides["use_sim_time"] = final_params["use_sim_time"]
    parameters.append(overrides)
    print(
        f"+ rko_lio odometry sidecar (config={config_file or '<none>'}, "
        f"lidar={lidar_topic or '<from config>'}, imu={imu_topic or '<from config>'})"
    )
    return parameters


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context).lower()
    odometry = common.flag(context, "odometry")
    sidecar_config = rko_lio_config(context) if odometry else {}

    merged = common.merge(
        common.config_file_parameters(context), common.cli_parameters(context, configurable_parameters)
    )

    if odometry:
        merged.setdefault("lidar_topic", sidecar_config.get("deskewed_scan_topic", RKO_LIO_SCAN_TOPIC))

    if common.flag(context, "autodetect"):
        autodetect_module = load_module(Path(get_package_share_directory("rko_lio")) / "launch" / "autodetect.py")
        merged = autodetect_module.autodetect_or_exit(
            merged,
            mode=mode,
            bag_path=merged.get("bag_path"),
            timeout=float(LaunchConfiguration("autodetect_timeout").perform(context)),
            wanted=("lidar_topic", "base_frame"),
        )

    validate_parameters(merged, mode=mode, odometry=odometry)
    final_params = common.node_parameters(applicable_parameters(mode), merged)
    odom_params = odometry_sidecar_parameters(context, final_params, sidecar_config) if odometry else []

    rviz_enabled = common.flag(context, "rviz")
    if rviz_enabled:
        rviz_config_file = Path(LaunchConfiguration("rviz_config_file").perform(context))
        if str(rviz_config_file) == common.default_for(configurable_parameters, "rviz_config_file"):
            # the shipped view shows the SLAM products, so this run has to publish them
            final_params["publish_sub_maps"] = True
            final_params["publish_keypose_graph"] = True
            final_params["publish_closure_maps"] = True
            odometry_layers = []
            if odometry:
                # Same for the sidecar's own layers, on the topics it was configured to use.
                odom_params[-1]["publish_local_map"] = True
                map_topic = sidecar_config.get("map_topic", RKO_LIO_MAP_TOPIC)
                odometry_layers = [
                    ("OdomLocalMap", map_topic, "142; 149; 163", 0.07),
                    ("DeskewedScan", final_params["lidar_topic"], "255; 255; 255", 0.05),
                ]
            rviz_config_file = prepare_rviz_config(
                rviz_config_file,
                final_params.get("map_frame", common.default_for(configurable_parameters, "map_frame")),
                final_params["base_frame"],
                odometry_layers,
            )

    print("\n" + "=" * 40 + "\n")
    print(f"rko_slam launch configuration (mode={mode}):\n")
    print(yaml.dump(final_params, sort_keys=False, default_flow_style=False, indent=4))

    print("=" * 40 + "\n")

    log_level = LaunchConfiguration("log_level")
    nodes = [
        launch_ros.actions.Node(
            package="rko_slam",
            executable=executable_for_mode[mode],
            parameters=[final_params],
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
            emulate_tty=True,
            on_exit=lambda event, context: (
                [Shutdown(reason=f"{executable_for_mode[mode]} exited with code {event.returncode}")]
                if event.returncode
                else []
            ),
        )
    ]

    if odometry:
        nodes.append(
            launch_ros.actions.Node(
                package="rko_lio",
                executable="online_node",
                parameters=odom_params,
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                emulate_tty=True,
            )
        )

    if rviz_enabled:
        nodes.append(
            launch_ros.actions.Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_file.as_posix()],
                output="screen",
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            *common.declare_arguments(configurable_parameters),
            OpaqueFunction(function=launch_setup),
        ]
    )
