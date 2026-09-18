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


common = load_module(Path(__file__).parent / "common.py")

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
        "description": "[offline] TUM trajectory of base_frame in odom_frame, used as the odometry in place of the bag's /tf",
    },
]

# Everything the rko_slam nodes declare_parameter().
configurable_parameters = [
    {
        "name": "lidar_topic",
        "default": "",
        "description": "PointCloud2 input topic (required), taken as already deskewed unless you set deskew",
        "required": True,
    },
    {
        "name": "base_frame",
        "default": "",
        "description": "Frame rko_slam works in, e.g. base_link; unset, the scan's frame",
    },
    {
        "name": "odom_frame",
        "default": "odom",
        "description": "Odometry frame published by the upstream odometry",
    },
    {
        "name": "map_frame",
        "default": "map",
        "description": "Map frame; rko_slam broadcasts map<-odom",
    },
    {
        "name": "invert_map_tf",
        "default": "false",
        "type": "bool",
        "description": "Invert the map transform so that the odom frame is the parent and the map frame is the child in the TF tree, for an odometry that publishes the odom frame as the child of the base frame",
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
        "modes": ("online",),
        "name": "tf_lookup_timeout_ms",
        "default": "80",
        "type": "int",
        "description": "Blocking timeout for odom_frame <- base_frame TF lookups",
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
        "default": "50.0",
        "type": "float",
        "description": "Straight-line distance (m) from the sub-map's keypose at which it is closed and the next one started",
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
        "name": "rviz",
        "default": "false",
        "type": "bool",
        "description": "Launch RViz with the rko_slam default view",
    },
    {
        "launch_only": True,
        "name": "rviz_config_file",
        "default": "config/default.rviz",
        "description": "Path to the RViz config. The default one is customised with this run's frames and forces the SLAM publish flags on. Any other path is passed to rviz unchanged",
    },
    *offline_only_parameters,
]

executable_for_mode = {"online": "online_node", "offline": "offline_node"}


def validate_mode(mode: str) -> None:
    if mode not in MODES:
        common.fail(f"[ERROR] unknown mode '{mode}'. Valid: {' | '.join(MODES)}.")


def applicable_parameters(table: list, mode: str) -> list:
    return [param for param in table if mode in param.get("modes", MODES) or param.get("launch_only")]


def using_default_rviz(context, table) -> bool:
    return common.flag(context, "rviz") and LaunchConfiguration("rviz_config_file").perform(
        context
    ) == common.default_for(table, "rviz_config_file")


def prepare_rviz_config(rviz_config_file: Path, map_frame: str, base_frame: str | None, extra_displays: list) -> Path:
    """The shipped view is generic; whatever depends on this run's frames or options is set here."""
    with open(Path(get_package_share_directory("rko_slam")) / rviz_config_file) as f:
        rviz_cfg = yaml.safe_load(f)
    rviz_cfg["Visualization Manager"]["Global Options"]["Fixed Frame"] = map_frame
    if base_frame:
        rviz_cfg["Visualization Manager"]["Views"]["Current"]["Target Frame"] = base_frame
    rviz_cfg["Visualization Manager"]["Displays"].extend(extra_displays)

    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".rviz", delete=False)  # noqa: SIM115
    yaml.safe_dump(rviz_cfg, tmp)
    tmp.flush()
    return Path(tmp.name)


def slam_nodes(context, table: list, mode: str, slam_params: dict, extra_displays: list) -> list:
    rviz_enabled = common.flag(context, "rviz")
    rviz_config_file = Path(LaunchConfiguration("rviz_config_file").perform(context))
    default_rviz = using_default_rviz(context, table)

    applicable = applicable_parameters(table, mode)
    common.check_required(applicable, slam_params)
    final_params = common.node_parameters(applicable, slam_params)

    if default_rviz:
        final_params["publish_sub_maps"] = True
        final_params["publish_keypose_graph"] = True
        final_params["publish_closure_maps"] = True
        rviz_config_file = prepare_rviz_config(
            rviz_config_file,
            final_params.get("map_frame", common.default_for(table, "map_frame")),
            final_params.get("base_frame"),
            extra_displays,
        )

    print("\n" + "=" * 40 + "\n")
    print(f"rko_slam launch configuration (mode={mode}):\n")
    print(yaml.dump(final_params, sort_keys=False, default_flow_style=False, indent=4))
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

    print("=" * 40 + "\n")

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


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context).lower()
    validate_mode(mode)
    merged = common.merge(
        common.config_file_parameters(context), common.cli_parameters(context, configurable_parameters)
    )
    return slam_nodes(context, configurable_parameters, mode, merged, extra_displays=[])


def generate_launch_description():
    return LaunchDescription(
        [
            *common.declare_arguments(configurable_parameters),
            OpaqueFunction(function=launch_setup),
        ]
    )
