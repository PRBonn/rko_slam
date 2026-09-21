import importlib.util
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch import LaunchDescription


def load_module(path: Path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


here = Path(__file__).parent
common = load_module(here / "common.py")
slam = load_module(here / "slam.launch.py")
rko_lio_launch = Path(get_package_share_directory("rko_lio")) / "launch"
rko_lio_odometry = load_module(rko_lio_launch / "odometry.launch.py")

FORCED = {
    "lidar_topic": "rko_lio's deskewed scan topic",
    "imu_topic": "rko_lio's imu_topic",
    "deskew": "false, since that scan is already deskewed",
    "invert_map_tf": "rko_lio's invert_odom_tf",
}
SHARED = ("base_frame", "odom_frame", "use_sim_time")


def forced_parameter(param: dict) -> dict:
    return {
        **param,
        "description": f"Set by this launch file to {FORCED[param['name']]}; passing it is an error",
        "required": False,
    }


configurable_parameters = [
    *[
        forced_parameter(param) if param["name"] in FORCED else param
        for param in slam.applicable_parameters(slam.configurable_parameters, "online")
        if param["name"] != "mode"
    ],
    {
        "launch_only": True,
        "name": "rko_lio_config_file",
        "default": "",
        "description": "YAML config for the rko_lio settings this launch file does not set. Without it, rko_lio autodetects its topics and frames",
    },
]


def rko_lio_value(odom_params: dict, name: str):
    table = rko_lio_odometry.configurable_parameters
    if name in odom_params:
        return odom_params[name]
    # rko_lio declares its defaults as launch argument strings, so "false" needs casting before it is read
    return common.cast(next(param for param in table if param["name"] == name), common.default_for(table, name))


def rko_lio_parameters(context, shared: dict, using_default_rviz: bool) -> dict:
    if config_file := LaunchConfiguration("rko_lio_config_file").perform(context):
        # rko_lio configs use the flat launch-file format, not the ros__parameters wrapper.
        with open(config_file) as f:
            odom_params = yaml.safe_load(f) or {}
        if odom_params.get("publish_deskewed_scan") is False:
            common.fail(
                "[ERROR] rko_lio_config_file sets publish_deskewed_scan: false, which rko_slam subscribes to.",
                "Drop it, or run rko_lio and slam.launch.py yourself.",
            )
        return {**odom_params, **shared, "publish_deskewed_scan": True}

    autodetect = load_module(rko_lio_launch / "autodetect.py")
    timeout = common.default_for(rko_lio_odometry.configurable_parameters, "autodetect_timeout")
    try:
        found = autodetect.autodetect(dict(shared), mode="online", bag_path=None, timeout=float(timeout))
    except autodetect.AutodetectError as error:
        common.fail(
            "[ERROR] rko_lio autodetect failed:",
            str(error),
            "Pass rko_lio_config_file, or run rko_lio and slam.launch.py yourself.",
        )
    return {**found, "publish_deskewed_scan": True, "publish_local_map": using_default_rviz}


def point_cloud_display(name: str, topic: str, colour: str, size_m: float) -> dict:
    return {
        "Class": "rviz_default_plugins/PointCloud2",
        "Enabled": True,
        "Name": name,
        # rko_lio publishes its scan and local map at SystemDefaults/keep_last(1)/volatile
        "Topic": {"Value": topic, "Durability Policy": "Volatile", "Reliability Policy": "Reliable"},
        "Color Transformer": "FlatColor",
        "Color": colour,
        # Flat Squares as the shipped displays use: the Points style sizes in pixels
        "Size (m)": size_m,
        "Style": "Flat Squares",
    }


def launch_setup(context, *args, **kwargs):
    given = common.merge(
        common.config_file_parameters(context), common.cli_parameters(context, configurable_parameters)
    )
    for name, decided in FORCED.items():
        if name in given:
            common.fail(
                f"[ERROR] {name} is set by this launch file to {decided}.",
                "Run rko_lio and slam.launch.py yourself if you need it otherwise.",
            )

    default_rviz = slam.using_default_rviz(context, configurable_parameters)
    shared = {name: given[name] for name in SHARED if given.get(name) not in ("", None)}
    odom_params = rko_lio_parameters(context, shared, default_rviz)

    slam_params = {
        **given,
        **{name: odom_params[name] for name in SHARED if name in odom_params},
        "lidar_topic": rko_lio_value(odom_params, "deskewed_scan_topic"),
        "imu_topic": rko_lio_value(odom_params, "imu_topic"),
        "deskew": False,
        "invert_map_tf": rko_lio_value(odom_params, "invert_odom_tf"),
    }

    extra_displays = []
    if default_rviz:
        extra_displays = [point_cloud_display("DeskewedScan", slam_params["lidar_topic"], "255; 255; 255", 0.05)]
        if rko_lio_value(odom_params, "publish_local_map"):
            extra_displays.insert(
                0, point_cloud_display("OdomLocalMap", rko_lio_value(odom_params, "map_topic"), "142; 149; 163", 0.07)
            )

    print("\n" + "=" * 40 + "\n")
    print("rko_lio launch configuration:\n")
    print(yaml.dump(odom_params, sort_keys=False, default_flow_style=False, indent=4))
    print("=" * 40 + "\n")

    log_level = LaunchConfiguration("log_level")
    return [
        rko_lio_odometry.rko_lio_node(odom_params, "online_node", log_level),
        *slam.slam_nodes(context, configurable_parameters, "online", slam_params, extra_displays),
    ]


def generate_launch_description():
    return LaunchDescription(
        [*common.declare_arguments(configurable_parameters), OpaqueFunction(function=launch_setup)],
    )
