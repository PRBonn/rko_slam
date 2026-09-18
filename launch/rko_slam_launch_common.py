import sys

import yaml
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# A parameter whose default differs between the two entrypoints stays in their own tables.
# The launch-level knobs both entrypoints take.
LAUNCH_PARAMETERS = [
    {
        "launch_only": True,
        "name": "config_file",
        "default": "",
        "description": "YAML file with any of these parameters; explicit CLI args override it",
    },
    {
        "launch_only": True,
        "name": "log_level",
        "default": "info",
        "description": "ROS log level [DEBUG|INFO|WARN|ERROR|FATAL]",
    },
]

CLOSURE_PARAMETERS = [
    {
        "name": "density_map_resolution",
        "default": "0.5",
        "type": "float",
        "description": "Cell size (m) of the top-down image closures are detected in",
    },
    {
        "name": "density_threshold",
        "default": "0.05",
        "type": "float",
        "description": "How occupied a cell must be to count when looking for a closure",
    },
    {
        "name": "hamming_distance_threshold",
        "default": "50",
        "type": "int",
        "description": "How different two places may look and still be called a match; lower is stricter",
    },
    {
        "name": "inliers_threshold",
        "default": "5",
        "type": "int",
        "description": "How many points must agree before a candidate closure is checked properly",
    },
    {
        "name": "overlap_threshold",
        "default": "0.4",
        "type": "float",
        "description": "How much the two sub-maps must overlap to accept the closure. Raise it where many places look alike",
    },
]

POSE_GRAPH_PARAMETERS = [
    {
        "name": "rotation_info_scale",
        "default": "100.0",
        "type": "float",
        "description": "How much more the optimizer trusts rotation than translation. 100 is about 5.7 degrees of assumed error",
    },
    {
        "name": "closure_info_scale",
        "default": "1.0",
        "type": "float",
        "description": "How much the optimizer trusts closures against odometry; above 1 trusts closures more",
    },
    {
        "name": "closure_kernel_delta",
        "default": "1.0",
        "type": "float",
        "description": "Closures that disagree by more than this (m) are given less weight rather than believed",
    },
]


def fail(*lines):
    print("\n\n" + "=" * 40)
    for line in lines:
        print(line)
    print("=" * 40 + "\n\n")
    sys.exit(1)


def declare_arguments(table):
    return [
        DeclareLaunchArgument(
            param["name"],
            default_value=param.get("default", ""),
            description=param.get("description", ""),
        )
        for param in table
    ]


def cast(param, value):
    """DeclareLaunchArgument only carries strings."""
    kind = param.get("type")
    if kind == "bool":
        return str(value).lower() == "true"
    if kind == "int":
        return int(value)
    if kind == "float":
        return float(value)
    return value


def default_for(table, name) -> str:
    return next(param["default"] for param in table if param["name"] == name)


def flag(context, name) -> bool:
    return cast({"type": "bool"}, LaunchConfiguration(name).perform(context))


def cli_parameters(context, table):
    """Unset parameters are dropped, not defaulted: the node keeps its C++ value, the table default only feeds `-s`."""
    explicit = {arg.split(":=")[0] for arg in getattr(context, "argv", []) if ":=" in arg}
    unknown = sorted(explicit - {param["name"] for param in table})
    if unknown:
        fail(
            "[ERROR] unknown parameter(s) on the command line:",
            *[f"  - {name}" for name in unknown],
            "Run the launch file with -s to list every parameter it accepts.",
        )
    return {
        param["name"]: cast(param, LaunchConfiguration(param["name"]).perform(context))
        for param in table
        if param["name"] in explicit
    }


def config_file_parameters(context):
    path = LaunchConfiguration("config_file").perform(context)
    if path == "":
        return {}
    with open(path) as f:
        return yaml.safe_load(f) or {}


def merge(file_params, cli_params):
    merged = dict(file_params)
    for name, value in cli_params.items():
        if value not in ("", None):
            merged[name] = value
    return merged


def check_required(table, merged):
    missing = [param["name"] for param in table if param.get("required") and not merged.get(param["name"])]
    if missing:
        fail(
            "[ERROR] missing required parameter(s):",
            *[f"  - {name}" for name in missing],
            "Provide them via cli (param:=value) or a config file.",
        )


def node_parameters(table, merged):
    forwarded = {param["name"] for param in table if not param.get("launch_only")}
    consumed = {param["name"] for param in table if param.get("launch_only")}
    ignored = sorted(set(merged) - forwarded - consumed)
    if ignored:
        print(f"[WARN] not forwarded to the node: {ignored}")
    return {name: value for name, value in merged.items() if name in forwarded}
