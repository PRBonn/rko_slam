import importlib.util
from pathlib import Path

import launch_ros.actions
import yaml
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch import LaunchDescription


def load_module(path: Path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


common = load_module(Path(__file__).parent / "common.py")

# Everything align_sessions declares.
configurable_parameters = [
    {
        "name": "run_dirs",
        "default": "",
        "description": "Run directories to merge, comma-separated (required)",
        "required": True,
    },
    {
        "name": "results_dir",
        "default": "results",
        "description": "Where to write the merged run",
    },
    {
        "name": "run_name",
        "default": "aligned",
        "description": "Name of the run folder under results_dir; a number is appended so nothing is overwritten",
    },
    *common.CLOSURE_PARAMETERS,
    *common.POSE_GRAPH_PARAMETERS,
    {
        "name": "no_of_sub_maps_to_skip",
        "default": "0",
        "type": "int",
        "description": "How many of the most recent sub-maps to ignore when looking for a closure. 0 here, since neighbouring sub-maps from different sessions are real candidates",
    },
    {
        "name": "max_iterations",
        "default": "100",
        "type": "int",
        "description": "Most optimizer iterations to run over the merged graph",
    },
    *common.LAUNCH_PARAMETERS,
]


def parse_run_dirs(value) -> list:
    if isinstance(value, (list, tuple)):
        entries = [str(entry).strip() for entry in value]
    else:
        entries = [entry.strip().strip("'\"") for entry in str(value).strip().strip("[]").split(",")]
    return [entry for entry in entries if entry]


def launch_setup(context, *args, **kwargs):
    merged = common.merge(
        common.config_file_parameters(context), common.cli_parameters(context, configurable_parameters)
    )
    merged["run_dirs"] = parse_run_dirs(merged.get("run_dirs", ""))
    common.check_required(configurable_parameters, merged)
    if len(merged["run_dirs"]) < 2:
        common.fail("[ERROR] run_dirs needs at least 2 run directories.")

    final_params = common.node_parameters(configurable_parameters, merged)
    print("\n" + "=" * 40 + "\n")
    print("rko_slam align configuration:\n")
    print(yaml.dump(final_params, sort_keys=False, default_flow_style=False, indent=4))
    print("=" * 40 + "\n")

    return [
        launch_ros.actions.Node(
            package="rko_slam",
            executable="align_sessions",
            parameters=[final_params],
            output="screen",
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            emulate_tty=True,
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            *common.declare_arguments(configurable_parameters),
            OpaqueFunction(function=launch_setup),
        ]
    )
