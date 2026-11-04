import sys
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).parent))
from model import model


def launch_executor(context):
    job = LaunchConfiguration("job").perform(context)
    mode = LaunchConfiguration("mode").perform(context)
    moveit = model(yaml.safe_load(Path(job).read_text()), mode)
    return [
        Node(
            package="piper_drawing",
            executable="draw_executor",
            output="screen",
            parameters=[
                moveit.to_dict(),
                {
                    "job": job,
                    "mode": mode,
                    "execute": LaunchConfiguration("execute").perform(context) == "true",
                    "report": LaunchConfiguration("report").perform(context),
                },
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("job"),
            DeclareLaunchArgument("mode", default_value="mock", choices=["mock", "real"]),
            DeclareLaunchArgument("execute", default_value="true"),
            DeclareLaunchArgument("report", default_value="run_report.yaml"),
            OpaqueFunction(function=launch_executor),
        ]
    )
