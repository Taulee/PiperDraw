import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from piper_draw.device import DEFAULT_DEVICE_CONFIG, load_device_config

sys.path.insert(0, str(Path(__file__).parent))
from model import model


def launch_nodes(context):
    mode = LaunchConfiguration("mode").perform(context)
    config_path = LaunchConfiguration("config").perform(context)
    config = yaml.safe_load(Path(config_path).read_text())
    device = None
    if mode == "real":
        device = load_device_config(
            LaunchConfiguration("device_config").perform(context),
            can=LaunchConfiguration("can_port").perform(context) or None,
            protocol=LaunchConfiguration("protocol").perform(context) or None,
            dh_version=LaunchConfiguration("dh_version").perform(context) or None,
        )
        if config["robot"]["dh_version"] != device.dh_version:
            raise ValueError("绘画配置的 robot.dh_version 与设备配置不一致，请核对模型与标定")
    moveit = model(config, mode)
    package = Path(get_package_share_directory("piper_drawing"))
    nodes = []
    if mode == "real":
        urdf = ET.fromstring(moveit.robot_description["robot_description"])
        limits = [urdf.find(f"joint[@name='joint{i}']/limit") for i in range(1, 7)]
        nodes.append(
            Node(
                package="piper_drawing",
                executable="piper_bridge",
                name="piper_bridge",
                output="screen",
                parameters=[
                    {
                        "can_port": device.can,
                        "protocol": device.protocol,
                        "dh_version": device.dh_version,
                        "lock_dir": str(device.lock_dir),
                        "firmware_speed_percent": device.firmware_speed_percent,
                        "gripper_present": device.gripper_present,
                        "joint_lower": [float(item.get("lower")) for item in limits],
                        "joint_upper": [float(item.get("upper")) for item in limits],
                    }
                ],
            )
        )
    controller = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit.robot_description, str(package / "config/ros2_controllers.yaml")],
        output="screen",
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit.to_dict(),
            {
                "publish_robot_description": True,
                "publish_robot_description_semantic": True,
                "use_sim_time": False,
            },
        ],
    )
    nodes.extend(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[moveit.robot_description],
                output="screen",
            ),
            Node(
                package="piper_drawing",
                executable="drawing_scene",
                parameters=[{"config": config_path}],
                output="screen",
            ),
            controller,
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "arm_controller",
                    "--controller-manager-timeout",
                    "60",
                ],
                output="screen",
            ),
            move_group,
        ]
    )
    # 驱动、控制器或规划进程退出时关闭整套后端。
    critical = [controller, move_group] + (nodes[:1] if mode == "real" else [])
    handlers = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=node,
                on_exit=[EmitEvent(event=Shutdown(reason="绘画后端核心进程退出"))],
            )
        )
        for node in critical
    ]
    return [*handlers, *nodes]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="mock", choices=["mock", "real"]),
            DeclareLaunchArgument("config"),
            DeclareLaunchArgument("device_config", default_value=str(DEFAULT_DEVICE_CONFIG)),
            DeclareLaunchArgument("can_port", default_value=""),
            DeclareLaunchArgument("protocol", default_value=""),
            DeclareLaunchArgument("dh_version", default_value=""),
            OpaqueFunction(function=launch_nodes),
        ]
    )
