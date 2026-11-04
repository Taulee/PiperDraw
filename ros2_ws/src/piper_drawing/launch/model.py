import math
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def model(config, mode):
    robot_config = config["robot"]
    if robot_config["group"] != "arm" or robot_config["base_link"] != "base_link":
        raise ValueError("Piper 配置需要 group: arm、base_link: base_link")
    if robot_config["flange_link"] != "link6":
        raise ValueError("Piper 法兰坐标系必须为 link6；笔尖位置由 TCP 设置")
    dh_version = robot_config["dh_version"]
    if dh_version not in ("new", "old"):
        raise ValueError("robot.dh_version 必须为 new 或 old")
    suffix = "_old" if dh_version == "old" else ""
    description = Path(get_package_share_directory("piper_description"))
    urdf = description / f"urdf/piper_description{suffix}.urdf"
    moveit = (
        MoveItConfigsBuilder("piper", package_name="piper_drawing")
        .robot_description(file_path=str(urdf))
        .robot_description_semantic(file_path="config/piper.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], load_all=False)
        .to_moveit_configs()
    )
    robot = ET.fromstring(moveit.robot_description["robot_description"])
    velocity = robot_config["joint_velocity_rad_s"]
    acceleration = robot_config["joint_acceleration_rad_s2"]
    if not all(math.isfinite(x) and x > 0 for x in (velocity, acceleration)):
        raise ValueError("绘画关节速度、加速度上限必须是有限正数")
    initial = robot_config["initial_positions"]
    if len(initial) != 6 or not all(math.isfinite(x) for x in initial):
        raise ValueError("仿真初始关节角必须是 6 个有限数值，单位 rad")
    width = robot_config["gripper_width_mm"] / 1000
    # 官方夹爪 URDF 每侧行程 35 mm；不扩大官方模型的限位。
    if not math.isfinite(width) or not 0 <= width <= 0.07:
        raise ValueError("模型夹爪总开口必须在 0–70 mm 内")
    control = ET.SubElement(robot, "ros2_control", name="PiperDrawing", type="system")
    hardware = ET.SubElement(control, "hardware")
    plugin = "piper_drawing/PiperHardware" if mode == "real" else "mock_components/GenericSystem"
    ET.SubElement(hardware, "plugin").text = plugin
    joint_velocities = {}
    for i in range(1, 9):
        name = f"joint{i}"
        limit = robot.find(f"joint[@name='{name}']/limit")
        position = initial[i - 1] if i <= 6 else width / 2 * (1 if i == 7 else -1)
        if not float(limit.get("lower")) <= position <= float(limit.get("upper")):
            raise ValueError(f"{name} 仿真初始位置超出官方关节限位")
        joint = ET.SubElement(control, "joint", name=name)
        if i <= 6:
            # 取消项目自设的 1 rad/s 封顶；仍保留官方 URDF 各轴速度上限。
            joint_velocities[name] = min(velocity, float(limit.get("velocity")))
            limit.set("velocity", str(joint_velocities[name]))
            command = ET.SubElement(joint, "command_interface", name="position")
            for bound in ("min", "max"):
                ET.SubElement(command, "param", name=bound).text = limit.get(
                    "lower" if bound == "min" else "upper"
                )
        state = ET.SubElement(joint, "state_interface", name="position")
        ET.SubElement(state, "param", name="initial_value").text = str(position)
        ET.SubElement(joint, "state_interface", name="velocity")
    moveit.robot_description["robot_description"] = ET.tostring(robot, encoding="unicode")
    moveit.joint_limits["robot_description_planning"]["joint_limits"] = {
        f"joint{i}": {
            "has_velocity_limits": True,
            "max_velocity": joint_velocities[f"joint{i}"],
            "has_acceleration_limits": True,
            "max_acceleration": acceleration,
        }
        for i in range(1, 7)
    }
    moveit.robot_description_kinematics["robot_description_kinematics"]["arm"][
        "kinematics_solver_timeout"
    ] = 0.05
    # 返程净空是一个宽半空间约束；在关节空间采样，避免对大体积盒子反复求 IK。
    moveit.planning_pipelines["ompl"].setdefault("arm", {})["enforce_joint_model_state_space"] = True
    return moveit
