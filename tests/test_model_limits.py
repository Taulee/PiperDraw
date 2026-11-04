"""验证模型的实际约束输出；替代 MoveIt 配置构建器，不启动 ROS。"""

import importlib.util
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml


@pytest.fixture
def model_environment(monkeypatch):
    root = Path(__file__).resolve().parents[1]
    description = root / "vendor/piper_ros/src/piper_description"
    moveit = SimpleNamespace(
        robot_description={},
        joint_limits={"robot_description_planning": {}},
        robot_description_kinematics={"robot_description_kinematics": {"arm": {}}},
        planning_pipelines={"ompl": {}},
    )
    builder = Mock()
    for name in ("robot_description_semantic", "robot_description_kinematics", "joint_limits",
                 "trajectory_execution", "planning_pipelines"):
        getattr(builder, name).return_value = builder

    def load_urdf(*, file_path):
        moveit.robot_description["robot_description"] = Path(file_path).read_text()
        return builder

    builder.robot_description.side_effect = load_urdf
    builder.to_moveit_configs.return_value = moveit
    monkeypatch.setitem(sys.modules, "ament_index_python.packages", SimpleNamespace(
        get_package_share_directory=lambda name: str(description),
    ))
    monkeypatch.setitem(sys.modules, "moveit_configs_utils", SimpleNamespace(
        MoveItConfigsBuilder=lambda *args, **kwargs: builder,
    ))
    spec = importlib.util.spec_from_file_location(
        "speed_test_model", root / "ros2_ws/src/piper_drawing/launch/model.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    config = yaml.safe_load((root / "config/real.yaml").read_text())
    return SimpleNamespace(model=module.model, config=config)


@pytest.mark.parametrize("dh", ["new", "old"])
def test_limits_above_one_are_used_by_model_and_planner(model_environment, dh):
    e = model_environment
    e.config["robot"].update(dh_version=dh, joint_velocity_rad_s=2.0,
                             joint_acceleration_rad_s2=4.0)
    result = e.model(e.config, "real")
    assert result.planning_pipelines["ompl"]["arm"]["enforce_joint_model_state_space"] is True
    urdf = ET.fromstring(result.robot_description["robot_description"])
    limits = result.joint_limits["robot_description_planning"]["joint_limits"]
    for i in range(1, 7):
        assert float(urdf.find(f"joint[@name='joint{i}']/limit").get("velocity")) == 2.0
        assert limits[f"joint{i}"]["max_velocity"] == 2.0
        assert limits[f"joint{i}"]["max_acceleration"] == 4.0


def test_requested_velocity_preserves_per_joint_official_limits(model_environment):
    e = model_environment
    e.config["robot"]["joint_velocity_rad_s"] = 10.0
    result = e.model(e.config, "real")
    urdf = ET.fromstring(result.robot_description["robot_description"])
    limits = result.joint_limits["robot_description_planning"]["joint_limits"]
    for i in range(1, 7):
        expected = 3.0 if i == 6 else 5.0
        assert float(urdf.find(f"joint[@name='joint{i}']/limit").get("velocity")) == expected
        assert limits[f"joint{i}"]["max_velocity"] == expected


@pytest.mark.parametrize("key", ["joint_velocity_rad_s", "joint_acceleration_rad_s2"])
@pytest.mark.parametrize("value", [0, -1, math.nan, math.inf])
def test_nonpositive_or_nonfinite_limits_remain_invalid(model_environment, key, value):
    e = model_environment
    e.config["robot"][key] = value
    with pytest.raises(ValueError, match="有限正数"):
        e.model(e.config, "real")
