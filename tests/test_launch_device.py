"""验证真机 launch 的配置传递及模型一致性；不启动 ROS。"""

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest


@pytest.fixture
def launch_environment(tmp_path, monkeypatch):
    def action(*arguments, **options):
        return SimpleNamespace(**options)

    joints = "".join(
        f'<joint name="joint{i}"><limit lower="-1" upper="1"/></joint>'
        for i in range(1, 7)
    )
    model = Mock(return_value=SimpleNamespace(
        robot_description={"robot_description": f"<robot>{joints}</robot>"},
        to_dict=dict,
    ))
    modules = {
        "ament_index_python.packages": SimpleNamespace(get_package_share_directory=lambda name: str(tmp_path)),
        "launch": SimpleNamespace(LaunchDescription=action),
        "launch.actions": SimpleNamespace(**{name: action for name in (
            "DeclareLaunchArgument", "EmitEvent", "OpaqueFunction", "RegisterEventHandler"
        )}),
        "launch.event_handlers": SimpleNamespace(OnProcessExit=action),
        "launch.events": SimpleNamespace(Shutdown=action),
        "launch.substitutions": SimpleNamespace(
            LaunchConfiguration=lambda name: SimpleNamespace(perform=lambda context: context[name])
        ),
        "launch_ros.actions": SimpleNamespace(Node=action),
        "model": SimpleNamespace(model=model),
    }
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setattr(sys, "path", sys.path.copy())
    path = Path(__file__).resolve().parents[1] / "ros2_ws/src/piper_drawing/launch/bringup.launch.py"
    spec = importlib.util.spec_from_file_location("bringup_test", path)
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    drawing = tmp_path / "drawing.yaml"
    drawing.write_text("robot:\n  dh_version: new\n")
    device = tmp_path / "device.yaml"
    device.write_text("can: can1\nprotocol: v3\ndh_version: new\nlock_dir: locks\n")
    context = {
        "mode": "real", "config": str(drawing), "device_config": str(device),
        "can_port": "", "protocol": "", "dh_version": "",
    }
    return SimpleNamespace(launch=launch, context=context, model=model, directory=tmp_path)


def test_real_launch_passes_shared_configuration_to_bridge(launch_environment):
    e = launch_environment
    nodes = e.launch.launch_nodes(e.context)
    bridge = next(node for node in nodes if getattr(node, "executable", None) == "piper_bridge")
    parameters = bridge.parameters[0]
    assert parameters["can_port"] == "can1"
    assert parameters["protocol"] == "v3"
    assert parameters["dh_version"] == "new"
    assert parameters["lock_dir"] == str(e.directory / "locks")
    assert parameters["joint_lower"] == [-1.0] * 6
    assert parameters["firmware_speed_percent"] == 100
    assert parameters["gripper_present"] is True


def test_real_launch_passes_explicit_firmware_speed(launch_environment):
    e = launch_environment
    with (e.directory / "device.yaml").open("a") as stream:
        stream.write("firmware_speed_percent: 35\n")
    nodes = e.launch.launch_nodes(e.context)
    bridge = next(node for node in nodes if getattr(node, "executable", None) == "piper_bridge")
    assert bridge.parameters[0]["firmware_speed_percent"] == 35


def test_model_mismatch_fails_before_launching_nodes(launch_environment):
    e = launch_environment
    e.context["dh_version"] = "old"
    with pytest.raises(ValueError, match="与设备配置不一致"):
        e.launch.launch_nodes(e.context)
    e.model.assert_not_called()


def test_mock_launch_does_not_require_device_configuration(launch_environment):
    e = launch_environment
    e.context["mode"] = "mock"
    e.context["device_config"] = str(e.directory / "missing.yaml")
    nodes = e.launch.launch_nodes(e.context)
    assert not any(getattr(node, "executable", None) == "piper_bridge" for node in nodes)
    e.model.assert_called_once()
