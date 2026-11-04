"""待机保持到新绘画的交接，不连接 ROS 或硬件。"""

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from piper_draw import idle_ros


@pytest.mark.parametrize("holding", [False, True])
def test_new_drawing_releases_existing_idle_hold_after_alignment(monkeypatch, holding):
    modules = {
        "rclpy": NS(),
        "rclpy.action": NS(ActionClient=object),
        "rclpy.context": NS(Context=object),
        "rclpy.executors": NS(SingleThreadedExecutor=object),
        "controller_manager_msgs.srv": NS(ListControllers=object),
        "moveit_msgs.action": NS(MoveGroup=object),
        "rcl_interfaces.srv": NS(GetParameters=object),
        "std_msgs.msg": NS(String=object),
        "std_srvs.srv": NS(Trigger=object),
    }
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(__file__).parents[1] / "src/piper_draw/workflow_ros.py"
    spec = importlib.util.spec_from_file_location("piper_draw.workflow_ros_idle_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    connection = object.__new__(module.RobotConnection)
    status = {"arm_status": 0, "session_enabled": True, "holding": holding}
    connection.executor = NS(spin_once=lambda **kwargs: setattr(connection, "last_status", status))
    events = []
    monkeypatch.setattr(idle_ros, "trigger", lambda connection, name, check: events.append(name))
    connection.enable(lambda: None, lambda: events.append("motion_authorized"))
    assert events == (["motion_authorized", "/piper/resume_commands"] if holding else ["motion_authorized"])
