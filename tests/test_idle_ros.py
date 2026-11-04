"""验证取消时先保持、完整规划离纸和返程后才执行，失败则继续保持。"""

import sys
from types import SimpleNamespace as NS

import pytest

from piper_draw import idle_ros


class Message:
    def __init__(self, **values):
        self.__dict__.update(values)


@pytest.fixture
def messages(monkeypatch):
    class SolidPrimitive(Message):
        BOX = 1

    for module, values in {
        "geometry_msgs.msg": {k: Message for k in ("Point", "Pose", "Quaternion", "Vector3")},
        "moveit_msgs.msg": {k: Message for k in ("RobotState", "BoundingVolume", "Constraints", "PositionConstraint", "JointConstraint", "MotionPlanRequest")},
        "sensor_msgs.msg": {"JointState": Message},
        "std_msgs.msg": {"Header": Message},
        "shape_msgs.msg": {"SolidPrimitive": SolidPrimitive},
        "moveit_msgs.srv": {k: NS(Request=Message) for k in ("GetCartesianPath", "GetMotionPlan")},
    }.items():
        monkeypatch.setitem(sys.modules, module, NS(**values))


@pytest.fixture
def plan():
    return {
        "positions_rad": [0.5] * 6,
        "settings": {"min_tip_height_mm": 4.0, "velocity_scale": 0.2, "acceleration_scale": 0.4, "timeout_s": 120},
        "job": {
            "robot": {"group": "arm", "base_link": "base_link", "flange_link": "link6"},
            "motion": {"lift_mm": 8.0, "velocity_scale": 0.4, "acceleration_scale": 0.8,
                       "cartesian_step_mm": 1.0, "max_joint_step_rad": 0.15,
                       "joint_tolerance_rad": 0.03, "lift_mm_s": 15, "lower_mm_s": 5, "travel_mm_s": 200},
            "paper_origin": [0, 0, 0], "paper_normal": [0, 0, 1],
            "table": {"pose": [0, 0, -0.01, 0, 0, 0, 1]},
            "tool": {"tcp_offset_mm": [0, 0, 190]},
        },
    }


@pytest.mark.parametrize("planner_ok", [True, False])
def test_contact_cancel_lifts_then_returns_only_after_both_paths_are_validated(messages, plan, monkeypatch, planner_ok):
    events = []
    requests = []
    joints = NS(joint_names=[f"joint{i}" for i in range(1, 7)], points=[NS(positions=[0.1] * 6)])
    lift, path = NS(name="lift", joint_trajectory=joints), NS(name="return", joint_trajectory=joints)
    statuses = iter([{"positions_rad": [0.0] * 6}, {"positions_rad": [0.5] * 6}])
    monkeypatch.setattr(idle_ros, "hold_and_cancel", lambda *args: events.append("hold"))
    monkeypatch.setattr(idle_ros, "fresh_status", lambda *args: next(statuses))
    monkeypatch.setattr(idle_ros, "tip_height", lambda *args: (0.0, {
        "position_mm": [1, 2, 190], "quaternion_xyzw": [0, 1, 0, 0],
    }))
    monkeypatch.setattr(idle_ros, "validate_trajectory", lambda trajectory, *args: events.append("validate_" + trajectory.name))
    monkeypatch.setattr(idle_ros, "limit_tip_speed", lambda *args: None)
    monkeypatch.setattr(idle_ros, "trigger", lambda connection, name, *args: events.append(name))
    monkeypatch.setattr(idle_ros, "execute", lambda connection, trajectory, *args: events.append("execute_" + trajectory.name))

    def service(connection, kind, name, request, *args):
        requests.append(request)
        if name == "/compute_cartesian_path":
            events.append("plan_lift")
            return NS(error_code=NS(val=1), fraction=1.0, solution=lift)
        events.append("plan_return")
        return NS(motion_plan_response=NS(error_code=NS(val=1 if planner_ok else -1), trajectory=path))

    monkeypatch.setattr(idle_ros, "service", service)
    if planner_ok:
        result = idle_ros.return_to_idle(NS(), plan)
        assert result["holding"]
        assert events == ["hold", "plan_lift", "validate_lift", "plan_return", "validate_return",
                          "/piper/resume_commands", "execute_lift", "execute_return", "/piper/hold"]
    else:
        with pytest.raises(RuntimeError, match="完整路径规划失败"):
            idle_ros.return_to_idle(NS(), plan)
        assert events == ["hold", "plan_lift", "validate_lift", "plan_return", "hold"]
    assert requests[0].avoid_collisions is True
    assert requests[0].waypoints[0].position.z == pytest.approx(0.198)
    constraint = requests[1].motion_plan_request.path_constraints.position_constraints[0]
    assert constraint.target_point_offset.z == 0.19
    assert constraint.constraint_region.primitive_poses[0].position.z == pytest.approx(2.004)


def test_speed_limit_updates_time_velocity_and_acceleration_consistently(monkeypatch):
    def point(position, seconds):
        return NS(positions=[position] * 6, velocities=[2.0] * 6, accelerations=[4.0] * 6,
                  time_from_start=NS(sec=seconds, nanosec=0))

    points = [point(0, 0), point(1, 1)]
    trajectory = NS(joint_trajectory=NS(joint_names=[f"joint{i}" for i in range(1, 7)], points=points))
    monkeypatch.setattr(idle_ros, "tip_height", lambda job, positions: (0, {"tip_position_mm": [positions[0] * 100, 0, 0]}))
    idle_ros.limit_tip_speed(trajectory, {}, 50)
    assert points[1].time_from_start.sec == 2
    assert points[1].velocities == [1.0] * 6
    assert points[1].accelerations == [1.0] * 6


@pytest.mark.parametrize("changes", [{"session_enabled": False}, {"motors_enabled": [False]*6},
                                   {"arm_status": 1}, {"error_code": 1}, {"motors_enabled": []}])
def test_fault_feedback_does_not_authorize_return(changes):
    status = {"session_enabled": True, "motors_enabled": [True]*6, "arm_status": 0, "error_code": 0, **changes}
    connection = NS(last_status=None)
    connection.executor = NS(spin_once=lambda **kwargs: setattr(connection, "last_status", status))
    with pytest.raises(RuntimeError):
        idle_ros.fresh_status(connection, lambda: None)
