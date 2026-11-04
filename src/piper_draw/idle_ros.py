"""在原 ROS 控制链路内保持、取消、离纸和返回待机位；不切换 SDK 控制权。"""

import math
import time


def service(connection, service_type, name, request, timeout=5.0, check=lambda: None):
    client = connection.node.create_client(service_type, name)
    try:
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f"服务未就绪：{name}")
        response = connection.call(client, request, timeout, check)
        if response is None:
            raise RuntimeError(f"服务响应超时：{name}")
        return response
    finally:
        connection.node.destroy_client(client)


def trigger(connection, name, check=lambda: None):
    from std_srvs.srv import Trigger

    response = service(connection, Trigger, name, Trigger.Request(), check=check)
    if not response.success:
        raise RuntimeError(response.message)


def hold_and_cancel(connection, check=lambda: None):
    from action_msgs.srv import CancelGoal

    trigger(connection, "/piper/hold", check)
    for name in ("/move_action", "/execute_trajectory", "/arm_controller/follow_joint_trajectory"):
        response = service(connection, CancelGoal, name + "/_action/cancel_goal",
                           CancelGoal.Request(), check=check)
        if response.return_code != 0:
            raise RuntimeError(f"无法取消原轨迹：{name}，{response.return_code}")
    status = fresh_status(connection, check)
    if not status.get("holding"):
        raise RuntimeError("未确认反馈位置保持，不能自动返程")


def fresh_status(connection, check):
    connection.last_status = None
    deadline = time.monotonic() + 3
    while connection.last_status is None and time.monotonic() < deadline:
        check()
        connection.executor.spin_once(timeout_sec=0.1)
    status = connection.last_status
    if not status or status.get("preflight_error") or status.get("fault"):
        raise RuntimeError("无有效反馈，无法受控返回待机位")
    motors = status.get("motors_enabled", [])
    if status.get("arm_status") != 0 or status.get("error_code") or len(motors) != 6 or not all(motors):
        raise RuntimeError("驱动或控制器异常，无法受控返回待机位")
    if not status.get("session_enabled"):
        raise RuntimeError("会话未使能，不能自动发起返程")
    return status


def robot_state(positions):
    from moveit_msgs.msg import RobotState
    from sensor_msgs.msg import JointState

    return RobotState(is_diff=True, joint_state=JointState(
        name=[f"joint{i}" for i in range(1, 7)], position=list(positions)
    ))


def tip_height(job, positions):
    from .calibration import flange_from_joints
    from scipy.spatial.transform import Rotation

    pose = flange_from_joints(positions, job["robot"]["dh_version"])
    offset = Rotation.from_quat(pose["quaternion_xyzw"]).apply(job["tool"]["tcp_offset_mm"])
    pose["tip_position_mm"] = [a + b for a, b in zip(pose["position_mm"], offset)]
    height = sum((a + b - o * 1000) * n for a, b, o, n in zip(
        pose["position_mm"], offset, job["paper_origin"], job["paper_normal"]
    ))
    return height, pose


def limit_tip_speed(trajectory, job, speed_mm_s):
    joints = trajectory.joint_trajectory
    previous_tip, previous_time, factor = None, 0.0, 1.0
    for point in joints.points:
        positions = dict(zip(joints.joint_names, point.positions))
        _, pose = tip_height(job, [positions[f"joint{i}"] for i in range(1, 7)])
        stamp = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
        current = pose["tip_position_mm"]
        if previous_tip is not None:
            if stamp <= previous_time:
                raise RuntimeError("轨迹时间未递增，不能重计时")
            factor = max(factor, math.dist(previous_tip, current) / (stamp - previous_time) / speed_mm_s)
        previous_tip, previous_time = current, stamp
    for point in joints.points:
        stamp_ns = round((point.time_from_start.sec + point.time_from_start.nanosec / 1e9) * factor * 1e9)
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(stamp_ns, 1_000_000_000)
        point.velocities = [v / factor for v in point.velocities]
        point.accelerations = [a / factor**2 for a in point.accelerations]


def clearance_constraint(job, height_mm):
    from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
    from moveit_msgs.msg import BoundingVolume, Constraints, PositionConstraint
    from shape_msgs.msg import SolidPrimitive
    from std_msgs.msg import Header

    # 纸面姿态的局部 Z 为法向；4 m 高的盒子表示机器人工作域内的净空半空间。
    center = [o + n * (2.0 + height_mm / 1000) for o, n in zip(job["paper_origin"], job["paper_normal"])]
    quaternion = job["table"]["pose"][3:]
    return Constraints(position_constraints=[PositionConstraint(
        header=Header(frame_id=job["robot"]["base_link"]),
        link_name=job["robot"]["flange_link"],
        target_point_offset=Vector3(**dict(zip(("x", "y", "z"), [v / 1000 for v in job["tool"]["tcp_offset_mm"]]))),
        constraint_region=BoundingVolume(
            primitives=[SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[4.0, 4.0, 4.0])],
            primitive_poses=[Pose(position=Point(**dict(zip(("x", "y", "z"), center))),
                                  orientation=Quaternion(**dict(zip(("x", "y", "z", "w"), quaternion))))],
        ), weight=1.0,
    )])


def validate_trajectory(trajectory, job, minimum_height_mm):
    joints = trajectory.joint_trajectory
    names = [f"joint{i}" for i in range(1, 7)]
    if set(joints.joint_names) != set(names) or not joints.points:
        raise RuntimeError("返程轨迹未包含六轴")
    previous = -1.0
    for point in joints.points:
        stamp = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
        if not math.isfinite(stamp) or stamp <= previous:
            raise RuntimeError("返程轨迹时间未严格递增")
        previous = stamp
        positions = dict(zip(joints.joint_names, point.positions))
        height, _ = tip_height(job, [positions[name] for name in names])
        if height < minimum_height_mm - 1e-3:
            raise RuntimeError("返程轨迹未满足纸面净空")
    if previous <= 0:
        raise RuntimeError("返程轨迹未完成时间参数化")


def execute(connection, trajectory, timeout, check):
    from moveit_msgs.action import ExecuteTrajectory
    from rclpy.action import ActionClient

    client = ActionClient(connection.node, ExecuteTrajectory, "/execute_trajectory")
    handle = None
    completed = False
    try:
        if not client.wait_for_server(timeout_sec=3):
            raise RuntimeError("返程执行服务未就绪")
        future = client.send_goal_async(ExecuteTrajectory.Goal(trajectory=trajectory))
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            check()
            connection.executor.spin_once(timeout_sec=0.1)
        if not future.done():
            raise RuntimeError("返程目标接收超时")
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError("返程目标被拒绝")
        result = handle.get_result_async()
        while not result.done() and time.monotonic() < deadline:
            check()
            connection.executor.spin_once(timeout_sec=0.1)
        if not result.done() or result.result().result.error_code.val != 1:
            raise RuntimeError("返程执行失败或超时")
        completed = True
    finally:
        if handle is not None and not completed:
            handle.cancel_goal_async()
        client.destroy()


def return_to_idle(connection, plan, check=lambda: None):
    from geometry_msgs.msg import Point, Pose, Quaternion
    from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
    from moveit_msgs.srv import GetCartesianPath, GetMotionPlan
    from std_msgs.msg import Header

    job, settings = plan["job"], plan["settings"]
    try:
        hold_and_cancel(connection, check)
        status = fresh_status(connection, check)
        positions = status["positions_rad"]
        height, flange = tip_height(job, positions)
        minimum = settings["min_tip_height_mm"]
        if height >= minimum and max(abs(a - b) for a, b in zip(positions, plan["positions_rad"])) <= job["motion"]["joint_tolerance_rad"]:
            return {"positions_rad": positions, "holding": True}
        # 取消可能发生在纸面接触中，先沿法向离纸，再做关节返程。
        target_height = max(job["motion"]["lift_mm"], minimum + 1.0)
        lift = None
        if height < target_height - 0.1:
            target = [p / 1000 + n * (target_height - height) / 1000
                      for p, n in zip(flange["position_mm"], job["paper_normal"])]
            request = GetCartesianPath.Request(
                header=Header(frame_id=job["robot"]["base_link"]),
                start_state=robot_state(positions), group_name=job["robot"]["group"],
                link_name=job["robot"]["flange_link"], avoid_collisions=True,
                waypoints=[Pose(position=Point(**dict(zip(("x", "y", "z"), target))),
                                orientation=Quaternion(**dict(zip(("x", "y", "z", "w"), flange["quaternion_xyzw"]))))],
                max_step=job["motion"]["cartesian_step_mm"] / 1000,
                revolute_jump_threshold=job["motion"]["max_joint_step_rad"],
                max_velocity_scaling_factor=min(settings["velocity_scale"], job["motion"]["velocity_scale"]),
                max_acceleration_scaling_factor=min(settings["acceleration_scale"], job["motion"]["acceleration_scale"]),
            )
            response = service(connection, GetCartesianPath, "/compute_cartesian_path", request, 15, check)
            if response.error_code.val != 1 or response.fraction < 1.0 - 1e-9:
                raise RuntimeError("取消后的离纸路径不完整")
            lift = response.solution
            validate_trajectory(lift, job, height - 0.1)
            limit_tip_speed(lift, job, job["motion"].get("lift_mm_s", job["motion"]["lower_mm_s"]))
            end = dict(zip(lift.joint_trajectory.joint_names, lift.joint_trajectory.points[-1].positions))
            positions = [end[f"joint{i}"] for i in range(1, 7)]
        request = GetMotionPlan.Request(motion_plan_request=MotionPlanRequest(
            group_name=job["robot"]["group"], pipeline_id="ompl", num_planning_attempts=1,
            start_state=robot_state(positions), allowed_planning_time=10.0,
            max_velocity_scaling_factor=settings["velocity_scale"],
            max_acceleration_scaling_factor=settings["acceleration_scale"],
            goal_constraints=[Constraints(joint_constraints=[JointConstraint(
                joint_name=f"joint{i}", position=float(value), tolerance_above=0.005,
                tolerance_below=0.005, weight=1.0,
            ) for i, value in enumerate(plan["positions_rad"], 1)])],
            path_constraints=clearance_constraint(job, minimum),
        ))
        response = service(connection, GetMotionPlan, "/plan_kinematic_path", request, 15, check)
        if response.motion_plan_response.error_code.val != 1:
            raise RuntimeError("返回待机位的完整路径规划失败，保持当前位置")
        path = response.motion_plan_response.trajectory
        validate_trajectory(path, job, minimum)
        limit_tip_speed(path, job, job["motion"]["travel_mm_s"])
        # 原轨迹取消后控制器应已切换为反馈位置保持；未同步就不放开桥接目标锁。
        trigger(connection, "/piper/resume_commands", check)
        if lift is not None:
            execute(connection, lift, settings["timeout_s"], check)
        execute(connection, path, settings["timeout_s"], check)
        final = fresh_status(connection, check)
        if max(abs(a - b) for a, b in zip(final["positions_rad"], plan["positions_rad"])) > job["motion"]["joint_tolerance_rad"]:
            raise RuntimeError("返程结束后关节未到待机位")
        trigger(connection, "/piper/hold", check)
        return {"positions_rad": final["positions_rad"], "holding": True}
    except BaseException:
        try:
            hold_and_cancel(connection)
        except (RuntimeError, ValueError):
            pass
        raise
