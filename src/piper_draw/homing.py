"""通过固件 MOVE J 低速移动到指定关节姿态。"""

import math
import time

from .piper_backend import STATE_FRAMES, validate_firmware


def move_to_joint_target(transport, *, target_rad=(0.0,) * 6, speed_percent=3, timeout=90,
                         clock=time.monotonic, sleep=time.sleep, progress=None, gripper_present=True):
    if type(speed_percent) is not int or not 1 <= speed_percent <= 5:
        raise ValueError("低速移动速度必须为 1–5%")
    if not math.isfinite(timeout) or not 5 <= timeout <= 120:
        raise ValueError("移动超时必须为 5–120 秒")
    if len(target_rad) != 6 or not all(map(math.isfinite, target_rad)):
        raise ValueError("目标必须包含六个有限关节角")
    target_deg = [math.degrees(angle) for angle in target_rad]
    command = [round(angle * 1000) for angle in target_deg]

    required = (*STATE_FRAMES, 0x2A8) if gripper_present else STATE_FRAMES

    def checked(*, enabled=False):
        transport.frames.require(required)
        feedback = transport.read()
        if len(feedback.positions) != 6 or not all(map(math.isfinite, feedback.positions)):
            raise RuntimeError("六轴反馈无效")
        if feedback.error_code or feedback.arm_status != 0:
            raise RuntimeError(f"控制器异常：status={feedback.arm_status}, error={feedback.error_code}")
        if enabled and not all(feedback.enabled):
            raise RuntimeError("移动期间电机失能")
        return feedback

    deadline = clock() + 3
    while True:
        try:
            transport.frames.require(required)
            break
        except RuntimeError:
            if clock() >= deadline:
                raise
            sleep(0.05)
    before = checked()
    validate_firmware(before.firmware, "v3", "new")
    result = {"before": dict(vars(before)), "target_deg": target_deg, "speed_percent": speed_percent}

    def target():
        # 固件自行规划 MOVE J；不是把远距离目标当作流式伺服小步指令。
        transport.sdk.MotionCtrl_2(0x01, 0x01, speed_percent, 0x00)
        transport.sdk.JointCtrl(*command)

    active = False
    try:
        # 先设置低速与目标，再使能，避免沿用旧的高速或旧目标。
        active = True
        target()
        enable_deadline = clock() + 3
        while True:
            feedback = checked()
            if all(feedback.enabled):
                break
            if clock() >= enable_deadline:
                raise RuntimeError("3 秒内未确认六轴使能")
            transport.sdk.EnableArm(7)
            sleep(0.05)
            target()

        started = clock()
        previous_time, previous = started, feedback.positions
        initial_error = max(abs(a - b) for a, b in zip(feedback.positions, target_rad))
        progress_at = started
        settled_at = None
        peak_speed = 0.0
        while True:
            now = clock()
            feedback = checked(enabled=True)
            dt = now - previous_time
            if dt > 0.25:
                raise RuntimeError("移动监测循环超过 250 ms 未运行")
            if dt >= 0.01:
                speed = max(abs(a - b) / dt for a, b in zip(feedback.positions, previous))
                peak_speed = max(peak_speed, speed)
            error = max(abs(a - b) for a, b in zip(feedback.positions, target_rad))
            if now - started > timeout:
                raise RuntimeError("移动超时")
            if now - started > 5 and error > math.radians(1) and initial_error - error < math.radians(0.2):
                raise RuntimeError("移动指令发出后未检测到向目标收敛")
            target()
            if error <= math.radians(0.5):
                settled_at = now if settled_at is None else settled_at
                if now - settled_at >= 1:
                    return {**result, "status": "reached", "after": vars(feedback),
                            "seconds": round(now - started, 3),
                            "peak_measured_speed_deg_s": math.degrees(peak_speed)}
            else:
                settled_at = None
            if progress is not None and now >= progress_at:
                progress(feedback, now - started)
                progress_at = now + 2
            previous_time, previous = now, feedback.positions
            sleep(0.02)
    except BaseException:
        if active:
            # 异常时只请求快速停止，不自动 reset、失能或操作夹爪。
            transport.sdk.MotionCtrl_1(0x01, 0, 0)
        raise
