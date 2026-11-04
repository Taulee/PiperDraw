"""官方 SDK 的单位转换、会话使能和反馈看门狗；不实现 CAN 协议或逆解。"""

import math
import re
import threading
import time
from contextlib import ExitStack
from dataclasses import dataclass

from .device import device_lock, validate_firmware_speed_percent

ARM_NAMES = tuple(f"joint{i}" for i in range(1, 7))
JOINT_FRAMES = (0x2A5, 0x2A6, 0x2A7)
MOTOR_FRAMES = tuple(range(0x261, 0x267))
STATE_FRAMES = (*JOINT_FRAMES, 0x2A1, *MOTOR_FRAMES)


class FeedbackUnavailable(RuntimeError):
    """反馈尚未到达或已超时，可在初始化期限内等待。"""


class FirmwareNotReady(RuntimeError):
    """固件查询尚未返回可识别版本。"""


class FeedbackClock:
    """分别记录每组 CAN 反馈，避免一个活跃关节掩盖其他关节断流。"""

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.timestamps = {}
        self.lock = threading.Lock()

    def received(self, frame_id):
        with self.lock:
            self.timestamps[frame_id] = self.clock()

    def require(self, frame_ids, timeout=0.5):
        now = self.clock()
        with self.lock:
            stale = [i for i in frame_ids if now - self.timestamps.get(i, -math.inf) > timeout]
        if stale:
            raise FeedbackUnavailable("Piper 反馈未就绪或超时：" + ", ".join(hex(i) for i in stale))


@dataclass
class Feedback:
    positions: tuple[float, ...]
    gripper_width_mm: float
    enabled: tuple[bool, ...]
    arm_status: int
    error_code: int
    firmware: str


class SDKTransport:
    def __init__(self, can_port, protocol, dh_version, *, lock_dir, firmware_speed_percent=100):
        if protocol not in ("v2", "v3") or dh_version not in ("new", "old"):
            raise ValueError("Piper 协议必须是 v2/v3，DH 版本必须是 new/old")
        self.firmware_speed_percent = validate_firmware_speed_percent(firmware_speed_percent)
        with ExitStack() as resources:
            resources.enter_context(device_lock(can_port, lock_dir))
            from piper_sdk import C_PiperInterface_V2, C_PiperInterface_V3

            self.frames = FeedbackClock()
            frames = self.frames
            base = C_PiperInterface_V3 if protocol == "v3" else C_PiperInterface_V2

            class ObservedPiper(base):
                def ParseCANFrame(self, message):
                    super().ParseCANFrame(message)
                    if (
                        message is not None
                        and len(message.data) == 8
                        and not message.is_error_frame
                        and not message.is_remote_frame
                    ):
                        frames.received(message.arbitration_id)

            self.sdk = ObservedPiper(
                can_name=can_port, judge_flag=False, dh_is_offset=int(dh_version == "new")
            )
            resources.callback(self.sdk.DisconnectPort)
            # 初始化仅查询参数与固件，不自动使能、不复位、不设夹爪零点。
            self.sdk.ConnectPort()
            self._resources = resources.pop_all()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def read(self):
        joints = self.sdk.GetArmJointMsgs().joint_state
        motors = self.sdk.GetArmLowSpdInfoMsgs()
        status = self.sdk.GetArmStatus().arm_status
        return Feedback(
            tuple(math.radians(getattr(joints, f"joint_{i}") / 1000) for i in range(1, 7)),
            self.sdk.GetArmGripperMsgs().gripper_state.grippers_angle / 1000,
            tuple(
                bool(getattr(motors, f"motor_{i}").foc_status.driver_enable_status)
                for i in range(1, 7)
            ),
            status.arm_status,
            status.err_code,
            str(self.sdk.GetPiperFirmwareVersion()),
        )

    def send(self, positions):
        self.sdk.MotionCtrl_2(0x01, 0x01, self.firmware_speed_percent, 0x00)
        self.sdk.JointCtrl(*(round(math.degrees(value) * 1000) for value in positions))

    def close(self):
        self._resources.close()


def validate_firmware(firmware, protocol, dh_version):
    match = re.search(r"V?(\d+)\.(\d+)[.-](\d+)", firmware)
    if match is None:
        raise FirmwareNotReady(f"尚未读到可识别的 Piper 固件版本：{firmware}")
    version = tuple(map(int, match.groups()))
    expected_dh = "new" if version >= (1, 6, 3) else "old"
    expected_protocol = "v3" if version >= (1, 8, 8) else "v2"
    if dh_version != expected_dh or protocol != expected_protocol:
        raise RuntimeError(
            f"固件 {firmware} 需要 dh_version={expected_dh}、protocol={expected_protocol}；"
            "修改配置并重启后端"
        )


class PiperSession:
    def __init__(
        self, transport, lower, upper, protocol="v2", dh_version="new", clock=time.monotonic,
        *, gripper_present=True,
    ):
        if len(lower) != 6 or len(upper) != 6:
            raise ValueError("Piper 需要六轴官方关节限位")
        self.transport = transport
        self.lower, self.upper = tuple(lower), tuple(upper)
        self.protocol, self.dh_version = protocol, dh_version
        self.clock = clock
        self.gripper_present = gripper_present
        self.enabled = False
        self.fault = ""
        self.last_command = None
        self.last_command_time = -math.inf
        self.last_sent = None
        self.holding = False
        self.hold_reason = ""
        self.hold_target = None

    def feedback(self, *, motors=False, gripper=True):
        required = STATE_FRAMES if motors else JOINT_FRAMES
        self.transport.frames.require((*required, 0x2A8) if gripper and self.gripper_present else required)
        feedback = self.transport.read()
        self.validate_positions(feedback.positions)
        return feedback

    def validate_positions(self, positions):
        if len(positions) != 6:
            raise ValueError("关节目标必须包含六轴")
        for i, (value, lower, upper) in enumerate(zip(positions, self.lower, self.upper), 1):
            if not math.isfinite(value) or not lower - 1e-4 <= value <= upper + 1e-4:
                raise ValueError(f"joint{i} 不是有限数值或超出官方关节限位：{value}")

    def preflight(self, *, require_enabled=True):
        feedback = self.feedback(motors=True)
        validate_firmware(feedback.firmware, self.protocol, self.dh_version)
        if feedback.error_code or feedback.arm_status not in (0, 1):
            raise RuntimeError(
                f"Piper 控制器异常：arm_status={feedback.arm_status}、err_code={feedback.error_code}"
            )
        if self.gripper_present and not 0 <= feedback.gripper_width_mm <= 70.1:
            raise RuntimeError("夹爪开口超出当前官方碰撞模型的 0–70 mm 范围")
        if require_enabled and (
            not self.enabled or not all(feedback.enabled) or feedback.arm_status != 0
        ):
            raise RuntimeError("当前会话未使能，请显式执行 piper-draw robot enable")
        if require_enabled and self.fault:
            raise RuntimeError(self.fault)
        return feedback

    def enable(self, *, allow_recovery=True):
        feedback = self.preflight(require_enabled=False)
        if not allow_recovery and (feedback.arm_status != 0 or self.fault):
            raise RuntimeError("机械臂处于停止或故障状态，请现场检查后恢复")
        if (
            self.last_command is None
            or self.clock() - self.last_command_time > 0.25
            or max(abs(a - b) for a, b in zip(self.last_command, feedback.positions)) > 0.03
        ):
            raise RuntimeError("控制器目标未与当前反馈对齐，请重启后端后再使能")
        if feedback.arm_status == 1:
            self.transport.sdk.MotionCtrl_1(0x02, 0, 0)
        try:
            self.transport.send(feedback.positions)
            deadline = self.clock() + 3
            while self.clock() < deadline:
                self.transport.sdk.EnableArm(7)
                self.transport.frames.require(MOTOR_FRAMES)
                if all(self.transport.read().enabled):
                    self.last_sent = feedback.positions
                    self.last_command_time = self.clock()
                    self.enabled = True
                    self.holding = False
                    self.hold_reason = ""
                    self.fault = ""
                    return
                time.sleep(0.05)
            raise RuntimeError("Piper 六轴使能未在 3 秒内确认")
        except (ValueError, RuntimeError) as error:
            # 使能过程可能已改变电机状态；失败时仍须明确发送停止请求。
            self.stop(str(error))
            raise

    def command(self, positions):
        positions = tuple(positions)
        self.validate_positions(positions)
        self.last_command = positions
        self.last_command_time = self.clock()
        if not self.enabled:
            return
        if self.holding:
            return
        feedback = self.preflight()
        if max(abs(a - b) for a, b in zip(positions, self.last_sent)) > 0.1:
            raise RuntimeError("伺服目标发生超过 0.1 rad 的跳变")
        if max(abs(a - b) for a, b in zip(positions, feedback.positions)) > 0.15:
            raise RuntimeError("关节跟踪偏差超过 0.15 rad")
        self.transport.send(positions)
        self.last_sent = positions

    def watchdog(self):
        if self.enabled:
            self.preflight()
            if self.holding:
                self.transport.send(self.hold_target)
                return
            if self.clock() - self.last_command_time > 0.25:
                raise RuntimeError("Piper 伺服指令断流超过 250 ms")

    def hold(self, reason="保持当前位置"):
        feedback = self.preflight()
        if not self.holding:
            self.hold_target = feedback.positions
            self.transport.send(self.hold_target)
            self.last_sent = self.hold_target
        self.holding = True
        self.hold_reason = reason

    def resume_commands(self):
        feedback = self.preflight()
        if self.last_command is None or self.clock() - self.last_command_time > 0.25:
            raise RuntimeError("位置保持尚未与控制器新鲜目标同步")
        if max(abs(a - b) for a, b in zip(self.last_command, feedback.positions)) > 0.03:
            raise RuntimeError("位置保持尚未与控制器目标对齐")
        self.transport.send(feedback.positions)
        self.last_sent = feedback.positions
        self.holding = False
        self.hold_reason = ""

    def stop(self, reason="用户停止"):
        self.enabled = False
        self.holding = False
        self.fault = reason
        self.transport.sdk.MotionCtrl_1(0x01, 0, 0)

    def gripper(self, width_mm, torque_nm):
        if not math.isfinite(width_mm) or not 0 <= width_mm <= 70:
            raise ValueError("夹爪总开口必须在 0–70 mm 内，与官方碰撞模型一致")
        if not math.isfinite(torque_nm) or not 0 < torque_nm <= 1:
            raise ValueError("夹笔力矩必须在 (0, 1] N·m 内")
        self.feedback(motors=True)
        validate_firmware(self.transport.read().firmware, self.protocol, self.dh_version)
        self.transport.sdk.GripperCtrl(round(width_mm * 1000), round(torque_nm * 1000), 0x01, 0)
