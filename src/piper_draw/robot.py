"""可供 CLI 或 Web 后端调用的设备操作：返回数据，失败抛出异常。"""

import json
import platform
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

from .device import DeviceConfig
from .homing import move_to_joint_target
from .piper_backend import (
    FeedbackUnavailable,
    FirmwareNotReady,
    PiperSession,
    SDKTransport,
    validate_firmware,
)
from .recovery import resume as resume_transport

PARKING_PATH = Path("output/hardware/parking.json")


def doctor(config: DeviceConfig, *, configure_slave=False):
    """读取连接与反馈；仅 configure_slave=True 时显式设置从臂角色。"""
    if platform.system() != "Linux":
        raise RuntimeError("Piper SocketCAN 真机驱动需要 Linux")
    if not (Path("/sys/class/net") / config.can).exists():
        raise RuntimeError(f"未找到 CAN 接口 {config.can}；先运行 ip -details link show")
    details = json.loads(subprocess.check_output(
        ["ip", "-details", "-json", "link", "show", config.can], text=True
    ))[0]
    link = details.get("linkinfo", {})
    if link.get("info_kind") != "can":
        raise RuntimeError(f"{config.can} 不是 CAN 接口")
    if "UP" not in details.get("flags", []):
        raise RuntimeError(f"{config.can} 尚未启用；先运行 sudo bash scripts/can_up.sh {config.can}")
    if link.get("info_data", {}).get("bittiming", {}).get("bitrate") != 1_000_000:
        raise RuntimeError(f"{config.can} 必须配置为 1 Mbit/s；先运行 scripts/can_up.sh")
    with SDKTransport(config.can, config.protocol, config.dh_version,
                      lock_dir=config.lock_dir) as transport:
        session = PiperSession(
            transport, [-4.0] * 6, [4.0] * 6, config.protocol, config.dh_version,
            gripper_present=config.gripper_present,
        )
        deadline = time.monotonic() + 3
        configured = False
        while True:
            try:
                validate_firmware(transport.read().firmware, config.protocol, config.dh_version)
                if configure_slave and not configured:
                    transport.sdk.MasterSlaveConfig(0xFC, 0, 0, 0)
                    configured = True
                feedback = session.preflight(require_enabled=False)
                return {
                    "status": "checked",
                    **vars(feedback),
                    "can": config.can,
                    "protocol": config.protocol,
                    "dh_version": config.dh_version,
                    "checked_at": datetime.now(timezone.utc).isoformat(),
                }
            except (FirmwareNotReady, FeedbackUnavailable):
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.05)


def home(config: DeviceConfig, *, speed_percent=3, progress=None, target_rad=(0.0,) * 6):
    if (config.protocol, config.dh_version) != ("v3", "new"):
        raise ValueError("低速关节移动目前仅支持 v3/new")
    with SDKTransport(config.can, config.protocol, config.dh_version,
                      lock_dir=config.lock_dir) as transport:
        return move_to_joint_target(transport, target_rad=target_rad, speed_percent=speed_percent,
                                    progress=progress, gripper_present=config.gripper_present)


def resume(config: DeviceConfig):
    if (config.protocol, config.dh_version) != ("v3", "new"):
        raise ValueError("急停恢复目前仅支持 v3/new")
    with SDKTransport(config.can, config.protocol, config.dh_version,
                      lock_dir=config.lock_dir) as transport:
        return resume_transport(transport, gripper_present=config.gripper_present)


def parking_target(config: DeviceConfig, saved):
    from .calibration import flange_from_joints

    if (config.protocol, config.dh_version) != ("v3", "new"):
        raise ValueError("停放位目前仅支持 v3/new")
    if any(saved[key] != getattr(config, key) for key in ("can", "protocol", "dh_version")):
        raise ValueError("停放位的设备配置已变化，请重新保存")
    positions = saved["positions_rad"]
    flange_from_joints(positions, config.dh_version)
    return tuple(positions)
