"""设备连接配置与进程间控制权；CLI、SDK 操作和 ROS 共用。"""

import os
import re
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path

import yaml

from .completion import IdleSettings, load_idle_settings

DEFAULT_DEVICE_CONFIG = Path("config/device.yaml")


@dataclass(frozen=True)
class DeviceConfig:
    can: str
    protocol: str
    dh_version: str
    lock_dir: Path
    gripper_present: bool = True
    firmware_speed_percent: int = 100
    return_to_idle: IdleSettings = IdleSettings()


def validate_firmware_speed_percent(value):
    if type(value) is not int or not 1 <= value <= 100:
        raise ValueError("firmware_speed_percent 必须是 1–100 的整数")
    return value


def load_device_config(path=DEFAULT_DEVICE_CONFIG, *, can=None, protocol=None, dh_version=None):
    path = Path(path)
    values = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(values, dict):
        raise TypeError("设备配置必须是 YAML 映射")
    can = values["can"] if can is None else can
    protocol = values["protocol"] if protocol is None else protocol
    dh_version = values["dh_version"] if dh_version is None else dh_version
    if not isinstance(can, str) or re.fullmatch(r"[a-zA-Z0-9_.:-]{1,15}", can) is None:
        raise ValueError("CAN 接口名必须是 1–15 个字母、数字或 _.:- 字符")
    if protocol not in ("v2", "v3") or dh_version not in ("new", "old"):
        raise ValueError("设备协议必须是 v2/v3，DH 版本必须是 new/old")
    lock_dir = values["lock_dir"]
    if not isinstance(lock_dir, str) or not lock_dir.strip():
        raise ValueError("设备配置 lock_dir 必须是非空路径字符串")
    gripper_present = values.get("gripper_present", True)
    if type(gripper_present) is not bool:
        raise ValueError("gripper_present 必须为 true 或 false")
    firmware_speed = validate_firmware_speed_percent(values.get("firmware_speed_percent", 100))
    return DeviceConfig(
        can, protocol, dh_version, (path.parent / lock_dir).resolve(), gripper_present,
        firmware_speed,
        load_idle_settings(values.get("return_to_idle", {})),
    )


class DeviceBusyError(RuntimeError):
    """同一 CAN 接口已被本项目的另一控制连接持有。"""


@contextmanager
def device_lock(can, directory):
    import fcntl

    directory.mkdir(parents=True, exist_ok=True)
    # 保留文件，避免 unlink 后两个进程锁住不同 inode。
    with (directory / f"{can}.lock").open("a+", encoding="utf-8") as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            handle.seek(0)
            owner = handle.read().strip()
            raise DeviceBusyError(f"{can} 正被占用（{owner}）；先停止持有连接的进程") from error
        try:
            handle.seek(0)
            handle.truncate()
            handle.write(f"pid={os.getpid()}")
            handle.flush()
            yield
        finally:
            fcntl.flock(handle, fcntl.LOCK_UN)
