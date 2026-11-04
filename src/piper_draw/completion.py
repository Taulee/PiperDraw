"""绘画完成、取消和可受控故障的待机位配置；本模块不发送运动。"""

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class IdleSettings:
    enabled: bool = False
    velocity_scale: float = 0.2
    acceleration_scale: float = 0.4
    min_tip_height_mm: float = 4.0
    timeout_s: float = 120.0
    positions_rad: tuple[float, ...] | None = None


def load_idle_settings(values):
    if not isinstance(values, dict):
        raise ValueError("return_to_idle 必须是 YAML 映射")
    settings = {**asdict(IdleSettings()), **values}
    if type(settings["enabled"]) is not bool:
        raise ValueError("return_to_idle.enabled 必须为 true 或 false")
    for name in ("velocity_scale", "acceleration_scale", "min_tip_height_mm", "timeout_s"):
        value = settings[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"return_to_idle.{name} 必须是有限正数")
        if name.endswith("_scale") and value > 1:
            raise ValueError(f"return_to_idle.{name} 必须在 (0, 1] 内")
        settings[name] = float(value)
    positions = settings["positions_rad"]
    if positions is not None:
        if not isinstance(positions, (list, tuple)) or len(positions) != 6 or not all(
            not isinstance(x, bool) and isinstance(x, (int, float)) and math.isfinite(x)
            for x in positions
        ):
            raise ValueError("return_to_idle.positions_rad 必须是六个有限关节角，单位 rad")
        settings["positions_rad"] = tuple(float(v) for v in positions)
    try:
        return IdleSettings(**settings)
    except TypeError as error:
        raise ValueError(f"未知待机位配置：{error}") from error


def prepare_idle_return(job, device, parking_path=Path("output/hardware/parking.json")):
    """执行前固定待机目标，并用本次 TCP、纸面和官方关节限位核对。"""
    settings = device.return_to_idle
    if not settings.enabled:
        return None
    from .calibration import flange_from_joints
    from scipy.spatial.transform import Rotation

    positions = settings.positions_rad
    if positions is None:
        saved = json.loads(Path(parking_path).read_text(encoding="utf-8"))
        if tuple(saved.get(k) for k in ("can", "protocol", "dh_version")) != (
            device.can, device.protocol, device.dh_version
        ):
            raise ValueError("待机位与当前设备配置不同，请重新保存待机位")
        positions = saved["positions_rad"]
    if job["robot"]["dh_version"] != device.dh_version:
        raise ValueError("待机位返回使用的 DH 与绘画模型不同")
    pose = flange_from_joints(positions, device.dh_version)
    positions = [float(v) for v in positions]
    tip = [a + b for a, b in zip(
        pose["position_mm"], Rotation.from_quat(pose["quaternion_xyzw"]).apply(job["tool"]["tcp_offset_mm"])
    )]
    height = sum((a / 1000 - b) * n for a, b, n in zip(tip, job["paper_origin"], job["paper_normal"])) * 1000
    if height < settings.min_tip_height_mm:
        raise ValueError("待机位的笔尖低于本次纸面净空要求，请重新保存待机位")
    return {"positions_rad": list(positions), "settings": asdict(settings), "job": job}


def finish_run(plan, report_path, *, reason="completed"):
    """独立 run 的统一收尾；失败时保留反馈位置保持，绝不自动关闭后端。"""
    import os
    import signal
    import time
    from .calibration_cli import _save_yaml
    from .workflow_ros import RobotConnection

    path = Path(report_path)
    report = yaml.safe_load(path.read_text()) if path.is_file() else {}
    report = report or {}
    report["drawing_status"] = report.get("status", reason)
    report["status"] = "returning_to_idle"
    report["idle_return"] = {"status": "returning", "reason": reason}
    _save_yaml(path, report)
    connection = None
    started = time.monotonic()
    previous = signal.getsignal(signal.SIGTERM)

    def interrupted(_signum, _frame):
        raise KeyboardInterrupt("返程被中断，保持当前位置")

    signal.signal(signal.SIGTERM, interrupted)
    try:
        connection = RobotConnection(domain_id=int(os.environ.get("ROS_DOMAIN_ID", "106")))
        result = connection.return_to_idle(plan)
        report["idle_return"].update(status="returned", seconds=time.monotonic() - started, **result)
        report["status"] = "executed" if reason == "completed" else reason
    except BaseException as error:
        report["idle_return"].update(status="failed", error=str(error))
        report["status"] = "failed"
        report["error"] = f"返回待机位失败：{error}"
        raise
    finally:
        signal.signal(signal.SIGTERM, previous)
        _save_yaml(path, report)
        if connection is not None:
            connection.close()
