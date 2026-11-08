"""标定 CLI；采样文件和绘画配置是可供后续 Web 后端复用的 YAML 数据。"""

import argparse
import json
from pathlib import Path
from tempfile import NamedTemporaryFile

import yaml

from .device import DEFAULT_DEVICE_CONFIG, load_device_config


def add_calibration_commands(commands):
    parser = commands.add_parser("calibrate", help="只读采点、TCP 标定、纸面定位与 A5 选区")
    actions = parser.add_subparsers(dest="calibration_command", required=True)
    for name, description in (
        ("capture", "手动定位并保持静止后，追加一次实际法兰位姿"),
        ("tcp", "由同一点的多个姿态求 TCP，撤销旧纸面标定"),
        ("paper", "由左下、右下、右上、左上四点定位纸面，在四边形内选取 A5"),
    ):
        action = actions.add_parser(name, help=description, description=description)
        action.add_argument("--config", type=Path, default=Path("config/real.yaml"))
        action.add_argument("--samples", type=Path, required=True, help="本组法兰位姿 YAML")
        if name == "capture":
            action.add_argument("--source", choices=["sdk", "ros"], default="sdk")
            action.add_argument("--device-config", type=Path, default=DEFAULT_DEVICE_CONFIG)
            action.add_argument("--can")
            action.add_argument("--protocol", choices=["v2", "v3"])
        else:
            action.add_argument("--output", type=Path, required=True, help="输出完整绘画配置 YAML")
        if name == "tcp":
            action.add_argument("--max-error-mm", type=float, default=1.0)
        elif name == "paper":
            action.add_argument("--orientation", choices=["auto", "portrait", "landscape"],
                                default="auto")
            action.add_argument("--max-plane-error-mm", type=float, default=1.0,
                                help="四角拟合平面的最大残差，单位 mm")
            action.add_argument("--confirm-motion", action="store_true", help=argparse.SUPPRESS)


def _save_yaml(path, values):
    path.parent.mkdir(parents=True, exist_ok=True)
    with NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent, delete=False) as file:
        temporary = Path(file.name)
        try:
            yaml.safe_dump(values, file, allow_unicode=True, sort_keys=False)
            file.close()
            temporary.replace(path)
        finally:
            temporary.unlink(missing_ok=True)


def calibration_command(args):
    from .calibration import calibrate_paper, calibrate_tcp, capture_pose, robot_identity

    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    identity = robot_identity(config)
    if args.samples.resolve() == args.config.resolve():
        raise ValueError("采样文件不能与绘画配置使用同一路径")
    if args.calibration_command == "capture" and not args.samples.exists():
        observations = {"robot": identity, "samples": []}
    else:
        observations = yaml.safe_load(args.samples.read_text(encoding="utf-8"))
    if observations["robot"] != identity:
        raise ValueError("样本坐标系或 DH 版本与绘画配置不一致")
    if not isinstance(observations["samples"], list):
        raise TypeError("samples 必须是法兰位姿列表")
    if args.calibration_command == "capture":
        device = load_device_config(
            args.device_config, can=args.can, protocol=args.protocol
        ) if args.source == "sdk" else None
        sample = capture_pose(config, source=args.source, device=device)
        observations["samples"].append(sample)
        _save_yaml(args.samples, observations)
        result = {"samples": str(args.samples), "count": len(observations["samples"]),
                  "pose": sample}
    else:
        if args.output.resolve() == args.samples.resolve():
            raise ValueError("输出配置不能覆盖采样文件")
        if args.calibration_command == "tcp":
            updated = calibrate_tcp(config, observations, max_error_mm=args.max_error_mm)
        else:
            updated = calibrate_paper(
                config, observations, orientation=args.orientation,
                max_plane_error_mm=args.max_plane_error_mm,
            )
        _save_yaml(args.output, updated)
        result = {"config": str(args.output), "calibrated": updated["calibrated"],
                  "result": updated["calibration"][args.calibration_command]}
    print(json.dumps(result, ensure_ascii=False, indent=2))
