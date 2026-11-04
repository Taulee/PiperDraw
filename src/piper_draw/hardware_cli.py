"""设备命令的参数、终端输出和运行记录；业务操作位于 robot.py。"""

import argparse
import json
import math
import signal
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from tempfile import NamedTemporaryFile

from . import robot
from .artifacts import timestamped_path
from .device import DEFAULT_DEVICE_CONFIG, load_device_config


def add_robot_commands(commands):
    parser = commands.add_parser("robot", help="设备诊断、回零、恢复与 ROS 操作")
    actions = parser.add_subparsers(dest="robot_command", required=True)
    connection = argparse.ArgumentParser(add_help=False)
    connection.add_argument("--device-config", type=Path, default=DEFAULT_DEVICE_CONFIG)
    connection.add_argument("--can", help="覆盖设备配置中的 CAN 接口")
    connection.add_argument("--protocol", choices=["v2", "v3"])
    connection.add_argument("--dh-version", choices=["new", "old"])
    for name, description in (
        ("doctor", "查询固件与反馈，不使能、不运动"),
        ("feedback-on", "显式配置从臂角色并检查反馈，不使能、不运动"),
        ("home", "以 1–5% 速度使能并执行六轴回零，保持夹爪状态"),
        ("resume", "解除软件急停，可能失能下落，不自动使能"),
        ("save-parking", "读取六轴角度并保存为停放位"),
        ("park", "以 1–5% 速度使能并移动到停放位"),
    ):
        action = actions.add_parser(
            name, parents=[connection], help=description, description=description
        )
        action.add_argument("--report", type=Path, help="指定本次 JSON 记录路径")
        if name in ("home", "park"):
            action.add_argument("--speed-percent", type=int, choices=range(1, 6), default=3)
    action = actions.add_parser("listen", parents=[connection], help="被动监听 CAN，不占控制连接")
    action.add_argument("--seconds", type=int, default=5)
    actions.add_parser("status", help="读取运行中的 ROS 后端状态")
    actions.add_parser("enable", help="通过 ROS 后端显式使能，保持当前位置")
    actions.add_parser("stop", help="请求 ROS 后端快速停止，保留夹爪夹持")
    action = actions.add_parser("gripper", help="通过 ROS 后端调整夹爪")
    action.add_argument("--width-mm", type=float, required=True)
    action.add_argument("--torque-nm", type=float, default=0.3)


def _write_json(path, report):
    path.parent.mkdir(parents=True, exist_ok=True)
    with NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent, delete=False) as file:
        temporary = Path(file.name)
        try:
            json.dump(report, file, ensure_ascii=False, indent=2)
            file.write("\n")
            file.close()
            temporary.replace(path)
        finally:
            temporary.unlink(missing_ok=True)


def _run_recorded(args):
    operation = args.robot_command
    directory = Path("output/hardware")
    path = args.report or timestamped_path(directory, operation, ".json")
    report = {"operation": operation, "started_at": datetime.now(timezone.utc).isoformat()}
    try:
        config = load_device_config(
            args.device_config, can=args.can, protocol=args.protocol, dh_version=args.dh_version
        )
        report.update(can=config.can, protocol=config.protocol, dh_version=config.dh_version)
        if operation in ("doctor", "feedback-on"):
            result = robot.doctor(config, configure_slave=operation == "feedback-on")
        elif operation == "save-parking":
            feedback = robot.doctor(config)
            saved = {key: getattr(config, key) for key in ("can", "protocol", "dh_version")}
            saved.update(positions_rad=feedback["positions"], saved_at=feedback["checked_at"])
            robot.parking_target(config, saved)
            _write_json(robot.PARKING_PATH, saved)
            result = {"status": "saved", "parking": saved, "positions": feedback["positions"],
                      "firmware": feedback["firmware"]}
        elif operation in ("home", "park"):
            report["speed_percent"] = args.speed_percent
            report["samples"] = []

            def progress(feedback, seconds):
                report["samples"].append({"seconds": seconds, "feedback": dict(vars(feedback))})
                angles = [round(math.degrees(p), 2) for p in feedback.positions]
                print(f"{seconds:.1f}s  六轴角度(°): {angles}", file=sys.stderr, flush=True)

            options = {}
            if operation == "park":
                if not robot.PARKING_PATH.exists():
                    raise ValueError("请先保存停放位")
                saved = json.loads(robot.PARKING_PATH.read_text(encoding="utf-8"))
                options["target_rad"] = robot.parking_target(config, saved)
            result = robot.home(config, speed_percent=args.speed_percent, progress=progress, **options)
        else:
            result = robot.resume(config)
        report.update(result)
        return report
    finally:
        error = sys.exc_info()[1]
        if error is not None:
            report.update(status="failed", error=str(error))
        report["finished_at"] = datetime.now(timezone.utc).isoformat()
        _write_json(path, report)
        _write_json(directory / "latest.json", report)
        print(f"记录：{path}", file=sys.stderr)


def hardware_command(args):
    if args.robot_command in ("status", "enable", "stop", "gripper"):
        from .ros_client import command

        options = {}
        if args.robot_command == "gripper":
            options = {"width_mm": args.width_mm, "torque_nm": args.torque_nm}
        result = command(args.robot_command, **options)
    elif args.robot_command == "listen":
        if args.seconds < 1:
            raise ValueError("监听秒数必须为正整数")
        config = load_device_config(
            args.device_config, can=args.can, protocol=args.protocol, dh_version=args.dh_version
        )
        process = subprocess.run([
            "timeout", "--signal=INT", str(args.seconds), "candump", "-L", config.can
        ], check=False)
        if process.returncode not in (0, 124):
            raise subprocess.CalledProcessError(process.returncode, process.args)
        return
    else:
        def interrupt(signum, frame):
            raise KeyboardInterrupt(f"收到信号 {signum}")

        previous = signal.signal(signal.SIGTERM, interrupt)
        try:
            result = _run_recorded(args)
        finally:
            signal.signal(signal.SIGTERM, previous)
    print(json.dumps(result, ensure_ascii=False, indent=2))
