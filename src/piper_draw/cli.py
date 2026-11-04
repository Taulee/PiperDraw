import argparse
import os
import shutil
import signal
import subprocess
from pathlib import Path
from tempfile import TemporaryDirectory

import yaml

from .app_config import load_app_config
from .artifacts import Artifacts, timestamped_path
from .calibration_cli import add_calibration_commands
from .device import DEFAULT_DEVICE_CONFIG
from .hardware_cli import add_robot_commands


def launch(file, **parameters):
    local_launch = Path(__file__).resolve().parents[2] / "ros2_ws/src/piper_drawing/launch" / file
    arguments = (["ros2", "launch", str(local_launch)] if local_launch.is_file()
                 else ["ros2", "launch", "piper_drawing", file])
    for name, value in parameters.items():
        value = str(value).lower() if isinstance(value, bool) else str(value)
        arguments.append(f"{name}:={value}")
    process = subprocess.Popen(arguments, start_new_session=True)
    previous = signal.getsignal(signal.SIGTERM)

    def interrupted(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    try:
        try:
            code = process.wait()
        except KeyboardInterrupt:
            # 原执行器先退出，避免它的延迟取消请求影响随后返程。
            for signum, timeout in ((signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
                if process.poll() is not None:
                    break
                try:
                    os.killpg(process.pid, signum)
                except ProcessLookupError:
                    break
                try:
                    process.wait(timeout=timeout)
                    break
                except subprocess.TimeoutExpired:
                    continue
            raise
        if code:
            raise subprocess.CalledProcessError(code, arguments)
    finally:
        signal.signal(signal.SIGTERM, previous)


def _generate_line_art(args, artifacts: Artifacts) -> Path:
    from .seedream import SeedreamAPIError, generate_line_art, load_seedream_config

    prompt = args.prompt.read_text(encoding="utf-8")
    config_text = args.seedream_config.read_text(encoding="utf-8")
    config = load_seedream_config(args.seedream_config)
    source_path = artifacts.path("source", args.image.suffix.lower())
    shutil.copyfile(args.image, source_path)
    reference_paths = []
    for index, reference in enumerate(args.reference or (), start=1):
        reference_path = artifacts.path(f"reference_{index}", reference.suffix.lower())
        shutil.copyfile(reference, reference_path)
        reference_paths.append(reference_path)
    artifacts.path("prompt", ".txt").write_text(prompt, encoding="utf-8")
    artifacts.path("seedream", ".yaml").write_text(config_text, encoding="utf-8")
    generation_options = {"reference_images": reference_paths} if reference_paths else {}
    try:
        line_art = generate_line_art(
            source_path, prompt, config, artifacts.path("line_art", ".png"), **generation_options
        )
    except SeedreamAPIError as error:
        raise SystemExit(f"{error}\n生成配置：{args.seedream_config}") from None
    print(f"线稿：{line_art}")
    return line_art


def main(argv=None):
    parser = argparse.ArgumentParser(prog="piper-draw", description="PIPER 照片转线稿与绘画")
    commands = parser.add_subparsers(dest="command", required=True)
    p = commands.add_parser("draw", help="从图片或历史任务开始完整绘画")
    p.add_argument("input", help="图片路径，或 history 显示的任务 ID")
    p.add_argument("--history", action="store_true", help="输入是历史任务 ID")
    p.add_argument("--mode", choices=["real", "mock"], default="real")
    p.add_argument("--config", type=Path)
    p.add_argument("--device-config", type=Path)
    sources = p.add_mutually_exclusive_group(required=True)
    sources.add_argument("--from-line-art", dest="source", action="store_const", const="line_art")
    sources.add_argument("--from-photo", dest="source", action="store_const", const="photo")
    sources.add_argument("--reuse-path", dest="source", action="store_const", const="path")
    p = commands.add_parser("history", help="查看共用绘画历史")
    p.add_argument("--delete", nargs="+", metavar="ID", help="删除指定任务")
    commands.add_parser("stop", help="停止当前绘画")
    image_options = argparse.ArgumentParser(add_help=False)
    image_options.add_argument("--seedream-config", type=Path, default=Path("config/seedream.yaml"))
    image_options.add_argument("--app-config", type=Path, default=Path("config/app.yaml"))
    image_options.add_argument("--prompt", type=Path, help="覆盖应用配置中的提示词")
    image_options.add_argument(
        "--reference", type=Path, action="append", help="覆盖配置参考图，可重复指定，按入参顺序发送"
    )
    p = commands.add_parser(
        "line-art", parents=[image_options], help="调用 Seedream 将照片转为线稿"
    )
    p.add_argument("image", type=Path)
    p = commands.add_parser("prepare", parents=[image_options], allow_abbrev=False,
                            help="生成笔画预览和机器人任务")
    p.add_argument("image", type=Path)
    p.add_argument("--config", type=Path, default=Path("config/sim.yaml"))
    p.add_argument("--output-dir", type=Path, help="任务产物的父目录，默认 output")
    source_options = p.add_mutually_exclusive_group()
    source_options.add_argument(
        "--from-photo",
        dest="source",
        action="store_const",
        const="photo",
        help="覆盖应用配置，先调用 Seedream 将照片转为线稿",
    )
    source_options.add_argument(
        "--from-line-art",
        dest="source",
        action="store_const",
        const="line_art",
        help="覆盖应用配置，直接处理已有线稿，不调用生成服务",
    )
    p = commands.add_parser("tune", help="离线评估线稿路径参数，为全部输入选择同一配置")
    p.add_argument("images", type=Path, nargs="+")
    p.add_argument("--config", type=Path, default=Path("config/sim.yaml"))
    p.add_argument("--tuning-config", type=Path, default=Path("config/tuning.yaml"))
    p = commands.add_parser("regress", help="按固定提示词链调用 API，重复回归并联合调参")
    p.add_argument("images", type=Path, nargs="+")
    p.add_argument("--workflow", type=Path, default=Path("config/regression.yaml"))
    p.add_argument("--repeats", type=int, default=1)
    p.add_argument("--config", type=Path, default=Path("config/sim.yaml"))
    p.add_argument("--tuning-config", type=Path, default=Path("config/tuning.yaml"))
    p.add_argument("--seedream-config", type=Path, default=Path("config/seedream.yaml"))
    p.add_argument("--app-config", type=Path, default=Path("config/app.yaml"))
    p.add_argument(
        "--reference", type=Path, action="append", help="覆盖配置参考图，可重复指定，按入参顺序发送"
    )
    p = commands.add_parser("bringup", help="启动无头模型、场景、MoveIt 和控制器")
    p.add_argument("--mode", choices=["mock", "real"], default="mock")
    p.add_argument("--config", type=Path, default=Path("config/sim.yaml"))
    p.add_argument("--device-config", type=Path, default=DEFAULT_DEVICE_CONFIG)
    p.add_argument("--can", help="覆盖设备配置中的 SocketCAN 接口")
    p.add_argument("--protocol", choices=["v2", "v3"])
    p.add_argument("--dh-version", choices=["new", "old"])
    add_robot_commands(commands)
    add_calibration_commands(commands)
    p = commands.add_parser("web", help="启动本地 Web 控制台，支持 SSH 端口转发")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--config", type=Path, default=Path("config/real.yaml"),
                   help="首次启动时复制到 Web 工作配置")
    p.add_argument("--device-config", type=Path, default=DEFAULT_DEVICE_CONFIG)
    for command, help_text in (("plan", "规划任务"), ("run", "规划并执行任务")):
        p = commands.add_parser(command, help=help_text)
        p.add_argument("job", type=Path)
        p.add_argument("--mode", choices=["mock", "real"], default="mock")
        p.add_argument("--report", type=Path, help="自定义报告路径；默认在任务同级保存时间戳报告")
        p.add_argument("--device-config", type=Path, default=DEFAULT_DEVICE_CONFIG)
        p.add_argument("--no-return-to-idle", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.command in ("line-art", "prepare", "regress"):
        app_config = load_app_config(args.app_config)
        if args.command == "prepare":
            args.source = args.source or app_config.source
            if args.source == "line_art" and args.reference:
                parser.error("prepare 使用 --reference 时不能选择线稿输入")
        if args.command != "regress" and args.prompt is None:
            args.prompt = app_config.prompt
        if args.reference is None:
            args.reference = app_config.references
    if args.command in ("draw", "history", "stop"):
        import json

        from .workflow import WORK_CONFIG, WORK_DEVICE, delete_history, draw, history, stop_drawing

        try:
            if args.command == "draw":
                args.device_config = args.device_config or (WORK_DEVICE if WORK_DEVICE.exists() else DEFAULT_DEVICE_CONFIG)
                args.config = args.config or (WORK_CONFIG if args.mode == "real" else Path("config/sim.yaml"))
                if args.mode == "real" and not args.config.exists() and args.config == WORK_CONFIG:
                    args.config = Path("config/real.yaml")
                draw(args)
            elif args.command == "history":
                result = delete_history(args.delete) if args.delete else history()
                print(json.dumps(result, ensure_ascii=False, indent=2))
            else:
                print("已请求停止" if stop_drawing() else "当前没有绘画任务")
        except (OSError, RuntimeError, ValueError) as error:
            parser.exit(1, f"{error}\n")
    elif args.command == "line-art":
        _generate_line_art(args, Artifacts.create())
    elif args.command == "prepare":
        from .job import prepare

        if args.source == "photo":
            args.config.resolve(strict=True)
            artifacts = Artifacts.create(args.output_dir or Path("output"))
            line_art = _generate_line_art(args, artifacts)
            prepare(line_art, args.config, artifacts)
        else:
            if args.output_dir is None:
                prepare(args.image, args.config)
            else:
                prepare(args.image, args.config, Artifacts.create(args.output_dir))
    elif args.command == "tune":
        from .tuning import tune_paths

        tune_paths(args.images, args.config, args.tuning_config)
    elif args.command == "regress":
        from .regression import regress
        from .seedream import SeedreamAPIError

        if args.repeats < 1:
            parser.error("--repeats 必须是正整数")
        try:
            regress(
                args.images,
                args.workflow,
                args.seedream_config,
                args.config,
                args.tuning_config,
                args.repeats,
                reference_images=args.reference,
            )
        except SeedreamAPIError as error:
            raise SystemExit(f"{error}\n生成配置：{args.seedream_config}") from None
    elif args.command == "web":
        from .web import serve

        serve(args.host, args.port, args.config, args.device_config)
    elif args.command == "bringup":
        parameters = {
            "mode": args.mode, "config": args.config.resolve(),
            "device_config": args.device_config.resolve(),
        }
        for key in ("can", "protocol", "dh_version"):
            value = getattr(args, key)
            if value is not None:
                parameters["can_port" if key == "can" else key] = value
        launch("bringup.launch.py", **parameters)
    elif args.command == "calibrate":
        from .calibration_cli import calibration_command

        try:
            calibration_command(args)
        except (OSError, RuntimeError, ValueError, TypeError, KeyError, yaml.YAMLError) as error:
            parser.exit(1, f"{error}\n")
    elif args.command == "robot":
        from .hardware_cli import hardware_command

        try:
            hardware_command(args)
        except (
            OSError, RuntimeError, ValueError, TypeError, KeyError,
            subprocess.CalledProcessError, yaml.YAMLError,
        ) as error:
            parser.exit(1, f"{error}\n")
    else:
        execute = args.command == "run"
        job_path = args.job.resolve(strict=True)
        legacy_job = None
        if args.mode == "real":
            from .job import has_legacy_job_calibration

            values = yaml.safe_load(job_path.read_text(encoding="utf-8"))
            if isinstance(values, dict) and has_legacy_job_calibration(job_path, values):
                legacy_job = {**values, "calibrated": True}
        if args.report is None:
            name = "execute_report" if execute else "plan_report"
            report_path = timestamped_path(job_path.parent, name, ".yaml")
            # 独占创建，拒绝覆盖碰巧同名的旧报告。
            report_path.touch(exist_ok=False)
        else:
            report_path = args.report.resolve()
            report_path.parent.mkdir(parents=True, exist_ok=True)
            report_path.unlink(missing_ok=True)
        print(f"报告：{report_path}")
        idle_return = None
        if execute and args.mode == "real" and not args.no_return_to_idle:
            from .completion import prepare_idle_return
            from .device import load_device_config

            idle_return = prepare_idle_return(
                yaml.safe_load(job_path.read_text()), load_device_config(args.device_config)
            )
            if idle_return:
                from .ros_client import command

                status = command("status")
                if status.get("session_enabled") and status.get("holding"):
                    command("resume_commands")
        options = {"mode": args.mode, "execute": execute, "report": report_path}
        try:
            if legacy_job is None:
                launch("draw.launch.py", job=job_path, **options)
            else:
                with TemporaryDirectory(prefix="piper-legacy-job-") as directory:
                    runtime_job = Path(directory) / job_path.name
                    runtime_job.write_text(yaml.safe_dump(legacy_job), encoding="utf-8")
                    launch("draw.launch.py", job=runtime_job, **options)
        except (KeyboardInterrupt, subprocess.CalledProcessError) as error:
            if idle_return:
                from .completion import finish_run

                print("绘画已中断，正在保持并返回待机位", flush=True)
                finish_run(idle_return, report_path,
                           reason="cancelled" if isinstance(error, KeyboardInterrupt) else "failed")
                print("已回待机位，持续保持位置控制", flush=True)
            raise
        report = yaml.safe_load(report_path.read_text(encoding="utf-8"))
        if not isinstance(report, dict) or report.get("status") not in (
            "planned",
            "executed",
            "failed",
        ):
            raise RuntimeError(f"绘画后端未生成有效报告：{report_path}")
        if report["status"] == "failed":
            if idle_return:
                from .completion import finish_run

                finish_run(idle_return, report_path, reason="failed")
            raise SystemExit(report["error"])
        if idle_return and report["status"] == "executed":
            from .completion import finish_run

            print("绘画完成，正在返回待机位", flush=True)
            finish_run(idle_return, report_path)
            print("已回待机位，持续保持位置控制", flush=True)


if __name__ == "__main__":
    main()
