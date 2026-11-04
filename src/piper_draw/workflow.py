"""One drawing lifecycle and history for the CLI and Web."""
import fcntl
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from contextlib import contextmanager
from pathlib import Path
from uuid import uuid4

import yaml

from .artifacts import Artifacts
from .calibration_cli import _save_yaml
from .device import load_device_config

OUTPUT = Path("output")
RUNS = OUTPUT / "draws"
WORK_CONFIG = OUTPUT / "web/drawing.yaml"
WORK_DEVICE = OUTPUT / "web/device.yaml"
WORK_SEEDREAM = OUTPUT / "web/seedream.yaml"
BACKEND = OUTPUT / ".draw-backend.yaml"
STAMP = re.compile(r"\d{8}_\d{6}_\d{6}")
TERMINAL = {"done", "failed", "stopped", "interrupted", "stop_unconfirmed"}


class WorkflowBusy(RuntimeError):
    pass


class Cancelled(Exception):
    pass


@contextmanager
def workflow_lock(root=Path(".")):
    (root / OUTPUT).mkdir(exist_ok=True)
    with (root / OUTPUT / ".draw.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise WorkflowBusy("机械臂正在处理另一个任务") from None
        yield


def drawing_active(root=Path(".")):
    try:
        with workflow_lock(root):
            return False
    except WorkflowBusy:
        return True


def history(root=Path(".")):
    items = []
    for base in (root / RUNS, root / OUTPUT / "web/jobs", root / OUTPUT):
        for directory in base.iterdir() if base.is_dir() and not base.is_symlink() else ():
            if not STAMP.fullmatch(directory.name) or not directory.is_dir() or directory.is_symlink():
                continue
            if not directory.resolve().is_relative_to((root / OUTPUT).resolve()):
                continue
            files = Artifacts(directory, directory.name)
            status_path = directory / "workflow.yaml"
            job = files.path("job", ".yaml")
            if not (status_path.is_file() or job.is_file()):
                continue
            line = files.path("line_art", ".png")
            photo = next((p for p in directory.glob(f"{directory.name}_source.*") if p.is_file()), None)
            if status_path.is_file():
                state = read_yaml(status_path) or {}
            else:
                reports = sorted(directory.glob("*_execute_report.yaml"), reverse=True)
                result = read_yaml(reports[0]) if reports else {}
                state = {"phase": {"executed": "done", "failed": "failed"}.get(
                    (result or {}).get("status"), "prepared")}
            items.append({
                **state, "id": directory.relative_to(root).as_posix(), "time": directory.name,
                "line_art": line.relative_to(root).as_posix() if line.is_file() else None,
                "photo": photo.relative_to(root).as_posix() if photo else None,
                "job": job.relative_to(root).as_posix() if job.is_file() else None,
                "preview": files.path("preview", ".svg").relative_to(root).as_posix()
                if files.path("preview", ".svg").is_file() else None,
            })
    return sorted(items, key=lambda row: row["time"], reverse=True)


def read_yaml(path):
    try:
        return yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError):
        return None


def report_progress(report):
    segments = report.get("segments") or []
    done = report.get("completed_segments", 0)
    if not segments:
        return {}
    lifts = [isinstance(segment, dict) and segment.get("kind") == "lift" for segment in segments]
    return {
        "completed_strokes": sum(lifts[:done]),
        "total_strokes": sum(lifts),
    }


def find_record(identifier, root=Path(".")):
    for record in history(root):
        if record["id"] == identifier:
            return record
    raise ValueError("找不到选中的历史任务")


def delete_history(identifiers, root=Path(".")):
    if not isinstance(identifiers, list) or not identifiers or not all(isinstance(item, str) for item in identifiers):
        raise ValueError("请选择要删除的历史任务")
    with workflow_lock(root):
        known = {row["id"] for row in history(root)}
        if not set(identifiers) <= known:
            raise ValueError("部分历史任务已不存在，请刷新后重试")
        paths = [root / item for item in dict.fromkeys(identifiers)]
        for path in paths:
            shutil.rmtree(path)
    return len(paths)


def active_record(root=Path(".")):
    base = root / RUNS
    directories = sorted(base.glob("*/workflow.yaml"), reverse=True)
    if not directories:
        return None
    path = directories[0]
    state = read_yaml(path) or {}
    preview = Artifacts(path.parent, path.parent.name).path("preview", ".svg")
    state.update(id=path.parent.relative_to(root).as_posix(),
                 preview=preview.relative_to(root).as_posix() if preview.is_file() else None)
    if state.get("phase") not in TERMINAL:
        if not drawing_active(root):
            state["phase"] = "interrupted"
            state["message"] = "上次绘画已中断，请重新发起"
        else:
            report = read_yaml(root / state["id"] / "execute.yaml") or {}
            if report.get("status") == "executing":
                state["phase"] = "drawing"
                state.update(report_progress(report))
            if report.get("status") == "returning_to_idle":
                state.update(phase="returning_to_idle", message="正在返回待机位")
                state.update(report_progress(report))
            if (path.parent / "stop").exists() and state.get("phase") != "returning_to_idle":
                state.update(phase="stopping", message="正在请求停止")
    return state


def stop_drawing(root=Path(".")):
    record = active_record(root)
    if not record or record.get("phase") in TERMINAL:
        return False
    (root / record["id"] / "stop").touch()
    return True


def child_command(*args):
    return [sys.executable, "-u", "-m", "piper_draw.cli", *map(str, args)]


def end_process(process):
    if process is None or process.poll() is not None:
        return
    for sig, timeout in ((signal.SIGINT, 15), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
        try:
            os.killpg(process.pid, sig)
            process.wait(timeout=timeout)
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            continue


def managed_backend(root=Path(".")):
    data = read_yaml(root / BACKEND)
    if not isinstance(data, dict):
        return None
    pid, token = data.get("pid"), data.get("token")
    if not isinstance(pid, int) or pid <= 1 or not isinstance(token, str):
        return None
    process_env = Path(f"/proc/{pid}/environ")
    expected = f"PIPER_DRAW_BACKEND={token}".encode()
    try:
        if expected in process_env.read_bytes().split(b"\0"):
            return pid, expected
    except OSError:
        pass
    return None


def stop_managed_backend(root=Path(".")):
    """Only signal a backend started by this workflow in this process namespace."""
    backend = managed_backend(root)
    if backend is None:
        return False
    pid, expected = backend
    process_env = Path(f"/proc/{pid}/environ")
    try:
        os.killpg(pid, signal.SIGINT)
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            if not process_env.exists() or expected not in process_env.read_bytes().split(b"\0"):
                return True
            time.sleep(0.1)
        raise RuntimeError("机械臂连接尚未释放，请检查状态后重试")
    except (ProcessLookupError, FileNotFoundError):
        return True
    except PermissionError:
        return False
    finally:
        if managed_backend(root) is None:
            (root / BACKEND).unlink(missing_ok=True)


class Draw:
    def __init__(self, args, files):
        self.args = args
        self.files = files
        self.directory = files.directory
        self.status_path = self.directory / "workflow.yaml"
        self.backend = None
        self.process = None
        self.motion = False
        self.had_motion = False
        self.idle_return = None
        self.return_attempted = False
        self.return_error = None
        self.return_cancelled = False
        self.cancelled = False
        self.state = {"mode": args.mode, "source": args.source}
        self.ros = None
        self.env = dict(os.environ)
        if args.mode == "mock":
            # A simulation must not discover a real backend in the normal domain.
            real_domain = int(self.env.get("ROS_DOMAIN_ID", "106"))
            self.env["ROS_DOMAIN_ID"] = str(107 if real_domain != 107 else 108)

    def stage(self, phase, message, **fields):
        self.state.update(phase=phase, message=message, **fields)
        _save_yaml(self.status_path, self.state)
        print(message, flush=True)

    def check_stop(self):
        if self.cancelled or (self.directory / "stop").exists():
            raise Cancelled

    def spawn(self, log, *args, env=None):
        with (self.directory / log).open("wb") as stream:
            return subprocess.Popen(child_command(*args), stdin=subprocess.DEVNULL,
                                    stdout=stream, stderr=subprocess.STDOUT, start_new_session=True,
                                    env=self.env if env is None else env)

    def command(self, log, *args, report=None):
        self.check_stop()
        self.process = self.spawn(log, *args)
        while self.process.poll() is None:
            self.check_stop()
            time.sleep(0.15)
        self.check_stop()
        result = read_yaml(report) if report else None
        if self.process.returncode or (report and not result):
            output = (self.directory / log).read_text(encoding="utf-8", errors="replace").strip()
            error = (result or {}).get("error") or (output.splitlines()[-1] if output else "命令未返回结果")
            raise RuntimeError(error)
        return result

    def prepare(self):
        from .calibration import is_calibrated

        config = read_yaml(self.args.config)
        if not config:
            raise ValueError("找不到绘画配置，请先在「标定与姿态」完成标定")
        if self.args.mode == "real" and not is_calibrated(config):
            raise ValueError("请先在「标定与姿态」完成 TCP 和纸面定位")
        config_path = self.directory / "config.yaml"
        _save_yaml(config_path, config)
        device_path = self.directory / "device.yaml"
        device = load_device_config(self.args.device_config)
        device_values = read_yaml(self.args.device_config)
        _save_yaml(device_path, {**device_values, "lock_dir": str(device.lock_dir)})
        job = self.files.path("job", ".yaml")
        source = self.args.input
        if self.args.history:
            record = find_record(source)
            key = {"photo": "photo", "line_art": "line_art", "path": "job"}[self.args.source]
            if not record[key]:
                raise ValueError("该记录没有所选素材，请选择另一种方式或重新选图")
            source = record[key]
        elif self.args.source == "path":
            raise ValueError("复用路径时请选择历史任务")
        source = Path(source)
        self.state["title"] = source.name
        self.stage("preparing", "正在准备线稿")
        if self.args.source == "path":
            from .job import has_legacy_job_calibration

            values = read_yaml(source)
            if not values or values.get("calibration_id") != config.get("calibration_id"):
                raise ValueError("此路径使用旧标定，请用原线稿重新规划")
            if self.args.mode == "real" and not (
                values.get("calibrated") is True or has_legacy_job_calibration(source, values)
            ):
                raise ValueError("此路径缺少实机标定，请用原线稿重新规划")
            _save_yaml(job, {**values, "calibrated": True} if self.args.mode == "real" else values)
            preview = source.with_name(source.name.replace("_job.yaml", "_preview.svg"))
            if preview.is_file():
                shutil.copyfile(preview, self.files.path("preview", ".svg"))
            return config_path, device_path, device, job
        # prepare owns its own artifacts directory. Move only this invocation's files
        # into the shared record, using the history ID as their common filename prefix.
        scratch = self.directory / "prepare"
        flag = "--from-photo" if self.args.source == "photo" else "--from-line-art"
        options = ("--seedream-config", WORK_SEEDREAM) if WORK_SEEDREAM.is_file() else ()
        self.command("prepare.log", "prepare", source, flag, "--config", config_path,
                     "--output-dir", scratch, *options)
        produced, = scratch.iterdir()
        for item in produced.iterdir():
            suffix = item.name.removeprefix(produced.name)
            if suffix.startswith("_"):
                item.rename(self.directory / f"{self.files.timestamp}{suffix}")
        produced.rmdir()
        scratch.rmdir()
        if self.args.source == "line_art":
            import cv2

            image = cv2.imread(str(source))
            if image is None or not cv2.imwrite(str(self.files.path("line_art", ".png")), image):
                raise RuntimeError("无法保存本次线稿")
        return config_path, device_path, device, job

    def run(self):
        self.stage("checking", "正在检查任务")
        try:
            from .workflow_ros import RobotConnection
        except ImportError as error:
            raise RuntimeError("当前环境缺少绘画控制组件，请使用项目的 Docker 入口") from error
        config, device_path, device, job = self.prepare()
        if self.args.mode == "real":
            from .completion import prepare_idle_return

            self.idle_return = prepare_idle_return(read_yaml(job), device)
        self.stage("connecting", "正在连接机械臂" if self.args.mode == "real" else "正在启动仿真")
        self.ros = RobotConnection(domain_id=int(self.env.get("ROS_DOMAIN_ID", "106")))
        if not self.ros.controller_available():
            token = uuid4().hex
            self.backend = self.spawn("backend.log", "bringup", "--mode", self.args.mode,
                                      "--config", config, "--device-config", device_path,
                                      env={**self.env, "PIPER_DRAW_BACKEND": token})
            if self.args.mode == "real":
                _save_yaml(BACKEND, {"pid": self.backend.pid, "token": token})
        self.ros.wait_ready(self.args.mode, device, self.backend, self.check_stop)
        self.stage("planning", "正在规划")
        options = ("--mode", self.args.mode)
        plan = self.directory / "plan.yaml"
        planned = self.command("plan.log", "plan", job, *options, "--report", plan, report=plan)
        if planned.get("status") != "planned":
            raise RuntimeError("路径规划未完成")
        fields = {"estimated_seconds": planned.get("estimated_seconds"),
                  "total_strokes": sum(s["kind"] == "lift" for s in planned.get("segments", []))}
        if self.args.mode == "real":
            self.stage("enabling", "正在准备机械臂运动", **fields)
            self.check_stop()
            # From here a stop request must reach the robot before processes exit.
            self.ros.enable(self.check_stop, self.mark_motion)
        self.stage("starting", "正在核对起点并开始绘画", **fields)
        report = self.directory / "execute.yaml"
        result = self.command("execute.log", "run", job, *options, "--report", report,
                              "--no-return-to-idle", report=report)
        if result.get("status") != "executed":
            raise RuntimeError("绘画未完成")
        if self.idle_return:
            self.return_to_idle("completed")
        self.motion = False
        self.stage("done", "已画完", completed_strokes=fields["total_strokes"])

    def mark_motion(self):
        self.motion = True
        self.had_motion = True

    def return_to_idle(self, reason):
        self.return_attempted = True
        report_path = self.directory / "execute.yaml"
        report = read_yaml(report_path) or {}
        drawing_status = report.get("status", reason)
        report.update(status="returning_to_idle", drawing_status=drawing_status,
                      idle_return={"status": "returning", "reason": reason})
        _save_yaml(report_path, report)
        self.stage("returning_to_idle", "正在返回待机位", **report_progress(report))
        try:
            def check():
                if self.return_cancelled:
                    raise Cancelled("返程已中断，保持当前位置")

            result = self.ros.return_to_idle(self.idle_return, check)
            report["idle_return"].update(status="returned", **result)
            report["status"] = drawing_status
            self.motion = False
            self.state["idle_returned"] = True
        except (Cancelled, RuntimeError, ValueError) as error:
            report["idle_return"].update(status="failed", error=str(error))
            self.state["idle_returned"] = False
            self.return_error = str(error)
            raise
        finally:
            _save_yaml(report_path, report)

    def stop_motion(self):
        if not self.motion:
            return True
        try:
            self.ros.hold_and_cancel()
            return True
        except (RuntimeError, OSError):
            return False


def draw(args):
    with workflow_lock():
        files = Artifacts.create(RUNS)
        runner = Draw(args, files)

        def cancel(_signum, _frame):
            runner.cancelled = True
            if runner.state.get("phase") == "returning_to_idle":
                runner.return_cancelled = True

        signals = (signal.SIGINT, signal.SIGTERM)
        previous = [signal.signal(sig, cancel) for sig in signals]
        try:
            runner.run()
        except (Cancelled, OSError, RuntimeError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
            stopped = runner.stop_motion()
            end_process(runner.process)
            return_error = runner.return_error
            if stopped and runner.motion and runner.idle_return and not runner.return_attempted:
                try:
                    runner.return_to_idle("cancelled" if isinstance(error, Cancelled) else "failed")
                except (RuntimeError, ValueError) as failed_return:
                    return_error = str(failed_return)
            progress = report_progress(read_yaml(files.directory / "execute.yaml") or {})
            if not stopped:
                runner.stage("stop_unconfirmed", "无法确认位置保持，未自动返程；请检查机械臂", **progress)
            elif return_error:
                runner.stage("failed", f"返回待机位失败：{return_error}；请核对保持反馈", **progress)
            elif isinstance(error, Cancelled):
                runner.stage("stopped", "绘画已取消，已回待机位" if runner.state.get("idle_returned")
                             else "绘画已取消，保持当前位置", **progress)
            else:
                runner.stage("failed", f"{error}；已回待机位并保持" if runner.state.get("idle_returned")
                             else str(error), **progress)
            raise SystemExit(1) from None
        finally:
            end_process(runner.process)
            # 曾使能的真实后端继续保持位置；取消和返程失败也不能直接关驱动。
            if args.mode != "real" or not runner.had_motion:
                end_process(runner.backend)
            if runner.ros:
                runner.ros.close()
            for sig, handler in zip(signals, previous):
                signal.signal(sig, handler)
