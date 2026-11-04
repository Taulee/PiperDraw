"""本地 Web 控制台：静态页面、有限的命令接口和可停止的单任务进程。"""

import importlib.util
import ipaddress
import json
import mimetypes
import os
import re
import secrets
import shutil
import signal
import subprocess
import sys
import threading
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlsplit
from uuid import uuid4

import yaml

from .calibration_cli import _save_yaml
from .device import load_device_config
from .robot import PARKING_PATH
from .workflow import (
    WORK_DEVICE, WORK_SEEDREAM, WorkflowBusy, active_record, delete_history, drawing_active, history, stop_drawing,
    managed_backend, stop_managed_backend,
)

STATIC = Path(__file__).with_name("web_static")
IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".webp", ".bmp"}


class BusyError(RuntimeError):
    pass


@dataclass
class ProcessTask:
    operation: str
    process: subprocess.Popen
    log: Path
    mode: str = ""
    cancelled: bool = False

    def snapshot(self):
        code = self.process.poll()
        status = ("stopping" if self.cancelled else "running") if code is None else (
            "cancelled" if self.cancelled else "succeeded" if code == 0 else "failed"
        )
        with self.log.open("rb") as stream:
            stream.seek(max(0, self.log.stat().st_size - 128 * 1024))
            output = stream.read().decode("utf-8", errors="replace")
        return {"id": self.log.stem, "operation": self.operation, "status": status, "exit_code": code,
                "log": output, "mode": self.mode}

    def terminate(self):
        if self.process.poll() is None:
            self.cancelled = True
            # park 的 SIGTERM 处理会请求硬件快速停止；ROS 子进程响应 SIGINT。
            stop_signal = signal.SIGTERM if self.operation == "park" else signal.SIGINT
            try:
                os.killpg(self.process.pid, stop_signal)
            except ProcessLookupError:
                self.process.wait()
                return
            try:
                # 独立 run 收到中断后仍需受控返回待机位。
                if self.operation == "run":
                    return
                self.process.wait(timeout=8)
            except subprocess.TimeoutExpired as error:
                raise RuntimeError("任务未在 8 秒内退出，请检查日志与实体状态") from error


class WebApp:
    def __init__(self, root, config, device_config):
        self.root = Path(root).resolve()
        self.directory = self.root / "output/web"
        self.directory.mkdir(parents=True, exist_ok=True)
        self.config = self.directory / "drawing.yaml"
        self.parking_path = self.root / PARKING_PATH
        self.initial_device_config = Path(device_config).resolve()
        load_device_config(self.device_config)
        if not self.config.exists():
            values = self.read_drawing(Path(config))
            _save_yaml(self.config, values)
        self.token = secrets.token_urlsafe(32)
        self.lock = threading.RLock()
        self.task = None
        self.draw_request_id = None
        self.backend = None
        self.has_ros = bool(shutil.which("ros2") and importlib.util.find_spec("rclpy"))

    @property
    def device_config(self):
        path = self.root / WORK_DEVICE
        return path if path.is_file() else self.initial_device_config

    def config_text(self, kind):
        if kind == "drawing":
            return self.config.read_text(encoding="utf-8")
        if kind == "device":
            values = yaml.safe_load(self.device_config.read_text(encoding="utf-8"))
            values.pop("lock_dir", None)
            return yaml.safe_dump(values, allow_unicode=True, sort_keys=False)
        if kind == "image":
            override = self.root / WORK_SEEDREAM
            path = override if override.is_file() else self.root / "config/seedream.yaml"
            return path.read_text(encoding="utf-8")
        raise ValueError("未知配置类型")

    def save_profile(self, kind, text):
        from .seedream import load_seedream_config
        from tempfile import NamedTemporaryFile

        if kind not in {"device", "image"} or not isinstance(text, str):
            raise ValueError("请选择配置并填写 YAML")
        with self.lock:
            self.idle()
            values = yaml.safe_load(text)
            if not isinstance(values, dict):
                raise ValueError("配置必须是 YAML 映射")
            target = self.root / (WORK_DEVICE if kind == "device" else WORK_SEEDREAM)
            if kind == "device":
                values["lock_dir"] = os.path.relpath(load_device_config(self.device_config).lock_dir,
                                                     target.parent)
            with NamedTemporaryFile(dir=target.parent) as temp:
                _save_yaml(Path(temp.name), values)
                (load_device_config if kind == "device" else load_seedream_config)(Path(temp.name))
            _save_yaml(target, values)
        return {"message": "配置已保存"}

    @staticmethod
    def read_drawing(path):
        from .calibration import robot_identity
        from .geometry import PaperFrame, drawing_bounds

        values = yaml.safe_load(path.read_text(encoding="utf-8"))
        if not isinstance(values, dict) or not all(
            key in values for key in ("paper", "tool", "robot", "motion", "image", "calibrated")
        ):
            raise ValueError("需要完整的绘画配置，包含 paper、tool、robot、motion、image、calibrated")
        robot_identity(values)
        paper = values["paper"]
        PaperFrame.from_points(paper["origin_mm"], paper["x_point_mm"], paper["y_point_mm"])
        drawing_bounds(paper)
        if type(values["calibrated"]) is not bool:
            raise ValueError("calibrated 必须为 true 或 false")
        json.dumps(values, allow_nan=False)
        return values

    def relative(self, path):
        return str(path.relative_to(self.root))

    def path(self, value, directories, suffixes):
        if not isinstance(value, str) or not value:
            raise ValueError("文件路径不能为空")
        path = (self.root / value).resolve()
        if not any(path.is_relative_to(self.root / directory) for directory in directories):
            raise ValueError("文件必须位于项目的允许目录中")
        if path.suffix.lower() not in suffixes or not path.is_file():
            raise ValueError("文件不存在或类型不支持")
        return path

    def idle(self):
        if (self.task and self.task.process.poll() is None) or drawing_active(self.root):
            raise BusyError("已有任务运行，请等待完成或停止当前任务")

    def samples_path(self, kind):
        if kind not in ("tcp", "paper"):
            raise ValueError("采样类型必须为 tcp 或 paper")
        return self.directory / f"{kind}_samples.yaml"

    def invalidate_calibration(self, kind):
        values = yaml.safe_load(self.config.read_text())
        values["calibrated"] = False
        measured = values.get("calibration", {})
        measured.pop("paper", None)
        if kind == "tcp":
            measured.pop("tcp", None)
        _save_yaml(self.config, values)

    def state(self):
        from .calibration import is_calibrated

        with self.lock:
            config = yaml.safe_load(self.config.read_text())
            samples = {}
            for kind in ("tcp", "paper"):
                path = self.samples_path(kind)
                samples[kind] = yaml.safe_load(path.read_text())["samples"] if path.exists() else []
            return {
                "token": self.token, "ros_available": self.has_ros,
                "config_path": self.relative(self.config), "calibrated": is_calibrated(config),
                "calibration": config.get("calibration", {}), "samples": samples,
                "task": self.task.snapshot() if self.task else None,
                "drawing": active_record(self.root),
                "device_connected": self.real_backend_active(),
                "backend": self.backend.snapshot() if self.backend else None,
                "parking": json.loads(self.parking_path.read_text(encoding="utf-8"))
                if self.parking_path.exists() else None,
                "device": json.loads((self.root / "output/hardware/latest.json").read_text())
                if (self.root / "output/hardware/latest.json").exists() else None,
            }

    def files(self):
        def recent(paths):
            return [self.relative(path) for path in sorted(
                paths, key=lambda p: p.stat().st_mtime, reverse=True
            )[:100] if path.resolve().is_relative_to(self.root)]

        images = [p for base in (self.root / "input", self.directory / "uploads")
                  for p in base.glob("*") if p.is_file() and p.suffix.lower() in IMAGE_SUFFIXES]
        configs = [p for p in (self.root / "config").glob("*.yaml")
                   if p.name in ("real.yaml", "real.example.yaml", "sim.yaml")]
        configs += list((self.root / "output/hardware/calibration").glob("*.yaml"))
        jobs = list((self.root / "output").rglob("*_job.yaml"))
        return {"images": recent(images), "configs": recent(configs), "jobs": recent(jobs)}

    def _spawn(self, operation, arguments, mode=""):
        logs = self.directory / "logs"
        logs.mkdir(exist_ok=True)
        log = logs / f"{uuid4().hex}_{operation}.log"
        with log.open("wb") as stream:
            process = subprocess.Popen(
                [sys.executable, "-u", "-m", "piper_draw.cli", *map(str, arguments)],
                cwd=self.root, stdin=subprocess.DEVNULL, stdout=stream, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        return ProcessTask(operation, process, log, mode)

    def start(self, data):
        with self.lock:
            operation = data["operation"]
            request_id = data.get("request_id")
            if (operation == "draw" and request_id and request_id == self.draw_request_id
                    and self.task and self.task.operation == "draw"):
                return self.task.snapshot()
            self.idle()
            mode = data.get("mode", "real")
            if mode not in ("real", "mock"):
                raise ValueError("模式必须为 real 或 mock")
            ros_actions = {"status", "enable", "bringup", "plan", "run", "draw"}
            if operation in ros_actions and not self.has_ros:
                raise ValueError("当前运行环境没有 ROS；请在完整 ROS 环境启动同一个 Web 命令")
            if operation == "draw":
                options = {"line_art": "--from-line-art", "photo": "--from-photo", "path": "--reuse-path"}
                source = data.get("source")
                if source not in options:
                    raise ValueError("请先选择使用已有线稿或从照片生成")
                if "history" in data:
                    if not isinstance(data["history"], str) or not data["history"].startswith("output/"):
                        raise ValueError("请选择历史任务")
                    arguments = ["draw", data["history"], "--history"]
                else:
                    if source == "path":
                        raise ValueError("请从历史任务选择已有路径")
                    image = self.path(data["image"], ("input", "output"), IMAGE_SUFFIXES)
                    arguments = ["draw", image]
                config = self.config if mode == "real" else self.root / "config/sim.yaml"
                arguments += [options[source], "--mode", mode, "--config", config,
                              "--device-config", self.device_config]
            elif operation in ("doctor", "feedback-on", "resume", "save-parking", "park"):
                arguments = ["robot", operation, "--device-config", self.device_config]
                if operation == "park":
                    speed = data.get("speed_percent", 3)
                    if type(speed) is not int or not 1 <= speed <= 5:
                        raise ValueError("停放速度必须为 1–5% 的整数")
                    if not self.parking_path.exists():
                        raise ValueError("请先保存停放位")
                    arguments += ["--speed-percent", speed]
                if operation in ("feedback-on", "resume", "save-parking", "park"):
                    stop_managed_backend(self.root)
                if operation == "doctor" and self.real_backend_active():
                    arguments = ["robot", "status"]
            elif operation in ("status", "enable"):
                arguments = ["robot", operation]
            elif operation in ("capture", "tcp", "paper"):
                kind = data["kind"] if operation == "capture" else operation
                samples = self.samples_path(kind)
                arguments = ["calibrate", operation, "--config", self.config, "--samples", samples]
                if operation == "capture":
                    source = data.get("source", "sdk")
                    if source == "auto":
                        source = "ros" if self.real_backend_active() else "sdk"
                    if source not in ("sdk", "ros") or (source == "ros" and not self.has_ros):
                        raise ValueError("采点来源不可用")
                    if kind == "paper" and samples.exists() and len(
                        yaml.safe_load(samples.read_text())["samples"]
                    ) >= 3:
                        raise ValueError("纸面已采集三个点；请撤销末点或清空后再采集")
                    arguments += ["--source", source, "--device-config", self.device_config]
                    self.invalidate_calibration(kind)
                else:
                    arguments += ["--output", self.config]
                    if operation == "paper":
                        orientation = data.get("orientation", "auto")
                        if orientation not in ("auto", "portrait", "landscape"):
                            raise ValueError("A5 方向无效")
                        arguments += ["--orientation", orientation]
            elif operation == "prepare":
                image = self.path(data["image"], ("input", "output"), IMAGE_SUFFIXES)
                source = data.get("source", "line_art")
                if source not in ("line_art", "photo"):
                    raise ValueError("图片来源必须为 line_art 或 photo")
                arguments = ["prepare", image, "--config", self.config,
                             "--from-photo" if source == "photo" else "--from-line-art",
                             "--output-dir", self.directory / "jobs"]
                if (self.root / WORK_SEEDREAM).is_file():
                    arguments += ["--seedream-config", self.root / WORK_SEEDREAM]
            elif operation in ("plan", "run"):
                job = self.path(data["job"], ("output",), {".yaml"})
                if not job.name.endswith("_job.yaml"):
                    raise ValueError("请选择生成的 _job.yaml 文件")
                values = yaml.safe_load(job.read_text())
                if mode == "real":
                    from .calibration import is_calibrated
                    from .job import has_legacy_job_calibration

                    current = yaml.safe_load(self.config.read_text())
                    if not is_calibrated(current) or not (
                        values["calibrated"] or has_legacy_job_calibration(job, values)
                    ):
                        raise ValueError("实机需要完成 TCP 和纸面标定")
                    if current["calibration_id"] != values["calibration_id"]:
                        raise ValueError("任务的标定版本已过期，请重新生成任务")
                arguments = [operation, job, "--mode", mode]
                if operation == "run":
                    arguments += ["--device-config", self.device_config]
            elif operation == "bringup":
                if self.backend and self.backend.process.poll() is None:
                    raise BusyError("本页面启动的后端已在运行")
                arguments = ["bringup", "--mode", mode, "--config", self.config,
                             "--device-config", self.device_config]
                self.backend = self._spawn(operation, arguments, mode)
                return self.backend.snapshot()
            else:
                raise ValueError("未知操作")
            self.task = self._spawn(operation, arguments, mode)
            self.draw_request_id = request_id if operation == "draw" else None
            return self.task.snapshot()

    def real_backend_active(self):
        return bool(managed_backend(self.root) or (
            self.backend and self.backend.mode == "real" and self.backend.process.poll() is None
        ))

    def stop(self, source="sdk", *, backend=False):
        with self.lock:
            if source not in ("sdk", "ros"):
                raise ValueError("停止来源必须为 sdk 或 ros")
            active = self.task and self.task.process.poll() is None
            if stop_drawing(self.root):
                return {"message": "已请求取消，正在保持并返回待机位；驱动继续保持"}
            ros_stop = source == "ros" or self.real_backend_active() or (
                active and self.task.operation == "run" and self.task.mode == "real"
            )
            errors = []
            if active and self.task.operation == "park":
                self.task.terminate()
                active = False
            if ros_stop:
                try:
                    result = subprocess.run(
                        [sys.executable, "-m", "piper_draw.cli", "robot", "stop"],
                        cwd=self.root, capture_output=True, text=True, timeout=10, check=False,
                    )
                    if result.returncode:
                        errors.append(result.stderr.strip() or result.stdout.strip())
                except subprocess.TimeoutExpired:
                    errors.append("ROS 停止请求超时，实体停止状态未知")
            if active:
                self.task.terminate()
                if self.task.operation == "run":
                    if errors:
                        raise RuntimeError("；".join(errors))
                    return {"message": "已请求取消，正在返回待机位；驱动继续保持"}
            if errors:
                raise RuntimeError("；".join(errors))
            if ros_stop and backend:
                try:
                    status = subprocess.run(
                        [sys.executable, "-m", "piper_draw.cli", "robot", "status"],
                        cwd=self.root, capture_output=True, text=True, timeout=5, check=False,
                    )
                    readonly = status.returncode == 0 and json.loads(status.stdout).get("session_enabled") is False
                except (ValueError, subprocess.TimeoutExpired):
                    readonly = False
                if not readonly:
                    return {"message": "已保持当前位置；为避免下坠，保留已使能的控制后端"}
            if backend and self.backend:
                self.backend.terminate()
            if backend:
                stop_managed_backend(self.root)
            if errors:
                raise RuntimeError("；".join(errors))
            return {"message": "已处理任务停止请求；实体状态以现场反馈为准"}

    def edit_samples(self, data):
        with self.lock:
            self.idle()
            kind = data["kind"]
            path = self.samples_path(kind)
            action = data["action"]
            if action not in ("undo", "clear"):
                raise ValueError("采样操作必须为 undo 或 clear")
            if path.exists():
                values = yaml.safe_load(path.read_text())
                if action == "clear":
                    path.unlink()
                elif values["samples"]:
                    values["samples"].pop()
                    _save_yaml(path, values)
            self.invalidate_calibration(kind)
            return {"message": "采样已更新，需重新求解标定"}

    def save_config(self, data):
        with self.lock:
            self.idle()
            if "path" in data:
                source = self.path(data["path"], ("config", "output"), {".yaml"})
                values = self.read_drawing(source)
            else:
                text = data["text"]
                if not isinstance(text, str):
                    raise ValueError("配置必须为 YAML 文本")
                temporary = self.directory / "config-edit.yaml"
                try:
                    temporary.write_text(text, encoding="utf-8")
                    values = self.read_drawing(temporary)
                finally:
                    temporary.unlink(missing_ok=True)
                previous = yaml.safe_load(self.config.read_text())
                # 配置修改使旧任务过期，标定结果仍来自采样求解。
                if values != previous:
                    values["calibrated"] = False
                    values["calibration_id"] = "edited-" + uuid4().hex
                    if values["tool"]["tcp_offset_mm"] != previous["tool"]["tcp_offset_mm"] or (
                        values["robot"] != previous["robot"]
                    ):
                        values.pop("calibration", None)
                    elif "calibration" in previous:
                        values["calibration"] = previous["calibration"]
                    else:
                        values.pop("calibration", None)
            _save_yaml(self.config, values)
            return {"message": "工作配置已保存；请按标定状态生成新任务"}

    def upload(self, name, data):
        import cv2
        import numpy as np

        suffix = Path(name).suffix.lower()
        if suffix not in IMAGE_SUFFIXES:
            raise ValueError("支持 PNG、JPEG、WebP、BMP 图片")
        image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
        if image is None or min(image.shape[:2]) < 2 or image.shape[0] * image.shape[1] > 36_000_000:
            raise ValueError("图片无法读取或尺寸无效，最多 3600 万像素")
        directory = self.directory / "uploads"
        directory.mkdir(exist_ok=True)
        stem = re.sub(r"[^\w.-]", "_", Path(name).stem)[:80] or "image"
        path = directory / f"{uuid4().hex[:12]}_{stem}{suffix}"
        path.write_bytes(data)
        return {"path": self.relative(path)}


class Handler(BaseHTTPRequestHandler):
    server_version = "PiperWeb/1.0"

    @property
    def app(self):
        return self.server.app

    def log_message(self, format, *args):
        # 页面按秒读取状态；只保留错误请求，避免访问日志掩盖设备日志。
        if len(args) > 1 and str(args[1]).startswith(("4", "5")):
            super().log_message(format, *args)

    def reply(self, status, body, content_type="application/json; charset=utf-8"):
        if not isinstance(body, bytes):
            body = json.dumps(body, ensure_ascii=False, allow_nan=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy",
                         "default-src 'self'; img-src 'self' blob:; frame-ancestors 'none'")
        self.end_headers()
        self.wfile.write(body)

    def check_host(self):
        host = urlsplit("http://" + self.headers.get("Host", "")).hostname
        if host == "localhost":
            return
        try:
            address = ipaddress.ip_address(host or "")
        except ValueError as error:
            raise PermissionError("只接受 localhost 或 IP 地址访问") from error
        bind = self.server.server_address[0]
        if bind in ("127.0.0.1", "::1") and not address.is_loopback:
            raise PermissionError("此服务只接受本机地址访问")

    def body(self, limit):
        if self.headers.get("Transfer-Encoding"):
            raise ValueError("请求必须使用 Content-Length")
        size = int(self.headers.get("Content-Length", "0"))
        if not 0 < size <= limit:
            raise ValueError(f"请求大小必须在 1–{limit} 字节内")
        return self.rfile.read(size)

    def handle_request(self, post=False):
        try:
            self.check_host()
            url = urlsplit(self.path)
            query = parse_qs(url.query)
            if post:
                if not secrets.compare_digest(self.headers.get("X-Piper-Token", ""), self.app.token):
                    raise PermissionError("页面令牌无效，请刷新页面")
                origin = self.headers.get("Origin")
                if origin and urlsplit(origin).netloc != self.headers.get("Host"):
                    raise PermissionError("拒绝跨站操作")
                if url.path == "/api/upload":
                    result = self.app.upload(query.get("name", [""])[0], self.body(30 * 1024**2))
                else:
                    if self.headers.get_content_type() != "application/json":
                        raise ValueError("操作请求需要 application/json")
                    data = json.loads(self.body(1024**2))
                    if not isinstance(data, dict):
                        raise ValueError("请求必须是 JSON 对象")
                    if url.path == "/api/action":
                        result = self.app.start(data)
                    elif url.path == "/api/stop":
                        result = self.app.stop(data.get("source", "sdk"), backend=data.get("backend") is True)
                    elif url.path == "/api/samples":
                        result = self.app.edit_samples(data)
                    elif url.path == "/api/config":
                        kind = data.get("kind", "drawing")
                        result = (self.app.save_config(data) if kind == "drawing"
                                  else self.app.save_profile(kind, data.get("text")))
                    elif url.path == "/api/history/delete":
                        with self.app.lock:
                            self.app.idle()
                            result = {"deleted": delete_history(data.get("ids"), self.app.root)}
                    else:
                        self.reply(404, {"error": "接口不存在"})
                        return
                self.reply(200, result)
            elif url.path == "/api/state":
                self.reply(200, self.app.state())
            elif url.path == "/api/files":
                self.reply(200, self.app.files())
            elif url.path == "/api/history":
                self.reply(200, history(self.app.root))
            elif url.path == "/api/config":
                self.reply(200, {"text": self.app.config_text(query.get("kind", ["drawing"])[0])})
            elif url.path == "/files":
                path = self.app.path(query.get("path", [""])[0], ("input", "output"),
                                     IMAGE_SUFFIXES | {".svg", ".yaml", ".json", ".log"})
                self.reply(200, path.read_bytes(), mimetypes.guess_type(path.name)[0] or "text/plain")
            elif url.path in ("/", "/app.js", "/style.css"):
                path = STATIC / ("index.html" if url.path == "/" else unquote(url.path[1:]))
                self.reply(200, path.read_bytes(), mimetypes.guess_type(path.name)[0])
            else:
                self.reply(404, {"error": "页面不存在"})
        except PermissionError as error:
            self.reply(403, {"error": str(error)})
        except (BusyError, WorkflowBusy) as error:
            self.reply(409, {"error": str(error)})
        except (ValueError, TypeError, KeyError, yaml.YAMLError) as error:
            self.reply(400, {"error": str(error)})
        except (OSError, RuntimeError) as error:
            self.reply(500, {"error": str(error)})

    def do_GET(self):
        self.handle_request()

    def do_POST(self):
        self.handle_request(post=True)


def serve(host, port, config, device_config):
    if not 1 <= port <= 65535:
        raise ValueError("端口必须为 1–65535")
    app = WebApp(Path.cwd(), config, device_config)
    server = ThreadingHTTPServer((host, port), Handler)
    server.app = app

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    previous = signal.signal(signal.SIGTERM, interrupt)
    print(f"Piper Web：http://{host}:{port}；工作配置：{app.config}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            app.stop(backend=True)
        finally:
            server.server_close()
            signal.signal(signal.SIGTERM, previous)
