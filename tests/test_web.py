"""本地 HTTP、真实子进程与图片管线验证；不连接 CAN，不执行机器人运动。"""

import json
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest.mock import Mock
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import cv2
import numpy as np
import pytest
import yaml

from piper_draw.web import BusyError, Handler, ProcessTask, WebApp


@pytest.fixture
def app(tmp_path):
    project = Path(__file__).parents[1]
    (tmp_path / "config").mkdir()
    (tmp_path / "input").mkdir()
    for name in ("real.example.yaml", "sim.yaml", "device.yaml", "app.yaml"):
        shutil.copyfile(project / "config" / name, tmp_path / "config" / name)
    result = WebApp(tmp_path, tmp_path / "config/real.example.yaml", tmp_path / "config/device.yaml")
    yield result
    result.stop(backend=True)


@pytest.fixture
def server(app):
    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    server.app = app
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    yield f"http://127.0.0.1:{server.server_port}", app
    server.shutdown()
    server.server_close()
    thread.join(timeout=3)


def http(server, path, data=None, *, token=True, headers=None):
    base, app = server
    request_headers = {"X-Piper-Token": app.token} if token else {}
    if data is not None and not isinstance(data, bytes):
        data = json.dumps(data).encode()
        request_headers["Content-Type"] = "application/json"
    request_headers.update(headers or {})
    request = Request(base + path, data=data, headers=request_headers)
    try:
        response = urlopen(request, timeout=10)
    except HTTPError as error:
        response = error
    with response:
        body = response.read()
        if response.headers.get_content_type() == "application/json":
            body = json.loads(body)
        return response.status, body


def wait_until(condition, seconds=10):
    deadline = time.monotonic() + seconds
    while not condition():
        if time.monotonic() >= deadline:
            raise AssertionError("condition did not complete")
        time.sleep(0.02)


def test_static_page_and_state_do_not_connect_hardware(server, monkeypatch):
    constructor = Mock(side_effect=AssertionError("must not start a process"))
    monkeypatch.setattr(subprocess, "Popen", constructor)
    status, html = http(server, "/")
    assert status == 200 and "Piper 绘画控制台" in html.decode()
    assert http(server, "/app.js")[0] == http(server, "/style.css")[0] == 200
    status, state = http(server, "/api/state")
    assert status == 200 and state["task"] is None
    assert not state["calibrated"] and state["token"]
    constructor.assert_not_called()


def test_mutations_require_token_and_same_origin(server):
    assert http(server, "/api/action", {"operation": "doctor"}, token=False)[0] == 403
    assert http(server, "/api/action", {"operation": "doctor"},
                headers={"Origin": "https://attacker.example"})[0] == 403
    assert http(server, "/api/state", headers={"Host": "attacker.example"})[0] == 403


@pytest.mark.parametrize("path", ["../secret.yaml", "config/device.yaml", "api_key.txt"])
def test_file_endpoint_cannot_read_outside_data_directories(server, path):
    status, _ = http(server, "/files?path=" + path)
    assert status == 400


def test_symlinks_cannot_escape_files_boundary(server, tmp_path):
    _, app = server
    secret = tmp_path / "secret.yaml"
    secret.write_text("secret")
    (app.directory / "linked.yaml").symlink_to(secret)
    assert http(server, "/files?path=output/web/linked.yaml")[0] == 400


def test_unknown_operation_and_ros_unavailability_rejected_before_process(server, monkeypatch):
    _, app = server
    app.has_ros = False
    spawn = Mock()
    monkeypatch.setattr(app, "_spawn", spawn)
    assert http(server, "/api/action", {"operation": "anything; touch bad"})[0] == 400
    assert http(server, "/api/action", {"operation": "bringup"})[0] == 400
    assert http(server, "/api/action", {"operation": "park", "speed_percent": 20})[0] == 400
    spawn.assert_not_called()


def test_upload_prepare_preview_round_trip(server):
    _, app = server
    raster = np.full((120, 120), 255, np.uint8)
    cv2.line(raster, (15, 60), (105, 60), 0, 3)
    png = cv2.imencode(".png", raster)[1].tobytes()
    status, uploaded = http(server, "/api/upload?name=line.png", png,
                            headers={"Content-Type": "image/png"})
    assert status == 200
    assert http(server, "/files?path=" + uploaded["path"])[1] == png
    status, _ = http(server, "/api/action", {
        "operation": "prepare", "source": "line_art", "image": uploaded["path"]
    })
    assert status == 200
    wait_until(lambda: app.task.process.poll() is not None)
    assert app.task.snapshot()["status"] == "succeeded", app.task.snapshot()["log"]
    jobs = http(server, "/api/files")[1]["jobs"]
    assert len(jobs) == 1 and jobs[0].startswith("output/web/jobs/")
    job = yaml.safe_load(http(server, "/files?path=" + jobs[0])[1])
    assert job["segments"] and not job["calibrated"]
    preview = http(server, "/files?path=" + jobs[0].replace("_job.yaml", "_preview.svg"))
    assert preview[0] == 200 and b"<svg" in preview[1]


def test_upload_rejects_nonimage_and_disallowed_type(server):
    assert http(server, "/api/upload?name=broken.png", b"not a picture")[0] == 400
    assert http(server, "/api/upload?name=script.html", b"<script></script>")[0] == 400


def test_config_save_uses_work_copy_and_invalidates_old_execution(server):
    _, app = server
    original = (app.root / "config/real.example.yaml").read_text()
    values = yaml.safe_load(original)
    values["calibrated"] = True
    values["motion"]["draw_mm_s"] = 4
    status, _ = http(server, "/api/config", {"text": yaml.safe_dump(values)})
    assert status == 200
    written = yaml.safe_load(http(server, "/api/config")[1]["text"])
    assert written["motion"]["draw_mm_s"] == 4
    assert not written["calibrated"] and written["calibration_id"].startswith("edited-")
    assert (app.root / "config/real.example.yaml").read_text() == original
    saved = app.config.read_bytes()
    assert http(server, "/api/config", {"text": "wrong: ["})[0] == 400
    assert app.config.read_bytes() == saved


def test_clear_samples_invalidates_tcp_and_paper(server):
    _, app = server
    values = yaml.safe_load(app.config.read_text())
    values.update(calibrated=True, calibration={"tcp": {}, "paper": {}})
    app.config.write_text(yaml.safe_dump(values))
    app.samples_path("tcp").write_text("samples: []\n")
    assert http(server, "/api/samples", {"kind": "tcp", "action": "clear"})[0] == 200
    result = yaml.safe_load(app.config.read_text())
    assert not result["calibrated"] and result["calibration"] == {}
    assert not app.samples_path("tcp").exists()


def test_parking_requires_saved_pose_and_preserves_calibration(app, monkeypatch):
    config = yaml.safe_load(app.config.read_text())
    config.update(calibrated=True, calibration={"tcp": {}, "paper": {}})
    app.config.write_text(yaml.safe_dump(config))
    app.samples_path("tcp").write_text("samples: []\n")
    spawn = Mock()
    spawn.return_value.process.poll.return_value = 0
    monkeypatch.setattr(app, "_spawn", spawn)
    with pytest.raises(ValueError, match="未知操作"):
        app.start({"operation": "set-zero"})
    with pytest.raises(ValueError, match="先保存停放位"):
        app.start({"operation": "park"})
    spawn.assert_not_called()

    app.start({"operation": "save-parking"})
    assert spawn.call_args.args[1][:2] == ["robot", "save-parking"]
    app.parking_path.parent.mkdir(parents=True, exist_ok=True)
    saved = {"positions_rad": [0.1] * 6}
    app.parking_path.write_text(json.dumps(saved))
    assert app.state()["parking"] == saved
    app.start({"operation": "park"})
    assert spawn.call_args.args[1][:2] == ["robot", "park"]
    assert yaml.safe_load(app.config.read_text()) == config
    assert app.samples_path("tcp").exists()


def test_real_execution_rejects_old_calibration(app, monkeypatch):
    app.has_ros = True
    config = yaml.safe_load(app.config.read_text())
    config.update(calibrated=True, calibration_id="new")
    app.config.write_text(yaml.safe_dump(config))
    job = app.directory / "old_job.yaml"
    job.write_text("calibrated: true\ncalibration_id: old\n")
    spawn = Mock()
    monkeypatch.setattr(app, "_spawn", spawn)
    with pytest.raises(ValueError, match="过期"):
        app.start({"operation": "run", "job": app.relative(job), "mode": "real"})
    spawn.assert_not_called()


def test_park_task_cancel_runs_signal_handler_and_blocks_competing_commands(app):
    log = app.directory / "test-park.log"
    code = """
import signal, sys, time
def stop(signum, frame):
    print('stop handler ran', flush=True)
    sys.exit(2)
signal.signal(signal.SIGTERM, stop)
print('ready', flush=True)
while True:
    time.sleep(.02)
"""
    with log.open("wb") as stream:
        process = subprocess.Popen([sys.executable, "-u", "-c", code],
                                   stdout=stream, stderr=stream, start_new_session=True)
    app.task = ProcessTask("park", process, log)
    try:
        wait_until(lambda: "ready" in log.read_text())
        with pytest.raises(BusyError):
            app.start({"operation": "doctor"})
        with pytest.raises(BusyError):
            app.edit_samples({"kind": "tcp", "action": "clear"})
        app.stop()
        assert process.returncode == 2
        assert "stop handler ran" in app.task.snapshot()["log"]
        assert app.task.snapshot()["status"] == "cancelled"
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)


def test_ros_stop_failure_is_visible(app, monkeypatch):
    monkeypatch.setattr(subprocess, "run", Mock(return_value=subprocess.CompletedProcess(
        [], 1, "", "service unavailable"
    )))
    with pytest.raises(RuntimeError, match="service unavailable"):
        app.stop("ros")


@pytest.mark.parametrize("session_enabled", [False, True])
def test_backend_shutdown_preserves_enabled_hold_but_closes_readonly_backend(app, monkeypatch, session_enabled):
    from types import SimpleNamespace

    events = []
    app.backend = SimpleNamespace(mode="real", process=SimpleNamespace(poll=lambda: None),
                                  terminate=lambda: events.append("closed"))

    def run(arguments, **kwargs):
        return SimpleNamespace(returncode=0, stdout=json.dumps({"session_enabled": session_enabled}), stderr="")

    monkeypatch.setattr("piper_draw.web.subprocess.run", run)
    app.stop(backend=True)
    assert events == ([] if session_enabled else ["closed"])
    app.backend = None


def test_work_config_survives_service_restart(app):
    values = yaml.safe_load(app.config.read_text())
    values["motion"]["draw_mm_s"] = 2.5
    app.config.write_text(yaml.safe_dump(values))
    restarted = WebApp(app.root, app.root / "config/real.example.yaml", app.device_config)
    assert yaml.safe_load(restarted.config.read_text())["motion"]["draw_mm_s"] == 2.5
