import json
import shutil
import sys
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
import yaml

from piper_draw import cli, workflow
from piper_draw.web import WebApp


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def project(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    shutil.copytree(ROOT / "config", "config")
    device = yaml.safe_load(Path("config/device.yaml").read_text())
    device["return_to_idle"]["enabled"] = False
    Path("config/device.yaml").write_text(yaml.safe_dump(device))
    Path("input").mkdir()
    image = np.full((120, 120), 255, np.uint8)
    cv2.line(image, (15, 60), (105, 60), 0, 3)
    assert cv2.imwrite("input/test.png", image)
    return tmp_path


def test_shared_draw_history_and_explicit_redraw(project, monkeypatch, capsys):
    events = []

    class Connection:
        def __init__(self, domain_id):
            assert domain_id in (107, 108)

        def controller_available(self):
            return True

        def wait_ready(self, mode, device, backend, check):
            check()
            events.append(("ready", mode, device.lock_dir))

        def close(self):
            events.append(("close",))

    monkeypatch.setitem(sys.modules, "piper_draw.workflow_ros",
                        SimpleNamespace(RobotConnection=Connection))
    original = workflow.Draw.command

    def command(self, log, *args, report=None):
        if args[0] == "prepare":
            return original(self, log, *args, report=report)
        events.append((args[0],))
        assert Path(args[1]).is_file()
        return {"status": "planned" if args[0] == "plan" else "executed",
                "segments": [{"kind": "lift"}], "estimated_seconds": 8}

    monkeypatch.setattr(workflow.Draw, "command", command)
    with pytest.raises(SystemExit) as missing_choice:
        cli.main(["draw", "input/test.png", "--mode", "mock"])
    assert missing_choice.value.code == 2
    cli.main(["draw", "input/test.png", "--from-line-art", "--mode", "mock"])
    assert events == [("ready", "mock", project / "output/hardware/locks"),
                      ("plan",), ("run",), ("close",)]
    first, = workflow.history()
    assert first["phase"] == "done" and Path(first["line_art"]).is_file()
    assert workflow.history(project) == workflow.history()
    capsys.readouterr()
    cli.main(["history"])
    assert json.loads(capsys.readouterr().out) == [first]

    app = WebApp(project, project / "config/real.example.yaml", project / "config/device.yaml")
    app.has_ros = True
    launched = []
    monkeypatch.setattr(app, "_spawn", lambda operation, args, mode: launched.append(args)
                        or SimpleNamespace(snapshot=lambda: {}))
    app.start({"operation": "draw", "history": first["id"], "source": "line_art", "mode": "mock"})
    assert launched[0][:4] == ["draw", first["id"], "--history", "--from-line-art"]
    cli.main(list(map(str, launched[0])))
    newer, original_record = workflow.history()
    assert original_record == first and newer["phase"] == "done"
    with workflow.workflow_lock():
        with pytest.raises(workflow.WorkflowBusy):
            workflow.delete_history([first["id"]])
    assert workflow.delete_history([first["id"]]) == 1
    assert workflow.history() == [newer]


def test_stop_requested_before_enable(project, monkeypatch):
    class Connection:
        def __init__(self, domain_id):
            assert domain_id in (107, 108)

        def controller_available(self):
            return True

        def wait_ready(self, mode, device, backend, check):
            assert workflow.stop_drawing(project)
            check()

        def close(self):
            pass

    monkeypatch.setitem(sys.modules, "piper_draw.workflow_ros",
                        SimpleNamespace(RobotConnection=Connection))
    with pytest.raises(SystemExit) as stopped:
        cli.main(["draw", "input/test.png", "--from-line-art", "--mode", "mock"])
    assert stopped.value.code == 1
    record, = workflow.history()
    assert record["phase"] == "stopped"
    assert not (Path(record["id"]) / "execute.yaml").exists()
    assert not workflow.stop_drawing(project)


def test_device_settings_keep_the_shared_lock(project):
    app = WebApp(project, project / "config/real.example.yaml", project / "config/device.yaml")
    values = yaml.safe_load(app.config_text("device"))
    values.update(can="can1", lock_dir="/tmp/another-lock")
    app.save_profile("device", yaml.safe_dump(values))
    settings = workflow.load_device_config(app.device_config)
    assert settings.can == "can1"
    assert settings.lock_dir == project / "output/hardware/locks"


def test_real_plans_before_enabling_and_stops_on_execution_failure(project, monkeypatch):
    events = []
    config = Path("config/real.example.yaml")
    values = yaml.safe_load(config.read_text())
    config.write_text(yaml.safe_dump({**values, "calibrated": True}))

    class Connection:
        def __init__(self, domain_id):
            pass

        def controller_available(self):
            return True

        def wait_ready(self, mode, device, backend, check):
            events.append("ready")

        def enable(self, check, on_motion):
            check()
            events.append("enable")
            on_motion()

        def hold_and_cancel(self):
            events.append("hold")

        def close(self):
            events.append("close")

    monkeypatch.setitem(sys.modules, "piper_draw.workflow_ros",
                        SimpleNamespace(RobotConnection=Connection))
    original = workflow.Draw.command

    def command(self, log, *args, report=None):
        if args[0] == "prepare":
            return original(self, log, *args, report=report)
        events.append(args[0])
        if args[0] == "run":
            raise RuntimeError("轨迹执行失败")
        return {"status": "planned", "segments": [], "estimated_seconds": 8}

    monkeypatch.setattr(workflow.Draw, "command", command)
    with pytest.raises(SystemExit) as failed:
        cli.main(["draw", "input/test.png", "--from-line-art", "--config", str(config)])
    assert failed.value.code == 1
    assert events == ["ready", "plan", "enable", "run", "hold", "close"]
    record, = workflow.history()
    assert (record["phase"], record["message"]) == ("failed", "轨迹执行失败")


@pytest.mark.parametrize("outcome", ["completed", "cancelled", "failed"])
def test_real_success_cancel_and_failure_return_to_idle_without_stopping_backend(project, monkeypatch, outcome):
    events = []
    config = Path("config/real.example.yaml")
    values = yaml.safe_load(config.read_text())
    config.write_text(yaml.safe_dump({**values, "calibrated": True}))
    device_path = Path("config/device.yaml")
    device = yaml.safe_load(device_path.read_text())
    device["return_to_idle"].update(enabled=True, positions_rad=[1.57, 0.25, -0.42, 0.0, 0.73, 0.0])
    device_path.write_text(yaml.safe_dump(device))

    class Connection:
        def __init__(self, domain_id):
            pass

        def controller_available(self):
            return True

        def wait_ready(self, mode, device, backend, check):
            pass

        def enable(self, check, on_motion):
            on_motion()

        def hold_and_cancel(self):
            events.append("hold")

        def return_to_idle(self, plan, check=lambda: None):
            check()
            assert plan["positions_rad"] == [1.57, 0.25, -0.42, 0.0, 0.73, 0.0]
            events.append("return")
            return {"holding": True, "positions_rad": plan["positions_rad"]}

        def close(self):
            events.append("close")

    monkeypatch.setitem(sys.modules, "piper_draw.workflow_ros", SimpleNamespace(RobotConnection=Connection))
    original = workflow.Draw.command

    def command(self, log, *args, report=None):
        if args[0] == "prepare":
            return original(self, log, *args, report=report)
        result = {"status": "planned" if args[0] == "plan" else "executed",
                  "completed_segments": 1, "segments": [{"kind": "lift"}], "estimated_seconds": 8}
        if report:
            Path(report).write_text(yaml.safe_dump(result))
        if args[0] == "run":
            assert "--no-return-to-idle" in args
            if outcome == "cancelled":
                raise workflow.Cancelled
            if outcome == "failed":
                raise RuntimeError("轨迹故障")
        return result

    monkeypatch.setattr(workflow.Draw, "command", command)
    arguments = ["draw", "input/test.png", "--from-line-art", "--config", str(config)]
    if outcome == "completed":
        cli.main(arguments)
        assert events == ["return", "close"]
    else:
        with pytest.raises(SystemExit):
            cli.main(arguments)
        assert events == ["hold", "return", "close"]
    record, = workflow.history()
    assert record["idle_returned"] is True
    report = yaml.safe_load((Path(record["id"]) / "execute.yaml").read_text())
    assert report["idle_return"]["status"] == "returned"


def test_failed_return_is_not_retried_or_followed_by_backend_shutdown(project, monkeypatch):
    backend = object()
    closed = []
    returns = []

    def run(self):
        self.backend = backend
        self.motion = self.had_motion = True
        self.idle_return = {"target": "saved"}

        def fail(*args):
            returns.append("return")
            raise RuntimeError("路径不可达")

        self.ros = SimpleNamespace(return_to_idle=fail, hold_and_cancel=lambda: None, close=lambda: None)
        self.return_to_idle("completed")

    monkeypatch.setattr(workflow.Draw, "run", run)
    monkeypatch.setattr(workflow, "end_process", lambda process: closed.append(process))
    with pytest.raises(SystemExit):
        cli.main(["draw", "input/test.png", "--from-line-art"])
    assert returns == ["return"]
    assert backend not in closed
    record, = workflow.history()
    assert record["idle_returned"] is False
