"""设备 CLI 的数据、失败记录、依赖隔离与停止语义。"""

import json
import math
import signal
import subprocess
import sys
from pathlib import Path
from unittest.mock import MagicMock

import pytest

from piper_draw import cli, robot
from piper_draw.device import DeviceConfig
from piper_draw.piper_backend import STATE_FRAMES, Feedback, FeedbackClock


@pytest.fixture
def environment(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    Path("config").mkdir()
    Path("config/device.yaml").write_text(
        "can: can0\nprotocol: v3\ndh_version: new\nlock_dir: ../output/hardware/locks\n"
    )
    frames = FeedbackClock()
    for frame in (*STATE_FRAMES, 0x2A8):
        frames.received(frame)
    transport = MagicMock()
    transport.__enter__.return_value = transport
    transport.frames = frames
    transport.read.return_value = Feedback((0,) * 6, 10, (False,) * 6, 0, 0, "S-V1.8-9")
    monkeypatch.setattr(robot, "SDKTransport", MagicMock(return_value=transport))
    monkeypatch.setattr(robot.platform, "system", lambda: "Linux")
    # 使用真实的临时接口目录，避免全局替换 Path.exists。
    network = tmp_path / "net"
    (network / "can0").mkdir(parents=True)
    original_path = robot.Path
    monkeypatch.setattr(robot, "Path", lambda path: network if path == "/sys/class/net" else original_path(path))
    monkeypatch.setattr(robot.subprocess, "check_output", lambda *args, **kwargs: json.dumps([{
        "flags": ["UP"],
        "linkinfo": {"info_kind": "can", "info_data": {"bittiming": {"bitrate": 1_000_000}}},
    }]))
    return transport


def test_doctor_returns_json_and_shared_configuration(environment, capsys):
    cli.main(["robot", "doctor"])
    result = json.loads(capsys.readouterr().out)
    assert result["status"] == "checked"
    assert result["protocol"] == "v3"
    assert result["enabled"] == [False] * 6
    assert result == json.loads(Path("output/hardware/latest.json").read_text())
    runs = list(Path("output/hardware").glob("*_doctor.json"))
    assert len(runs) == 1
    assert result == json.loads(runs[0].read_text())
    robot.SDKTransport.assert_called_once_with(
        "can0", "v3", "new", lock_dir=Path("output/hardware/locks").resolve()
    )
    assert environment.sdk.mock_calls == []


def test_failed_attempt_replaces_previous_success(environment, capsys):
    cli.main(["robot", "doctor"])
    with pytest.raises(SystemExit) as failure:
        cli.main(["robot", "home", "--protocol", "v2"])
    assert failure.value.code == 1
    result = json.loads(Path("output/hardware/latest.json").read_text())
    assert result["operation"] == "home"
    assert result["status"] == "failed"
    assert "仅支持 v3/new" in result["error"]
    assert len(list(Path("output/hardware").glob("*_*.json"))) == 2
    assert "仅支持 v3/new" in capsys.readouterr().err
    assert environment.sdk.mock_calls == []


def test_feedback_configuration_is_explicit(environment):
    cli.main(["robot", "feedback-on"])
    environment.sdk.MasterSlaveConfig.assert_called_once_with(0xFC, 0, 0, 0)
    assert len(environment.sdk.mock_calls) == 1


def test_cli_invalid_speed_cannot_open_device(environment):
    with pytest.raises(SystemExit) as failure:
        cli.main(["robot", "home", "--speed-percent", "6"])
    assert failure.value.code == 2
    robot.SDKTransport.assert_not_called()


def test_save_parking_reads_only_and_park_uses_saved_pose(environment, monkeypatch):
    positions = tuple(map(math.radians, (25, 40, -30, 0, 5, -10)))
    environment.read.return_value.positions = positions
    cli.main(["robot", "save-parking"])
    saved = json.loads(robot.PARKING_PATH.read_text())
    assert tuple(saved["positions_rad"]) == positions
    assert environment.sdk.mock_calls == []

    motion = MagicMock(return_value={"status": "reached"})
    monkeypatch.setattr(robot, "home", motion)
    cli.main(["robot", "park", "--speed-percent", "1"])
    assert motion.call_args.kwargs["target_rad"] == positions
    assert motion.call_args.kwargs["speed_percent"] == 1

    saved["positions_rad"][0] = math.tau
    robot.PARKING_PATH.write_text(json.dumps(saved))
    motion.reset_mock()
    with pytest.raises(SystemExit):
        cli.main(["robot", "park"])
    motion.assert_not_called()


def test_sigterm_is_recorded_and_handler_restored(environment, monkeypatch):
    def interrupted_home(*args, **kwargs):
        signal.raise_signal(signal.SIGTERM)

    previous = signal.getsignal(signal.SIGTERM)
    monkeypatch.setattr(robot, "home", interrupted_home)
    with pytest.raises(KeyboardInterrupt):
        cli.main(["robot", "home"])
    assert signal.getsignal(signal.SIGTERM) == previous
    result = json.loads(Path("output/hardware/latest.json").read_text())
    assert result["status"] == "failed"
    assert "收到信号" in result["error"]


def test_hardware_help_does_not_import_drawing_ros_or_sdk():
    code = """
import builtins
original = builtins.__import__
def guarded(name, *args, **kwargs):
    if name.split('.')[0] in {'numpy', 'cv2', 'scipy', 'rclpy', 'piper_sdk'}:
        raise AssertionError('unexpected dependency: ' + name)
    return original(name, *args, **kwargs)
builtins.__import__ = guarded
from piper_draw.cli import main
main(['robot', 'home', '--help'])
"""
    result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True,
                            timeout=10, check=False)
    assert result.returncode == 0, result.stderr
    assert "--speed-percent" in result.stdout


@pytest.mark.parametrize("operation", [robot.home, robot.resume])
def test_unsupported_operations_reject_before_connecting(tmp_path, monkeypatch, operation):
    constructor = MagicMock()
    monkeypatch.setattr(robot, "SDKTransport", constructor)
    with pytest.raises(ValueError, match="仅支持 v3/new"):
        operation(DeviceConfig("can0", "v2", "old", tmp_path))
    constructor.assert_not_called()
