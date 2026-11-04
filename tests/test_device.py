"""设备配置与真实进程锁行为；不使用 CAN。"""

import subprocess
import sys
from types import SimpleNamespace

import pytest

from piper_draw.device import DeviceBusyError, device_lock, load_device_config
from piper_draw.piper_backend import SDKTransport


@pytest.fixture
def device_file(tmp_path):
    path = tmp_path / "device.yaml"
    path.write_text("can: can0\nprotocol: v3\ndh_version: new\nlock_dir: locks\n")
    return path


def test_configuration_and_explicit_overrides(device_file):
    configured = load_device_config(device_file)
    assert (configured.can, configured.protocol, configured.dh_version) == ("can0", "v3", "new")
    assert configured.lock_dir == device_file.parent / "locks"
    assert configured.firmware_speed_percent == 100
    overridden = load_device_config(device_file, can="can1", protocol="v2", dh_version="old")
    assert (overridden.can, overridden.protocol, overridden.dh_version) == ("can1", "v2", "old")
    assert overridden.lock_dir == configured.lock_dir


def test_firmware_speed_configuration_is_loaded(device_file):
    with device_file.open("a") as stream:
        stream.write("firmware_speed_percent: 35\n")
    assert load_device_config(device_file).firmware_speed_percent == 35


@pytest.mark.parametrize("value", ["0", "101", "-1", "true", "30.5", "'30'", ".nan"])
def test_invalid_firmware_speed_is_rejected(device_file, value):
    with device_file.open("a") as stream:
        stream.write(f"firmware_speed_percent: {value}\n")
    with pytest.raises(ValueError, match="firmware_speed_percent"):
        load_device_config(device_file)


@pytest.mark.parametrize("option", [
    {"can": "../can0"}, {"can": ""}, {"can": "x" * 16},
    {"protocol": "v1"}, {"dh_version": "unknown"},
])
def test_invalid_connection_parameters_are_rejected(device_file, option):
    with pytest.raises(ValueError):
        load_device_config(device_file, **option)


def test_missing_configuration_is_not_replaced_with_defaults(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_device_config(tmp_path / "missing.yaml")


def test_process_lock_conflicts_then_releases(tmp_path):
    code = """
import sys
from pathlib import Path
from piper_draw.device import device_lock, DeviceBusyError
try:
    with device_lock('can0', Path(sys.argv[1])):
        print('acquired')
except DeviceBusyError:
    sys.exit(7)
"""
    def acquire():
        return subprocess.run([sys.executable, "-c", code, str(tmp_path)],
                              capture_output=True, text=True, timeout=10, check=False)

    with device_lock("can0", tmp_path):
        assert acquire().returncode == 7
        with device_lock("can1", tmp_path):
            assert acquire().returncode == 7
    result = acquire()
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "acquired"
    assert (tmp_path / "can0.lock").exists()


@pytest.fixture
def fake_sdk(monkeypatch):
    events = []

    class SDK:
        fail_connect = False

        def __init__(self, **options):
            events.append("constructed")

        def ConnectPort(self):
            events.append("connected")
            if self.fail_connect:
                raise RuntimeError("connection failed")

        def DisconnectPort(self):
            events.append("disconnected")

        def MotionCtrl_2(self, *values):
            events.append(("motion", values))

        def JointCtrl(self, *values):
            events.append(("joints", values))

    monkeypatch.setitem(sys.modules, "piper_sdk", SimpleNamespace(
        C_PiperInterface_V2=SDK, C_PiperInterface_V3=SDK
    ))
    return SDK, events


def test_busy_transport_never_constructs_sdk(tmp_path, fake_sdk):
    _, events = fake_sdk
    with device_lock("can0", tmp_path), pytest.raises(DeviceBusyError):
        SDKTransport("can0", "v3", "new", lock_dir=tmp_path)
    assert events == []


def test_transport_failure_and_close_release_control(tmp_path, fake_sdk):
    sdk, events = fake_sdk
    sdk.fail_connect = True
    with pytest.raises(RuntimeError, match="connection failed"):
        SDKTransport("can0", "v3", "new", lock_dir=tmp_path)
    assert events == ["constructed", "connected", "disconnected"]
    sdk.fail_connect = False
    with (
        SDKTransport("can0", "v3", "new", lock_dir=tmp_path) as transport,
        pytest.raises(DeviceBusyError),
        device_lock("can0", tmp_path),
    ):
        pass
    transport.close()
    assert events.count("disconnected") == 2
    with device_lock("can0", tmp_path):
        pass


def test_lock_released_when_operation_raises(tmp_path, fake_sdk):
    with (
        pytest.raises(ValueError, match="operation failed"),
        SDKTransport("can0", "v3", "new", lock_dir=tmp_path),
    ):
        raise ValueError("operation failed")
    with device_lock("can0", tmp_path):
        pass


@pytest.mark.parametrize("speed", [1, 35, 100])
def test_every_joint_command_uses_configured_firmware_speed(tmp_path, fake_sdk, speed):
    _, events = fake_sdk
    with SDKTransport("can0", "v3", "new", lock_dir=tmp_path,
                      firmware_speed_percent=speed) as transport:
        # 连接仍只查询；只有显式发送目标才设置固件速度。
        assert events == ["constructed", "connected"]
        transport.send((0.0,) * 6)
        transport.send((0.1,) * 6)
    assert [value for kind, value in (item for item in events if isinstance(item, tuple))
            if kind == "motion"] == [(0x01, 0x01, speed, 0x00)] * 2


@pytest.mark.parametrize("speed", [0, 101, True, 1.5, "100"])
def test_invalid_transport_speed_never_opens_sdk(tmp_path, fake_sdk, speed):
    _, events = fake_sdk
    with pytest.raises(ValueError, match="firmware_speed_percent"):
        SDKTransport("can0", "v3", "new", lock_dir=tmp_path,
                     firmware_speed_percent=speed)
    assert events == []
