"""待机位在执行前校验，成功、取消和故障的收尾均不关闭控制后端。"""

import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from piper_draw.completion import IdleSettings, finish_run, load_idle_settings, prepare_idle_return
from piper_draw.device import DeviceConfig
from piper_draw.job import make_job


@pytest.mark.parametrize("values", [None, [], {"enabled": 1}, {"velocity_scale": 2},
    {"acceleration_scale": 0}, {"timeout_s": float("nan")}, {"min_tip_height_mm": -1},
    {"positions_rad": [0] * 5}, {"positions_rad": [False] * 6}, {"unknown": 1}])
def test_invalid_idle_configuration_is_rejected(values):
    with pytest.raises(ValueError):
        load_idle_settings(values)


def test_saved_idle_pose_is_bound_to_device_and_checked_against_paper(tmp_path):
    config = yaml.safe_load((Path(__file__).parents[1] / "config/real.yaml").read_text())
    job = make_job([], config)
    device = DeviceConfig("can0", "v3", "new", tmp_path, return_to_idle=IdleSettings(enabled=True))
    parking = tmp_path / "parking.json"
    target = [1.57, 0.25, -0.42, 0.0, 0.73, 0.0]
    values = {"can": "can0", "protocol": "v3", "dh_version": "new", "positions_rad": target}
    parking.write_text(json.dumps(values))
    assert prepare_idle_return(job, device, parking)["positions_rad"] == target
    values["can"] = "can1"
    parking.write_text(json.dumps(values))
    with pytest.raises(ValueError, match="当前设备配置不同"):
        prepare_idle_return(job, device, parking)
    high_paper = {**job, "paper_origin": [0, 0, 10]}
    explicit = DeviceConfig("can0", "v3", "new", tmp_path,
        return_to_idle=IdleSettings(enabled=True, positions_rad=tuple(target)))
    with pytest.raises(ValueError, match="低于本次纸面净空"):
        prepare_idle_return(high_paper, explicit)


@pytest.mark.parametrize("reason", ["completed", "cancelled", "failed"])
def test_standalone_run_finishes_without_stop_or_shutdown(tmp_path, monkeypatch, reason):
    events = []
    class Connection:
        def __init__(self, **kwargs):
            pass

        def return_to_idle(self, plan):
            events.append("return")
            return {"holding": True}

        def close(self):
            events.append("close_client")

    monkeypatch.setitem(sys.modules, "piper_draw.workflow_ros", SimpleNamespace(RobotConnection=Connection))
    path = tmp_path / "report.yaml"
    path.write_text("status: executed\n")
    finish_run({"target": "saved"}, path, reason=reason)
    assert events == ["return", "close_client"]
    report = yaml.safe_load(path.read_text())
    assert report["idle_return"]["status"] == "returned"
    assert report["status"] == ("executed" if reason == "completed" else reason)
