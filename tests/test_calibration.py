"""用已知 TCP 和倾斜纸面验证标定、配置与绘画接口；不连接实体设备。"""

from copy import deepcopy
from pathlib import Path
from unittest.mock import Mock

import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

from piper_draw import calibration, cli, robot, ros_client
from piper_draw.device import DeviceConfig
from piper_draw.geometry import PaperFrame
from piper_draw.job import make_job
from piper_draw.vectorize import Stroke, image_frame


@pytest.fixture
def config():
    return yaml.safe_load((Path(__file__).parents[1] / "config/real.example.yaml").read_text())


@pytest.fixture
def tcp_samples():
    offset = np.array([12.0, -8.0, 170.0])
    center = np.array([300.0, 20.0, 30.0])
    rotations = Rotation.from_euler("xyz", [
        [180, 0, 0], [155, 0, 0], [180, 30, 0],
        [180, 0, 40], [160, -25, 15], [200, 15, -20],
    ], degrees=True)
    return [
        {"position_mm": (center - rotation.apply(offset)).tolist(),
         "quaternion_xyzw": rotation.as_quat().tolist()}
        for rotation in rotations
    ]


def observations(config, samples):
    return {"robot": calibration.robot_identity(config), "samples": samples}


def paper_samples(offset, width=190, height=277, skew=0):
    frame = Rotation.from_euler("xyz", [12, -8, 25], degrees=True)
    origin = np.array([250, -120, 15])
    rotations = frame * Rotation.from_euler("xyz", [
        [180, 0, 0], [170, 10, 0], [190, -15, 10]
    ], degrees=True)
    points = origin + frame.apply([[0, 0, 0], [width, 0, 0], [skew, height, 0]])
    samples = [
        {"position_mm": (point - rotation.apply(offset)).tolist(),
         "quaternion_xyzw": rotation.as_quat().tolist()}
        for point, rotation in zip(points, rotations)
    ]
    return samples, points, frame


def test_pivot_recovers_offset_and_unknown_fixed_point(tcp_samples):
    result = calibration.solve_tcp(tcp_samples)
    np.testing.assert_allclose(result["tcp_offset_mm"], [12, -8, 170], atol=1e-10)
    np.testing.assert_allclose(result["contact_point_mm"], [300, 20, 30], atol=1e-10)
    assert result["max_error_mm"] < 1e-10


def test_pivot_with_measurement_noise_and_bad_contact(tcp_samples):
    random = np.random.default_rng(4)
    for sample in tcp_samples:
        sample["position_mm"] = (np.array(sample["position_mm"]) + random.normal(0, 0.05, 3)).tolist()
    result = calibration.solve_tcp(tcp_samples)
    np.testing.assert_allclose(result["tcp_offset_mm"], [12, -8, 170], atol=0.3)
    assert 0 < result["rms_error_mm"] < 0.2
    tcp_samples[-1]["position_mm"][0] += 10
    with pytest.raises(ValueError, match="最大残差"):
        calibration.solve_tcp(tcp_samples)


@pytest.mark.parametrize("angles", [[0, 0, 0, 0], [0, 30, 60, 90], [0, 0.1, 0.2, 0.3]])
def test_pivot_rejects_unobservable_axis(angles):
    samples = [{"position_mm": [0, 0, 0], "quaternion_xyzw": r.as_quat().tolist()}
               for r in Rotation.from_euler("z", angles, degrees=True)]
    with pytest.raises(ValueError, match="姿态变化不足"):
        calibration.solve_tcp(samples)


@pytest.mark.parametrize("change", [
    lambda samples: samples.pop(),
    lambda samples: samples[0].update(position_mm=[float("nan"), 0, 0]),
    lambda samples: samples[0].update(quaternion_xyzw=[0, 0, 0, 0]),
])
def test_pivot_rejects_invalid_inputs(tcp_samples, change):
    samples = deepcopy(tcp_samples[:4])
    change(samples)
    with pytest.raises(ValueError):
        calibration.solve_tcp(samples)


def test_tilted_paper_uses_tcp_and_each_samples_orientation():
    samples, points, rotation = paper_samples([12, -8, 170])
    result = calibration.locate_paper(samples, [12, -8, 170])
    paper = result["paper"]
    assert paper["width_mm"] == pytest.approx(190)
    assert paper["height_mm"] == pytest.approx(277)
    np.testing.assert_allclose(paper["drawing_area"]["origin_mm"], [21, 33.5])
    np.testing.assert_allclose(result["normal"], rotation.apply([0, 0, 1]))
    np.testing.assert_allclose(result["fourth_corner_mm"], points[1] + points[2] - points[0])
    recovered = rotation * Rotation.from_quat(result["quaternion_in_paper"])
    expected = Rotation.from_quat(samples[0]["quaternion_xyzw"])
    assert (recovered.inv() * expected).magnitude() < 1e-12


def test_paper_landscape_and_insufficient_room():
    samples, _, _ = paper_samples([0, 0, 170], width=280, height=180)
    assert calibration.locate_paper(samples, [0, 0, 170])["orientation"] == "landscape"
    with pytest.raises(ValueError, match="放不下"):
        calibration.locate_paper(samples, [0, 0, 170], orientation="portrait")
    samples, _, _ = paper_samples([0, 0, 170], width=140, height=200)
    with pytest.raises(ValueError, match="放不下"):
        calibration.locate_paper(samples, [0, 0, 170])


def test_exact_a5_fit_is_not_rejected_by_rotation_roundoff():
    samples, _, _ = paper_samples([0, 0, 170], width=148, height=210)
    result = calibration.locate_paper(samples, [0, 0, 170])
    assert result["orientation"] == "portrait"
    np.testing.assert_allclose(result["paper"]["drawing_area"]["origin_mm"], [0, 0], atol=1e-10)


def test_paper_rejects_wrong_order_skew_and_collinearity():
    samples, _, _ = paper_samples([0, 0, 170])
    with pytest.raises(ValueError, match="法向"):
        calibration.locate_paper([samples[0], samples[2], samples[1]], [0, 0, 170])
    with pytest.raises(ValueError, match="共线"):
        calibration.locate_paper([samples[0], samples[1], samples[1]], [0, 0, 170])
    samples, _, _ = paper_samples([0, 0, 170], skew=30)
    with pytest.raises(ValueError, match="直角"):
        calibration.locate_paper(samples, [0, 0, 170])


def test_calibration_to_a5_strokes_to_flange_targets(config, tcp_samples):
    calibrated_tcp = calibration.calibrate_tcp(config, observations(config, tcp_samples))
    samples, points, rotation = paper_samples([12, -8, 170])
    located = calibration.calibrate_paper(calibrated_tcp, observations(config, samples))
    assert located["calibrated"] and not calibrated_tcp["calibrated"]
    # 129×191 的图像覆盖 A5 内留白 10 mm 后的 128×190 mm 区域。
    factor, offset = image_frame((191, 129), located["paper"])
    assert factor == pytest.approx(1)
    np.testing.assert_allclose(offset, [31, 43.5])
    stroke = Stroke(np.array([[0, 0], [128, 190]]) * factor + offset)
    job = make_job([stroke], located)
    assert job["calibrated"]
    np.testing.assert_allclose(job["table"]["size"][:2], [0.190, 0.277])
    np.testing.assert_allclose(job["paper_origin"], points[0] / 1000)
    for segment, xy, height in [
        (job["segments"][0], stroke.points[0], 8),
        (job["segments"][1], stroke.points[0], 0),
        (job["segments"][2], stroke.points[1], 0),
    ]:
        pose = segment["poses"][0]
        actual_tip = np.array(pose[:3]) * 1000 + Rotation.from_quat(pose[3:]).apply([12, -8, 170])
        expected_tip = points[0] + rotation.apply([*xy, height])
        np.testing.assert_allclose(actual_tip, expected_tip, atol=1e-9)
    # 求解与写配置不修改调用方输入；重新标 TCP 取消纸面许可。
    assert not config["calibrated"] and "calibration" not in config
    reset = calibration.calibrate_tcp(located, observations(config, tcp_samples))
    assert not reset["calibrated"] and "paper" not in reset["calibration"]


def test_paper_requires_matching_tcp(config, tcp_samples):
    samples, _, _ = paper_samples([12, -8, 170])
    data = observations(config, samples)
    with pytest.raises(ValueError, match="先完成"):
        calibration.calibrate_paper(config, data)
    calibrated = calibration.calibrate_tcp(config, observations(config, tcp_samples))
    result = calibration.calibrate_paper(calibrated, data)
    assert result["calibrated"]
    calibrated["tool"]["tcp_offset_mm"][2] += 1
    with pytest.raises(ValueError, match="TCP 已被修改"):
        calibration.calibrate_paper(calibrated, data)


def test_reject_model_mixing(config, tcp_samples):
    data = observations(config, tcp_samples)
    data["robot"]["dh_version"] = "old"
    with pytest.raises(ValueError, match="DH"):
        calibration.calibrate_tcp(config, data)


def test_image_frame_rejects_outside_drawing_area(config):
    config["paper"]["drawing_area"] = {
        "origin_mm": [100, 0], "width_mm": 148, "height_mm": 210
    }
    with pytest.raises(ValueError, match="超出纸面"):
        image_frame((100, 100), config["paper"])


@pytest.mark.parametrize("dh_version", ["new", "old"])
def test_urdf_fk_agrees_with_official_sdk_within_model_rounding(dh_version):
    from piper_sdk.kinematics.piper_fk import C_PiperForwardKinematics

    joints = [0.3, 1.0, -1.2, 0.2, 0.4, -0.3]
    actual = calibration.flange_from_joints(joints, dh_version)
    expected = C_PiperForwardKinematics(int(dh_version == "new")).CalFK(joints)[-1]
    # URDF 有更精细的 link6 平移及四位小数角度，与 SDK DH 的舍入不同。
    np.testing.assert_allclose(actual["position_mm"], expected[:3], atol=0.2)
    rotation = Rotation.from_quat(actual["quaternion_xyzw"])
    reference = Rotation.from_euler("xyz", expected[3:], degrees=True)
    assert (rotation.inv() * reference).magnitude() < 0.001


def test_capture_uses_sdk_readonly_interface_and_rejects_dh_mismatch(config, monkeypatch, tmp_path):
    doctor = Mock(return_value={"positions": [0, 0.7, -1, 0, 0.3, 0]})
    monkeypatch.setattr(robot, "doctor", doctor)
    device = DeviceConfig("can0", "v3", "new", tmp_path)
    sample = calibration.capture_pose(config, device=device)
    doctor.assert_called_once_with(device)
    assert sample["source"] == "sdk" and len(sample["position_mm"]) == 3
    doctor.reset_mock()
    with pytest.raises(ValueError, match="DH"):
        calibration.capture_pose(config, device=DeviceConfig("can0", "v2", "old", tmp_path))
    doctor.assert_not_called()


def test_capture_ros_checks_feedback_and_dh(config, monkeypatch):
    status = {"positions_rad": [0, 0.7, -1, 0, 0.3, 0],
              "dh_version": "new", "preflight_error": ""}
    command = Mock(return_value=status)
    monkeypatch.setattr(ros_client, "command", command)
    calibration.capture_pose(config, source="ros")
    command.assert_called_once_with("status")
    status["preflight_error"] = "反馈超时"
    with pytest.raises(RuntimeError, match="反馈超时"):
        calibration.capture_pose(config, source="ros")
    status.update(preflight_error="", dh_version="old")
    with pytest.raises(ValueError, match="DH"):
        calibration.capture_pose(config, source="ros")


def test_cli_capture_solve_and_failed_solve_preserves_file(config, tcp_samples, tmp_path,
                                                         monkeypatch, capsys):
    source = tmp_path / "config.yaml"
    source.write_text(yaml.safe_dump(config))
    samples = tmp_path / "tcp_samples.yaml"
    monkeypatch.setattr(calibration, "capture_pose", Mock(side_effect=tcp_samples))
    for _ in tcp_samples:
        cli.main(["calibrate", "capture", "--config", str(source), "--samples", str(samples),
                  "--source", "ros"])
    assert len(yaml.safe_load(samples.read_text())["samples"]) == 6
    output = tmp_path / "tcp.yaml"
    cli.main(["calibrate", "tcp", "--config", str(source), "--samples", str(samples),
              "--output", str(output)])
    tcp_config = yaml.safe_load(output.read_text())
    assert not tcp_config["calibrated"]
    paper, _, _ = paper_samples(tcp_config["tool"]["tcp_offset_mm"])
    samples.write_text(yaml.safe_dump(observations(config, paper)))
    final = tmp_path / "real.yaml"
    cli.main(["calibrate", "paper", "--config", str(output), "--samples", str(samples),
              "--output", str(final)])
    saved = final.read_text()
    assert yaml.safe_load(saved)["calibrated"]
    samples.write_text(yaml.safe_dump(observations(config, paper[:2])))
    with pytest.raises(SystemExit) as error:
        cli.main(["calibrate", "paper", "--config", str(output), "--samples", str(samples),
                  "--output", str(final)])
    assert error.value.code == 1 and "恰好 3" in capsys.readouterr().err
    assert final.read_text() == saved


def test_frame_rejects_nonfinite_points():
    with pytest.raises(ValueError, match="有限"):
        PaperFrame.from_points([0, 0, float("nan")], [100, 0, 0], [0, 100, 0])
