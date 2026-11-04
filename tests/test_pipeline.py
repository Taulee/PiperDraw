from copy import deepcopy
from itertools import pairwise
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

from piper_draw.artifacts import Artifacts
from piper_draw.geometry import PaperFrame, flange_pose, resample
from piper_draw.job import make_job, prepare
from piper_draw.vectorize import Stroke, trace, vectorize


@pytest.fixture
def config():
    return yaml.safe_load((Path(__file__).parents[1] / "config/sim.yaml").read_text())


def test_tilted_paper_frame_and_tcp_offset():
    frame = PaperFrame.from_points([100, 200, 300], [200, 200, 300], [100, 300, 400])
    assert np.allclose(frame.rotation.T @ frame.rotation, np.eye(3))
    assert np.isclose(np.linalg.det(frame.rotation), 1)
    tip = frame.point([10, 20], 8)
    quaternion = [0, 1, 0, 0]
    pose = flange_pose(tip, quaternion, [0, 0, 80])
    recovered = np.array(pose[:3]) + Rotation.from_quat(pose[3:]).apply([0, 0, 0.08])
    assert np.allclose(tip, recovered)


def test_collinear_calibration_is_rejected():
    with pytest.raises(ValueError, match="共线"):
        PaperFrame.from_points([0, 0, 0], [100, 0, 0], [200, 0, 0])


def test_graph_visits_t_junction_edges_once():
    mask = np.zeros((7, 7), np.uint8)
    mask[1, 1:6] = 1
    mask[1:6, 3] = 1
    chains = trace(mask)
    edges = [tuple(sorted((tuple(a), tuple(b)))) for p, _, _ in chains for a, b in pairwise(p)]
    assert len(chains) == 3
    assert len(edges) == len(set(edges)) == 8


def test_loop_is_closed_and_resampling_preserves_closure():
    mask = np.zeros((8, 8), np.uint8)
    mask[1, 1:7] = mask[6, 1:7] = 1
    mask[1:7, 1] = mask[1:7, 6] = 1
    chains = trace(mask)
    assert len(chains) == 1 and chains[0][1]
    points = resample(chains[0][0].astype(float), 0.4)
    assert np.array_equal(points[0], points[-1])
    assert np.linalg.norm(np.diff(points, axis=0), axis=1).max() <= 0.4 + 1e-8


def test_thick_line_yields_one_centerline(tmp_path, config):
    image = np.full((101, 101), 255, np.uint8)
    cv2.line(image, (10, 25), (90, 25), 0, 5)
    path = tmp_path / "line_art.png"
    cv2.imwrite(str(path), image)
    strokes, _ = vectorize(path, config)
    assert len(strokes) == 1
    assert np.ptp(strokes[0].points[:, 1]) < 1e-5
    assert strokes[0].points[:, 1].mean() > config["paper"]["height_mm"] / 2


def test_job_lifts_between_disconnected_strokes(config):
    strokes = [
        Stroke(np.array([[5.0, 5.0], [20.0, 5.0]])),
        Stroke(np.array([[30.0, 30.0], [40.0, 40.0]])),
    ]
    job = make_job(strokes, config)
    assert [s["kind"] for s in job["segments"]] == [
        "approach",
        "lower",
        "draw",
        "lift",
        "transit",
        "lower",
        "draw",
        "lift",
    ]
    for i in (0, 4):
        high = np.array(job["segments"][i]["poses"][0][:3])
        low = np.array(job["segments"][i + 1]["poses"][0][:3])
        assert np.allclose(high - low, np.array(job["paper_normal"]) * 0.008)
    assert not job["calibrated"]
    assert [segment["speed_m_s"] for segment in job["segments"]] == [
        0.045,
        0.005,
        0.01,
        0.01,
        0.045,
        0.005,
        0.01,
        0.01,
    ]


def test_lift_speed_is_independent_and_old_config_keeps_lower_speed(config):
    strokes = [Stroke(np.array([[30.0, 30.0], [40.0, 40.0]]))]
    legacy = deepcopy(config)
    for name in ("lift_mm_s", "travel_velocity_scale", "travel_acceleration_scale"):
        del legacy["motion"][name]
    legacy["motion"]["travel_mm_s"] = 30.0
    old_job = make_job(strokes, legacy)
    assert [segment["speed_m_s"] for segment in old_job["segments"]] == [
        0.03,
        0.005,
        0.01,
        0.005,
    ]
    config["motion"]["lift_mm_s"] = 12.0
    new_job = make_job(strokes, config)
    assert new_job["segments"][-1]["speed_m_s"] == 0.012
    assert new_job["segments"][1]["speed_m_s"] == 0.005
    assert new_job["motion"]["travel_velocity_scale"] == 0.3
    assert new_job["motion"]["velocity_scale"] == 0.2
    assert [segment["poses"] for segment in new_job["segments"]] == [
        segment["poses"] for segment in old_job["segments"]
    ]


def test_tool_orientation_follows_paper(config):
    tilted = deepcopy(config)
    tilted["paper"]["y_point_mm"] = [300.0, 60.0, 150.0]
    job = make_job([Stroke(np.array([[0.0, 0.0], [5.0, 5.0]]))], tilted)
    pose = job["segments"][0]["poses"][0]
    axis = Rotation.from_quat(pose[3:]).apply([0, 0, 1])
    assert np.allclose(axis, -np.array(job["paper_normal"]))


def test_png_to_job_and_svg_round_trip(tmp_path, config):
    image = np.full((160, 160), 255, np.uint8)
    cv2.rectangle(image, (30, 30), (130, 130), 0, 3)
    path = tmp_path / "line_art.png"
    cv2.imwrite(str(path), image)
    settings = tmp_path / "config.yaml"
    settings.write_text(yaml.safe_dump(config))
    artifacts = Artifacts.create(tmp_path / "output")
    job = prepare(path, settings, artifacts)
    assert yaml.safe_load(artifacts.path("job", ".yaml").read_text()) == job
    draw = next(s for s in job["segments"] if s["kind"] == "draw")
    lower = next(s for s in job["segments"] if s["kind"] == "lower")
    assert np.allclose(draw["poses"][-1], lower["poses"][0])
    assert artifacts.path("preview", ".svg").read_text().startswith("<svg")
    assert yaml.safe_load(artifacts.path("drawing_config", ".yaml").read_text()) == config
