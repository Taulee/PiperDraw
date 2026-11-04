"""从栅格到最终任务的去锯齿回归，以及采样和图连接保护。"""

import json
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml
from scipy.interpolate import BSpline

from piper_draw.artifacts import Artifacts
from piper_draw.curves import _fit_spline, protect_connections, smooth_path
from piper_draw.geometry import resample, sample_curve
from piper_draw.job import prepare
from piper_draw.stroke_graph import trace_strokes
from piper_draw.vectorize import image_frame, vectorize


def config():
    return yaml.safe_load((Path(__file__).parents[1] / "config/sim.yaml").read_text())


def distance(samples, path):
    delta = np.diff(path, axis=0)
    squared = np.sum(delta * delta, axis=1)
    distances = []
    for point in samples:
        along = np.clip(np.sum((point - path[:-1]) * delta, axis=1) / squared, 0, 1)
        distances.append(np.linalg.norm(point - path[:-1] - along[:, None] * delta, axis=1).min())
    return np.array(distances)


def test_bezier_with_midpoint_on_chord_still_subdivides():
    controls = np.array([[0.0, 0.0], [1.0, 3.0], [2.0, -3.0], [3.0, 0.0]])
    curve = BSpline([0, 0, 0, 0, 1, 1, 1, 1], controls, 3)
    points, parameters = sample_curve(curve, 100, 0.03, fixed_parameters=[0.37])
    assert len(points) > 4
    assert 0.37 in parameters
    assert distance(curve(np.linspace(0, 1, 2001)), points).max() <= 0.03


def test_sampling_limits_step_and_tracks_arc_length():
    curve = BSpline([0, 0, 0, 0, 1, 1, 1, 1], [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [10.0, 0.0]], 3)
    points, parameters = sample_curve(curve, 1, 0.03)
    assert np.linalg.norm(np.diff(points, axis=0), axis=1).max() <= 1 + 1e-10
    # x=10u^3，等参数会产生极不均匀点距。
    assert np.ptp(np.diff(points[:, 0])) < 0.01
    assert np.array_equal(points, curve(parameters))


def test_periodic_fit_has_matching_position_velocity_and_acceleration():
    theta = np.linspace(0, 2 * np.pi, 101)
    points = 10 * np.column_stack((np.cos(theta), np.sin(theta)))
    points[-1] = points[0]
    lengths = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    curve = _fit_spline(points, lengths, 3, 0.6, [], True)
    for derivative in range(3):
        assert np.allclose(curve(0, nu=derivative), curve(lengths[-1], nu=derivative), atol=1e-9)


def test_internal_graph_anchor_is_exact_and_reported_fit_is_bounded():
    x = np.linspace(0, 20, 201)
    points = np.column_stack((x, np.sin(x / 5) + 0.04 * np.cos(40 * x)))
    records = []
    result = smooth_path(points, False, 0.15, fixed_points=points[[71]], diagnostics=records)
    assert any(np.array_equal(point, points[71]) for point in result)
    assert np.array_equal(result[[0, -1]], points[[0, -1]])
    assert records[0]["method"] == "spline"
    assert records[0]["max_deviation_mm"] <= 0.15
    assert distance(resample(result, 0.01), points).max() <= 0.15 + 1e-8
    assert distance(resample(points, 0.01), result).max() <= 0.15 + 1e-8


def test_trace_exposes_internal_junction_without_changing_default_contract():
    mask = np.zeros((13, 13), np.uint8)
    mask[3, 1:12] = mask[3:12, 6] = 255
    ordinary = trace_strokes(mask, 0.2, 0)
    annotated = trace_strokes(mask, 0.2, 0, with_anchors=True)
    assert all(len(stroke) == 3 for stroke in ordinary)
    for (points, closed, protected), (other, c, p, anchors) in zip(ordinary, annotated):
        assert np.array_equal(points, other) and closed == c and protected == p
        assert [3, 6] in other[anchors].tolist()
    through = next(stroke for stroke in annotated if np.all(stroke[0][:, 0] == 3))
    assert 0 < through[3][0] < len(through[0]) - 1


def test_new_interstroke_connection_is_rejected_and_not_counted_as_success():
    original = np.array([[0.0, 0.0], [1.0, 1.0], [2.0, 0.0]])
    nearby = np.array([[0.5, 0.2], [1.5, 0.2]])
    # 原折线在此线段上方，候选曲线新增交点。
    candidate = np.array([[0.0, 0.0], [1.0, 0.3], [2.0, 0.0]])
    rows = [{"stroke": 0, "method": "spline"}]
    paths = protect_connections(
        [(candidate, False), (nearby, False)], [(original, False), (nearby, False)], rows
    )
    assert np.array_equal(paths[0][0], original)
    assert rows[0]["method"] == "preserved"
    assert rows[0]["reason"] == "new_interstroke_connection"


def test_existing_graph_connection_and_collinear_overlap_are_allowed():
    first = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]])
    second = np.array([[1.0, 0.0], [1.0, 1.0]])
    overlap = np.array([[0.25, 0.0], [1.75, 0.0]])
    original = [(first, False), (second, False), (overlap, False)]
    result = protect_connections([(first[[0, -1]], False), *original[1:]], original)
    assert len(result[0][0]) == 2


def test_failed_support_is_explicit_preservation():
    x = np.linspace(0, 10, 51)
    points = np.column_stack((x, 0.1 * np.sin(x)))
    rows = []
    result = smooth_path(
        points, False, 0.15, supported=lambda path: np.array_equal(path, points), diagnostics=rows
    )
    assert np.array_equal(result, points)
    assert rows[0]["method"] == "preserved"
    assert rows[0]["reason"] == "black_support"


@pytest.mark.parametrize(
    "a,b", [((60, 180), (660, 740)), ((60, 400), (660, 480)), ((300, 60), (380, 940))]
)
def test_raster_line_to_task_has_low_normal_wave(tmp_path, a, b):
    values = config()
    image = np.full((1024, 724), 255, np.uint8)
    cv2.line(image, a, b, 0, 5, cv2.LINE_AA)
    path = tmp_path / "line.png"
    assert cv2.imwrite(str(path), image)
    strokes, _ = vectorize(path, values)
    assert len(strokes) == 1
    points = strokes[0].points
    factor, offset = image_frame(image.shape, values["paper"])
    start = np.array([a[0], 1023 - a[1]]) * factor + offset
    end = np.array([b[0], 1023 - b[1]]) * factor + offset
    tangent = (end - start) / np.linalg.norm(end - start)
    normal = np.array([-tangent[1], tangent[0]])
    along = (points - start) @ tangent
    low, high = sorted(along[[0, -1]])
    middle = points[(along > low + 2) & (along < high - 2)]
    # 从实际取线路径端部各排除 2 mm；旧算法峰峰值为 .207 / .164 / .162 mm。
    assert np.ptp((middle - start) @ normal) < 0.048
    assert np.linalg.norm(np.diff(points, axis=0), axis=1).max() <= 1 + 1e-9
    artifacts = Artifacts.create(tmp_path / "output")
    settings = tmp_path / "config.yaml"
    settings.write_text(yaml.safe_dump(values))
    job = prepare(path, settings, artifacts)
    assert len([segment for segment in job["segments"] if segment["kind"] == "draw"]) == 1
    report = json.loads(artifacts.path("smoothing", ".json").read_text())
    assert report["algorithm"] == "constrained_cubic_bspline_v1"
    assert all(row["method"] != "preserved" for row in report["spans"])


def test_legacy_window_warns_and_new_chord_tolerance_is_validated(tmp_path):
    values = config()
    values["image"]["smoothing_span_mm"] = 4
    values["image"]["curve_chord_error_mm"] = 0
    with (
        pytest.warns(UserWarning, match="smoothing_span_mm"),
        pytest.raises(ValueError, match="curve_chord_error_mm"),
    ):
        vectorize(tmp_path / "unused.png", values)


def test_raster_circle_preserves_radius_and_smooth_closed_seam(tmp_path):
    values = config()
    image = np.full((1024, 724), 255, np.uint8)
    cv2.circle(image, (362, 512), 130, 0, 5, cv2.LINE_AA)
    path = tmp_path / "circle.png"
    assert cv2.imwrite(str(path), image)
    rows = []
    strokes, _ = vectorize(path, values, diagnostics=rows)
    assert len(strokes) == 1 and strokes[0].closed
    points = strokes[0].points
    assert np.array_equal(points[0], points[-1])
    factor, offset = image_frame(image.shape, values["paper"])
    center = np.array([362, 1023 - 512]) * factor + offset
    assert np.max(np.abs(np.linalg.norm(points - center, axis=1) - 130 * factor)) < 0.2
    deltas = np.diff(points, axis=0)
    directions = np.arctan2(deltas[:, 1], deltas[:, 0])
    turns = np.angle(np.exp(1j * (np.roll(directions, -1) - directions)))
    assert np.abs(turns).sum() < 2 * np.pi * 1.05
    assert rows[0]["method"] == "spline"


def test_raster_junction_is_still_fixed_in_final_ordered_paths(tmp_path):
    values = config()
    values["image"].update(spur_length_mm=0, small_hole_area_mm2=0)
    image = np.full((201, 201), 255, np.uint8)
    cv2.line(image, (20, 75), (180, 115), 0, 3)
    cv2.line(image, (100, 95), (80, 175), 0, 3)
    path = tmp_path / "junction.png"
    assert cv2.imwrite(str(path), image)
    strokes, skeleton = vectorize(path, values)
    factor, offset = image_frame(image.shape, values["paper"])
    chains = trace_strokes(skeleton, factor, 0, with_anchors=True)
    anchors = np.vstack([pixels[indices] for pixels, _, _, indices in chains if len(indices)])
    assert len(anchors)
    fixed = np.column_stack((anchors[:, 1], 200 - anchors[:, 0])) * factor + offset
    output = np.vstack([stroke.points for stroke in strokes])
    assert all(np.any(np.all(output == point, axis=1)) for point in fixed)
