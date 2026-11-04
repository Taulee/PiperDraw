from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from piper_draw.curves import _has_intersection, smooth_path
from piper_draw.geometry import resample
from piper_draw.simple_lines import (
    HighlightClearanceError,
    fill_small_holes,
    prune_spurs,
    smooth_supported_path,
)
from piper_draw.stroke_graph import trace
from piper_draw.vectorize import _supported_path, vectorize


@pytest.fixture
def config():
    values = yaml.safe_load((Path(__file__).parents[1] / "config/sim.yaml").read_text())
    values["paper"].update(width_mm=40.0, height_mm=40.0, margin_mm=0.0)
    # 保留白色高光作为基准；测试填孔行为时再显式开启。
    values["image"].update(
        mode="simple", spur_length_mm=0.8, smoothing_span_mm=4.0, small_hole_area_mm2=0
    )
    return values


def write_image(directory, image):
    path = directory / "lineart.png"
    assert cv2.imwrite(str(path), image)
    return path


def turning_variation(points):
    directions = np.unwrap(np.arctan2(*np.diff(points, axis=0)[:, ::-1].T))
    return np.abs(np.diff(directions)).sum()


def distance_to_path(samples, points):
    starts, delta = points[:-1], np.diff(points, axis=0)
    squared_lengths = np.sum(delta * delta, axis=1)
    result = []
    for sample in samples:
        along = np.clip(np.sum((sample - starts) * delta, axis=1) / squared_lengths, 0, 1)
        result.append(np.linalg.norm(sample - starts - along[:, None] * delta, axis=1).min())
    return np.asarray(result)


def test_spur_pruning_keeps_main_endpoints_and_short_independent_feature():
    skeleton = np.zeros((25, 35), np.uint8)
    skeleton[10, 2:33] = 255
    skeleton[7:10, 15] = 255
    skeleton[20, 4:7] = 255
    result = prune_spurs(skeleton, 0.2, 0.8)

    assert np.all(result[10, 2:33])
    assert not result[7:10, 15].any()
    assert np.all(result[20, 4:7])
    assert np.count_nonzero(result) == np.count_nonzero(skeleton) - 3


def test_pruning_preserves_closed_loop_and_short_bridge_between_junctions():
    skeleton = np.zeros((30, 30), np.uint8)
    skeleton[2, 2:8] = skeleton[7, 2:8] = 255
    skeleton[2:8, 2] = skeleton[2:8, 7] = 255
    skeleton[11:28, 15] = skeleton[11:28, 18] = 255
    skeleton[19, 15:19] = 255
    result = prune_spurs(skeleton, 0.2, 0.8)

    assert np.array_equal(result, skeleton)
    assert any(closed for _, closed, _ in trace(result))


def test_pruning_removes_short_attached_spur_without_opening_loop():
    skeleton = np.zeros((31, 31), np.uint8)
    skeleton[10, 10:21] = skeleton[20, 10:21] = 255
    skeleton[10:21, 10] = skeleton[10:21, 20] = 255
    skeleton[7:10, 15] = 255
    result = prune_spurs(skeleton, 0.2, 0.8)

    assert not result[7:10, 15].any()
    assert any(closed for _, closed, _ in trace(result))
    assert np.array_equal(result[10:21, 10:21], skeleton[10:21, 10:21])


def test_single_pass_pruning_does_not_erase_a_small_y_feature():
    skeleton = np.zeros((11, 11), np.uint8)
    skeleton[3:6, 5] = skeleton[5, 3:8] = 255
    assert np.array_equal(prune_spurs(skeleton, 0.2, 0.8), skeleton)
    assert np.array_equal(prune_spurs(skeleton, 0.2, 0), skeleton)


def test_simple_mode_keeps_short_eye_nose_and_mouth_lines(tmp_path, config):
    image = np.full((201, 201), 255, np.uint8)
    for x, y in ((75, 75), (120, 75), (100, 95), (100, 120)):
        cv2.line(image, (x, y), (x + 5, y), 0, 2)
    config["image"]["min_stroke_mm"] = 9.0
    strokes, _ = vectorize(write_image(tmp_path, image), config)

    assert len(strokes) == 4
    assert all(
        0 < np.linalg.norm(np.diff(stroke.points, axis=0), axis=1).sum() < 1.4 for stroke in strokes
    )


def test_simple_mode_preserves_solid_eye_dots_without_double_tracing(tmp_path, config):
    image = np.full((201, 201), 255, np.uint8)
    cv2.circle(image, (75, 80), 3, 0, -1)
    cv2.circle(image, (125, 80), 3, 0, -1)
    strokes, _ = vectorize(write_image(tmp_path, image), config)

    assert len(strokes) == 2
    assert all(len(stroke.points) == 2 and not stroke.closed for stroke in strokes)
    assert all(
        0 < np.linalg.norm(stroke.points[-1] - stroke.points[0]) <= 0.25 for stroke in strokes
    )
    assert sorted(np.mean(stroke.points[:, 0]) for stroke in strokes) == [15.0, 25.0]


def test_large_compact_solid_dot_is_summarized_instead_of_silently_dropped(tmp_path, config):
    image = np.full((201, 201), 255, np.uint8)
    cv2.circle(image, (100, 100), 6, 0, -1)
    strokes, _ = vectorize(write_image(tmp_path, image), config)

    assert len(strokes) == 2
    (outline,) = [stroke for stroke in strokes if stroke.closed]
    (center,) = [stroke for stroke in strokes if not stroke.closed]
    assert np.ptp(outline.points[:, 0]) == pytest.approx(2.4)
    assert np.ptp(outline.points[:, 1]) == pytest.approx(2.4)
    assert np.linalg.norm(center.points[-1] - center.points[0]) == pytest.approx(1.9)
    assert np.array_equal(np.mean(center.points, axis=0), [20.0, 20.0])


def test_attached_thick_pupil_stays_visible_and_eyelid_is_not_double_traced(tmp_path, config):
    image = np.full((201, 201), 255, np.uint8)
    cv2.line(image, (20, 95), (180, 95), 0, 5)
    cv2.circle(image, (100, 103), 11, 0, -1)
    strokes, _ = vectorize(write_image(tmp_path, image), config)

    assert any(stroke.closed for stroke in strokes)
    pupil_points = np.vstack([stroke.points for stroke in strokes if stroke.closed])
    assert np.ptp(pupil_points[:, 1]) >= 3.0
    assert any(
        not stroke.closed
        and np.ptp(stroke.points[:, 1]) < 1e-8
        and np.ptp(stroke.points[:, 0]) > 1.0
        and np.mean(stroke.points[:, 1]) < 21
        for stroke in strokes
    )
    for x in (8.0, 32.0):
        nearby = [point for stroke in strokes for point in stroke.points if abs(point[0] - x) < 0.3]
        assert nearby
        assert np.ptp(np.asarray(nearby)[:, 1]) < 0.5


def test_thick_line_uses_one_centerline_and_does_not_join_blank_gap(tmp_path, config):
    image = np.full((201, 201), 255, np.uint8)
    cv2.line(image, (20, 50), (180, 50), 0, 7)
    cv2.line(image, (20, 150), (75, 150), 0, 3)
    cv2.line(image, (85, 150), (180, 150), 0, 3)
    strokes, _ = vectorize(write_image(tmp_path, image), config)

    assert len(strokes) == 3
    top = [stroke for stroke in strokes if np.mean(stroke.points[:, 1]) > 20]
    assert len(top) == 1
    assert np.ptp(top[0].points[:, 1]) == 0
    for stroke in strokes:
        if np.mean(stroke.points[:, 1]) < 20:
            assert stroke.points[:, 0].max() < 16 or stroke.points[:, 0].min() > 16


@pytest.mark.parametrize("radius", [1, 6])
def test_centerline_and_local_smoothing_keep_white_highlight(tmp_path, config, radius):
    image = np.full((201, 201), 255, np.uint8)
    cv2.circle(image, (100, 100), 20, 0, -1)
    cv2.circle(image, (100, 92), radius, 255, -1)
    hole = np.zeros_like(image)
    cv2.circle(hole, (100, 92), radius, 255, -1)
    white_pixels = np.argwhere(hole)
    white_points = np.column_stack((white_pixels[:, 1], 200 - white_pixels[:, 0])) * 0.2
    strokes, _ = vectorize(write_image(tmp_path, image), config)

    assert any(stroke.closed for stroke in strokes)
    for stroke in strokes:
        assert (
            np.min(distance_to_path(white_points, stroke.points))
            > config["image"]["pen_width_mm"] / 2
        )
        dense = resample(stroke.points, 0.025) / 0.2
        dense[:, 1] = 200 - dense[:, 1]
        pixels = np.rint(dense).astype(int)
        assert not np.any(hole[pixels[:, 1], pixels[:, 0]])


@pytest.mark.parametrize("thickness", [1, 2])
def test_thin_empty_eye_loop_keeps_centerline_without_promising_ink_clearance(
    tmp_path, config, thickness
):
    image = np.full((201, 201), 255, np.uint8)
    cv2.circle(image, (100, 100), 10, 0, thickness)
    strokes, _ = vectorize(write_image(tmp_path, image), config)

    assert len(strokes) == 1 and strokes[0].closed
    assert _supported_path(strokes[0].points, 255 - image, 0.2, np.zeros(2))


def test_thick_block_with_highlight_too_near_border_is_rejected(tmp_path, config):
    image = np.full((201, 201), 255, np.uint8)
    cv2.circle(image, (100, 100), 10, 0, -1)
    cv2.circle(image, (108, 100), 1, 255, -1)
    with pytest.raises(HighlightClearanceError, match="白色高光无法容纳当前笔宽"):
        vectorize(write_image(tmp_path, image), config)


@pytest.mark.parametrize("area_mm2", [0.8, 1.0])
def test_small_hole_filling_keeps_large_lens_and_border_connected_white_region(area_mm2):
    mask = np.zeros((101, 101), np.uint8)
    mask[10:90, 10:90] = 255
    mask[20:24, 20:24] = 0  # 0.64 mm² 的微孔。
    mask[40:60, 40:60] = 0  # 16 mm² 的镜片白区。
    mask[10:14, 75:79] = 0  # 与外白背景连接，即使面积很小也不能填。
    result = fill_small_holes(mask, 0.2, area_mm2)

    assert np.all(result[20:24, 20:24] == 255)
    assert not result[40:60, 40:60].any()
    assert not result[10:14, 75:79].any()
    assert np.count_nonzero(result != mask) == 16
    assert np.array_equal(fill_small_holes(mask, 0.2, 0), mask)


def test_small_hole_area_threshold_is_inclusive_and_distinguishes_08_from_10():
    mask = np.full((31, 31), 255, np.uint8)
    mask[10:15, 10:15] = 0  # 25 × 0.2² = 1 mm²。

    assert np.array_equal(fill_small_holes(mask, 0.2, 0.8), mask)
    assert np.all(fill_small_holes(mask, 0.2, 1.0))


def test_small_highlight_can_be_omitted_before_the_pen_width_guard(tmp_path, config):
    image = np.full((201, 201), 255, np.uint8)
    cv2.circle(image, (100, 100), 10, 0, -1)
    cv2.circle(image, (108, 100), 1, 255, -1)
    config["image"]["small_hole_area_mm2"] = 0.8
    strokes, _ = vectorize(write_image(tmp_path, image), config)

    assert any(stroke.closed for stroke in strokes)
    assert any(not stroke.closed for stroke in strokes)


def test_large_highlight_with_an_independent_black_line_is_not_rejected(tmp_path, config):
    image = np.full((151, 151), 255, np.uint8)
    cv2.circle(image, (75, 75), 30, 0, -1)
    cv2.circle(image, (75, 75), 5, 255, -1)
    cv2.line(image, (73, 75), (77, 75), 0, 1)
    config["paper"].update(width_mm=22.5, height_mm=22.5)
    config["image"]["min_component_pixels"] = 1
    strokes, _ = vectorize(write_image(tmp_path, image), config)

    assert any(stroke.closed for stroke in strokes)
    (line,) = [
        stroke for stroke in strokes if not stroke.closed and np.ptp(stroke.points[:, 0]) < 1
    ]
    assert np.array_equal(line.points[:, 1], [11.25, 11.25])
    assert np.linalg.norm(line.points[-1] - line.points[0]) == pytest.approx(0.6)


def test_local_support_failure_does_not_restore_the_entire_jagged_arc():
    theta = np.linspace(0, np.pi / 2, 301)
    pixels = np.round(80 * np.column_stack((np.cos(theta), np.sin(theta))) + 20).astype(int)
    pixels = pixels[np.r_[True, np.any(np.diff(pixels, axis=0), axis=1)]]
    mask = np.zeros((121, 121), np.uint8)
    cv2.polylines(mask, [pixels], False, 255, 5)
    points = np.column_stack((pixels[:, 0], 120 - pixels[:, 1])) * 0.1
    candidate = smooth_path(points, False, 0.2)
    candidate_pixels = resample(candidate, 0.01) / 0.1
    candidate_pixels[:, 1] = 120 - candidate_pixels[:, 1]
    candidate_pixels = np.unique(np.rint(candidate_pixels).astype(int), axis=0)
    original_pixels = set(map(tuple, pixels))
    carved = False
    for x, y in candidate_pixels:
        if (x, y) in original_pixels or not 45 < x < 90:
            continue
        test_mask = mask.copy()
        test_mask[y, x] = 0
        if _supported_path(points, test_mask, 0.1, np.zeros(2)):
            mask = test_mask
            carved = True
            break
    assert carved
    assert not _supported_path(candidate, mask, 0.1, np.zeros(2))
    result = smooth_supported_path(
        points, False, 0.2, 4.0, lambda path: _supported_path(path, mask, 0.1, np.zeros(2))
    )

    assert _supported_path(result, mask, 0.1, np.zeros(2))
    assert np.array_equal(result[[0, -1]], points[[0, -1]])
    assert turning_variation(result) < 0.7 * turning_variation(points)
    assert distance_to_path(resample(result, 0.025), points).max() <= 0.2 + 1e-8
    assert distance_to_path(resample(points, 0.025), result).max() <= 0.2 + 1e-8
    assert not _has_intersection(result, False)


def test_local_smoothing_preserves_sharp_corner_and_existing_crossing():
    right_angle = np.array([[0.0, 0.0], [10.0, 0.0], [10.0, 10.0]])
    crossing = np.array([[0.0, 0.0], [4.0, 4.0], [0.0, 4.0], [4.0, 0.0]])
    for points in (right_angle, crossing):
        result = smooth_supported_path(points, False, 0.2, 4.0, lambda _: True)
        assert np.array_equal(result, points)


def test_local_smoothing_does_not_add_intersection_in_nearby_fold():
    points = np.array(
        [
            [-0.02, -0.09],
            [-0.36, -0.46],
            [0.72, -1.69],
            [0.36, -1.57],
            [-0.26, -2.65],
            [-0.87, -0.47],
            [0.09, 0.11],
            [1.41, -0.82],
            [1.88, -2.36],
        ]
    )
    assert not _has_intersection(points, False)
    result = smooth_supported_path(points, False, 0.8, 4.0, lambda _: True)
    assert not _has_intersection(result, False)
    assert np.array_equal(result[[0, -1]], points[[0, -1]])


def test_small_positive_window_still_advances_at_large_arc_lengths():
    points = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [3.0, 0.0]])
    result = smooth_supported_path(points, False, 0.2, np.finfo(float).eps, lambda _: True)
    assert np.array_equal(result, points)


@pytest.mark.parametrize(
    "name,value",
    [
        ("spur_length_mm", -1),
        ("spur_length_mm", np.inf),
        ("smoothing_span_mm", 0),
        ("small_hole_area_mm2", -1),
        ("small_hole_area_mm2", np.inf),
    ],
)
def test_simple_parameters_reject_invalid_values(tmp_path, config, name, value):
    image = np.full((201, 201), 255, np.uint8)
    cv2.line(image, (20, 100), (180, 100), 0, 2)
    config["image"][name] = value
    with pytest.raises(ValueError, match=name):
        vectorize(write_image(tmp_path, image), config)


def test_unknown_vectorization_mode_is_rejected(tmp_path, config):
    config["image"]["mode"] = "simpe"
    with pytest.raises(ValueError, match="image.mode"):
        vectorize(tmp_path / "unused.png", config)
