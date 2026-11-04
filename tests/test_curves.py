from itertools import pairwise

import numpy as np
import pytest

from piper_draw.curves import smooth_path


def _dense(points, step=0.01):
    samples = []
    for start, end in pairwise(points):
        count = max(1, int(np.ceil(np.linalg.norm(end - start) / step)))
        samples.extend(start + (end - start) * index / count for index in range(count))
    return np.vstack([*samples, points[-1]])


def _distance_to_path(samples, points):
    starts, delta = points[:-1], np.diff(points, axis=0)
    length_squared = np.sum(delta * delta, axis=1)
    distances = []
    for sample in samples:
        along = np.clip(np.sum((sample - starts) * delta, axis=1) / length_squared, 0, 1)
        distances.append(np.linalg.norm(sample - starts - along[:, None] * delta, axis=1).min())
    return np.asarray(distances)


def _turning_variation(points):
    directions = np.unwrap(np.arctan2(*np.diff(points, axis=0)[:, ::-1].T))
    return np.abs(np.diff(directions)).sum()


def test_pixel_noise_on_arc_is_smoothed_without_moving_endpoints():
    theta = np.linspace(0, np.pi / 2, 151)
    points = 15 * np.column_stack((np.cos(theta), np.sin(theta)))
    points = np.round(points / 0.15) * 0.15
    points = points[np.r_[True, np.any(np.diff(points, axis=0) != 0, axis=1)]]
    result = smooth_path(points, False, 0.2)
    assert np.array_equal(result[[0, -1]], points[[0, -1]])
    assert _turning_variation(result) < 0.5 * _turning_variation(points)
    assert _distance_to_path(_dense(result), points).max() <= 0.2 + 1e-8
    assert _distance_to_path(_dense(points), result).max() <= 0.2 + 1e-8


def test_straight_line_stays_straight():
    points = np.column_stack((np.linspace(0, 30, 100), np.linspace(0, 60, 100)))
    result = smooth_path(points, False, 0.2)
    assert np.array_equal(result, points[[0, -1]])


def test_smooth_arc_keeps_small_direction_changes_after_sparsification():
    theta = np.linspace(0, np.pi / 2, 201)
    points = 15 * np.column_stack((np.cos(theta), np.sin(theta)))
    result = smooth_path(points, False, 0.2)
    directions = np.unwrap(np.arctan2(*np.diff(result, axis=0)[:, ::-1].T))
    assert np.abs(np.diff(directions)).max() < np.deg2rad(12)
    assert _distance_to_path(_dense(points), result).max() <= 0.2 + 1e-8


@pytest.mark.parametrize("end", ([10.0, 10.0], [1.0, 1.0]))
def test_right_angle_and_sharp_tip_are_fixed(end):
    points = np.array([[0.0, 0.0], [10.0, 0.0], end])
    result = smooth_path(points, False, 0.2)
    assert np.array_equal(result, points)


def test_closed_circle_is_exactly_closed():
    theta = np.linspace(0, 2 * np.pi, 101)
    points = 10 * np.column_stack((np.cos(theta), np.sin(theta)))
    points[-1] = points[0]
    result = smooth_path(points, True, 0.2)
    assert np.array_equal(result[0], result[-1])
    assert len(result) >= 4
    assert np.min(np.linalg.norm(result[:-1], axis=1)) > 9.8


@pytest.mark.parametrize("points", (np.array([[1.0, 2.0]]), np.array([[1.0, 2.0], [1.2, 2.3]])))
def test_short_paths_are_kept(points):
    assert np.array_equal(smooth_path(points, False, 0.2), points)


def test_long_path_is_not_truncated():
    x = np.linspace(0, 500, 2001)
    points = np.column_stack((x, 2 * np.sin(x / 10)))
    result = smooth_path(points, False, 0.2)
    assert np.array_equal(result[[0, -1]], points[[0, -1]])
    assert np.max(result[:, 0]) == 500
    assert len(result) > 50


def test_coarse_arc_obeys_small_deviation_budget():
    theta = np.linspace(0, np.pi, 13)
    points = 5 * np.column_stack((np.cos(theta), np.sin(theta)))
    result = smooth_path(points, False, 0.03)
    assert _distance_to_path(_dense(result), points).max() <= 0.03 + 1e-8
    assert _distance_to_path(_dense(points), result).max() <= 0.03 + 1e-8


def test_existing_intersection_preserves_original_path():
    points = np.array([[0.0, 0.0], [4.0, 4.0], [0.0, 4.0], [4.0, 0.0]])
    assert np.array_equal(smooth_path(points, False, 0.5), points)


def test_smoothing_that_would_cross_a_nearby_fold_is_rejected():
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
    assert np.array_equal(smooth_path(points, False, 0.8), points)


def test_duplicate_points_are_removed_and_unclosed_loop_is_closed():
    points = np.array([[0.0, 0.0], [0.0, 0.0], [5.0, 0.0], [5.0, 5.0], [0.0, 5.0]])
    result = smooth_path(points, True, 0.2)
    assert np.array_equal(result[0], result[-1])
    assert np.all(np.linalg.norm(np.diff(result, axis=0), axis=1) > 0)


def test_zero_tolerance_preserves_original_points():
    points = np.array([[0.0, 0.0], [1.0, 0.1], [2.0, 0.0]])
    assert np.array_equal(smooth_path(points, False, 0), points)
