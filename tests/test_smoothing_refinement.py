"""Geometry regressions: remove repeated turns without introducing broad waves."""

import numpy as np

from piper_draw.curves import protect_refinements, smooth_path


def _quantized_ellipse(a, b, grid):
    theta = np.linspace(0, 2 * np.pi, 801)
    points = np.round(np.column_stack((a * np.cos(theta), b * np.sin(theta))) / grid) * grid
    points = points[np.r_[True, np.any(np.diff(points, axis=0) != 0, axis=1)]]
    points[-1] = points[0]
    return points


def _at_arc_spacing(points, spacing):
    lengths = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    samples = np.linspace(0, lengths[-1], max(2, round(lengths[-1] / spacing)) + 1)
    return np.column_stack([np.interp(samples, lengths, points[:, axis]) for axis in range(2)])


def _ellipse_normal_error(points, a, b):
    theta = np.arctan2(points[:, 1] / b, points[:, 0] / a)
    ideal = np.column_stack((a * np.cos(theta), b * np.sin(theta)))
    normal = np.column_stack((np.cos(theta) / a, np.sin(theta) / b))
    normal /= np.linalg.norm(normal, axis=1)[:, None]
    return np.sum((points - ideal) * normal, axis=1)


def _broad_wave(error):
    # Samples are 0.1 mm apart: average over approximately 5 mm of a closed path.
    return sum(np.roll(error, shift) for shift in range(-25, 26)) / 51


def test_noisy_ellipse_loses_reverse_turns_without_growing_broad_error():
    source = _quantized_ellipse(30, 5, 0.2)
    result = smooth_path(source, True, 0.15)
    assert np.array_equal(result[0], result[-1])
    samples = _at_arc_spacing(result, 0.25)
    deltas = np.diff(samples, axis=0)
    directions = np.arctan2(deltas[:, 1], deltas[:, 0])
    turns = np.angle(np.exp(1j * (np.roll(directions, -1) - directions)))
    # The ideal ellipse is convex. The former retry strategy accumulated 8.45 rad
    # of reverse turning on this input despite meeting its positional tolerance.
    assert -turns[turns < 0].sum() < 5
    error = _ellipse_normal_error(_at_arc_spacing(result, 0.1)[:-1], 30, 5)
    assert np.max(np.abs(error)) < 0.11
    assert np.max(np.abs(_broad_wave(error))) < 0.05


def test_quantized_circle_does_not_exchange_pixel_noise_for_a_broad_wave():
    source = _quantized_ellipse(15, 15, 0.125)
    result = smooth_path(source, True, 0.15)
    samples = _at_arc_spacing(result, 0.1)[:-1]
    radial_error = np.linalg.norm(samples, axis=1) - 15
    assert np.ptp(radial_error) < 0.04
    assert np.max(np.abs(_broad_wave(radial_error))) < 0.02


def test_smooth_open_contour_keeps_its_endpoints_and_internal_junction():
    x = np.linspace(0, 50, 501)
    source = np.round(np.column_stack((x, 3 * np.sin(x / 9))) / 0.125) * 0.125
    source = source[np.r_[True, np.any(np.diff(source, axis=0) != 0, axis=1)]]
    anchor = source[np.argmin(np.abs(source[:, 0] - 23))]
    result = smooth_path(source, False, 0.15, fixed_points=[anchor])
    assert np.array_equal(result[[0, -1]], source[[0, -1]])
    assert np.any(np.all(result == anchor, axis=1))
    samples = _at_arc_spacing(result, 0.1)
    assert np.all(np.diff(samples[:, 0]) > 0)
    assert np.max(np.abs(samples[:, 1] - 3 * np.sin(samples[:, 0] / 9))) < 0.08


def test_refinement_connection_failure_keeps_valid_spline_baseline():
    original = np.array([[0.0, 0.0], [0.5, 0.18], [1.0, 0.3], [1.5, 0.18], [2.0, 0.0]])
    baseline = np.array([[0.0, 0.0], [1.0, 0.25], [2.0, 0.0]])
    refined = np.array([[0.0, 0.0], [1.0, 0.18], [2.0, 0.0]])
    neighbor = np.array([[0.8, 0.16], [1.2, 0.16]])
    records = [
        {"stroke": 0, "method": "spline", "refined": True, "max_deviation_mm": 0.12},
        {"stroke": 1, "method": "line", "max_deviation_mm": 0.0},
    ]
    result = protect_refinements(
        [(refined, False), (neighbor, False)],
        [(original, False), (neighbor, False)],
        [
            (baseline, [{"method": "spline", "refined": False, "max_deviation_mm": 0.05}]),
            (neighbor, [{"method": "line", "max_deviation_mm": 0.0}]),
        ],
        records,
    )
    # Both the raw and baseline paths miss the neighbor; refinement crosses it.
    # Keep the already accepted smooth curve instead of restoring pixel corners.
    assert np.array_equal(result[0][0], baseline)
    assert np.array_equal(result[1][0], neighbor)
    first = next(record for record in records if record["stroke"] == 0)
    assert first["method"] == "spline"
    assert first["max_deviation_mm"] == 0.05
    assert first["refined"] is False
    assert first["refinement_rejected"] == "new_interstroke_connection"
