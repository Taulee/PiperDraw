from collections import Counter
from itertools import pairwise

import numpy as np
import pytest

from piper_draw.vectorize import Stroke, order_strokes


def edge_counts(strokes):
    return Counter(
        tuple(sorted((tuple(first), tuple(second))))
        for stroke in strokes
        for first, second in pairwise(stroke.points)
    )


def test_shared_endpoint_merges_after_reversing_next_stroke():
    strokes = [
        Stroke(np.array([[0.0, 0.0], [1.0, 0.0]])),
        Stroke(np.array([[2.0, 1.0], [1.0, 0.0]])),
    ]
    ordered = order_strokes(strokes)

    assert len(ordered) == 1
    assert np.array_equal(ordered[0].points, [[0.0, 0.0], [1.0, 0.0], [2.0, 1.0]])
    assert not ordered[0].closed
    assert edge_counts(ordered) == edge_counts(strokes)


@pytest.mark.parametrize("start_x", [2.0, np.nextafter(1.0, 2.0)])
def test_distinct_endpoints_keep_lift_even_when_extremely_close(start_x):
    strokes = [
        Stroke(np.array([[0.0, 0.0], [1.0, 0.0]])),
        Stroke(np.array([[start_x, 0.0], [3.0, 0.0]])),
    ]
    ordered = order_strokes(strokes)

    assert len(ordered) == 2
    assert edge_counts(ordered) == edge_counts(strokes)


def test_rotated_closed_loop_continues_from_exact_endpoint():
    strokes = [
        Stroke(np.array([[0.0, 0.0], [1.0, 0.0]])),
        Stroke(np.array([[2.0, 0.0], [2.0, 1.0], [1.0, 0.0], [2.0, 0.0]]), True),
    ]
    ordered = order_strokes(strokes)

    assert len(ordered) == 1
    assert not ordered[0].closed
    assert np.array_equal(ordered[0].points[0], [0.0, 0.0])
    assert np.array_equal(ordered[0].points[-1], [1.0, 0.0])
    assert edge_counts(ordered) == edge_counts(strokes)


def test_closed_loop_keeps_lift_when_nearest_point_is_not_equal():
    strokes = [
        Stroke(np.array([[0.0, 0.0], [1.0, 0.0]])),
        Stroke(np.array([[2.0, 0.0], [2.0, 1.0], [1.1, 0.0], [2.0, 0.0]]), True),
    ]
    ordered = order_strokes(strokes)

    assert len(ordered) == 2
    assert ordered[1].closed
    assert edge_counts(ordered) == edge_counts(strokes)


def test_open_strokes_form_one_closed_path_without_repeating_edges():
    strokes = [
        Stroke(np.array([[0.0, 0.0], [1.0, 0.0]])),
        Stroke(np.array([[1.0, 0.0], [1.0, 1.0]])),
        Stroke(np.array([[1.0, 1.0], [0.0, 0.0]])),
    ]
    ordered = order_strokes(strokes)

    assert len(ordered) == 1
    assert ordered[0].closed
    assert len(ordered[0].points) == 4
    assert edge_counts(ordered) == edge_counts(strokes)
    assert set(edge_counts(ordered).values()) == {1}


def test_closed_path_followed_by_open_stroke_is_open():
    strokes = [
        Stroke(np.array([[0.0, 0.0], [1.0, 1.0], [0.0, 1.0], [0.0, 0.0]]), True),
        Stroke(np.array([[0.0, 0.0], [2.0, 0.0]])),
    ]
    ordered = order_strokes(strokes)

    assert len(ordered) == 1
    assert not ordered[0].closed
    assert np.array_equal(ordered[0].points[-1], [2.0, 0.0])
    assert edge_counts(ordered) == edge_counts(strokes)


def test_sorting_reversing_rotating_and_merging_do_not_modify_input():
    strokes = [
        Stroke(np.array([[0.0, 0.0], [1.0, 0.0]])),
        Stroke(np.array([[2.0, 0.0], [2.0, 1.0], [1.0, 0.0], [2.0, 0.0]]), True),
        Stroke(np.array([[3.0, 0.0], [1.0, 0.0]])),
    ]
    originals = [(stroke, stroke.points.copy(), stroke.closed) for stroke in strokes]
    ordered = order_strokes(strokes)

    assert len(ordered) == 1
    for stroke, (original, points, closed) in zip(strokes, originals):
        assert stroke is original
        assert np.array_equal(stroke.points, points)
        assert stroke.closed == closed
        assert not np.shares_memory(ordered[0].points, stroke.points)
    assert edge_counts(ordered) == edge_counts(strokes)


def travel_length(strokes):
    current = np.zeros(2)
    length = 0.0
    for stroke in strokes:
        length += np.linalg.norm(stroke.points[0] - current)
        current = stroke.points[-1]
    return length


def test_local_sorting_preserves_edges_closure_and_never_increases_travel():
    random = np.random.default_rng(605)
    strokes = [Stroke(random.uniform(0, 100, (2, 2))) for _ in range(30)]
    strokes.append(Stroke(np.array([[50, 50], [55, 50], [50, 55], [50, 50]]), True))
    originals = [stroke.points.copy() for stroke in strokes]
    nearest = order_strokes(strokes)
    optimized = order_strokes(strokes, passes=2)
    repeated = order_strokes(strokes, passes=2)

    assert travel_length(optimized) < travel_length(nearest)
    assert edge_counts(optimized) == edge_counts(strokes)
    assert sum(stroke.closed for stroke in optimized) == 1
    for first, second in zip(optimized, repeated):
        assert np.array_equal(first.points, second.points)
    for original, stroke in zip(originals, strokes):
        assert np.array_equal(original, stroke.points)
