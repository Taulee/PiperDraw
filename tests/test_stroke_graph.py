from collections import Counter
from itertools import pairwise

import numpy as np

from piper_draw.stroke_graph import trace, trace_strokes


def edges(strokes):
    return Counter(
        tuple(sorted((tuple(first), tuple(second))))
        for points, _, _ in strokes
        for first, second in pairwise(points)
    )


def test_trace_keeps_original_t_junction_contract():
    mask = np.zeros((9, 9), np.uint8)
    mask[2, 1:8] = 1
    mask[2:8, 4] = 1
    strokes = trace(mask)
    assert len(strokes) == 3
    assert sum(edges(strokes).values()) == len(edges(strokes)) == 11


def test_t_junction_continues_straight_and_covers_each_edge_once():
    mask = np.zeros((9, 9), np.uint8)
    mask[2, 1:8] = 1
    mask[2:8, 4] = 1
    strokes = trace_strokes(mask, 0.2, 0)
    assert len(strokes) == 2
    assert edges(strokes) == edges(trace(mask))
    assert any(np.all(points[:, 0] == 2) for points, _, _ in strokes)


def test_diagonal_x_junction_becomes_two_continuous_strokes():
    mask = np.zeros((13, 13), np.uint8)
    indices = np.arange(1, 12)
    mask[indices, indices] = 1
    mask[indices, 12 - indices] = 1
    strokes = trace_strokes(mask, 0.2, 0)
    assert len(strokes) == 2
    assert edges(strokes) == edges(trace(mask))
    for points, closed, _ in strokes:
        assert not closed
        assert np.array_equal(points[0] + points[-1], [12, 12])


def test_y_junction_without_straight_continuation_stays_separate():
    mask = np.zeros((13, 13), np.uint8)
    for distance in range(6):
        mask[6 - distance, 6 - distance] = 1
        mask[6 + distance, 6 - distance] = 1
        mask[6, 6 + distance] = 1
    strokes = trace_strokes(mask, 0.2, 0)
    assert len(strokes) == 3
    assert edges(strokes) == edges(trace(mask))


def test_closed_loop_remains_closed_without_junctions():
    mask = np.zeros((9, 9), np.uint8)
    mask[1, 1:8] = mask[7, 1:8] = 1
    mask[1:8, 1] = mask[1:8, 7] = 1
    strokes = trace_strokes(mask, 0.2, 0)
    assert len(strokes) == 1 and strokes[0][1]
    assert np.array_equal(strokes[0][0][0], strokes[0][0][-1])
    assert edges(strokes) == edges(trace(mask))


def test_paired_cycles_close_and_do_not_repeat_external_edges():
    mask = np.zeros((13, 13), np.uint8)
    mask[2, 2:7] = mask[6, 2:7] = 1
    mask[2:7, 2] = mask[2:7, 6] = 1
    mask[6, 6:11] = mask[10, 6:11] = 1
    mask[6:11, 6] = mask[6:11, 10] = 1
    strokes = trace_strokes(mask, 0.2, 0)
    assert len(strokes) == 1 and strokes[0][1]
    assert edges(strokes) == edges(trace(mask))


def test_separate_nearby_lines_do_not_gain_connections():
    mask = np.zeros((9, 13), np.uint8)
    mask[3, 1:6] = mask[3, 7:12] = 1
    mask[5, 1:12] = 1
    strokes = trace_strokes(mask, 0.2, 0)
    assert len(strokes) == 3
    assert edges(strokes) == edges(trace(mask))


def test_compact_junction_internal_edges_do_not_become_tiny_strokes():
    mask = np.zeros((14, 14), np.uint8)
    mask[6:8, 6:8] = 1
    mask[6, 1:6] = mask[7, 8:13] = 1
    mask[1:6, 7] = mask[8:13, 6] = 1
    strokes = trace_strokes(mask, 0.2, 0)
    assert len(strokes) == 2
    assert all(not closed and len(points) > 10 for points, closed, _ in strokes)
    junction = {(6, 6), (6, 7), (7, 6), (7, 7)}

    def external(edge):
        return not all(point in junction for point in edge)

    original_edges = {edge: count for edge, count in edges(trace(mask)).items() if external(edge)}
    cleaned_edges = {edge: count for edge, count in edges(strokes).items() if external(edge)}
    assert cleaned_edges == original_edges
    assert set(cleaned_edges.values()) == {1}


def test_short_bridge_between_distinct_junctions_is_preserved():
    mask = np.zeros((13, 13), np.uint8)
    mask[1:12, 4] = mask[1:12, 7] = 1
    mask[6, 4:8] = 1
    strokes = trace_strokes(mask, 0.2, 1.0)
    assert len(strokes) == 3
    (bridge,) = [stroke for stroke in strokes if stroke[2]]
    assert np.array_equal(bridge[0], [[6, 4], [6, 5], [6, 6], [6, 7]])
    assert not bridge[1]
    assert edges(strokes) == edges(trace(mask))


def test_short_isolated_line_is_filtered_but_not_recursively_pruned():
    mask = np.zeros((12, 12), np.uint8)
    mask[2, 1:3] = 1
    mask[6, 1:11] = 1
    mask[6:9, 5] = 1
    strokes = trace_strokes(mask, 0.2, 1.0)
    assert len(strokes) == 1
    assert np.all(strokes[0][0][:, 0] == 6)


def test_join_angle_controls_continuation_without_crossing_white_background():
    mask = np.zeros((31, 31), np.uint8)
    mask[2:16, 15] = 1
    indices = np.arange(1, 14)
    mask[15 + indices, 15 + indices] = 1
    mask[15, 15:29] = 1
    strict = trace_strokes(mask, 0.2, 0, join_angle_deg=0)
    relaxed = trace_strokes(mask, 0.2, 0, join_angle_deg=35)
    assert len(strict) == 3 and len(relaxed) == 2
    for points, _, _ in relaxed:
        assert np.all(mask[points[:, 0], points[:, 1]])
    junction = {(15, 15), (15, 16)}
    original = {edge for edge in edges(trace(mask)) if not set(edge) <= junction}
    cleaned = {edge for edge in edges(relaxed) if not set(edge) <= junction}
    assert original == cleaned
