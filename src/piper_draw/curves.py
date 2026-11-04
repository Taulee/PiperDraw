"""保留尖角、端点与拓扑的毫米折线平滑。"""

from itertools import pairwise

import numpy as np


def _has_intersection(points, closed):
    starts, ends = points[:-1], points[1:]
    lower, upper = np.minimum(starts, ends), np.maximum(starts, ends)
    for i, (start, end) in enumerate(zip(starts, ends)):
        candidates = (
            np.flatnonzero(
                np.all(lower[i] <= upper[i + 2 :], axis=1)
                & np.all(lower[i + 2 :] <= upper[i], axis=1)
            )
            + i
            + 2
        )
        if closed and i == 0:
            candidates = candidates[candidates != len(starts) - 1]
        if not len(candidates):
            continue
        other_starts, other_ends = starts[candidates], ends[candidates]
        delta = end - start
        other_delta = other_ends - other_starts
        side1 = np.cross(delta, other_starts - start)
        side2 = np.cross(delta, other_ends - start)
        side3 = np.cross(other_delta, start - other_starts)
        side4 = np.cross(other_delta, end - other_starts)
        if np.any((side1 * side2 <= 0) & (side3 * side4 <= 0)):
            return True
    return False


def _deviation(source, source_parameters, points, parameters):
    # 两条参数折线之差在相邻断点间为线性；范数最大值必在断点取得。
    knots = np.union1d(source_parameters, parameters)
    source_values = np.column_stack(
        [np.interp(knots, source_parameters, source[:, axis]) for axis in range(2)]
    )
    values = np.column_stack([np.interp(knots, parameters, points[:, axis]) for axis in range(2)])
    return float(np.linalg.norm(source_values - values, axis=1).max())


def _simplify_indices(points, tolerance):
    keep = {0, len(points) - 1}
    pending = [(0, len(points) - 1)]
    while pending:
        first, last = pending.pop()
        if last <= first + 1:
            continue
        start, end = points[first], points[last]
        delta = end - start
        length_squared = float(delta @ delta)
        inside = points[first + 1 : last]
        if length_squared:
            along = np.clip((inside - start) @ delta / length_squared, 0, 1)
            distances = np.linalg.norm(inside - start - along[:, None] * delta, axis=1)
        else:
            distances = np.linalg.norm(inside - start, axis=1)
        index = int(np.argmax(distances))
        if distances[index] > tolerance:
            middle = first + 1 + index
            keep.add(middle)
            pending.extend(((first, middle), (middle, last)))
    return np.asarray(sorted(keep))


def _smooth_span(source, tolerance):
    parameters = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(source, axis=0), axis=1))]
    source_parameters = parameters.copy()
    points = source.copy()
    for _ in range(3):
        candidate = np.empty((2 * len(points), 2))
        candidate[0], candidate[-1] = points[0], points[-1]
        candidate[1:-1:2] = 0.75 * points[:-1] + 0.25 * points[1:]
        candidate[2:-1:2] = 0.25 * points[:-1] + 0.75 * points[1:]
        candidate_parameters = np.empty(2 * len(points))
        candidate_parameters[0], candidate_parameters[-1] = parameters[0], parameters[-1]
        candidate_parameters[1:-1:2] = 0.75 * parameters[:-1] + 0.25 * parameters[1:]
        candidate_parameters[2:-1:2] = 0.25 * parameters[:-1] + 0.75 * parameters[1:]
        if _deviation(source, source_parameters, candidate, candidate_parameters) > tolerance:
            break
        points, parameters = candidate, candidate_parameters

    # 稀疏化也验证完整参数折线，避免只验证顶点而漏掉原线段的凸起。
    for fraction in (0.25, 0.125, 0.0625):
        indices = _simplify_indices(points, tolerance * fraction)
        if _deviation(source, source_parameters, points[indices], parameters[indices]) <= tolerance:
            return points[indices]
    return points


def _corner_indices(points, tolerance, closed):
    parameters = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    length = parameters[-1]
    step = np.median(np.diff(parameters))
    radius = max(3 * tolerance, 4 * step)
    positions = parameters[:-1] if closed else parameters
    before = positions - radius
    after = positions + radius
    arguments = {"period": length} if closed else {}
    incoming = points[: len(positions)] - np.column_stack(
        [np.interp(before, parameters, points[:, axis], **arguments) for axis in range(2)]
    )
    outgoing = (
        np.column_stack(
            [np.interp(after, parameters, points[:, axis], **arguments) for axis in range(2)]
        )
        - points[: len(positions)]
    )
    norms = np.linalg.norm(incoming, axis=1) * np.linalg.norm(outgoing, axis=1)
    cosine = np.ones(len(positions))
    np.divide(np.sum(incoming * outgoing, axis=1), norms, out=cosine, where=norms > 0)
    angles = np.arccos(np.clip(cosine, -1, 1))
    candidates = np.flatnonzero(angles >= np.deg2rad(55))
    corners = []
    for index in candidates[np.argsort(-angles[candidates], kind="stable")]:
        distances = np.abs(positions[corners] - positions[index])
        if closed:
            distances = np.minimum(distances, length - distances)
        if np.all(distances >= radius):
            corners.append(int(index))
    return sorted(corners), angles


def smooth_path(points: np.ndarray, closed: bool, tolerance_mm: float) -> np.ndarray:
    """平滑原始毫米点列，连续偏差不超过容差，已有自交路径保留原样。

    在跨像素窗口识别尖角并固定，用三轮 Chaikin 角切整理其间的曲线。
    误差界针对最终输出折线而非假定的样条；新增交叉时返回原路径。
    """
    points = np.asarray(points, dtype=float)
    if tolerance_mm < 0:
        raise ValueError("曲线平滑容差不能为负数")
    if len(points) < 3 or tolerance_mm == 0:
        return points.copy()
    points = points[np.r_[True, np.any(np.diff(points, axis=0) != 0, axis=1)]]
    if closed and not np.array_equal(points[0], points[-1]):
        points = np.vstack((points, points[0]))
    if len(points) < 3 or _has_intersection(points, closed):
        return points.copy()
    original = points.copy()
    corners, angles = _corner_indices(points, tolerance_mm, closed)
    if closed:
        start = corners[0] if corners else int(np.argmin(angles))
        points = np.roll(points[:-1], -start, axis=0)
        points = np.vstack((points, points[0]))
        corners = sorted((index - start) % (len(points) - 1) for index in corners)
    anchors = sorted({0, len(points) - 1, *corners})
    spans = [_smooth_span(points[a : b + 1], tolerance_mm) for a, b in pairwise(anchors)]
    result = np.vstack([spans[0], *[span[1:] for span in spans[1:]]])
    if closed and len(result) < 4:
        return original
    if _has_intersection(result, closed):
        return original
    return result
