"""保留结构点、黑线支撑与最终误差界的三次样条去噪。"""

from itertools import pairwise

import numpy as np
from scipy.interpolate import BSpline
from scipy.linalg import LinAlgError, lstsq, null_space, solve

from .geometry import sample_curve


def _covered_segment(first, last, source):
    """相交点／重叠区间必须已经同时存在于原路径上。"""
    starts, delta = source[:-1], np.diff(source, axis=0)
    squared = np.sum(delta * delta, axis=1)
    segment = last - first
    length = np.linalg.norm(segment)
    if length <= 1e-8:
        along = np.zeros(len(starts))
        np.divide(np.sum((first - starts) * delta, axis=1), squared, out=along, where=squared > 0)
        return (
            np.min(np.linalg.norm(first - starts - np.clip(along, 0, 1)[:, None] * delta, axis=1))
            <= 1e-7
        )
    direction = segment / length
    distances = np.abs(np.cross(direction, source - first))
    collinear = (distances[:-1] <= 1e-7) & (distances[1:] <= 1e-7)
    projections = (source - first) @ direction
    intervals = sorted(
        (min(a, b), max(a, b))
        for a, b in zip(projections[:-1][collinear], projections[1:][collinear])
    )
    covered = 0.0
    for low, high in intervals:
        if low > covered + 1e-7:
            break
        covered = max(covered, high)
        if covered >= length - 1e-7:
            return True
    return False


def _new_connections(candidate, other, original, original_other):
    starts, ends = other[:-1], other[1:]
    lower, upper = np.minimum(starts, ends), np.maximum(starts, ends)
    for first, last in pairwise(candidate):
        possible = np.flatnonzero(
            np.all(lower <= np.maximum(first, last) + 1e-9, axis=1)
            & np.all(upper >= np.minimum(first, last) - 1e-9, axis=1)
        )
        delta = last - first
        for index in possible:
            start, end = starts[index], ends[index]
            direction = end - start
            cross = float(np.cross(delta, direction))
            if abs(cross) <= 1e-12:
                if abs(np.cross(delta, start - first)) > 1e-9:
                    continue
                squared = float(delta @ delta)
                if squared <= 1e-16:
                    continue
                along = np.sort(
                    np.array([(start - first) @ delta, (end - first) @ delta]) / squared
                )
                low, high = max(0, along[0]), min(1, along[1])
                if low > high + 1e-10:
                    continue
                contact = (first + low * delta, first + high * delta)
            else:
                t = float(np.cross(start - first, direction) / cross)
                u = float(np.cross(start - first, delta) / cross)
                if not (-1e-9 <= t <= 1 + 1e-9 and -1e-9 <= u <= 1 + 1e-9):
                    continue
                point = first + np.clip(t, 0, 1) * delta
                contact = (point, point)
            if not all(_covered_segment(*contact, path) for path in (original, original_other)):
                return True
    return False


def protect_connections(paths, originals, diagnostics=None, *, fallback_records=None):
    """逐笔接受候选，对尚未处理的笔画使用原路径，保证不引入新连接。"""
    accepted = list(originals)
    bounds = [(points.min(axis=0), points.max(axis=0)) for points, _ in originals]
    for index, (candidate, closed) in enumerate(paths):
        original = originals[index][0]
        if np.array_equal(candidate, original):
            continue
        low, high = candidate.min(axis=0), candidate.max(axis=0)
        rejected = False
        for other_index, (other, _) in enumerate(accepted):
            if index == other_index:
                continue
            other_low, other_high = bounds[other_index]
            if np.any(high < other_low) or np.any(low > other_high):
                continue
            if _new_connections(candidate, other, original, originals[other_index][0]):
                rejected = True
                if diagnostics is not None:
                    # 替换该笔之前的成功记录，不能把被撤销的拟合统计成成功。
                    diagnostics[:] = [row for row in diagnostics if row.get("stroke") != index]
                    if fallback_records is not None:
                        diagnostics.extend(
                            dict(row, refinement_rejected="new_interstroke_connection")
                            for row in fallback_records
                            if row.get("stroke") == index
                        )
                    else:
                        diagnostics.append(
                            {
                                "stroke": index,
                                "method": "preserved",
                                "reason": "new_interstroke_connection",
                                "source_length_mm": float(
                                    np.linalg.norm(np.diff(original, axis=0), axis=1).sum()
                                ),
                                "start_xy_mm": original[0].tolist(),
                                "end_xy_mm": original[-1].tolist(),
                            }
                        )
                break
        if not rejected:
            accepted[index] = (candidate, closed)
            bounds[index] = (low, high)
    return accepted


def protect_refinements(paths, originals, baselines, diagnostics=None):
    """先保护完整基线，再尝试细化；新接触不能让已平滑笔画退回像素折线。"""
    baseline_paths = [(points, original[1]) for (points, _), original in zip(baselines, originals)]
    baseline_records = [
        dict(row, stroke=index) for index, (_, records) in enumerate(baselines) for row in records
    ]
    safe_baselines = protect_connections(baseline_paths, originals, baseline_records)
    return protect_connections(
        paths, safe_baselines, diagnostics, fallback_records=baseline_records
    )


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


def _fit_spline(source, parameters, spacing, smoothing_length, fixed, periodic):
    # 参数单位为 mm；数据项用弧长权重，正则项系数的单位为 mm^4。
    # 这样笔画长度或输入点密度变化不会隐式改变正则化的物理尺度。
    length = parameters[-1]
    count = max(1, int(np.ceil(length / spacing)))
    if count > 800:
        raise ValueError("样条控制点数量超出预算")
    breaks = np.union1d(np.linspace(0.0, length, count + 1), parameters[fixed])
    knots = np.r_[np.repeat(0.0, 3), breaks, np.repeat(length, 3)]
    basis = BSpline(knots, np.eye(len(knots) - 4), 3)
    design = basis(parameters)
    weights = (
        np.r_[np.diff(parameters)[0], parameters[2:] - parameters[:-2], np.diff(parameters)[-1]] / 2
    )
    # 三点 Gauss 积分精确覆盖每个三次多项式的二阶导平方。
    nodes, quadrature = np.polynomial.legendre.leggauss(3)
    half = np.diff(breaks) / 2
    samples = ((breaks[:-1] + half)[:, None] + half[:, None] * nodes).ravel()
    integration = (half[:, None] * quadrature).ravel()
    derivatives = basis(samples, nu=2)
    matrix = design.T @ (weights[:, None] * design)
    matrix += smoothing_length**4 * derivatives.T @ (integration[:, None] * derivatives)
    rhs = design.T @ (weights[:, None] * source)
    constraints = list(design[fixed])
    values = list(source[fixed])
    if periodic:
        for derivative in range(3):
            constraints.append(basis(0.0, nu=derivative) - basis(length, nu=derivative))
            values.append(np.zeros(2))
    constraints = np.asarray(constraints)
    values = np.asarray(values)
    # 在等式约束的零空间内求解，避免 KKT 系统在局部密集约束时病态。
    scales = np.linalg.norm(constraints, axis=1)
    constraints = constraints / scales[:, None]
    values = values / scales[:, None]
    particular = lstsq(constraints, values)[0]
    if np.max(np.abs(constraints @ particular - values)) > 1e-8:
        raise ValueError("样条固定约束不可满足")
    free = null_space(constraints)
    solution = particular
    if free.shape[1]:
        reduced = free.T @ matrix @ free
        solution = particular + free @ solve(
            reduced, free.T @ (rhs - matrix @ particular), assume_a="pos"
        )
    if not np.isfinite(solution).all():
        raise ValueError("样条系数包含非有限数")
    return BSpline(knots, solution, 3)


def _bending_energy(points, closed):
    """在固定毫米尺度重采样最终折线，避免用路点数量衡量平滑程度。"""
    lengths = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    energy = []
    for step in (0.25, 0.5, 1.0):
        count = max(3, int(np.ceil(lengths[-1] / step)))
        samples = np.linspace(0, lengths[-1], count + 1)
        values = np.column_stack([np.interp(samples, lengths, points[:, i]) for i in range(2)])
        delta = np.diff(values, axis=0)
        norms = np.linalg.norm(delta, axis=1)
        directions = np.arctan2(delta[:, 1], delta[:, 0])
        if closed:
            turns = np.roll(directions, -1) - directions
            intervals = (norms + np.roll(norms, -1)) / 2
        else:
            turns = np.diff(directions)
            intervals = (norms[:-1] + norms[1:]) / 2
        turns = np.arctan2(np.sin(turns), np.cos(turns))
        energy.append(float(np.sum(turns**2 / np.maximum(intervals, 1e-12))))
    return np.asarray(energy)


def _sample_fit(curve, source, parameters, fixed, periodic, max_step, chord_error, tolerance):
    candidate, origins = sample_curve(
        curve, max_step, min(chord_error, tolerance / 4), parameters[fixed]
    )
    # 消除求解器舍入，结构点必须精确保留。
    for index in fixed:
        candidate[np.searchsorted(origins, parameters[index])] = source[index]
    if periodic:
        candidate[-1] = candidate[0]
    return candidate, origins


def _refine_fit(
    source,
    parameters,
    candidate,
    deviation,
    tolerance,
    supported,
    *,
    spacing,
    smoothing_length,
    fixed,
    periodic,
    max_step,
    chord_error,
):
    """保留合格基线；固定控制点布局，独立搜索更强正则化。

    首轮使用的 0.6 mm 平滑尺度不再扩大。只修复重试中减弱正则导致的
    过拟合，不放宽几何误差或支撑约束，也不增加额外像素锚点。
    """
    baseline = _bending_energy(candidate, periodic)
    best = candidate, deviation, smoothing_length, baseline
    evaluated = {float(smoothing_length)}
    attempts = 0

    def evaluate(strength):
        nonlocal best, attempts
        strength = float(strength)
        if strength in evaluated:
            return
        evaluated.add(strength)
        attempts += 1
        try:
            curve = _fit_spline(source, parameters, spacing, strength, fixed, periodic)
            points, origins = _sample_fit(
                curve, source, parameters, fixed, periodic, max_step, chord_error, tolerance
            )
        except (LinAlgError, ValueError):
            return
        error = _deviation(source, parameters, points, origins)
        if error > tolerance or not supported(points) or _has_intersection(points, periodic):
            return
        energy = _bending_energy(points, periodic)
        # 多尺度总能量必须下降，并限制单个尺度退化；这不是墨迹摆幅保证。
        if energy.sum() < best[3].sum() * (1 - 1e-6) and np.all(energy <= baseline * 1.05 + 1e-10):
            best = points, error, strength, energy

    if smoothing_length < 0.6:
        grid = np.geomspace(smoothing_length, 0.6, 5)
        for strength in grid[1:]:
            evaluate(strength)
        # 不假定支撑/拓扑判据随正则强度单调；检查整张粗网格后再局部细化。
        index = int(np.argmin(np.abs(grid - best[2])))
        for neighbor in (index - 1, index + 1):
            if 0 <= neighbor < len(grid):
                evaluate(np.sqrt(grid[index] * grid[neighbor]))
    points, error, strength, energy = best
    return points, {
        "max_deviation_mm": error,
        "baseline_max_deviation_mm": deviation,
        "control_spacing_mm": spacing,
        "baseline_smoothing_length_mm": smoothing_length,
        "smoothing_length_mm": strength,
        "refinement_attempts": attempts,
        "refined": strength != smoothing_length,
        "bending_energy_before": baseline.tolist(),
        "bending_energy_after": energy.tolist(),
        "bending_scales_mm": [0.25, 0.5, 1.0],
    }


def _fit_span(source, tolerance, supported, max_step, chord_error, fixed, periodic, baseline=None):
    parameters = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(source, axis=0), axis=1))]
    length = float(parameters[-1])
    if not periodic:
        fixed = sorted({0, len(source) - 1, *fixed})
        # 直线参数保留原输入的弧长对应关系，最终仍检查整段参数折线。
        if len(fixed) == 2:
            points = source[[0, -1]].copy()
            deviation = _deviation(source, parameters, points, parameters[[0, -1]])
            # 更严格地接受直线，避免骨架端点偏移把整个近直线段拉斜。
            if deviation <= min(tolerance / 2, 0.05) and supported(points):
                return points, {"method": "line", "max_deviation_mm": deviation}
    else:
        fixed = sorted(set(fixed) - {len(source) - 1})
    base_fixed = list(fixed)
    reason = "fit_budget"
    # 每轮整段重拟合，新增约束仍处在同一 C2 样条内，不制造窗口接缝。
    for attempt in range(6):
        spacing = max(0.3, 3.0 / 2 ** min(attempt, 4))
        smoothing_length = 0.6 / 2 ** min(attempt, 4)
        try:
            curve = _fit_spline(source, parameters, spacing, smoothing_length, fixed, periodic)
            candidate, origins = _sample_fit(
                curve, source, parameters, fixed, periodic, max_step, chord_error, tolerance
            )
        except (LinAlgError, ValueError):
            reason = "fit_or_sampling_budget"
            continue
        deviation = _deviation(source, parameters, candidate, origins)
        if deviation > tolerance:
            reason = "geometry_deviation"
            continue
        if not supported(candidate):
            reason = "black_support"
            bad = [i for i in range(len(candidate) - 1) if not supported(candidate[i : i + 2])]
            extra = set(fixed)
            for index in bad:
                first = max(0, int(np.searchsorted(parameters, origins[index])) - 1)
                last = min(
                    len(source) - 1, int(np.searchsorted(parameters, origins[index + 1])) + 1
                )
                extra.update(range(first, last + 1))
            if periodic:
                extra.discard(len(source) - 1)
            if len(extra) > 128:
                break
            fixed = sorted(extra)
            continue
        if _has_intersection(candidate, periodic):
            reason = "new_self_intersection"
            continue
        record = {
            "method": "spline",
            "max_deviation_mm": deviation,
            "support_anchor_count": len(fixed) - len(base_fixed),
            "attempts": attempt + 1,
            "control_spacing_mm": spacing,
            "smoothing_length_mm": smoothing_length,
            "refined": False,
        }
        if baseline is not None:
            baseline.append((candidate, dict(record)))
        candidate, refinement = _refine_fit(
            source,
            parameters,
            candidate,
            deviation,
            tolerance,
            supported,
            spacing=spacing,
            smoothing_length=smoothing_length,
            fixed=fixed,
            periodic=periodic,
            max_step=max_step,
            chord_error=chord_error,
        )
        return candidate, {
            **record,
            **refinement,
        }
    return source.copy(), {"method": "preserved", "reason": reason, "length_mm": length}


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


def smooth_path(
    points: np.ndarray,
    closed: bool,
    tolerance_mm: float,
    *,
    supported=None,
    fixed_points=(),
    max_step_mm=1.0,
    chord_error_mm=0.03,
    diagnostics=None,
    baseline=None,
) -> np.ndarray:
    """结构点固定的三次 B 样条拟合，检查最终折线而非只检查样条。

    无尖角闭环采用 C2 周期约束。支撑失败时有限次重拟合仍保持曲线连续；
    不能通过的结构段明确保留并记入 diagnostics。baseline 可接收整笔细化前的
    路径和诊断，供全图连接保护使用。
    """
    points = np.asarray(points, dtype=float)
    if not np.isfinite(tolerance_mm) or tolerance_mm < 0:
        raise ValueError("曲线平滑容差必须是有限非负数")
    supported = supported if supported is not None else lambda _: True
    records = []

    def finish(result, baseline_points=None, baseline_records=None):
        for rows in (records, baseline_records or []):
            for record in rows:
                record.setdefault(
                    "source_length_mm",
                    float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
                    if len(points)
                    else 0.0,
                )
                if len(points):
                    record.setdefault("start_xy_mm", points[0].tolist())
                    record.setdefault("end_xy_mm", points[-1].tolist())
        if diagnostics is not None:
            diagnostics.extend(records)
        if baseline is not None:
            baseline.append(
                (
                    result if baseline_points is None else baseline_points,
                    [
                        dict(row)
                        for row in (records if baseline_records is None else baseline_records)
                    ],
                )
            )
        return result

    if len(points) < 3 or tolerance_mm == 0:
        records.append({"method": "preserved", "reason": "short_or_disabled"})
        return finish(points.copy())
    points = points[np.r_[True, np.any(np.diff(points, axis=0) != 0, axis=1)]]
    if closed and not np.array_equal(points[0], points[-1]):
        points = np.vstack((points, points[0]))
    if len(points) < 3 or _has_intersection(points, closed):
        records.append({"method": "preserved", "reason": "short_or_existing_intersection"})
        return finish(points.copy())
    original = points.copy()
    fixed_coordinates = set(map(tuple, fixed_points))
    corners, _ = _corner_indices(points, tolerance_mm, closed)
    if closed and corners:
        start = corners[0]
        points = np.roll(points[:-1], -start, axis=0)
        points = np.vstack((points, points[0]))
        corners = sorted((index - start) % (len(points) - 1) for index in corners)
    fixed = [i for i, point in enumerate(points) if tuple(point) in fixed_coordinates]
    periodic = closed and not corners
    anchors = [0, len(points) - 1] if periodic else sorted({0, len(points) - 1, *corners})
    spans, baseline_spans, baseline_records = [], [], []
    for first, last in pairwise(anchors):
        source = points[first : last + 1]
        local_fixed = [i - first for i in fixed if first <= i <= last]
        span_baseline = []
        if len(source) <= 2:
            candidate, record = source.copy(), {"method": "preserved", "reason": "short"}
        else:
            candidate, record = _fit_span(
                source,
                tolerance_mm,
                supported,
                max_step_mm,
                chord_error_mm,
                local_fixed,
                periodic,
                baseline=span_baseline,
            )
        baseline_points, baseline_record = (
            span_baseline[0] if span_baseline else (candidate, dict(record))
        )
        metadata = {
            "source_start": first,
            "source_end": last,
            "source_length_mm": float(np.linalg.norm(np.diff(source, axis=0), axis=1).sum()),
            "start_xy_mm": source[0].tolist(),
            "end_xy_mm": source[-1].tolist(),
        }
        record.update(metadata)
        baseline_record.update(metadata)
        records.append(record)
        spans.append(candidate)
        baseline_records.append(baseline_record)
        baseline_spans.append(baseline_points)
    result = np.vstack([spans[0], *[span[1:] for span in spans[1:]]])
    baseline_result = np.vstack([baseline_spans[0], *[span[1:] for span in baseline_spans[1:]]])

    def valid(path):
        return (
            (not closed or len(path) >= 4)
            and not _has_intersection(path, closed)
            and supported(path)
        )

    if not valid(baseline_result):
        baseline_result = original
        baseline_records = [
            {
                "method": "preserved",
                "reason": "final_structure_or_support",
                "source_length_mm": float(np.linalg.norm(np.diff(original, axis=0), axis=1).sum()),
            }
        ]
    if not valid(result):
        result = baseline_result
        records = [
            dict(row, refinement_rejected="final_structure_or_support") for row in baseline_records
        ]
    return finish(result, baseline_result, baseline_records)
