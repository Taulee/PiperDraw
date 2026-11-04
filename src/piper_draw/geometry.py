from dataclasses import dataclass
from itertools import pairwise

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class PaperFrame:
    origin: np.ndarray
    rotation: np.ndarray

    @classmethod
    def from_points(cls, origin_mm, x_point_mm, y_point_mm):
        points = np.asarray([origin_mm, x_point_mm, y_point_mm], dtype=float)
        if points.shape != (3, 3) or not np.isfinite(points).all():
            raise ValueError("纸面标定点必须是三个有限的三维坐标")
        origin = points[0]
        x, y = points[1:] - origin
        if np.linalg.norm(x) < 1 or np.linalg.norm(np.cross(x, y)) < 1:
            raise ValueError("纸面标定点重合或共线")
        x /= np.linalg.norm(x)
        z = np.cross(x, y)
        z /= np.linalg.norm(z)
        return cls(origin / 1000, np.column_stack((x, np.cross(z, x), z)))

    def point(self, xy_mm, height_mm=0):
        return self.origin + self.rotation @ (np.array([*xy_mm, height_mm]) / 1000)

    @property
    def quaternion(self):
        return Rotation.from_matrix(self.rotation).as_quat().tolist()


def drawing_bounds(paper):
    """返回纸面内选区的二维原点、宽高和选区内留白，单位 mm。"""
    paper_size = np.array([paper["width_mm"], paper["height_mm"]], dtype=float)
    area = paper.get("drawing_area")
    origin = np.zeros(2) if area is None else np.asarray(area["origin_mm"], dtype=float)
    size = paper_size if area is None else np.array([area["width_mm"], area["height_mm"]])
    margin = paper["margin_mm"]
    if (
        origin.shape != (2,)
        or not np.isfinite(origin).all()
        or not np.isfinite(paper_size).all()
        or not np.isfinite(size).all()
        or not np.isfinite(margin)
        or margin < 0
    ):
        raise ValueError("纸面、选区尺寸和边距必须是有限数，边距不能为负数")
    if np.any(size <= 2 * margin):
        raise ValueError("纸面选区宽高必须大于两倍边距")
    if np.any(origin < 0) or np.any(origin + size > paper_size + 1e-8):
        raise ValueError("绘画选区超出纸面范围")
    return origin, size, margin


def flange_pose(tip_position, quaternion, tcp_offset_mm):
    rotation = Rotation.from_quat(quaternion)
    position = tip_position - rotation.apply(np.asarray(tcp_offset_mm) / 1000)
    return [*position.tolist(), *rotation.as_quat().tolist()]


def resample(points, max_step):
    result = [points[0]]
    for a, b in pairwise(points):
        delta = b - a
        count = max(1, int(np.ceil(np.linalg.norm(delta) / max_step)))
        result.extend(a + delta * t / count for t in range(1, count + 1))
    return np.asarray(result)


def _bezier_controls(curve, first, last):
    """三次样条在单个多项式区间内的精确 Bézier 控制点。"""
    ends = curve(np.array([first, last]))
    derivatives = curve(np.array([first, last]), nu=1) * ((last - first) / 3)
    return np.array([ends[0], ends[0] + derivatives[0], ends[1] - derivatives[1], ends[1]])


def _chord_bound(controls, first, last):
    delta = last - first
    squared = float(delta @ delta)
    along = np.clip((controls - first) @ delta / squared, 0, 1) if squared else 0
    return float(
        np.linalg.norm(controls - first - np.asarray(along)[..., None] * delta, axis=-1).max()
    )


def sample_curve(curve, max_step, chord_error, fixed_parameters=()):
    """按数值弧长取点，再用各多项式段控制点包络约束整段弦误差。

    返回点与其来源参数。弧长表是数值近似；弦误差界来自 Bézier 凸包，
    不依赖只检查中点，也不把均匀参数采样误认为等弧长。
    """
    if not np.isfinite([max_step, chord_error]).all() or min(max_step, chord_error) <= 0:
        raise ValueError("曲线点距和弦误差必须是有限正数")
    knots = np.unique(curve.t[curve.k : -curve.k])
    table = [float(knots[0])]

    def flatten(first, last, depth=0):
        controls = _bezier_controls(curve, first, last)
        polygon_length = np.linalg.norm(np.diff(controls, axis=0), axis=1).sum()
        if (
            polygon_length <= min(max_step / 4, 0.1)
            and _chord_bound(controls, controls[0], controls[-1]) <= chord_error / 8
        ):
            table.append(float(last))
            return
        if depth >= 20:
            raise ValueError("曲线弧长表超出细分预算")
        middle = (first + last) / 2
        flatten(first, middle, depth + 1)
        flatten(middle, last, depth + 1)

    for first, last in pairwise(knots):
        flatten(first, last)
    table = np.array(table)
    lengths = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(curve(table), axis=0), axis=1))]
    if lengths[-1] <= 1e-12:
        raise ValueError("拟合曲线长度为零")
    targets = np.linspace(0, lengths[-1], max(1, int(np.ceil(lengths[-1] / (0.99 * max_step)))) + 1)
    parameters = np.union1d(np.interp(targets, lengths, table), fixed_parameters)
    result = [parameters[0]]

    def refine(first, last, depth=0):
        start, end = curve(np.array([first, last]))
        pieces = np.r_[first, knots[(knots > first) & (knots < last)], last]
        bound = max(
            _chord_bound(_bezier_controls(curve, a, b), start, end) for a, b in pairwise(pieces)
        )
        if np.linalg.norm(end - start) <= max_step * (1 + 1e-12) and bound <= chord_error:
            result.append(last)
            return
        if depth >= 20:
            raise ValueError("曲线弦误差超出细分预算")
        distance = (np.interp(first, table, lengths) + np.interp(last, table, lengths)) / 2
        middle = float(np.interp(distance, lengths, table))
        refine(first, middle, depth + 1)
        refine(middle, last, depth + 1)

    for first, last in pairwise(parameters):
        refine(first, last)
    result = np.asarray(result)
    return curve(result), result
