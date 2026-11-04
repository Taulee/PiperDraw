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
    if (origin.shape != (2,) or not np.isfinite(origin).all()
            or not np.isfinite(paper_size).all() or not np.isfinite(size).all()
            or not np.isfinite(margin) or margin < 0):
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
