import warnings
from dataclasses import dataclass
from itertools import pairwise

import cv2
import numpy as np

from .curves import protect_connections, smooth_path
from .geometry import drawing_bounds, resample
from .marks import extract_marks, mark_support_mask
from .stroke_graph import trace, trace_strokes

__all__ = ["Stroke", "order_strokes", "trace", "vectorize"]


class NoDrawableLinesError(ValueError):
    """线稿或过滤后的候选没有可绘制笔画。"""


@dataclass
class Stroke:
    points: np.ndarray
    closed: bool = False


def _orient_stroke(stroke, current):
    if stroke.closed:
        points = stroke.points[:-1]
        index = int(np.argmin(np.linalg.norm(points - current, axis=1)))
        points = np.roll(points, -index, axis=0)
        return Stroke(np.vstack((points, points[0])), True)
    if np.linalg.norm(stroke.points[-1] - current) < np.linalg.norm(stroke.points[0] - current):
        return Stroke(stroke.points[::-1])
    return stroke


def _travel_length(strokes, current):
    length = 0.0
    for stroke in strokes:
        length += np.linalg.norm(stroke.points[0] - current)
        current = stroke.points[-1]
    return float(length)


def _improve_order(strokes, passes):
    """交换最多六笔的局部窗口，重选方向；只接受严格缩短空移的结果。"""
    strokes = list(strokes)
    budget = 6000
    for _ in range(passes):
        changed = False
        for first in range(len(strokes) - 1):
            before = np.zeros(2) if first == 0 else strokes[first - 1].points[-1]
            for last in range(first + 1, min(len(strokes), first + 6)):
                if budget == 0:
                    return strokes
                budget -= 1
                old = strokes[first : last + 1]
                after = strokes[last + 1].points[0] if last + 1 < len(strokes) else None
                old_length = _travel_length(old, before)
                candidate = []
                current = before
                for stroke in [old[-1], *old[1:-1], old[0]]:
                    oriented = _orient_stroke(stroke, current)
                    candidate.append(oriented)
                    current = oriented.points[-1]
                length = _travel_length(candidate, before)
                if after is not None:
                    old_length += np.linalg.norm(after - old[-1].points[-1])
                    length += np.linalg.norm(after - current)
                if length + 1e-9 < old_length:
                    strokes[first : last + 1] = candidate
                    changed = True
        if not changed:
            break
    return strokes


def order_strokes(strokes, passes=0):
    remaining = list(strokes)
    ordered = []
    current = np.zeros(2)
    while remaining:
        choices = []
        for i, stroke in enumerate(remaining):
            candidates = stroke.points[:-1] if stroke.closed else stroke.points[[0, -1]]
            distances = np.linalg.norm(candidates - current, axis=1)
            k = int(np.argmin(distances))
            choices.append((distances[k], i, k))
        _, i, k = min(choices)
        stroke = remaining.pop(i)
        if stroke.closed:
            points = np.roll(stroke.points[:-1], -k, axis=0)
            stroke = Stroke(np.vstack((points, points[0])), True)
        elif k == 1:
            stroke = Stroke(stroke.points[::-1])
        if ordered and np.array_equal(ordered[-1].points[-1], stroke.points[0]):
            # 已到达下一笔起点；共享点只保留一次，不增加连线或回描。
            points = np.vstack((ordered[-1].points, stroke.points[1:]))
            ordered[-1] = Stroke(points, np.array_equal(points[0], points[-1]))
        else:
            ordered.append(stroke)
        current = stroke.points[-1]
    return _improve_order(ordered, passes) if passes else ordered


def _supported_path(points, mask, factor, offset):
    """候选曲线须留在原黑线区域，避免跨过断口或黑块中的高光。"""
    pixels = (points - offset) / factor
    pixels[:, 1] = mask.shape[0] - 1 - pixels[:, 1]
    indices = np.rint(pixels).astype(int)
    x, y = indices.T
    height, width = mask.shape
    if np.any((x < 0) | (x >= width) | (y < 0) | (y >= height)):
        return False
    if not np.all(mask[y, x]):
        return False
    # 在所有半整数栅格边界处分段；每段中点代表整段所在像素，
    # 避免固定间距采样漏掉擦过白孔角落的极短区间。
    for first, last in pairwise(pixels):
        if np.array_equal(np.rint(first), np.rint(last)):
            continue
        delta = last - first
        crossings = [np.array([0.0, 1.0])]
        for axis in range(2):
            if delta[axis] != 0:
                low, high = sorted((first[axis], last[axis]))
                boundaries = np.arange(np.floor(low - 0.5) + 1, np.ceil(high - 0.5)) + 0.5
                crossings.append((boundaries - first[axis]) / delta[axis])
        parameters = np.sort(np.concatenate(crossings))
        # 毫米坐标往返会使同一个像素角的 x/y 交点差几个浮点 ulp；
        # 合并数值重合的交点，避免把零长白角误判成实际跨白区间。
        parameters = parameters[np.r_[True, np.diff(parameters) > 1e-12]]
        middle = (parameters[:-1] + parameters[1:]) / 2
        indices = np.rint(first + middle[:, None] * delta).astype(int)
        if not np.all(mask[indices[:, 1], indices[:, 0]]):
            return False
    return True


def read_mask(path, max_pixels):
    """按同一透明底色和 Otsu 规则读取线稿；供取路径与独立评估共用。"""
    image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError(f"无法读取图像：{path}")
    if image.dtype != np.uint8:
        raise ValueError("线稿必须是 8 位图像")
    if image.ndim == 3:
        if image.shape[2] == 4:
            alpha = image[:, :, 3:4].astype(float) / 255
            image = (image[:, :, :3] * alpha + 255 * (1 - alpha)).astype(np.uint8)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    scale = min(1, max_pixels / max(image.shape))
    if np.min(np.rint(np.asarray(image.shape) * scale)) < 2:
        raise ValueError("缩放后的线稿宽高必须至少为 2 像素")
    image = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    return cv2.threshold(image, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]


def image_frame(shape, paper):
    height, width = shape
    origin, size, margin = drawing_bounds(paper)
    factor = min((size - 2 * margin) / np.array([width - 1, height - 1]))
    offset = origin + (size - np.array([width - 1, height - 1]) * factor) / 2
    return factor, offset


def vectorize(path, config, *, diagnostics=None):
    options = config["image"]
    if "smoothing_span_mm" in options:
        warnings.warn(
            "image.smoothing_span_mm 已废弃：连续样条不再按固定窗口分段；请从配置中移除此字段",
            UserWarning,
            stacklevel=2,
        )
    if options.get("mode", "faithful") not in ("faithful", "simple"):
        raise ValueError("image.mode 必须是 faithful 或 simple")
    pen_width = options.get("pen_width_mm", 0.5)
    for name, value, minimum in (
        ("max_pixels", options["max_pixels"], 2),
        ("min_component_pixels", options["min_component_pixels"], 1),
        ("step_mm", options["step_mm"], np.finfo(float).eps),
        ("pen_width_mm", pen_width, np.finfo(float).eps),
        ("simplify_mm", options["simplify_mm"], 0),
        ("curve_chord_error_mm", options.get("curve_chord_error_mm", 0.03), np.finfo(float).eps),
        ("min_stroke_mm", options["min_stroke_mm"], 0),
        ("join_angle_deg", options.get("join_angle_deg", 35.0), 0),
        ("direction_lookahead_mm", options.get("direction_lookahead_mm", 0.8), 0),
        ("junction_span_mm", options.get("junction_span_mm", 0.5), 0),
    ):
        if not np.isfinite(value) or value < minimum:
            raise ValueError(f"image.{name} 必须是有限数且不小于 {minimum:g}")
    if options.get("join_angle_deg", 35.0) > 90:
        raise ValueError("image.join_angle_deg 不能大于 90")
    passes = options.get("order_passes", 0)
    if isinstance(passes, bool) or not isinstance(passes, int) or not 0 <= passes <= 2:
        raise ValueError("image.order_passes 必须是 0 到 2 的整数")
    mask = read_mask(path, options["max_pixels"])
    _, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    keep = stats[:, cv2.CC_STAT_AREA] >= options["min_component_pixels"]
    keep[0] = False
    mask = keep[labels].astype(np.uint8) * 255
    height = mask.shape[0]
    factor, offset = image_frame(mask.shape, config["paper"])
    if options.get("mode") == "simple":
        from .simple_lines import simple_paths

        paths, skeleton = simple_paths(mask, factor, offset, options, diagnostics=diagnostics)
        strokes = [Stroke(resample(points, options["step_mm"]), closed) for points, closed in paths]
        if not strokes:
            raise NoDrawableLinesError("图像中没有可绘制的线条")
        return order_strokes(strokes, passes), skeleton
    line_mask, marks = extract_marks(mask, factor, pen_width)
    skeleton = cv2.ximgproc.thinning(line_mask)
    chains = trace_strokes(
        skeleton,
        factor,
        options["min_stroke_mm"],
        options.get("join_angle_deg", 35.0),
        options.get("direction_lookahead_mm", 0.8),
        options.get("junction_span_mm", 0.5),
        with_anchors=True,
    )
    # 黑块中的短笔和小闭环承载细节，不能再按普通笔画长度过滤。
    mark_mask = mark_support_mask(mask, factor, pen_width)
    paths = [(pixels, closed, mask, anchors) for pixels, closed, _, anchors in chains] + [
        (pixels, closed, mark_mask, np.array([], dtype=int)) for pixels, closed in marks
    ]
    candidates, originals = [], []
    for pixels, closed, support_mask, anchors in paths:
        stroke_index = len(originals)
        points = np.column_stack((pixels[:, 1], height - 1 - pixels[:, 0])) * factor + offset
        if len(points) < 2:
            continue
        records = []
        candidate = smooth_path(
            points,
            closed,
            options["simplify_mm"],
            supported=lambda path, support=support_mask: _supported_path(
                path, support, factor, offset
            ),
            fixed_points=points[anchors],
            max_step_mm=options["step_mm"],
            chord_error_mm=options.get("curve_chord_error_mm", 0.03),
            diagnostics=records,
        )
        if diagnostics is not None:
            diagnostics.extend(dict(record, stroke=stroke_index) for record in records)
        originals.append((points, closed))
        candidates.append((candidate, closed))
    strokes = [
        Stroke(resample(points, options["step_mm"]), closed)
        for points, closed in protect_connections(candidates, originals, diagnostics)
    ]
    if not strokes:
        raise NoDrawableLinesError("图像中没有可绘制的线条")
    return order_strokes(strokes, passes), skeleton
