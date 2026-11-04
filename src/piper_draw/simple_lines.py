"""简笔画中心线：保护短五官与闭环，只修剪附着毛刺并局部平滑。"""

from itertools import pairwise

import cv2
import numpy as np

from .curves import _corner_indices, _has_intersection, _smooth_span
from .marks import extract_marks, mark_support_mask
from .stroke_graph import _neighbors, trace, trace_strokes


class HighlightClearanceError(ValueError):
    """厚黑块的封闭高光无法留出当前笔宽。"""


def fill_small_holes(mask, pixel_size_mm, area_mm2):
    """按毫米面积填平封闭微小白孔，保留连边背景及更大的眼白、镜片。"""
    result = mask.copy()
    if area_mm2 == 0:
        return result
    _, labels, stats, _ = cv2.connectedComponentsWithStats(
        (mask == 0).astype(np.uint8), connectivity=4
    )
    # 容纳面积换算的浮点舍入，使阈值相等的孔仍符合“不大于”。
    limit = area_mm2 * (1 + 8 * np.finfo(float).eps)
    fill = stats[:, cv2.CC_STAT_AREA] * pixel_size_mm**2 <= limit
    fill[0] = False
    border = np.unique(np.concatenate((labels[0], labels[-1], labels[:, 0], labels[:, -1])))
    fill[border] = False
    result[fill[labels]] = 255
    return result


def prune_spurs(skeleton, pixel_size_mm, spur_length_mm):
    """单轮删除端点到分叉的短支路，保留分叉本身和两条有效主支路。

    独立短线、闭环及分叉间短桥均不属于毛刺。单轮判断始终依据原图，
    避免删除一条支路后，把真实结构继续当作新毛刺递归侵蚀。
    """
    result = skeleton.copy()
    if spur_length_mm == 0:
        return result
    neighbors = _neighbors(skeleton)
    chains = trace(skeleton)
    lengths = [
        float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum() * pixel_size_mm)
        for points, _, _ in chains
    ]
    substantial = {}
    for (points, closed, protected), length in zip(chains, lengths):
        if length > spur_length_mm or closed or protected:
            endpoints = (tuple(points[0]), tuple(points[-1]))
            for point in endpoints if closed else set(endpoints):
                substantial[point] = substantial.get(point, 0) + 1
    for (points, closed, protected), length in zip(chains, lengths):
        if closed or protected or length > spur_length_mm:
            continue
        first, last = tuple(points[0]), tuple(points[-1])
        if len(neighbors[first]) > 2 and len(neighbors[last]) == 1:
            points = points[::-1]
            first, last = last, first
        if (
            len(neighbors[first]) == 1
            and len(neighbors[last]) > 2
            and substantial.get(last, 0) >= 2
        ):
            result[tuple(points[:-1].T)] = 0
    return result


def smooth_supported_path(points, closed, tolerance_mm, span_mm, supported):
    """以毫米窗口平滑，误差有界；支撑失败只拆分并回退所在窗口。

    supported 必须校验完整折线是否留在黑线内。端点、尖角和窗口接点固定，
    每个候选沿用现有参数折线误差界；最终全路径检查阻止新增自交。
    """
    points = np.asarray(points, dtype=float)
    if len(points) < 3 or tolerance_mm == 0 or _has_intersection(points, closed):
        return points.copy()
    corners, _ = _corner_indices(points, tolerance_mm, closed)
    lengths = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    anchors = {0, len(points) - 1, *corners}
    for start, end in pairwise(sorted(anchors)):
        current = start
        while lengths[end] - lengths[current] > span_mm:
            following = max(
                current + 1, min(int(np.searchsorted(lengths, lengths[current] + span_mm)), end)
            )
            anchors.add(following)
            current = following

    def smooth_span(first, last):
        source = points[first : last + 1]
        candidate = _smooth_span(source, tolerance_mm)
        if supported(candidate):
            return candidate
        if last - first <= 2:
            return source
        middle = int(np.searchsorted(lengths, (lengths[first] + lengths[last]) / 2))
        middle = max(first + 1, min(middle, last - 1))
        left, right = smooth_span(first, middle), smooth_span(middle, last)
        return np.vstack((left, right[1:]))

    spans = [smooth_span(first, last) for first, last in pairwise(sorted(anchors))]
    result = np.vstack([spans[0], *[span[1:] for span in spans[1:]]])
    if (closed and len(result) < 4) or _has_intersection(result, closed):
        return points.copy()
    return result


def _dot_paths(mask, skeleton, pixel_size_mm, pen_width_mm):
    """无骨架边的实心点保留可见尺寸；较大点使用外轮廓及一条中心短线。"""
    _, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    counts = np.bincount(labels[skeleton != 0], minlength=len(stats))
    distance = cv2.distanceTransform(np.pad(mask, 1), cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[
        1:-1, 1:-1
    ]
    paths = []
    for label in range(1, len(stats)):
        x, y, width, height, _ = stats[label]
        if counts[label] > 1:
            continue
        component = (labels[y : y + height, x : x + width] == label).astype(np.uint8)
        pixels = np.argwhere(component)
        _, (major, minor), _ = cv2.minAreaRect(pixels[:, ::-1].astype(np.float32))
        if min(major, minor) == 0 or max(major, minor) / min(major, minor) > 1.5:
            continue
        contours, hierarchy = cv2.findContours(component, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        if np.any(hierarchy[0, :, 3] != -1):
            continue
        local_distance = distance[y : y + height, x : x + width] * component
        center_y, center_x = np.unravel_index(np.argmax(local_distance), component.shape)
        xs = np.flatnonzero(component[center_y])
        diameter_mm = (xs[-1] - xs[0]) * pixel_size_mm
        if diameter_mm >= 3 * pen_width_mm:
            contour = max(contours, key=cv2.contourArea).reshape(-1, 2)[:, ::-1]
            contour = contour + [y, x]
            paths.append((np.vstack((contour, contour[0])), True))
            length_mm = diameter_mm - pen_width_mm
        else:
            length_mm = max(pixel_size_mm / 2, min(pen_width_mm / 2, diameter_mm - pen_width_mm))
        half = min(length_mm / pixel_size_mm / 2, center_x - xs[0], xs[-1] - center_x)
        if half > 0:
            paths.append(
                (
                    np.array(
                        [[y + center_y, x + center_x - half], [y + center_y, x + center_x + half]],
                        dtype=float,
                    ),
                    False,
                )
            )
    return paths


def _visible_marks(mask, pixel_size_mm, pen_width_mm):
    """明显厚于笔宽的紧凑黑点保留轮廓，每个区域最多补一条中心短线。"""
    line_mask, marks = extract_marks(
        mask,
        pixel_size_mm,
        pen_width_mm,
        core_radius_mm=max(0.65, 2.6 * pen_width_mm),
    )
    removed = ((mask != 0) & (line_mask == 0)).astype(np.uint8)
    _, labels, _, centers = cv2.connectedComponentsWithStats(removed)
    outlines = [(points, closed) for points, closed in marks if closed]
    central = {}
    for points, closed in marks:
        if closed:
            continue
        midpoint = (points[0] + points[-1]) / 2
        label = labels[tuple(np.rint(midpoint).astype(int))]
        distance = np.linalg.norm(midpoint - centers[label, ::-1])
        if label not in central or distance < central[label][0]:
            central[label] = distance, points
    return line_mask, outlines + [(points, False) for _, points in central.values()]


def _guard_block_highlights(mask, pixel_size_mm, pen_width_mm):
    """厚黑块的封闭小白孔留出笔宽；薄线闭环仅保持中心线与拓扑。

    复用厚核心判别，不把薄镜框和空心眼睛当作黑块。保护区先退出骨架，
    避免平滑失败回退到本来就会涂过高光的原始中心线。
    """
    line_mask, _ = extract_marks(mask, pixel_size_mm, pen_width_mm)
    blocks = ((mask != 0) & (line_mask == 0)).astype(np.uint8) * 255
    _, labels, stats, _ = cv2.connectedComponentsWithStats(
        (blocks == 0).astype(np.uint8), connectivity=4
    )
    protected = stats[:, cv2.CC_STAT_AREA] * pixel_size_mm**2 <= max(16, 64 * pen_width_mm**2)
    protected[0] = False
    border = np.unique(np.concatenate((labels[0], labels[-1], labels[:, 0], labels[:, -1])))
    protected[border] = False
    # 厚块背景还可能包含孔内保留的独立细黑线；只比较原白区的连通性。
    holes = protected[labels] & (mask == 0)
    if not holes.any():
        return mask.copy()
    _, original_white_labels = cv2.connectedComponents((mask == 0).astype(np.uint8), connectivity=4)
    expected_holes = len(np.unique(original_white_labels[holes]))
    safe_blocks = mark_support_mask(blocks, pixel_size_mm, pen_width_mm)
    result = mask.copy()
    result[blocks != 0] = safe_blocks[blocks != 0]
    _, white_labels = cv2.connectedComponents((result == 0).astype(np.uint8), connectivity=4)
    border = np.unique(
        np.concatenate((white_labels[0], white_labels[-1], white_labels[:, 0], white_labels[:, -1]))
    )
    remaining_holes = np.unique(white_labels[holes])
    if np.intersect1d(remaining_holes, border).size or len(remaining_holes) != expected_holes:
        raise HighlightClearanceError(
            "简笔画黑块的白色高光无法容纳当前笔宽；请减小 image.pen_width_mm 或扩大高光间隔"
        )
    return result


def simple_paths(mask, pixel_size_mm, offset, options):
    """输入 uint8 黑线掩码及纸面变换；返回毫米路径和修剪后的像素骨架。

    普通线只走中心线；紧凑厚黑点保留外轮廓及至多一条中心短笔，避免
    中心线把瞳孔吞成闭眼弧。min_stroke_mm 不删除短五官，仅由
    spur_length_mm 控制附着的短毛刺。
    """
    from .vectorize import _supported_path

    spur_length_mm = options.get("spur_length_mm", 0.8)
    span_mm = options.get("smoothing_span_mm", 4.0)
    small_hole_area_mm2 = options.get("small_hole_area_mm2", 0.0)
    for name, value, minimum in (
        ("spur_length_mm", spur_length_mm, 0),
        ("smoothing_span_mm", span_mm, np.finfo(float).eps),
        ("small_hole_area_mm2", small_hole_area_mm2, 0),
    ):
        if not np.isfinite(value) or value < minimum:
            raise ValueError(f"image.{name} 必须是有限数且不小于 {minimum:g}")
    pen_width_mm = options.get("pen_width_mm", 0.5)
    mask = fill_small_holes(mask, pixel_size_mm, small_hole_area_mm2)
    # 厚黑块高光的保护属于输入几何转换，不依赖平滑候选是否被接受。
    # 普通薄闭环以指定笔宽重绘，中心线不跨白；实际墨迹可侵占原白边。
    support = _guard_block_highlights(mask, pixel_size_mm, pen_width_mm)
    line_mask, marks = _visible_marks(support, pixel_size_mm, pen_width_mm)
    original_skeleton = cv2.ximgproc.thinning(line_mask)
    skeleton = prune_spurs(original_skeleton, pixel_size_mm, spur_length_mm)
    chains = trace_strokes(
        skeleton,
        pixel_size_mm,
        0,
        options.get("join_angle_deg", 35.0),
        options.get("direction_lookahead_mm", 0.8),
        options.get("junction_span_mm", 0.5),
    )
    height = mask.shape[0]
    paths = []
    for pixels, closed, _ in chains:
        points = np.column_stack((pixels[:, 1], height - 1 - pixels[:, 0])) * pixel_size_mm + offset
        candidate = smooth_supported_path(
            points,
            closed,
            options["simplify_mm"],
            span_mm,
            lambda candidate: _supported_path(candidate, support, pixel_size_mm, offset),
        )
        paths.append((candidate, closed))
    for pixels, closed in marks + _dot_paths(
        line_mask, original_skeleton, pixel_size_mm, pen_width_mm
    ):
        points = np.column_stack((pixels[:, 1], height - 1 - pixels[:, 0])) * pixel_size_mm + offset
        paths.append((points, closed))
    return paths, skeleton
