"""将可信的局部黑块转为边界与疏排线，普通线条仍交给中心线追踪。"""

import cv2
import numpy as np


def mark_support_mask(mask, pixel_size_mm, pen_width_mm=0.5):
    """黑块笔尖可用区域：保护不连边、面积至多 max(16 mm², 64×笔宽²) 的白孔。"""
    # 前景使用八邻域，白区采用对应的四邻域，保住只隔一像素的孔壁。
    _, labels, stats, _ = cv2.connectedComponentsWithStats(
        (mask == 0).astype(np.uint8), connectivity=4
    )
    keep = np.zeros(len(stats), bool)
    keep[1:] = stats[1:, cv2.CC_STAT_AREA] * pixel_size_mm**2 <= max(16, 64 * pen_width_mm**2)
    border_labels = np.unique(np.concatenate((labels[0], labels[-1], labels[:, 0], labels[:, -1])))
    keep[border_labels] = False
    holes = keep[labels]
    if not holes.any():
        return mask.copy()
    clearance = cv2.distanceTransform((~holes).astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
    # 两像素余量覆盖白像素面积、取整与半像素采样，内环向黑区退让。
    safe = clearance >= pen_width_mm / (2 * pixel_size_mm) + 2
    return ((mask != 0) & safe).astype(np.uint8) * 255


def extract_marks(mask, pixel_size_mm, pen_width_mm=0.5, *, core_radius_mm=None):
    """返回剩余线条掩码及黑块笔画；笔画坐标为浮点 [y, x] 像素点列。

    core_radius_mm 仅覆盖厚核心判别尺度；安全区与排线仍按实际笔宽计算。
    """
    if mask.ndim != 2 or mask.dtype != np.uint8:
        raise ValueError("黑块提取需要二维 uint8 二值图")
    if not np.isfinite(pixel_size_mm) or pixel_size_mm <= 0:
        raise ValueError("像素尺寸必须为正数")
    if not np.isfinite(pen_width_mm) or pen_width_mm <= 0:
        raise ValueError("笔宽必须为正数")
    if core_radius_mm is not None and (not np.isfinite(core_radius_mm) or core_radius_mm <= 0):
        raise ValueError("厚核心半径必须为有限正数")

    # 以物理线宽判断厚区域，白色外边框保证贴边黑块的距离仍然有限。
    distance = cv2.distanceTransform(np.pad(mask, 1), cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[
        1:-1, 1:-1
    ]
    radius_mm = max(0.65, 1.3 * pen_width_mm) if core_radius_mm is None else core_radius_mm
    radius = max(2.0, radius_mm / pixel_size_mm)
    _, labels, stats, _ = cv2.connectedComponentsWithStats((distance >= radius).astype(np.uint8))
    line_mask = mask.copy()
    _, original_labels, original_stats, _ = cv2.connectedComponentsWithStats(mask)
    skeleton_pixels = np.argwhere(cv2.ximgproc.thinning(mask))
    skeleton_widths = distance[tuple(skeleton_pixels.T)]
    skeleton_components = original_labels[tuple(skeleton_pixels.T)]
    accepted = np.zeros(mask.shape, bool)
    support = mark_support_mask(mask, pixel_size_mm, pen_width_mm)
    grow = int(np.ceil(radius)) + 1
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * grow + 1, 2 * grow + 1))
    for label in range(1, len(stats)):
        core = labels == label
        pixels = np.argwhere(core)
        peak = float(distance[core].max())
        if len(pixels) < max(3, 0.25 * peak * peak):
            continue
        hull = cv2.convexHull(pixels[:, ::-1].astype(np.float32))
        hull_area = cv2.contourArea(hull)
        # 高光会让厚核心呈月牙形；允许这种凹形，交汇误判另由邻近线宽排除。
        if hull_area == 0 or len(pixels) / hull_area < 0.45:
            continue
        _, (width, height), _ = cv2.minAreaRect(hull)
        if min(width, height) == 0 or max(width, height) / min(width, height) > 2.8:
            continue

        # T 交汇处也有厚核心，但邻近线条几乎同样粗；瞳孔连接的眼睑明显更细。
        source_label = original_labels[tuple(pixels[0])]
        component = original_labels == source_label
        center = pixels.mean(axis=0)
        ranges = np.linalg.norm(skeleton_pixels - center, axis=1)
        nearby = skeleton_widths[
            (ranges >= 1.5 * peak) & (ranges <= 3.0 * peak) & (skeleton_components == source_label)
        ]
        if len(nearby) and np.median(nearby) > peak / 1.8:
            continue

        region = cv2.dilate(core.astype(np.uint8), kernel).astype(bool) & component
        component_stats = original_stats[source_label]
        # 高光会削掉厚核心的上缘。独立紧凑黑块可恢复完整原区域；
        # 连接长眼睑或包含其他厚核心时，仍只使用局部区域。
        extent = max(component_stats[cv2.CC_STAT_WIDTH], component_stats[cv2.CC_STAT_HEIGHT])
        core_extent = max(stats[label, cv2.CC_STAT_WIDTH], stats[label, cv2.CC_STAT_HEIGHT])
        if extent <= core_extent + 2 * grow and len(np.unique(labels[component])) == 2:
            region = component
        accepted |= region

    # 先合并区域再生成路径，避免相邻厚核心恢复范围相交时重复绘制。
    _, regions = cv2.connectedComponents(accepted.astype(np.uint8))
    marks = []
    for label in range(1, int(regions.max()) + 1):
        region = regions == label
        region_mask = region.astype(np.uint8) * support
        paths = _region_paths(region_mask, pixel_size_mm, pen_width_mm)
        # 原黑块全部退出骨架，包括无法容纳笔宽的保护边缘。
        line_mask[region] = 0
        marks.extend(paths)

    # 小实心点会细化为孤立像素，图追踪无法生成边。仅给独立、无孔、
    # 已接近圆形且没有有效骨架链的小区域补一笔，避免改变短线和空心环。
    skeleton_counts = np.bincount(skeleton_components, minlength=len(original_stats))
    for label in range(1, len(original_stats)):
        x, y, width, height, area = original_stats[label]
        if area < 8 or max(width - 1, height - 1) * pixel_size_mm > max(1.6, 4 * pen_width_mm):
            continue
        if skeleton_counts[label] > 2:
            continue
        component = original_labels[y : y + height, x : x + width] == label
        if accepted[y : y + height, x : x + width][component].any():
            continue
        component_distance = distance[y : y + height, x : x + width]
        if component_distance[component].max() < 2:
            continue
        pixels = np.argwhere(component)
        _, (major, minor), _ = cv2.minAreaRect(pixels[:, ::-1].astype(np.float32))
        if min(major, minor) == 0 or max(major, minor) / min(major, minor) > 1.5:
            continue
        _, hierarchy = cv2.findContours(
            component.astype(np.uint8), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE
        )
        if np.any(hierarchy[0, :, 3] != -1):
            continue
        center_y, center_x = np.unravel_index(
            np.argmax(component_distance * component), component.shape
        )
        xs = np.flatnonzero(component[center_y])
        diameter_mm = (xs[-1] - xs[0]) * pixel_size_mm
        length_mm = max(pixel_size_mm / 2, min(pen_width_mm / 2, diameter_mm - pen_width_mm))
        half = min(length_mm / pixel_size_mm / 2, center_x - xs[0], xs[-1] - center_x)
        if half > 0:
            points = np.array(
                [[y + center_y, x + center_x - half], [y + center_y, x + center_x + half]],
                dtype=float,
            )
            indices = np.rint(points).astype(int)
            if np.all(support[indices[:, 0], indices[:, 1]]):
                marks.append((points, False))
                line_mask[y : y + height, x : x + width][component] = 0
    return line_mask, marks


def _region_paths(region, pixel_size_mm, pen_width_mm):
    """完整边界保留孔洞；排线逐段裁剪，绝不跨过白色高光。"""
    contours, _ = cv2.findContours(region, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    paths = []
    for contour in contours:
        pixels = contour.reshape(-1, 2)[:, ::-1].astype(float)
        if len(pixels) >= 3:
            paths.append((np.vstack((pixels, pixels[0])), True))

    occupied_rows = np.flatnonzero(region.any(axis=1))
    if not len(occupied_rows):
        return paths
    first, last = occupied_rows[[0, -1]]
    # 最多约三条排线；实际笔宽较大时进一步拉开距离，避免密集填充。
    step = max(1, int(np.ceil(1.5 * pen_width_mm / pixel_size_mm)), (last - first + 3) // 4)
    for y in range(first + step, last, step):
        xs = np.flatnonzero(region[y])
        for span in np.split(xs, np.flatnonzero(np.diff(xs) > 1) + 1):
            if len(span) >= 2:
                points = np.column_stack((np.full(len(span), y), span)).astype(float)
                paths.append((points, False))
    return paths
