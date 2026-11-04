from itertools import pairwise

import cv2
import numpy as np
import pytest

from piper_draw.marks import extract_marks, mark_support_mask


def assert_paths_inside_mask(mask, paths):
    for pixels, closed in paths:
        assert pixels.ndim == 2 and pixels.shape[1] == 2
        assert pixels.dtype.kind == "f"
        indices = np.rint(pixels).astype(int)
        assert np.all(mask[indices[:, 0], indices[:, 1]] == 255)
        if closed:
            assert np.array_equal(pixels[0], pixels[-1])


def test_compact_black_circle_keeps_outline_and_sparse_shading():
    mask = np.zeros((81, 81), np.uint8)
    cv2.circle(mask, (40, 40), 15, 255, -1)
    line_mask, paths = extract_marks(mask, 0.1)

    assert not line_mask.any()
    assert len([path for path in paths if path[1]]) == 1
    assert 1 <= len([path for path in paths if not path[1]]) <= 3
    assert_paths_inside_mask(mask, paths)


def test_highlight_stays_white_and_splits_shading():
    mask = np.zeros((81, 81), np.uint8)
    cv2.circle(mask, (40, 40), 17, 255, -1)
    cv2.circle(mask, (40, 35), 4, 0, -1)
    _, paths = extract_marks(mask, 0.1)

    assert len([path for path in paths if path[1]]) == 2
    assert_paths_inside_mask(mask, paths)
    for pixels, _ in paths:
        # 检查整条线段而不只检查端点，排线不能直接穿过高光。
        for start, end in pairwise(pixels):
            samples = np.linspace(start, end, int(np.ceil(np.linalg.norm(end - start))) + 2)
            indices = np.rint(samples).astype(int)
            assert np.all(mask[indices[:, 0], indices[:, 1]] == 255)


@pytest.mark.parametrize("highlight_radius", [1, 2, 4])
def test_pen_footprint_preserves_every_original_highlight_pixel(highlight_radius):
    mask = np.zeros((81, 81), np.uint8)
    cv2.circle(mask, (40, 40), 17, 255, -1)
    highlight = np.zeros_like(mask)
    cv2.circle(highlight, (40, 35), highlight_radius, 255, -1)
    mask[highlight != 0] = 0

    _, paths = extract_marks(mask, 0.1, 0.5)
    assert paths
    footprint = np.zeros_like(mask)
    magnification = 8
    detailed_footprint = np.zeros((81 * magnification, 81 * magnification), np.uint8)
    for points, closed in paths:
        cv2.polylines(footprint, [np.rint(points[:, ::-1]).astype(np.int32)], closed, 255, 5)
        detailed_points = np.rint(points[:, ::-1] * magnification + (magnification - 1) / 2)
        cv2.polylines(
            detailed_footprint, [detailed_points.astype(np.int32)], closed, 255, 5 * magnification
        )
        # 同时检查整段到原白孔像素的最短距离，避免只验证笔尖采样点。
        hole_pixels = np.argwhere(highlight).astype(float)
        for start, end in pairwise(points):
            direction = end - start
            length_squared = np.dot(direction, direction)
            fractions = np.clip(((hole_pixels - start) @ direction) / length_squared, 0, 1)
            projected = start + fractions[:, None] * direction
            assert np.linalg.norm(hole_pixels - projected, axis=1).min() * 0.1 > 0.25
    assert not footprint[highlight != 0].any()
    detailed_highlight = np.repeat(
        np.repeat(highlight, magnification, axis=0), magnification, axis=1
    )
    assert not detailed_footprint[detailed_highlight != 0].any()
    assert_paths_inside_mask(mask, paths)


def test_large_enclosed_face_area_does_not_erode_ordinary_contour():
    mask = np.zeros((101, 101), np.uint8)
    cv2.circle(mask, (50, 50), 40, 255, 3)
    support = mark_support_mask(mask, 0.1, 0.5)

    assert np.array_equal(support, mask)
    assert support[50, 50] == 0


def test_highlight_support_excludes_black_pixels_too_close_for_the_pen():
    mask = np.zeros((81, 81), np.uint8)
    cv2.circle(mask, (40, 40), 17, 255, -1)
    cv2.circle(mask, (40, 35), 1, 0, -1)
    support = mark_support_mask(mask, 0.1, 0.5)

    assert mask[35, 42] == 255 and support[35, 42] == 0
    assert support[45, 40] == 255
    assert np.all(support[mask == 0] == 0)


@pytest.mark.parametrize("center_y", [26, 27])
def test_thin_black_wall_can_skip_inner_loop_without_painting_the_highlight(center_y):
    mask = np.zeros((81, 81), np.uint8)
    cv2.circle(mask, (40, 40), 17, 255, -1)
    highlight = np.zeros_like(mask)
    cv2.circle(highlight, (40, center_y), 2, 255, -1)
    mask[highlight != 0] = 0
    _, paths = extract_marks(mask, 0.1, 0.5)

    assert len([path for path in paths if path[1]]) == 1
    footprint = np.zeros_like(mask)
    for points, closed in paths:
        cv2.polylines(footprint, [np.rint(points[:, ::-1]).astype(np.int32)], closed, 255, 5)
    assert not footprint[highlight != 0].any()


def test_local_pupil_can_be_extracted_from_connected_eyelid():
    mask = np.zeros((101, 151), np.uint8)
    cv2.line(mask, (15, 46), (135, 46), 255, 3)
    cv2.circle(mask, (75, 58), 14, 255, -1)
    line_mask, paths = extract_marks(mask, 0.1)

    assert paths
    assert line_mask[58, 75] == 0
    assert np.array_equal(line_mask[:, :40], mask[:, :40])
    assert np.array_equal(line_mask[:, 110:], mask[:, 110:])
    assert_paths_inside_mask(mask, paths)


def test_neighboring_cores_are_drawn_once_after_region_union():
    mask = np.zeros((101, 131), np.uint8)
    cv2.circle(mask, (50, 50), 14, 255, -1)
    cv2.circle(mask, (74, 50), 14, 255, -1)
    _, paths = extract_marks(mask, 0.1)

    assert len([path for path in paths if path[1]]) == 1
    edges = [
        tuple(sorted((tuple(a), tuple(b)))) for points, _ in paths for a, b in pairwise(points)
    ]
    assert len(edges) == len(set(edges))
    assert_paths_inside_mask(mask, paths)


def test_disconnected_nearby_black_circles_do_not_reject_each_other():
    mask = np.zeros((101, 141), np.uint8)
    cv2.circle(mask, (45, 50), 14, 255, -1)
    cv2.circle(mask, (75, 50), 14, 255, -1)
    line_mask, paths = extract_marks(mask, 0.1)

    assert not line_mask.any()
    assert len([path for path in paths if path[1]]) == 2
    assert_paths_inside_mask(mask, paths)


@pytest.mark.parametrize("radius", [2, 3, 4, 5])
def test_small_filled_circle_keeps_one_nonzero_short_stroke(radius):
    mask = np.zeros((31, 31), np.uint8)
    cv2.circle(mask, (15, 15), radius, 255, -1)
    assert np.count_nonzero(cv2.ximgproc.thinning(mask)) == 1

    line_mask, paths = extract_marks(mask, 0.125)

    assert not line_mask.any()
    assert len(paths) == 1 and not paths[0][1]
    assert 0 < np.linalg.norm(np.diff(paths[0][0], axis=0), axis=1).sum() * 0.125 <= 0.5
    assert_paths_inside_mask(mask, paths)


def test_small_hollow_ring_keeps_its_centerline_input():
    mask = np.zeros((31, 31), np.uint8)
    cv2.circle(mask, (15, 15), 4, 255, 1)
    line_mask, paths = extract_marks(mask, 0.125)

    assert not paths
    assert np.array_equal(line_mask, mask)


def test_small_short_line_is_not_replaced_with_a_dot():
    mask = np.zeros((31, 31), np.uint8)
    cv2.line(mask, (10, 15), (15, 15), 255, 3)
    line_mask, paths = extract_marks(mask, 0.125)

    assert not paths
    assert np.array_equal(line_mask, mask)


@pytest.mark.parametrize("thickness", [3, 13, 25])
def test_t_junction_is_not_reclassified_as_black_mark(thickness):
    mask = np.zeros((151, 151), np.uint8)
    cv2.line(mask, (15, 35), (135, 35), 255, thickness)
    cv2.line(mask, (75, 35), (75, 135), 255, thickness)
    line_mask, paths = extract_marks(mask, 0.1)

    assert not paths
    assert np.array_equal(line_mask, mask)


def test_long_thick_line_stays_centerline_input():
    mask = np.zeros((101, 301), np.uint8)
    cv2.line(mask, (25, 50), (275, 50), 255, 25)
    line_mask, paths = extract_marks(mask, 0.1)

    assert not paths
    assert np.array_equal(line_mask, mask)


def test_thin_closed_outline_does_not_fill_white_interior():
    mask = np.zeros((101, 101), np.uint8)
    cv2.circle(mask, (50, 50), 35, 255, 3)
    line_mask, paths = extract_marks(mask, 0.1)

    assert not paths
    assert np.array_equal(line_mask, mask)
    assert line_mask[50, 50] == 0


def test_noise_dot_is_not_enlarged_or_given_a_fake_path():
    mask = np.zeros((31, 31), np.uint8)
    mask[15, 15] = 255
    line_mask, paths = extract_marks(mask, 0.1)

    assert not paths
    assert np.array_equal(line_mask, mask)


def test_black_region_at_image_boundary_keeps_valid_coordinates():
    mask = np.zeros((81, 81), np.uint8)
    cv2.circle(mask, (7, 40), 15, 255, -1)
    _, paths = extract_marks(mask, 0.1)

    assert paths
    assert_paths_inside_mask(mask, paths)


@pytest.mark.parametrize("pixel_size,pen_width", [(0, 0.5), (0.1, -1), (float("nan"), 0.5)])
def test_invalid_physical_sizes_are_rejected(pixel_size, pen_width):
    with pytest.raises(ValueError):
        extract_marks(np.zeros((3, 3), np.uint8), pixel_size, pen_width)
