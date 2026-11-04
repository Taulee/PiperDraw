from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from piper_draw.geometry import resample
from piper_draw.job import prepare
from piper_draw.vectorize import _supported_path, vectorize


@pytest.fixture
def config():
    values = yaml.safe_load((Path(__file__).parents[1] / "config/sim.yaml").read_text())
    values["paper"].update(width_mm=40.0, height_mm=40.0, margin_mm=0.0)
    # 混合描线测试固定 faithful 算法，不跟随版本默认风格切换。
    values["image"]["mode"] = "faithful"
    return values


def write_image(directory, image):
    path = directory / "lineart.png"
    assert cv2.imwrite(str(path), image)
    return path


def test_connected_pupil_is_drawn_without_double_tracing_the_eyelid(tmp_path, config):
    image = np.full((201, 201), 255, np.uint8)
    cv2.line(image, (20, 95), (180, 95), 0, 2)
    cv2.circle(image, (100, 103), 9, 0, -1)
    strokes, _ = vectorize(write_image(tmp_path, image), config)

    assert any(stroke.closed for stroke in strokes)
    # 两侧眼睑仍是中心线，黑块没有把整条连接区域都转为轮廓。
    for x in (8.0, 32.0):
        nearby = [point for stroke in strokes for point in stroke.points if abs(point[0] - x) < 0.3]
        assert nearby
        assert np.ptp(np.asarray(nearby)[:, 1]) < 0.5
    assert any(
        np.linalg.norm(stroke.points - [20.0, 19.4], axis=1).min() < 0.9 for stroke in strokes
    )


def test_smoothing_and_shading_do_not_cross_white_highlight(tmp_path, config):
    image = np.full((201, 201), 255, np.uint8)
    cv2.circle(image, (100, 100), 20, 0, -1)
    cv2.circle(image, (100, 92), 6, 255, -1)
    highlight = np.zeros_like(image)
    cv2.circle(highlight, (100, 92), 6, 255, -1)
    strokes, _ = vectorize(write_image(tmp_path, image), config)

    # 闭环可在共享点接续短笔，整个笔画开放时仍须保留内外轮廓。
    enclosing_areas = []
    for stroke in strokes:
        seen = {}
        for i, point in enumerate(stroke.points):
            key = tuple(point)
            if key in seen and i - seen[key] >= 3:
                loop = stroke.points[seen[key] : i + 1].astype(np.float32)
                if cv2.pointPolygonTest(loop, (20.0, 21.6), False) > 0:
                    enclosing_areas.append(cv2.contourArea(loop))
            seen[key] = i
    assert len(enclosing_areas) >= 2
    assert max(enclosing_areas) > min(enclosing_areas) > 0
    for stroke in strokes:
        dense = resample(stroke.points, 0.05) / 0.2
        dense[:, 1] = 200 - dense[:, 1]
        pixels = np.rint(dense).astype(int)
        assert not np.any(highlight[pixels[:, 1], pixels[:, 0]])


@pytest.mark.parametrize("radius", [1, 6])
def test_highlight_survives_actual_pen_width_after_smoothing(tmp_path, config, radius):
    image = np.full((201, 201), 255, np.uint8)
    cv2.circle(image, (100, 100), 20, 0, -1)
    cv2.circle(image, (100, 92), radius, 255, -1)
    white_mask = np.zeros_like(image)
    cv2.circle(white_mask, (100, 92), radius, 255, -1)
    strokes, _ = vectorize(write_image(tmp_path, image), config)
    white_pixels = np.argwhere(white_mask)
    highlight = np.column_stack((white_pixels[:, 1], 200 - white_pixels[:, 0])) * 0.2
    for stroke in strokes:
        start, delta = stroke.points[:-1], np.diff(stroke.points, axis=0)
        length_squared = np.sum(delta * delta, axis=1)
        for point in highlight:
            along = np.clip(np.sum((point - start) * delta, axis=1) / length_squared, 0, 1)
            distance = np.linalg.norm(point - start - along[:, None] * delta, axis=1)
            assert distance.min() > config["image"]["pen_width_mm"] / 2


def test_straight_continuation_keeps_t_shape_and_separate_gap(tmp_path, config):
    image = np.full((201, 201), 255, np.uint8)
    cv2.line(image, (20, 50), (180, 50), 0, 3)
    cv2.line(image, (100, 50), (100, 130), 0, 3)
    cv2.line(image, (20, 160), (75, 160), 0, 3)
    cv2.line(image, (85, 160), (180, 160), 0, 3)
    strokes, _ = vectorize(write_image(tmp_path, image), config)

    assert len(strokes) == 4
    assert any(np.ptp(stroke.points[:, 1]) < 1e-8 and len(stroke.points) > 20 for stroke in strokes)
    for stroke in strokes:
        if np.mean(stroke.points[:, 1]) < 10:
            assert stroke.points[:, 0].max() < 16 or stroke.points[:, 0].min() > 16


def test_preview_width_and_generated_job_use_the_same_paths(tmp_path, config, monkeypatch):
    image = np.full((101, 101), 255, np.uint8)
    cv2.line(image, (10, 20), (90, 20), 0, 3)
    path = write_image(tmp_path, image)
    settings = tmp_path / "config.yaml"
    config["image"]["pen_width_mm"] = 0.4
    settings.write_text(yaml.safe_dump(config))
    monkeypatch.chdir(tmp_path)
    job = prepare(path, settings)

    preview = next((tmp_path / "output").glob("*/*_preview.svg")).read_text()
    assert 'stroke-width="0.4"' in preview
    assert 'stroke-linecap="round"' in preview
    strokes, _ = vectorize(path, config)
    draws = [segment for segment in job["segments"] if segment["kind"] == "draw"]
    assert [len(segment["poses"]) for segment in draws] == [
        len(stroke.points) - 1 for stroke in strokes
    ]


@pytest.mark.parametrize("name,value", [("step_mm", 0), ("pen_width_mm", -1), ("simplify_mm", -1)])
def test_invalid_geometric_options_are_rejected(tmp_path, config, name, value):
    config["image"][name] = value
    image = np.full((31, 31), 255, np.uint8)
    cv2.line(image, (5, 15), (25, 15), 0, 2)
    with pytest.raises(ValueError, match=name):
        vectorize(write_image(tmp_path, image), config)


def test_margin_cannot_consume_the_entire_paper(tmp_path, config):
    config["paper"]["margin_mm"] = 20
    image = np.full((31, 31), 255, np.uint8)
    with pytest.raises(ValueError, match="两倍边距"):
        vectorize(write_image(tmp_path, image), config)


def test_extreme_aspect_ratio_is_rejected_before_resizing(tmp_path, config):
    config["image"]["max_pixels"] = 2
    image = np.full((10, 100), 255, np.uint8)
    with pytest.raises(ValueError, match="至少为 2 像素"):
        vectorize(write_image(tmp_path, image), config)


def test_path_support_detects_a_brief_crossing_of_white_pixel_corner():
    mask = np.full((11, 11), 255, np.uint8)
    pixels = np.array([[1.0, 1.98], [8.0, 8.98]])
    points = np.column_stack((pixels[:, 0], 10 - pixels[:, 1]))
    assert _supported_path(points, mask, 1.0, np.zeros(2))
    mask[5, 5] = 0
    assert not _supported_path(points, mask, 1.0, np.zeros(2))
