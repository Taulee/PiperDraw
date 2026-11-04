from copy import deepcopy
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from piper_draw import tuning
from piper_draw.artifacts import Artifacts
from piper_draw.simple_lines import HighlightClearanceError
from piper_draw.tuning import (
    load_tuning_config,
    path_metrics,
    reference_data,
    rejection_reasons,
    tune_paths,
)
from piper_draw.vectorize import Stroke, vectorize


@pytest.fixture
def settings():
    values = load_tuning_config(Path(__file__).parents[1] / "config/tuning.yaml")
    # 质量门槛测试以原始栅格为基准；填孔归一化由专门测试开启。
    values["reference_small_hole_area_mm2"] = 0
    return values


@pytest.fixture
def config():
    values = yaml.safe_load((Path(__file__).parents[1] / "config/sim.yaml").read_text())
    values["paper"].update(width_mm=40, height_mm=40, margin_mm=0)
    # 基准需要按 min_stroke_mm 过滤短线，候选再按测试选择 simple。
    values["image"]["mode"] = "faithful"
    return values


def write_image(tmp_path):
    image = np.full((201, 201), 255, np.uint8)
    cv2.line(image, (15, 30), (185, 30), 0, 2)
    cv2.line(image, (60, 110), (75, 110), 0, 2)
    cv2.line(image, (120, 145), (180, 145), 0, 2)
    path = tmp_path / "lines.png"
    assert cv2.imwrite(str(path), image)
    return path


def test_short_detail_loss_is_rejected_even_with_lower_drawing_cost(tmp_path, config, settings):
    path = write_image(tmp_path)
    strokes, _ = vectorize(path, config)
    reference = reference_data(path, config, settings)
    baseline = path_metrics(strokes, config, reference, 0.5)
    reduced = [stroke for stroke in strokes if np.ptp(stroke.points[:, 0]) > 5]
    metrics = path_metrics(reduced, config, reference, 0.5)
    assert metrics["nominal_seconds"] < baseline["nominal_seconds"]
    assert "总线条覆盖下降" in rejection_reasons(metrics, baseline, settings)
    assert any(
        "局部线条覆盖下降" in reason for reason in rejection_reasons(metrics, baseline, settings)
    )


def test_added_path_in_white_background_is_rejected(tmp_path, config, settings):
    path = write_image(tmp_path)
    strokes, _ = vectorize(path, config)
    reference = reference_data(path, config, settings)
    baseline = path_metrics(strokes, config, reference, 0.5)
    strokes.append(Stroke(np.array([[3, 3], [36, 3]])))
    metrics = path_metrics(strokes, config, reference, 0.5)
    assert "新增偏离原黑线的路径" in rejection_reasons(metrics, baseline, settings)


def test_reference_does_not_change_with_candidate_filtering(tmp_path, config, settings):
    path = write_image(tmp_path)
    candidate = deepcopy(config)
    candidate["image"].update(max_pixels=128, min_component_pixels=100, min_stroke_mm=10)
    first = reference_data(path, config, settings)
    second = reference_data(path, candidate, settings)
    assert np.array_equal(first[0], second[0])
    assert np.array_equal(first[1], second[1])


def test_cartoon_reference_is_fixed_and_preserves_large_white_regions(tmp_path, config, settings):
    image = np.full((201, 201), 255, np.uint8)
    for center, hole_radius in (((50, 100), 2), ((150, 100), 8)):
        cv2.circle(image, center, 15, 0, -1)
        cv2.circle(image, center, hole_radius, 255, -1)
    path = tmp_path / "holes.png"
    assert cv2.imwrite(str(path), image)
    raw = reference_data(path, config, settings)
    settings["reference_small_hole_area_mm2"] = 0.8
    first = reference_data(path, config, settings)
    candidate = deepcopy(config)
    candidate["image"].update(mode="simple", small_hole_area_mm2=0.25, min_stroke_mm=10)
    second = reference_data(path, candidate, settings)
    assert raw[0][100, 50] == 0 and first[0][100, 50] == 255
    assert first[0][100, 150] == 0 and first[0][0, 0] == 0
    assert np.array_equal(first[0], second[0])
    assert np.array_equal(first[1], second[1])


@pytest.mark.parametrize("error_type", [HighlightClearanceError, ValueError])
def test_only_known_geometry_failure_is_rejected_as_candidate(
    tmp_path, monkeypatch, config, settings, error_type
):
    path = write_image(tmp_path)
    config_path = tmp_path / "drawing.yaml"
    config_path.write_text(yaml.safe_dump(config))
    settings["profiles"] = {"blocked": {"mode": "simple"}}
    tuning_path = tmp_path / "tuning.yaml"
    tuning_path.write_text(yaml.safe_dump(settings))
    original = tuning.vectorize

    def vectorize_candidate(image, candidate):
        if candidate["image"].get("mode") == "simple":
            raise error_type("测试几何无法表示")
        return original(image, candidate)

    monkeypatch.setattr(tuning, "vectorize", vectorize_candidate)
    artifacts = Artifacts.create(tmp_path / "output")
    if error_type is ValueError:
        with pytest.raises(ValueError, match="测试几何"):
            tune_paths([path], config_path, tuning_path, artifacts)
    else:
        report, _ = tune_paths([path], config_path, tuning_path, artifacts)
        assert report["selected"]["profile"] == "baseline"
        assert not report["profiles"][1]["eligible"]
        assert report["profiles"][1]["relative_cost"] is None


def test_tuning_selects_common_profile_and_saves_reproducible_outputs(tmp_path, config, settings):
    path = write_image(tmp_path)
    config_path = tmp_path / "drawing.yaml"
    config_path.write_text(yaml.safe_dump(config))
    settings["profiles"] = {
        "ordered": {"order_passes": 2},
        "erase_detail": {"min_stroke_mm": 5},
        "erase_all": {"min_component_pixels": 100000},
    }
    tuning_path = tmp_path / "tuning.yaml"
    tuning_path.write_text(yaml.safe_dump(settings, sort_keys=False))
    artifacts = Artifacts.create(tmp_path / "output")
    report, selected_path = tune_paths([path, path], config_path, tuning_path, artifacts)
    rejected = next(item for item in report["profiles"] if item["profile"] == "erase_detail")
    assert not rejected["eligible"]
    erased = next(item for item in report["profiles"] if item["profile"] == "erase_all")
    assert not erased["eligible"] and erased["relative_cost"] is None
    assert report["selected"]["profile"] != "erase_detail"
    assert yaml.safe_load(selected_path.read_text())["paper"] == config["paper"]
    for index in (1, 2):
        sample = artifacts.directory / f"sample_{index:02d}"
        assert (sample / f"sample_{index:02d}_job.yaml").is_file()
        assert (sample / f"sample_{index:02d}_baseline.svg").is_file()
    assert artifacts.path("tuning_report", ".json").is_file()
    assert artifacts.path("metrics", ".csv").is_file()
    assert all(
        Path(item["snapshot"]).read_bytes() == path.read_bytes() for item in report["inputs"]
    )


@pytest.mark.parametrize(
    "overrides",
    [
        {"step_mm": 3},
        {"pen_width_mm": 0.1},
        {"max_pixels": 1},
        {"join_angle_deg": 91},
        {"order_passes": 3},
        {"min_stroke_mm": float("nan")},
    ],
)
def test_invalid_candidate_options_fail_before_running(tmp_path, settings, overrides):
    settings["profiles"] = {"invalid": overrides}
    path = tmp_path / "tuning.yaml"
    path.write_text(yaml.safe_dump(settings))
    with pytest.raises(ValueError):
        load_tuning_config(path)
