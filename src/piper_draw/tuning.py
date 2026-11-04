"""固定参考栅格评估取路径参数，并为整组样例选择同一配置。"""

import csv
import hashlib
import json
from copy import deepcopy
from pathlib import Path

import cv2
import numpy as np
import yaml

from .artifacts import Artifacts
from .job import prepare, write_preview
from .simple_lines import HighlightClearanceError, fill_small_holes
from .vectorize import NoDrawableLinesError, image_frame, read_mask, vectorize

TUNABLE_OPTIONS = {
    "mode",
    "spur_length_mm",
    "smoothing_span_mm",
    "small_hole_area_mm2",
    "max_pixels",
    "min_component_pixels",
    "min_stroke_mm",
    "simplify_mm",
    "join_angle_deg",
    "direction_lookahead_mm",
    "junction_span_mm",
    "order_passes",
}


def save_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def file_digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_tuning_config(path):
    settings = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(settings, dict):
        raise TypeError("调参配置必须是 YAML 映射")
    expected = {
        "evaluation_pixels",
        "coverage_tolerance_mm",
        "max_coverage_drop",
        "max_local_coverage_drop",
        "max_excess_increase",
        "max_time_increase",
        "profiles",
    }
    if expected - settings.keys() or settings.keys() - expected - {"reference_small_hole_area_mm2"}:
        raise ValueError(f"调参配置项必须为：{', '.join(sorted(expected))}")
    reference_area = settings.get("reference_small_hole_area_mm2", 0)
    if (
        isinstance(reference_area, bool)
        or not isinstance(reference_area, (int, float))
        or not np.isfinite(reference_area)
        or reference_area < 0
    ):
        raise ValueError("reference_small_hole_area_mm2 必须是有限非负数")
    pixels = settings["evaluation_pixels"]
    if isinstance(pixels, bool) or not isinstance(pixels, int) or pixels < 2:
        raise ValueError("evaluation_pixels 必须是至少为 2 的整数")
    for name in expected - {"profiles", "evaluation_pixels"}:
        value = settings[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value):
            raise ValueError(f"{name} 必须是有限数")
        if value < 0 or (name == "coverage_tolerance_mm" and value == 0):
            raise ValueError(f"{name} 超出有效范围")
        if name != "coverage_tolerance_mm" and value > 1:
            raise ValueError(f"{name} 不能大于 1")
    if not isinstance(settings["profiles"], dict) or not settings["profiles"]:
        raise ValueError("profiles 必须是非空的命名参数映射")
    for name, options in settings["profiles"].items():
        if not isinstance(name, str) or not name or name == "baseline":
            raise ValueError("参数组合名称必须是非空字符串，且不能为 baseline")
        if not isinstance(options, dict) or options.keys() - TUNABLE_OPTIONS:
            raise ValueError(f"参数组合 {name} 包含未知选项；笔宽、步长和纸面不参与调参")
        for option, value in options.items():
            if option == "mode":
                if value not in ("faithful", "simple"):
                    raise ValueError(f"{name}.mode 必须是 faithful 或 simple")
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise TypeError(f"{name}.{option} 必须是数值")
            if not np.isfinite(value) or value < 0:
                raise ValueError(f"{name}.{option} 必须是有限非负数")
            if option in {"max_pixels", "min_component_pixels", "order_passes"} and not isinstance(
                value, int
            ):
                raise TypeError(f"{name}.{option} 必须是整数")
            if option == "max_pixels" and value < 2:
                raise ValueError(f"{name}.max_pixels 必须至少为 2")
            if option == "min_component_pixels" and value < 1:
                raise ValueError(f"{name}.min_component_pixels 必须至少为 1")
            if option == "order_passes" and value > 2:
                raise ValueError(f"{name}.order_passes 不能大于 2")
            if option == "join_angle_deg" and value > 90:
                raise ValueError(f"{name}.join_angle_deg 不能大于 90")
            if option == "smoothing_span_mm" and value == 0:
                raise ValueError(f"{name}.smoothing_span_mm 必须大于 0")
    return settings


def reference_data(path, config, settings):
    # 固定评估分辨率，不跟随候选的 max_pixels 或过滤参数改变比较基准。
    mask = read_mask(path, settings["evaluation_pixels"])
    factor, offset = image_frame(mask.shape, config["paper"])
    mask = fill_small_holes(mask, factor, settings.get("reference_small_hole_area_mm2", 0))
    skeleton = cv2.ximgproc.thinning(mask) != 0
    if not skeleton.any():
        raise ValueError(f"评估图像没有有效线条：{path}")
    outside = cv2.distanceTransform((mask == 0).astype(np.uint8), cv2.DIST_L2, 5) * factor
    return mask, skeleton, factor, offset, outside


def path_metrics(strokes, config, reference, tolerance):
    mask, skeleton, factor, offset, outside = reference
    height, width = mask.shape
    raster = np.zeros(mask.shape, np.uint8)
    drawing_length = 0.0
    travel_length = 0.0
    short_strokes = 0
    turning = 0.0
    current = np.zeros(2)
    for stroke in strokes:
        pixels = (stroke.points - offset) / factor
        pixels[:, 1] = height - 1 - pixels[:, 1]
        cv2.polylines(raster, [np.rint(pixels).astype(np.int32)], False, 255, 1)
        length = float(np.linalg.norm(np.diff(stroke.points, axis=0), axis=1).sum())
        # 用固定 0.5 mm 弧长采样比较曲线转角，不用输出点数推断平滑质量。
        parameters = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(stroke.points, axis=0), axis=1))]
        samples = np.arange(0.0, length, 0.5)
        if len(samples) >= 3:
            values = np.column_stack(
                [np.interp(samples, parameters, stroke.points[:, axis]) for axis in range(2)]
            )
            deltas = np.diff(values, axis=0)
            norms = np.linalg.norm(deltas[:-1], axis=1) * np.linalg.norm(deltas[1:], axis=1)
            valid = norms > 1e-12
            cosines = np.sum(deltas[:-1] * deltas[1:], axis=1)[valid] / norms[valid]
            turning += float(np.arccos(np.clip(cosines, -1, 1)).sum())
        drawing_length += length
        short_strokes += length < 2.0
        travel_length += float(np.linalg.norm(stroke.points[0] - current))
        current = stroke.points[-1]
    distance = cv2.distanceTransform((raster == 0).astype(np.uint8), cv2.DIST_L2, 5) * factor
    covered = distance <= tolerance
    coverage = float(covered[skeleton].mean())
    local_coverage = []
    # 按画布均匀分块，保护小区域的线条；不使用只适配样例的五官框。
    for ys in np.array_split(np.arange(height), 6):
        for xs in np.array_split(np.arange(width), 4):
            local_skeleton = skeleton[np.ix_(ys, xs)]
            if local_skeleton.sum() * factor >= 1.0:
                local_coverage.append(float(covered[np.ix_(ys, xs)][local_skeleton].mean()))
            else:
                local_coverage.append(None)
    excess = float((outside[raster != 0] > tolerance).mean())
    motion = config["motion"]
    lift = motion["lift_mm"] - motion["contact_mm"]
    nominal_seconds = (
        drawing_length / motion["draw_mm_s"]
        + travel_length / motion["travel_mm_s"]
        + len(strokes)
        * lift
        * (1 / motion["lower_mm_s"] + 1 / motion.get("lift_mm_s", motion["lower_mm_s"]))
    )
    return {
        "stroke_count": len(strokes),
        "short_stroke_count": short_strokes,
        "point_count": sum(len(stroke.points) for stroke in strokes),
        "total_turn_rad": turning,
        "drawing_length_mm": drawing_length,
        "travel_length_mm": travel_length,
        "nominal_seconds": nominal_seconds,
        "coverage": coverage,
        "excess_fraction": excess,
        "local_coverage": local_coverage,
    }


def rejection_reasons(metrics, baseline, settings):
    reasons = []
    if metrics["coverage"] + settings["max_coverage_drop"] < baseline["coverage"]:
        reasons.append("总线条覆盖下降")
    for index, (local, original) in enumerate(
        zip(metrics["local_coverage"], baseline["local_coverage"])
    ):
        if original is not None and local + settings["max_local_coverage_drop"] < original:
            reasons.append(f"局部线条覆盖下降（区块 {index}）")
    if metrics["excess_fraction"] > baseline["excess_fraction"] + settings["max_excess_increase"]:
        reasons.append("新增偏离原黑线的路径")
    if metrics["nominal_seconds"] > baseline["nominal_seconds"] * (
        1 + settings["max_time_increase"]
    ):
        reasons.append("单图名义绘制成本增加")
    return reasons


def tune_paths(images, config_path, tuning_path, artifacts=None):
    """所有候选跑同一组线稿；逐图通过质量门槛后按平均相对成本选共同参数。"""
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    settings = load_tuning_config(tuning_path)
    images = [Path(image).resolve(strict=True) for image in images]
    if not images:
        raise ValueError("调参至少需要一张线稿")
    artifacts = artifacts or Artifacts.create()
    directory = artifacts.directory
    artifacts.path("baseline_config", ".yaml").write_text(
        yaml.safe_dump(config, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    artifacts.path("tuning", ".yaml").write_text(tuning_path.read_text(encoding="utf-8"))
    inputs = []
    for index, image in enumerate(images):
        snapshot = artifacts.path(f"tune_input_{index + 1}", image.suffix.lower())
        snapshot.write_bytes(image.read_bytes())
        inputs.append(
            {"path": str(image), "sha256": file_digest(snapshot), "snapshot": str(snapshot)}
        )
    images = [Path(item["snapshot"]) for item in inputs]
    baselines = []
    rows = []
    summaries = []
    profiles = {"baseline": {}, **settings["profiles"]}
    references = [reference_data(image, config, settings) for image in images]
    raw_references = references
    if settings.get("reference_small_hole_area_mm2", 0):
        raw_settings = {**settings, "reference_small_hole_area_mm2": 0}
        raw_references = [reference_data(image, config, raw_settings) for image in images]
        for index, reference in enumerate(references, start=1):
            target_path = artifacts.path(f"reference_{index}", ".png")
            if not cv2.imwrite(str(target_path), 255 - reference[0]):
                raise OSError(f"无法保存固定评估线稿：{target_path}")
    for name, overrides in profiles.items():
        candidate_config = deepcopy(config)
        candidate_config["image"].update(overrides)
        eligible = True
        costs = []
        for index, (image, reference) in enumerate(zip(images, references)):
            try:
                strokes, _ = vectorize(image, candidate_config)
            except (NoDrawableLinesError, HighlightClearanceError) as error:
                if name == "baseline":
                    raise
                eligible = False
                rows.append(
                    {
                        "profile": name,
                        "input_index": index,
                        "eligible": False,
                        "rejected": [str(error)],
                    }
                )
                continue
            metrics = path_metrics(
                strokes, candidate_config, reference, settings["coverage_tolerance_mm"]
            )
            raw_metrics = (
                metrics
                if raw_references is references
                else path_metrics(
                    strokes,
                    candidate_config,
                    raw_references[index],
                    settings["coverage_tolerance_mm"],
                )
            )
            metrics["raw_coverage"] = raw_metrics["coverage"]
            metrics["raw_local_coverage"] = raw_metrics["local_coverage"]
            if name == "baseline":
                baselines.append(metrics)
            reasons = rejection_reasons(metrics, baselines[index], settings)
            eligible &= not reasons
            costs.append(metrics["nominal_seconds"] / baselines[index]["nominal_seconds"])
            rows.append(
                {
                    "profile": name,
                    "input_index": index,
                    **metrics,
                    "eligible": not reasons,
                    "rejected": reasons,
                }
            )
        relative_cost = float(np.mean(costs)) if len(costs) == len(images) else None
        summaries.append({"profile": name, "eligible": eligible, "relative_cost": relative_cost})
        save_json(artifacts.path("tuning_progress", ".json"), {"profiles": summaries, "rows": rows})
        cost_text = f"{relative_cost:.4f}" if relative_cost is not None else "无有效比较"
        print(
            f"调参 {name}：成本比 {cost_text}；{'通过' if eligible else '未通过质量门槛'}",
            flush=True,
        )
    eligible = [summary for summary in summaries if summary["eligible"]]
    best = min(eligible, key=lambda item: item["relative_cost"])
    selected = deepcopy(config)
    selected["image"].update(profiles[best["profile"]])
    selected_path = artifacts.path("selected_config", ".yaml")
    selected_path.write_text(
        yaml.safe_dump(selected, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    for index, image in enumerate(images, start=1):
        sample_directory = directory / f"sample_{index:02d}"
        sample_directory.mkdir()
        sample_artifacts = Artifacts(sample_directory, f"sample_{index:02d}")
        prepare(image, selected_path, sample_artifacts)
        baseline_strokes, _ = vectorize(image, config)
        write_preview(
            baseline_strokes,
            config["paper"],
            sample_artifacts.path("baseline", ".svg"),
            config["image"].get("pen_width_mm", 0.5),
        )
    report = {"selected": best, "inputs": inputs, "profiles": summaries, "rows": rows}
    save_json(artifacts.path("tuning_report", ".json"), report)
    with artifacts.path("metrics", ".csv").open("w", encoding="utf-8", newline="") as output:
        fields = [
            name
            for name in rows[0]
            if name not in {"local_coverage", "raw_local_coverage", "rejected"}
        ]
        writer = csv.DictWriter(output, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    print(
        f"选用：{best['profile']}；配置：{selected_path}；报告：{artifacts.path('tuning_report', '.json')}"
    )
    return report, selected_path
