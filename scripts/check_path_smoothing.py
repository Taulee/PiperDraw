"""离线生成平滑前后对照；不启动 ROS，不连接或移动机械臂。"""

import argparse
import json
from collections import Counter
from copy import deepcopy
from pathlib import Path

import cv2
import numpy as np
import yaml

from piper_draw.artifacts import Artifacts
from piper_draw.job import prepare, write_preview
from piper_draw.tuning import (
    load_tuning_config,
    path_metrics,
    reference_data,
    rejection_reasons,
)
from piper_draw.vectorize import image_frame, vectorize


def overlay(before, after, paper, destination):
    height, width = paper["height_mm"], paper["width_mm"]
    paths = []
    for color, strokes in (("#d16b54", before), ("#0068b5", after)):
        for points in strokes:
            coordinates = " ".join(f"{x:.5f},{height - y:.5f}" for x, y in points)
            paths.append(f'<polyline points="{coordinates}" stroke="{color}"/>')
    destination.write_text(
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" '
        f'width="{width}mm" height="{height}mm"><rect width="100%" height="100%" '
        'fill="white"/><g fill="none" stroke-width="0.08">' + "".join(paths) + "</g></svg>\n"
    )


def diagnostics_summary(rows):
    return {
        "methods": dict(Counter(row["method"] for row in rows)),
        "preserved_reasons": dict(
            Counter(row.get("reason") for row in rows if row["method"] == "preserved")
        ),
        "preserved_length_mm": sum(
            row.get("source_length_mm", 0) for row in rows if row["method"] == "preserved"
        ),
        "source_length_mm": sum(row.get("source_length_mm", 0) for row in rows),
        "max_accepted_deviation_mm": max(
            (row.get("max_deviation_mm", 0) for row in rows), default=0
        ),
    }


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=root / "output/path-smoothing")
    parser.add_argument("--baseline", type=Path)
    parser.add_argument(
        "--portrait", type=Path, default=root / "output/draws/20261103_232337_135430"
    )
    parser.add_argument("--config", type=Path, default=root / "config/sim.yaml")
    parser.add_argument("--tolerance", type=float, default=0.15)
    parser.add_argument(
        "--capture-baseline",
        action="store_true",
        help="在旧版本模块下保存 before.json 与原始路径；不要求新诊断接口",
    )
    args = parser.parse_args()
    if args.capture_baseline and args.baseline:
        parser.error("--capture-baseline 与 --baseline 不能同时使用")
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    values = yaml.safe_load(args.config.read_text())
    values["image"].pop("smoothing_span_mm", None)
    values["image"].update(simplify_mm=args.tolerance, curve_chord_error_mm=0.03)
    baseline = json.loads(args.baseline.read_text()) if args.baseline else None
    prefix = "before" if args.capture_baseline else "after"
    records = {
        "tolerance_mm": args.tolerance,
        "end_exclusion_mm": 2,
        "end_exclusion_basis": "projected distance from traced endpoints",
        "lines": {},
    }
    for name, start, end in (
        ("diagonal", (60, 180), (660, 740)),
        ("shallow", (60, 400), (660, 480)),
        ("steep", (300, 60), (380, 940)),
    ):
        image = np.full((1024, 724), 255, np.uint8)
        cv2.line(image, start, end, 0, 5, cv2.LINE_AA)
        path = out / f"{name}.png"
        assert cv2.imwrite(str(path), image)
        rows = []
        strokes, _ = (
            vectorize(path, values)
            if args.capture_baseline
            else vectorize(path, values, diagnostics=rows)
        )
        points = np.vstack([stroke.points for stroke in strokes])
        factor, offset = image_frame(image.shape, values["paper"])
        first = np.array([start[0], 1023 - start[1]]) * factor + offset
        last = np.array([end[0], 1023 - end[1]]) * factor + offset
        tangent = (last - first) / np.linalg.norm(last - first)
        normal = np.array([-tangent[1], tangent[0]])
        along = (points - first) @ tangent
        low, high = sorted(along[[0, -1]])
        middle = points[(along > low + 2) & (along < high - 2)]
        wave = float(np.ptp((middle - first) @ normal))
        row = {
            "normal_ptp_mm": wave,
            "points": len(points),
            "strokes": len(strokes),
            "diagnostics": diagnostics_summary(rows),
        }
        if baseline:
            row["reduction_fraction"] = 1 - wave / baseline[name]["normal_ptp_mm"]
        records["lines"][name] = row
        write_preview(strokes, values["paper"], out / f"{prefix}-{name}.svg")
        np.savez(out / f"{prefix}-{name}.npz", *[stroke.points for stroke in strokes])
        if args.baseline and (args.baseline.parent / f"before-{name}.npz").exists():
            old = np.load(args.baseline.parent / f"before-{name}.npz")
            overlay(
                [old[key] for key in old.files],
                [stroke.points for stroke in strokes],
                values["paper"],
                out / f"compare-{name}.svg",
            )
    image = next(args.portrait.glob("*_line_art.png"))
    original_config = next(args.portrait.glob("*_drawing_config.yaml"))
    real = yaml.safe_load(original_config.read_text())
    candidate = deepcopy(real)
    candidate["image"].pop("smoothing_span_mm", None)
    candidate["image"].update(simplify_mm=args.tolerance, curve_chord_error_mm=0.03)
    config_path = out / "drawing-config.yaml"
    config_path.write_text(yaml.safe_dump(candidate, sort_keys=False, allow_unicode=True))
    rows = []
    strokes, _ = (
        vectorize(image, candidate)
        if args.capture_baseline
        else vectorize(image, candidate, diagnostics=rows)
    )
    settings = load_tuning_config(root / "config/tuning.yaml")
    reference = reference_data(image, candidate, settings)
    metrics = path_metrics(strokes, candidate, reference, settings["coverage_tolerance_mm"])
    records["portrait"] = {
        "metrics": metrics,
        "diagnostics": diagnostics_summary(rows),
        "rejected": rejection_reasons(metrics, baseline["portrait"], settings)
        if baseline
        else None,
    }
    write_preview(strokes, candidate["paper"], out / f"{prefix}-portrait.svg", 0.1)
    np.savez(out / f"{prefix}-portrait.npz", *[stroke.points for stroke in strokes])
    if args.baseline and (args.baseline.parent / "before-portrait.npz").exists():
        old = np.load(args.baseline.parent / "before-portrait.npz")
        overlay(
            [old[key] for key in old.files],
            [stroke.points for stroke in strokes],
            candidate["paper"],
            out / "compare-portrait.svg",
        )
    prepare(image, config_path, Artifacts(out, f"{prefix}-portrait"))
    if args.capture_baseline:
        records = {
            **records["lines"],
            "portrait": metrics,
            "end_exclusion_mm": 2,
            "tolerance_mm": args.tolerance,
            "end_exclusion_basis": records["end_exclusion_basis"],
        }
    (out / f"{prefix}.json").write_text(json.dumps(records, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(records, indent=2, ensure_ascii=False))
    if baseline:
        if any(row["reduction_fraction"] < 0.7 for row in records["lines"].values()):
            raise SystemExit("直线波动降幅未达到 70%")
        if records["portrait"]["rejected"]:
            raise SystemExit("人像质量门槛未通过")


if __name__ == "__main__":
    main()
