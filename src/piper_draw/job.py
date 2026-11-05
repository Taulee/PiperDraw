import json
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from .artifacts import Artifacts
from .calibration import is_calibrated
from .geometry import PaperFrame
from .vectorize import vectorize


def has_legacy_job_calibration(path, job):
    if job.get("calibrated") is not False or not path.name.endswith("_job.yaml"):
        return False
    snapshot_path = path.with_name(path.name.removesuffix("_job.yaml") + "_drawing_config.yaml")
    if not snapshot_path.is_file() or snapshot_path.is_symlink():
        return False
    snapshot = yaml.safe_load(snapshot_path.read_text(encoding="utf-8"))
    return (
        isinstance(snapshot, dict)
        and snapshot.get("calibrated") is False
        and all(
            key in job and snapshot.get(key) == job[key]
            for key in ("calibration_id", "robot", "tool", "motion")
        )
        and is_calibrated(snapshot)
    )


def make_job(strokes, config):
    paper, tool = config["paper"], config["tool"]
    frame = PaperFrame.from_points(paper["origin_mm"], paper["x_point_mm"], paper["y_point_mm"])
    rotation = Rotation.from_matrix(frame.rotation) * Rotation.from_quat(
        tool["quaternion_in_paper"]
    )
    # 保留原来的四元数归一化步骤，避免姿态末位浮点变化。
    flange_rotation = Rotation.from_quat(rotation.as_quat().tolist())
    quaternion = flange_rotation.as_quat().tolist()
    tcp_offset = flange_rotation.apply(np.asarray(tool["tcp_offset_mm"]) / 1000)
    motion = config["motion"]
    lift_speed = motion.get("lift_mm_s", motion["lower_mm_s"])

    def pose(xy, z):
        position = frame.point(xy, z) - tcp_offset
        return [*position.tolist(), *quaternion]

    segments = []
    for i, stroke in enumerate(strokes):
        start, end = stroke.points[0], stroke.points[-1]
        for kind, poses, speed in (
            (
                "approach" if i == 0 else "transit",
                [pose(start, motion["lift_mm"])],
                motion["travel_mm_s"],
            ),
            ("lower", [pose(start, motion["contact_mm"])], motion["lower_mm_s"]),
            (
                "draw",
                [pose(p, motion["contact_mm"]) for p in stroke.points[1:]],
                motion["draw_mm_s"],
            ),
            ("lift", [pose(end, motion["lift_mm"])], lift_speed),
        ):
            segments.append({"kind": kind, "stroke": i, "speed_m_s": speed / 1000, "poses": poses})
    center = frame.point(
        [paper["width_mm"] / 2, paper["height_mm"] / 2], -paper["thickness_mm"] / 2
    )
    return {
        "version": 1,
        "units": "m_rad",
        "calibration_id": config["calibration_id"],
        "calibrated": is_calibrated(config),
        "robot": config["robot"],
        "motion": motion,
        "tool": tool,
        "paper_origin": frame.origin.tolist(),
        "paper_normal": frame.rotation[:, 2].tolist(),
        "table": {
            "pose": [*center.tolist(), *frame.quaternion],
            "size": [
                paper["width_mm"] / 1000,
                paper["height_mm"] / 1000,
                paper["thickness_mm"] / 1000,
            ],
        },
        "obstacles": config.get("obstacles", []),
        "segments": segments,
    }


def write_preview(strokes, paper, output, pen_width_mm=0.5):
    width, height = paper["width_mm"], paper["height_mm"]
    paths = []
    for stroke in strokes:
        points = " ".join(f"{x:.3f},{height - y:.3f}" for x, y in stroke.points)
        paths.append(f'<polyline points="{points}"/>')
    output.write_text(
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}mm" height="{height}mm" '
        f'viewBox="0 0 {width} {height}"><rect width="100%" height="100%" fill="white"/>'
        f'<g fill="none" stroke="black" stroke-width="{pen_width_mm:g}" '
        'stroke-linejoin="round" stroke-linecap="round">' + "".join(paths) + "</g></svg>\n"
    )


def prepare(image_path: Path, config_path: Path, artifacts: Artifacts | None = None):
    config_text = config_path.read_text(encoding="utf-8")
    config = yaml.safe_load(config_text)
    smoothing = []
    strokes, skeleton = vectorize(image_path, config, diagnostics=smoothing)
    job = make_job(strokes, config)
    artifacts = artifacts if artifacts is not None else Artifacts.create()
    artifacts.path("drawing_config", ".yaml").write_text(config_text, encoding="utf-8")
    artifacts.path("smoothing", ".json").write_text(
        json.dumps(
            {
                "algorithm": "constrained_cubic_bspline_v1",
                "stroke_index_space": "before_ordering",
                "spans": smoothing,
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    skeleton_path = artifacts.path("skeleton", ".png")
    if not cv2.imwrite(str(skeleton_path), 255 - skeleton):
        raise OSError(f"无法保存骨架图：{skeleton_path}")
    write_preview(
        strokes,
        config["paper"],
        artifacts.path("preview", ".svg"),
        config["image"].get("pen_width_mm", 0.5),
    )
    job_path = artifacts.path("job", ".yaml")
    job_path.write_text(yaml.safe_dump(job, sort_keys=False, allow_unicode=True), encoding="utf-8")
    length = sum(np.linalg.norm(np.diff(s.points, axis=0), axis=1).sum() for s in strokes)
    print(f"{len(strokes)} 笔，绘制长度 {length:.1f} mm；任务：{job_path}")
    return job
