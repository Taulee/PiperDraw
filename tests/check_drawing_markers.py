"""只读核对已关闭 MCAP 中的纸张、渐进墨线和真实 TF，不创建 ROS 节点。"""

import argparse
import bisect
import json
from pathlib import Path

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

MARKER_TOPIC = "/drawing_markers"


def transform(position, quaternion):
    """用独立的 TF 刚体变换核对显示点，避免复用被测 FK 实现。"""
    x, y, z, w = np.asarray(quaternion, dtype=float) / np.linalg.norm(quaternion)
    result = np.eye(4)
    result[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    result[:3, 3] = position
    return result


def pose_transform(pose):
    p, q = pose.position, pose.orientation
    return transform([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def open_reader(path, topics=None):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path.resolve()), storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    if topics is not None:
        reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return reader


def project(points, origin, normal):
    return points - np.outer((points - origin) @ normal - 0.0001, normal)


def task_strokes(job, origin, normal):
    """从任务的落笔位置和 draw 目标构造各笔几何，仅用于检查断线。"""
    tcp = np.asarray(job["tool"]["tcp_offset_mm"], dtype=float) / 1000
    starts, strokes = {}, {}
    for segment in job["segments"]:
        stroke = int(segment["stroke"])
        poses = segment["poses"]
        tips = [(transform(pose[:3], pose[3:]) @ np.r_[tcp, 1])[:3] for pose in poses]
        if segment["kind"] == "lower" and tips:
            starts[stroke] = tips[-1]
        elif segment["kind"] == "draw" and tips:
            strokes.setdefault(stroke, []).extend(([starts[stroke]] if stroke in starts else []) + tips)
    return {key: project(np.asarray(points), origin, normal) for key, points in strokes.items()}


def polyline_error(points, polyline):
    """每个显示点到单笔目标折线的最近距离，检测抬笔空移被错误连入墨线。"""
    if len(polyline) == 1:
        return float(np.linalg.norm(points - polyline[0], axis=1).max())
    start, delta = polyline[:-1], np.diff(polyline, axis=0)
    length2 = np.sum(delta * delta, axis=1)
    relative = points[:, None, :] - start
    weights = np.clip(np.sum(relative * delta, axis=2) / np.maximum(length2, 1e-24), 0, 1)
    distances = np.linalg.norm(relative - weights[:, :, None] * delta, axis=2)
    return float(distances.min(axis=1).max())


def matched_strokes(candidates):
    """允许接触中断拆分同笔，并避免把同一墨线当作多笔完成。"""
    assigned = {}

    def assign(strip, seen):
        for stroke in candidates[strip]:
            if stroke in seen:
                continue
            seen.add(stroke)
            if stroke not in assigned or assign(assigned[stroke], seen):
                assigned[stroke] = strip
                return True
        return False

    for strip in candidates:
        assign(strip, set())
    return assigned


def validate(args):
    job = yaml.safe_load(args.job.read_text())
    origin = np.asarray(job["paper_origin"], dtype=float)
    normal = np.asarray(job["paper_normal"], dtype=float)
    normal /= np.linalg.norm(normal)
    base, flange = job["robot"]["base_link"], job["robot"]["flange_link"]
    tcp = np.asarray(job["tool"]["tcp_offset_mm"], dtype=float) / 1000
    table = transform(job["table"]["pose"][:3], job["table"]["pose"][3:])
    size = np.asarray(job["table"]["size"], dtype=float)
    expected_top = table[:3, 3] + table[:3, 2] * size[2] / 2
    targets = task_strokes(job, origin, normal)
    expected = len(targets) if args.expect_strokes is None else args.expect_strokes

    reader = open_reader(args.recording)
    metadata = {entry.name: entry for entry in reader.get_all_topics_and_types()}
    counts = dict.fromkeys(metadata, 0)
    tf_class = get_message("tf2_msgs/msg/TFMessage")
    marker_class = get_message("visualization_msgs/msg/MarkerArray")
    parents, tip_samples, errors = {}, [], []
    while reader.has_next():
        topic, data, recorded = reader.read_next()
        counts[topic] = counts.get(topic, 0) + 1
        if topic not in ("/tf", "/tf_static"):
            continue
        try:
            message = deserialize_message(data, tf_class)
            for item in message.transforms:
                value = item.transform
                p, q = value.translation, value.rotation
                parents[item.child_frame_id] = (item.header.frame_id,
                    transform([p.x, p.y, p.z], [q.x, q.y, q.z, q.w]))
            current, chain = flange, []
            while current != base and current in parents and len(chain) < 100:
                current, edge = parents[current]
                chain.append(edge)
            if topic == "/tf" and current == base and message.transforms:
                world = np.eye(4)
                for edge in reversed(chain):
                    world = world @ edge
                time = max(stamp_ns(item.header.stamp) for item in message.transforms) or recorded
                tip_samples.append((time, (world @ np.r_[tcp, 1])[:3]))
        except (RuntimeError, ValueError, IndexError, KeyError, TypeError) as error:
            if len(errors) < 20:
                errors.append(f"{topic}: {error}")

    tip_samples.sort(key=lambda sample: sample[0])
    tip_times = [sample[0] for sample in tip_samples]
    tips = np.asarray([sample[1] for sample in tip_samples]).reshape(-1, 3)
    projected_tips = project(tips, origin, normal)
    reader = open_reader(args.recording, [MARKER_TOPIC])
    flags = dict.fromkeys(("deleteall_first", "single_paper", "paper_geometry", "paper_white",
                          "base_frame", "valid_lines", "monotonic_prefix", "ink_plane", "actual_tcp"), True)
    previous, history, previous_time = {}, {}, None
    snapshots = first_ink_count = new_points = 0
    growth_events = 0
    max_plane_error = max_tcp_error = max_raw_tcp_error = max_paper_error = 0.0
    missing_tf_points, tcp_mismatches = 0, []
    initial_empty = None
    final = {}
    while reader.has_next():
        _, data, recorded = reader.read_next()
        try:
            array = deserialize_message(data, marker_class)
            snapshots += 1
            markers = array.markers
            flags["deleteall_first"] &= (bool(markers) and markers[0].action == 3
                                         and sum(item.action == 3 for item in markers) == 1)
            paper = [item for item in markers if item.action == 0 and item.ns == "drawing_paper"]
            flags["single_paper"] &= len(paper) == 1
            for item in paper:
                pose = pose_transform(item.pose)
                dimensions = np.array([item.scale.x, item.scale.y, item.scale.z])
                top = pose[:3, 3] + pose[:3, 2] * dimensions[2] / 2
                error = max(float(np.linalg.norm(top - expected_top)),
                            float(np.linalg.norm(pose[:3, 2] - normal)),
                            float(np.max(np.abs(pose[:3, :3] - table[:3, :3]))),
                            float(np.max(np.abs(dimensions[:2] - size[:2]))))
                max_paper_error = max(max_paper_error, error)
                flags["paper_geometry"] &= item.type == 1 and 0 < dimensions[2] <= 0.001 and error <= 1e-5
                flags["paper_white"] &= min(item.color.r, item.color.g, item.color.b) >= 0.95 and item.color.a >= 0.9
                flags["base_frame"] &= item.header.frame_id == base
            ink = [item for item in markers if item.action == 0 and item.ns == "drawing_ink"]
            if initial_empty is None:
                initial_empty = not ink
            if ink and not first_ink_count:
                first_ink_count = len(ink)
            current = {}
            snapshot_time = max((stamp_ns(item.header.stamp) for item in markers), default=0) or recorded
            start_time = previous_time if previous_time is not None else snapshot_time
            left = bisect.bisect_left(tip_times, start_time - int(args.time_slack * 1e9))
            right = bisect.bisect_right(tip_times, snapshot_time + int(args.time_slack * 1e9))
            for item in ink:
                key = (item.ns, item.id)
                points = np.array([[point.x, point.y, point.z] for point in item.points]).reshape(-1, 3)
                pose = pose_transform(item.pose)
                points = points @ pose[:3, :3].T + pose[:3, 3]
                flags["base_frame"] &= item.header.frame_id == base
                flags["valid_lines"] &= (item.type == 4 and len(points) >= 2 and key not in current
                                         and bool(np.isfinite(points).all()) and item.scale.x > 0
                                         and max(item.color.r, item.color.g, item.color.b) <= 0.1
                                         and item.color.a >= 0.9)
                old = previous.get(key, np.empty((0, 3)))
                prefix_ok = len(points) >= len(old) and np.allclose(points[:len(old)], old, atol=1e-9, rtol=0)
                flags["monotonic_prefix"] &= prefix_ok
                appended = points[len(old):] if prefix_ok else points
                if len(old) and len(appended):
                    growth_events += 1
                history.setdefault(key, []).append(len(points))
                current[key] = points
                if len(points):
                    plane_error = float(np.abs((points - origin) @ normal - 0.0001).max())
                    max_plane_error = max(max_plane_error, plane_error)
                    flags["ink_plane"] &= plane_error <= 1e-4
                new_points += len(appended)
                for point_index, point in enumerate(appended, start=len(old) if prefix_ok else 0):
                    if right <= left:
                        flags["actual_tcp"] = False
                        missing_tf_points += 1
                        continue
                    # 优先实际三维距离，避免选择空移时 XY 相同但笔尖悬空的样本。
                    distances = np.linalg.norm(tips[left:right] - point, axis=1)
                    best = int(distances.argmin()) + left
                    raw_error = float(distances[best - left])
                    error = float(np.linalg.norm(projected_tips[best] - point))
                    max_tcp_error = max(max_tcp_error, error)
                    max_raw_tcp_error = max(max_raw_tcp_error, raw_error)
                    flags["actual_tcp"] &= error <= args.tcp_tolerance and raw_error <= args.tcp_tolerance
                    if (error > args.tcp_tolerance or raw_error > args.tcp_tolerance) and len(tcp_mismatches) < 10:
                        tcp_mismatches.append({"strip_id": item.id, "point_index": point_index,
                            "snapshot_timestamp_ns": snapshot_time, "tf_timestamp_ns": tip_times[best],
                            "point": point.tolist(), "actual_tcp": tips[best].tolist(),
                            "raw_error_m": raw_error, "projected_error_m": error})
            flags["monotonic_prefix"] &= previous.keys() <= current.keys()
            previous, previous_time, final = current, snapshot_time, current
        except (RuntimeError, ValueError, IndexError, KeyError, TypeError) as error:
            if len(errors) < 20:
                errors.append(f"{MARKER_TOPIC}: {error}")

    candidates = {}
    strip_errors = {}
    for key, points in final.items():
        distances = {stroke: polyline_error(points, target) for stroke, target in targets.items()}
        candidates[key] = sorted((stroke for stroke, error in distances.items() if error <= args.tcp_tolerance),
                                 key=lambda stroke: distances[stroke])
        strip_errors[str(key[1])] = min(distances.values(), default=None)
    assigned = matched_strokes(candidates)
    checks = {
        "marker_topic_recorded": counts.get(MARKER_TOPIC, 0) > 0 and snapshots > 0,
        "marker_topic_type": MARKER_TOPIC in metadata and metadata[MARKER_TOPIC].type == "visualization_msgs/msg/MarkerArray",
        "messages_deserialize": not errors,
        **{key: bool(value) for key, value in flags.items()},
        "initial_snapshot_has_no_old_ink": initial_empty is True,
        "expected_strokes": len(assigned) >= expected and (expected != 0 or not final),
        "individual_strokes_without_travel_lines": all(candidates.values()),
        "progressive_growth": (growth_events > 0 and any(min(lengths) < max(lengths) for lengths in history.values())) if expected else not history,
        "tf_reference_available": bool(tip_samples) if expected else True,
    }
    qos = getattr(metadata.get(MARKER_TOPIC), "offered_qos_profiles", "")
    return {
        "pass": all(checks.values()), "checks": checks,
        "recording": str(args.recording.resolve()), "job": str(args.job.resolve()),
        "message_counts": counts, "marker_offered_qos_profiles": qos,
        "snapshot_count": snapshots, "expected_strokes": expected,
        "final_line_strip_count": len(final), "matched_task_stroke_count": len(assigned),
        "matched_task_strokes": sorted(assigned), "first_nonempty_snapshot_strip_count": first_ink_count,
        "new_ink_point_count": new_points, "growth_snapshot_events": growth_events,
        "point_count_ranges_by_strip": {str(key[1]): [min(values), max(values)] for key, values in history.items()},
        "tf_tcp_sample_count": len(tip_samples), "maximum_paper_geometry_error": max_paper_error,
        "maximum_ink_plane_error_m": max_plane_error,
        "maximum_projected_tcp_error_m": max_tcp_error, "maximum_raw_tcp_error_m": max_raw_tcp_error,
        "new_points_without_nearby_tf": missing_tf_points, "tcp_mismatch_examples": tcp_mismatches,
        "maximum_task_polyline_error_m_by_strip": strip_errors,
        "tcp_tolerance_m": args.tcp_tolerance, "time_slack_seconds": args.time_slack,
        "errors": errors,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path, help="已关闭的 MCAP 目录或文件")
    parser.add_argument("job", type=Path, help="本次任务 job.yaml")
    parser.add_argument("--expect-strokes", type=int, help="预计实际绘制笔数；仅规划时传 0")
    parser.add_argument("--output", type=Path, help="验证结果 JSON，默认写录制路径同级")
    parser.add_argument("--tcp-tolerance", type=float, default=0.001, help="实际 TF 笔尖位置容差，单位米")
    parser.add_argument("--time-slack", type=float, default=0.25, help="快照批量发送及反馈补端点的时间余量，单位秒")
    args = parser.parse_args()
    if args.expect_strokes is not None and args.expect_strokes < 0:
        parser.error("--expect-strokes 必须非负")
    if args.tcp_tolerance <= 0 or args.time_slack < 0:
        parser.error("TCP 容差必须为正，时间余量必须非负")
    output = args.output or args.recording.parent / "marker-validation.json"
    result = validate(args)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({key: result[key] for key in ("pass", "checks", "snapshot_count", "final_line_strip_count",
          "matched_task_stroke_count", "new_ink_point_count", "maximum_projected_tcp_error_m", "maximum_raw_tcp_error_m")},
          ensure_ascii=False, indent=2))
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
