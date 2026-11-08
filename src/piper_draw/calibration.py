"""TCP 与纸面标定业务接口；位置用 mm，四元数用 xyzw，不发送运动指令。"""

import xml.etree.ElementTree as ET
from copy import deepcopy
from datetime import datetime, timezone
from itertools import combinations
from pathlib import Path
from uuid import uuid4

import numpy as np
from scipy.spatial.transform import Rotation

from .geometry import drawing_bounds


def robot_identity(config):
    robot = config["robot"]
    if (robot["base_link"], robot["flange_link"], robot["group"]) != (
        "base_link", "link6", "arm"
    ) or robot["dh_version"] not in ("new", "old"):
        raise ValueError("标定需要 arm、base_link、link6 和 new/old DH 模型")
    return {key: robot[key] for key in ("base_link", "flange_link", "dh_version")}


def is_calibrated(config):
    """识别已求解的标定，包括旧版尚未勾选运动确认的配置。"""
    if config["calibrated"] is True:
        return True
    try:
        measured = config["calibration"]
        paper = measured["paper"]["paper"]
        return (
            measured["robot"] == robot_identity(config)
            and measured["tcp"]["tcp_offset_mm"] == config["tool"]["tcp_offset_mm"]
            and measured["paper"]["quaternion_in_paper"] == config["tool"]["quaternion_in_paper"]
            and all(paper[key] == config["paper"][key] for key in (
                "origin_mm", "x_point_mm", "y_point_mm", "width_mm", "height_mm", "drawing_area"
            ))
        )
    except (KeyError, TypeError, ValueError):
        return False


def flange_from_joints(positions_rad, dh_version):
    """按绘画使用的官方 URDF 求 base_link 到 link6 的正解。"""
    joints = np.asarray(positions_rad, dtype=float)
    if joints.shape != (6,) or not np.isfinite(joints).all():
        raise ValueError("采点需要六个有限关节角，单位 rad")
    if dh_version not in ("new", "old"):
        raise ValueError("DH 版本必须为 new/old")
    suffix = "_old" if dh_version == "old" else ""
    path = Path(__file__).resolve().parents[2] / (
        f"vendor/piper_ros/src/piper_description/urdf/piper_description{suffix}.urdf"
    )
    model = ET.parse(path)
    transform = np.eye(4)
    for i, angle in enumerate(joints, 1):
        joint = model.find(f"joint[@name='joint{i}']")
        limit = joint.find("limit")
        if not float(limit.get("lower")) - 1e-4 <= angle <= float(limit.get("upper")) + 1e-4:
            raise ValueError(f"joint{i} 反馈超出官方关节限位")
        origin = joint.find("origin")
        fixed = np.eye(4)
        fixed[:3, 3] = np.fromstring(origin.get("xyz"), sep=" ") * 1000
        fixed[:3, :3] = Rotation.from_euler(
            "xyz", np.fromstring(origin.get("rpy"), sep=" ")
        ).as_matrix()
        moving = np.eye(4)
        moving[:3, :3] = Rotation.from_rotvec(
            np.fromstring(joint.find("axis").get("xyz"), sep=" ") * angle
        ).as_matrix()
        transform = transform @ fixed @ moving
    return {
        "position_mm": transform[:3, 3].tolist(),
        "quaternion_xyzw": Rotation.from_matrix(transform[:3, :3]).as_quat().tolist(),
    }


def capture_pose(config, *, source="sdk", device=None):
    """读取实际反馈并求法兰位姿。SDK 使用既有连接锁；ROS 使用已有状态接口。"""
    identity = robot_identity(config)
    if source == "sdk":
        from .robot import doctor

        if device is None or device.dh_version != identity["dh_version"]:
            raise ValueError("采点设备 DH 与绘画配置不一致")
        feedback = doctor(device)
        positions = feedback["positions"]
    elif source == "ros":
        from .ros_client import command

        feedback = command("status")
        if feedback["preflight_error"]:
            raise RuntimeError(feedback["preflight_error"])
        if feedback["dh_version"] != identity["dh_version"]:
            raise ValueError("ROS 后端 DH 与绘画配置不一致")
        positions = feedback["positions_rad"]
    else:
        raise ValueError("采点来源必须为 sdk 或 ros")
    return {
        **flange_from_joints(positions, identity["dh_version"]),
        "positions_rad": list(positions),
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "source": source,
    }


def _poses(samples):
    positions = np.asarray([sample["position_mm"] for sample in samples], dtype=float)
    quaternions = np.asarray([sample["quaternion_xyzw"] for sample in samples], dtype=float)
    if positions.shape != (len(samples), 3) or not np.isfinite(positions).all():
        raise ValueError("法兰位置必须是三个有限数，单位 mm")
    if quaternions.shape != (len(samples), 4) or not np.isfinite(quaternions).all():
        raise ValueError("法兰姿态必须是四个有限数，顺序 xyzw")
    if np.any(np.abs(np.linalg.norm(quaternions, axis=1) - 1) > 1e-3):
        raise ValueError("法兰四元数必须归一化")
    return positions, Rotation.from_quat(quaternions).as_matrix()


def solve_tcp(samples, *, max_error_mm=1.0):
    """同一点、不同姿态的枢轴标定；返回偏移、接触点及可判读的拟合误差。"""
    if len(samples) < 4:
        raise ValueError("TCP 标定至少需要 4 个不同姿态的样本，建议 4–6 个")
    if not np.isfinite(max_error_mm) or max_error_mm <= 0:
        raise ValueError("TCP 最大误差阈值必须为有限正数")
    positions, rotations = _poses(samples)
    # 消去未知接触点；中心化使所有样本具有相同权重。
    matrix = (rotations - rotations.mean(axis=0)).reshape(-1, 3)
    target = -(positions - positions.mean(axis=0)).ravel()
    offset, _, rank, singular = np.linalg.lstsq(matrix, target, rcond=None)
    if rank < 3 or singular[-1] / np.sqrt(len(samples)) < 0.05:
        raise ValueError("TCP 姿态变化不足或仅绕同一轴旋转；请绕不同方向重新采点")
    condition = singular[0] / singular[-1]
    contacts = positions + np.einsum("nij,j->ni", rotations, offset)
    center = contacts.mean(axis=0)
    errors = np.linalg.norm(contacts - center, axis=1)
    if errors.max() > max_error_mm:
        raise ValueError(
            f"TCP 最大残差 {errors.max():.3f} mm 超过 {max_error_mm:g} mm；检查接触点或笔杆滑动"
        )
    return {
        "tcp_offset_mm": offset.tolist(),
        "contact_point_mm": center.tolist(),
        "rms_error_mm": float(np.sqrt(np.mean(errors**2))),
        "max_error_mm": float(errors.max()),
        "condition_number": float(condition),
        "sample_count": len(samples),
    }


def _rectangle_inside_polygon(origin, size, polygon):
    """中心在多边形内，且边界不进入矩形内部时，整个矩形都在多边形内。"""
    tolerance = 1e-8
    center = origin + size / 2
    inside = False
    for start, end in zip(polygon, np.roll(polygon, -1, axis=0)):
        if (start[1] > center[1]) != (end[1] > center[1]):
            crossing = start[0] + (center[1] - start[1]) * (
                end[0] - start[0]
            ) / (end[1] - start[1])
            if center[0] < crossing:
                inside = not inside
        # 裁剪每条边与矩形开集的交集；边界相切允许，穿过内部则拒绝。
        first, last = 0.0, 1.0
        for position, delta, lower, upper in zip(start, end - start, origin, origin + size):
            if abs(delta) < 1e-12:
                if position <= lower + tolerance or position >= upper - tolerance:
                    break
            else:
                limits = sorted(((lower + tolerance - position) / delta,
                                 (upper - tolerance - position) / delta))
                first, last = max(first, limits[0]), min(last, limits[1])
            if first >= last:
                break
        else:
            return False
    return inside


def _inscribed_rectangle(polygon, size):
    """寻找最靠近范围中心的固定尺寸矩形，也适用于凹四边形。

    可行选区的边界来自矩形角接触多边形边，或多边形角接触矩形边。
    最近解必为目标中心、这些直线上的投影或直线交点；逐个检查实际包含关系。
    """
    lower, upper = polygon.min(axis=0), polygon.max(axis=0) - size
    if np.any(lower > upper + 1e-8):
        return None
    target = (lower + upper) / 2
    corners = np.array([[0, 0], [size[0], 0], size, [0, size[1]]])
    lines = []
    for start, end in zip(polygon, np.roll(polygon, -1, axis=0)):
        edge = end - start
        normal = np.array([edge[1], -edge[0]]) / np.linalg.norm(edge)
        lines.extend((normal, float(normal @ (start - corner))) for corner in corners)
        lines.extend((normal, float(normal @ start - shift)) for normal, shift in (
            (np.array([1.0, 0.0]), 0), (np.array([1.0, 0.0]), size[0]),
            (np.array([0.0, 1.0]), 0), (np.array([0.0, 1.0]), size[1]),
        ))
    candidates = [target]
    candidates.extend(target + (offset - normal @ target) * normal for normal, offset in lines)
    for (first, a), (second, b) in combinations(lines, 2):
        matrix = np.array([first, second])
        if abs(np.linalg.det(matrix)) > 1e-12:
            candidates.append(np.linalg.solve(matrix, [a, b]))
    for origin in sorted(candidates, key=lambda point: np.linalg.norm(point - target)):
        if np.all(origin >= lower - 1e-8) and np.all(origin <= upper + 1e-8):
            origin = np.clip(origin, lower, np.maximum(upper, lower))
            if _rectangle_inside_polygon(origin, size, polygon):
                return origin
    return None


def locate_paper(samples, tcp_offset_mm, *, orientation="auto", max_plane_error_mm=1.0):
    """用四个环绕角点拟合纸面，在四边形内截取横向或纵向 A5。"""
    if len(samples) != 4:
        raise ValueError("纸面定位需要恰好 4 个样本，顺序为左下、右下、右上、左上")
    if not np.isfinite(max_plane_error_mm) or max_plane_error_mm <= 0:
        raise ValueError("纸面最大平面误差必须为有限正数")
    offset = np.asarray(tcp_offset_mm, dtype=float)
    if offset.shape != (3,) or not np.isfinite(offset).all():
        raise ValueError("TCP 偏移必须是三个有限数，单位 mm")
    positions, rotations = _poses(samples)
    points = positions + np.einsum("nij,j->ni", rotations, offset)
    center = points.mean(axis=0)
    centered = points - center
    _, singular, axes = np.linalg.svd(centered)
    area = np.cross(centered, np.roll(centered, -1, axis=0)).sum(axis=0)
    if singular[1] < 1 or np.linalg.norm(area) < 1:
        raise ValueError("纸面角点重合、共线或边界交叉")
    normal = axes[-1]
    if normal @ area < 0:
        normal = -normal
    errors = np.abs(centered @ normal)
    if errors.max() > max_plane_error_mm:
        raise ValueError(
            f"纸面最大平面残差 {errors.max():.3f} mm 超过 {max_plane_error_mm:g} mm；"
            "请检查纸张是否平整并重新采点"
        )
    x = points[1] - points[0]
    x -= (x @ normal) * normal
    if np.linalg.norm(x) < 1:
        raise ValueError("纸面角点重合或共线")
    x /= np.linalg.norm(x)
    rotation = np.column_stack((x, np.cross(normal, x), normal))
    polygon = (centered @ rotation)[:, :2]
    edges = np.roll(polygon, -1, axis=0) - polygon
    turns = np.cross(edges, np.roll(edges, -1, axis=0))
    if any(np.linalg.norm(a - b) < 1 for a, b in combinations(polygon, 2)) or np.any(
        np.abs(turns) < 1
    ):
        raise ValueError("纸面角点重合或共线")
    for i, j in ((0, 2), (1, 3)):
        a, b = polygon[i], polygon[(i + 1) % 4]
        c, d = polygon[j], polygon[(j + 1) % 4]
        if np.cross(b - a, c - a) * np.cross(b - a, d - a) < 0 and (
            np.cross(d - c, a - c) * np.cross(d - c, b - c) < 0
        ):
            raise ValueError("纸面四边形边界交叉；请按左下、右下、右上、左上顺序采点")
    if np.any((positions - points) @ normal <= 0):
        raise ValueError("纸面法向朝向错误；从纸面上方看，按左下、右下、右上、左上顺序采点")
    lower = polygon.min(axis=0)
    size = polygon.max(axis=0) - lower
    polygon -= lower
    origin = center + rotation[:, :2] @ lower
    choices = {"portrait": [148.0, 210.0], "landscape": [210.0, 148.0]}
    if orientation not in (*choices, "auto"):
        raise ValueError("A5 方向必须为 auto、portrait 或 landscape")
    candidates = choices.items() if orientation == "auto" else [(orientation, choices[orientation])]
    selected = None
    for name, dimensions in candidates:
        drawing_size = np.array(dimensions)
        drawing_origin = _inscribed_rectangle(polygon, drawing_size)
        if drawing_origin is not None:
            selected = name, drawing_size, drawing_origin
            break
    if selected is None:
        raise ValueError("采集的四边形内放不下指定方向的完整 A5，请扩大可画范围或调整方向")
    orientation, drawing_size, drawing_origin = selected
    # 现有预览和碰撞场景使用包围矩形，绘画选区严格限制在实测四边形内。
    paper = {
        "origin_mm": origin.tolist(),
        "x_point_mm": (origin + size[0] * rotation[:, 0]).tolist(),
        "y_point_mm": (origin + size[1] * rotation[:, 1]).tolist(),
        "width_mm": float(size[0]),
        "height_mm": float(size[1]),
        "drawing_area": {
            "origin_mm": drawing_origin.tolist(),
            "width_mm": float(drawing_size[0]),
            "height_mm": float(drawing_size[1]),
        },
    }
    return {
        "paper": paper,
        "measured_corners_mm": points.tolist(),
        "corners_in_paper_mm": polygon.tolist(),
        "normal": normal.tolist(),
        "plane_rms_error_mm": float(np.sqrt(np.mean(errors**2))),
        "plane_max_error_mm": float(errors.max()),
        "orientation": orientation,
        "quaternion_in_paper": Rotation.from_matrix(
            rotation.T @ rotations[0]
        ).as_quat().tolist(),
    }


def calibrate_tcp(config, observations, *, max_error_mm=1.0):
    """返回更新后的绘画配置；更新 TCP 会撤销旧纸面标定和实机许可。"""
    identity = robot_identity(config)
    if observations["robot"] != identity:
        raise ValueError("样本坐标系或 DH 版本与绘画配置不一致")
    result = solve_tcp(observations["samples"], max_error_mm=max_error_mm)
    if np.linalg.norm(result["tcp_offset_mm"]) <= config["tool"]["tip_clearance_mm"]:
        raise ValueError("标定笔杆长度必须大于笔尖接触余量")
    updated = deepcopy(config)
    updated["tool"]["tcp_offset_mm"] = result["tcp_offset_mm"].copy()
    updated["calibration"] = {"robot": identity, "tcp": result}
    updated["calibration_id"] = "tcp-" + uuid4().hex
    updated["calibrated"] = False
    return updated


def calibrate_paper(config, observations, *, orientation="auto", max_plane_error_mm=1.0):
    """返回已标定的纸面、A5 选区及实测笔姿。"""
    identity = robot_identity(config)
    calibration = config.get("calibration", {})
    if calibration.get("robot") != identity or "tcp" not in calibration:
        raise ValueError("请先完成当前模型的 TCP 标定")
    if observations["robot"] != identity:
        raise ValueError("样本坐标系或 DH 版本与绘画配置不一致")
    if config["tool"]["tcp_offset_mm"] != calibration["tcp"]["tcp_offset_mm"]:
        raise ValueError("TCP 已被修改，请重新标定")
    result = locate_paper(observations["samples"], config["tool"]["tcp_offset_mm"],
                          orientation=orientation, max_plane_error_mm=max_plane_error_mm)
    updated = deepcopy(config)
    updated["paper"].update(deepcopy(result["paper"]))
    drawing_bounds(updated["paper"])
    updated["tool"]["quaternion_in_paper"] = result["quaternion_in_paper"].copy()
    updated["calibration"]["paper"] = result
    updated["calibration_id"] = "paper-" + uuid4().hex
    updated["calibrated"] = True
    return updated
