"""在独立 ROS 域验证常驻纸面、任务快照、任务退出后的重连和新任务清场。"""

import copy
import json
import math
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Point
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray


def wait_for(node, condition, description):
    deadline = time.monotonic() + 8
    while not condition() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not condition():
        raise AssertionError(description)


def paper_in(message):
    papers = [item for item in message.markers if item.ns == "drawing_paper" and item.action == Marker.ADD]
    assert len(papers) == 1, "画面必须恰好包含一张纸"
    assert message.markers[0].action == Marker.DELETEALL, "完整快照缺少清场指令"
    return papers[0]


def has_ink(message):
    return any(item.ns == "drawing_ink" and item.action == Marker.ADD for item in message.markers)


def main():
    root = Path(__file__).resolve().parents[1]
    if os.environ.get("ROS_DOMAIN_ID") in (None, "96"):
        raise SystemExit("请设置独立的 ROS_DOMAIN_ID，例如 98，避免影响正在查看的仿真。")
    rclpy.init()
    node = rclpy.create_node("drawing_scene_check")
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    messages = []
    subscriptions = [node.create_subscription(MarkerArray, "/drawing_markers", messages.append, qos)]
    checks = {}
    process = None
    try:
        with tempfile.TemporaryDirectory(prefix="piper-scene-check-") as directory:
            config = yaml.safe_load((root / "config/sim.yaml").read_text())
            # 倾斜纸面用于独立核对三点标定、旋转和毫米到米的转换。
            config["paper"].update({
                "origin_mm": [100, 200, 300],
                "x_point_mm": [200, 200, 300],
                "y_point_mm": [100, 300, 400],
            })
            config_path = Path(directory) / "tilted.yaml"
            config_path.write_text(yaml.safe_dump(config))
            with (Path(directory) / "scene.log").open("w") as log:
                process = subprocess.Popen(
                    ["ros2", "run", "piper_drawing", "drawing_scene", "--ros-args",
                     "-p", f"config:={config_path}"],
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                )
                wait_for(node, lambda: bool(messages), "没有任务时未收到纸面")
                initial = copy.deepcopy(messages[-1])
                paper = paper_in(initial)
                assert not has_ink(initial)
                assert node.count_publishers("/drawing_markers") == 1
                checks["无需任务即可显示纸面"] = True
                diagonal = math.sqrt(0.5)
                half_height = config["paper"]["height_mm"] / 2000
                expected = [
                    0.1 + config["paper"]["width_mm"] / 2000,
                    0.2 + (half_height + 0.0001) * diagonal,
                    0.3 + (half_height - 0.0001) * diagonal,
                ]
                actual = [paper.pose.position.x, paper.pose.position.y, paper.pose.position.z]
                assert max(abs(a - b) for a, b in zip(actual, expected)) < 1e-10
                assert math.isclose(paper.pose.orientation.x, math.sin(math.pi / 8), abs_tol=1e-10)
                assert math.isclose(paper.pose.orientation.w, math.cos(math.pi / 8), abs_tol=1e-10)
                assert paper.header.frame_id == config["robot"]["base_link"]
                assert math.isclose(paper.scale.x, 0.148) and math.isclose(paper.scale.y, 0.21)
                assert paper.type == Marker.CUBE and paper.color.a == 1
                checks["倾斜纸面位置尺寸和姿态正确"] = True

                late_paper = []
                subscriptions.append(node.create_subscription(
                    MarkerArray, "/drawing_markers", late_paper.append, qos))
                wait_for(node, lambda: bool(late_paper), "晚连接未收到保留纸面")
                paper_in(late_paper[-1])
                checks["晚连接可取得纸面"] = True

                task_publisher = node.create_publisher(MarkerArray, "/drawing_task_markers", qos)
                wait_for(node, lambda: task_publisher.get_subscription_count() > 0, "场景未订阅任务快照")
                task = copy.deepcopy(initial)
                # 任务可携带自身纸面配置；场景应完整保留该画面。
                task.markers[1].pose.position.x += 0.01
                ink = Marker()
                ink.header.frame_id = paper.header.frame_id
                ink.ns, ink.id, ink.type, ink.action = "drawing_ink", 0, Marker.LINE_STRIP, Marker.ADD
                ink.pose.orientation.w = 1.0
                ink.scale.x = 0.0007
                ink.color.a = 1.0
                ink.points = [Point(x=0.11, y=0.2, z=0.3), Point(x=0.12, y=0.2, z=0.3)]
                task.markers.append(ink)
                task_publisher.publish(task)
                wait_for(node, lambda: has_ink(messages[-1]), "任务快照未传到显示主题")
                assert math.isclose(paper_in(messages[-1]).pose.position.x, paper.pose.position.x + 0.01)
                checks["任务快照正确更新纸面和笔迹"] = True
                node.destroy_publisher(task_publisher)

                late_ink = []
                subscriptions.append(node.create_subscription(
                    MarkerArray, "/drawing_markers", late_ink.append, qos))
                wait_for(node, lambda: bool(late_ink), "任务发布者退出后未收到场景快照")
                assert has_ink(late_ink[-1]) and len(late_ink[-1].markers[-1].points) == 2
                paper_in(late_ink[-1])
                checks["任务退出后重连仍可取得纸面和最后笔迹"] = True

                new_publisher = node.create_publisher(MarkerArray, "/drawing_task_markers", qos)
                wait_for(node, lambda: new_publisher.get_subscription_count() > 0, "新任务未连接场景")
                new_publisher.publish(initial)
                wait_for(node, lambda: not has_ink(messages[-1]), "新任务没有清除旧笔迹")
                paper_in(messages[-1])
                checks["新任务清除旧笔迹并保留纸面"] = True
                node.destroy_publisher(new_publisher)
    finally:
        if process and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=8)
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps({"通过": all(checks.values()), "检查": checks}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
