"""在独立 mock 后端验证首笔逆解分支筛选、完整规划和缓存轨迹执行。"""

import argparse
import copy
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node
from sensor_msgs.msg import JointState

from piper_draw.job import make_job
from piper_draw.vectorize import Stroke

sys.path.insert(0, str(Path(get_package_share_directory("piper_drawing")) / "launch"))
# ROS launch 的系统依赖由 apt 安装；uv 环境默认不包含这个目录。
sys.path.append("/usr/lib/python3/dist-packages")
from model import model


class PlannerCheck(Node):
    def __init__(self):
        super().__init__("drawing_planner_check")
        self.positions = None
        self.baseline = None
        self.max_joint_motion = 0.0
        self.joint_samples = 0
        self.create_subscription(JointState, "/joint_states", self.receive_joints, 10)

    def receive_joints(self, message):
        positions = dict(zip(message.name, message.position))
        names = [f"joint{i}" for i in range(1, 7)]
        if not all(name in positions for name in names):
            return
        self.positions = np.array([positions[name] for name in names])
        self.joint_samples += 1
        if self.baseline is not None:
            change = float(np.max(np.abs(self.positions - self.baseline)))
            self.max_joint_motion = max(self.max_joint_motion, change)


def stop_process(process):
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)
    finally:
        # launch 父进程若先退出，也要清理其进程组内的剩余后端节点。
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def wait_for_backend(node, backend):
    client = node.create_client(ListControllers, "/controller_manager/list_controllers")
    deadline = time.monotonic() + 30
    future = None
    sent_at = 0.0
    controllers = {}
    try:
        while time.monotonic() < deadline:
            assert backend.poll() is None, "mock 后端在就绪前退出"
            rclpy.spin_once(node, timeout_sec=0.1)
            if future is not None and future.done():
                controllers = {item.name: item.state for item in future.result().controller}
                future = None
            elif future is not None and time.monotonic() - sent_at > 2:
                # 就绪查询无副作用，未收到响应时在总等待预算内重新查询。
                client.remove_pending_request(future)
                future = None
            services = dict(node.get_service_names_and_types())
            if (node.positions is not None
                    and controllers.get("joint_state_broadcaster") == "active"
                    and controllers.get("arm_controller") == "active"
                    and "/apply_planning_scene" in services
                    and "/get_planning_scene" in services):
                node.baseline = node.positions.copy()
                return
            if future is None and client.service_is_ready():
                future = client.call_async(ListControllers.Request())
                sent_at = time.monotonic()
        raise AssertionError(f"mock 后端 30 秒内未就绪，控制器状态：{controllers}")
    finally:
        node.destroy_client(client)


def flatten_parameters(values, prefix=""):
    result = {}
    for name, value in values.items():
        key = f"{prefix}.{name}" if prefix else name
        if isinstance(value, dict):
            result.update(flatten_parameters(value, key))
        else:
            result[key] = value
    return result


def run_case(node, backend, executable, model_parameters, directory, name, job,
             *, should_fail=False, execute=False,
             expected_error="首笔逆解分支筛选失败"):
    job_path = directory / f"{name}-job.yaml"
    report_path = directory / f"{name}-report.yaml"
    log_path = directory / f"{name}.log"
    parameters_path = directory / f"{name}-parameters.yaml"
    job_path.write_text(yaml.safe_dump(job, sort_keys=False))
    parameters = {**model_parameters, "job": str(job_path), "mode": "mock",
                  "execute": execute, "report": str(report_path)}
    parameters_path.write_text(yaml.safe_dump({"/**": {"ros__parameters": parameters}}))
    # 显式输出目录可重复使用；不能把上一轮的报告误认为本轮结果。
    report_path.unlink(missing_ok=True)
    samples_before = node.joint_samples
    process = None
    try:
        with log_path.open("w") as log:
            try:
                process = subprocess.Popen(
                    [str(executable), "--ros-args", "--params-file", str(parameters_path)],
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                )
                deadline = time.monotonic() + 90
                while process.poll() is None and time.monotonic() < deadline:
                    assert backend.poll() is None, "mock 后端在测试期间退出"
                    rclpy.spin_once(node, timeout_sec=0.1)
                assert process.poll() is not None, "执行器 90 秒内未退出"
            finally:
                if process is not None:
                    stop_process(process)
        assert report_path.exists(), "执行器未生成报告"
        report = yaml.safe_load(report_path.read_text())
        assert node.joint_samples > samples_before, "测试期间没有收到新的完整关节反馈"
        if should_fail:
            assert process.returncode != 0, "失败任务被误报为成功"
            assert report["status"] == "failed", report
            assert report["completed_segments"] == 0, report
            assert "segments" not in report, "失败任务不能返回部分成功规划"
            assert expected_error in report["error"], report
        else:
            assert process.returncode == 0, f"执行器退出值：{process.returncode}"
            assert report["status"] == ("executed" if execute else "planned"), report
            planned = report["segments"]
            expected = [(item["stroke"], item["kind"]) for item in job["segments"]
                        if item["kind"] != "transit"]
            actual = [(item["stroke"], item["kind"]) for item in planned
                      if item["kind"] not in ("transit", "initial_lift")]
            assert actual == expected, "非空绘画阶段缺失或顺序错误"
            for item in planned:
                assert item["points"] > 1, item
                assert math.isfinite(item["seconds"]) and item["seconds"] > 0, item
                if item["kind"] == "draw":
                    targets = next(segment["poses"] for segment in job["segments"]
                                   if segment["kind"] == "draw"
                                   and segment["stroke"] == item["stroke"])
                    assert item["points"] >= len(targets) + 1, "draw 轨迹未覆盖所有目标点"
            if execute:
                assert len(planned) == 4, report
                assert report["completed_segments"] == 4, report
            else:
                assert report["completed_segments"] == 0, report
        if execute:
            assert node.max_joint_motion > 1e-3, "mock 执行未产生实际关节变化"
        else:
            assert node.max_joint_motion <= 1e-6, "仅规划或失败任务意外移动了机械臂"
        print(f"通过：{name}", flush=True)
        return report
    except (AssertionError, yaml.YAMLError) as error:
        report_text = report_path.read_text() if report_path.exists() else "未生成报告"
        log_text = log_path.read_text()
        raise AssertionError(
            f"{name}：{error}\n报告：\n{report_text}\n执行器日志：\n{log_text}"
        ) from error


def run_checks(directory):
    root = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((root / "config/sim.yaml").read_text())
    points = yaml.safe_load((root / "tests/data/planner_strokes.yaml").read_text())["strokes"]
    strokes = [Stroke(np.asarray(stroke, dtype=float)) for stroke in points]
    assert len(strokes) == 7 and len(strokes[5].points) == 18
    assert config["motion"]["max_joint_step_rad"] == 0.15
    job = make_job(strokes, config)
    last_draw = [item for item in job["segments"] if item["kind"] == "draw"][-1]
    assert min(pose[1] for pose in last_draw["poses"]) < 0
    assert max(pose[1] for pose in last_draw["poses"]) > 0
    config_path = directory / "config.yaml"
    config_path.write_text(yaml.safe_dump(config))
    model_parameters = flatten_parameters(model(config, "mock").to_dict())
    executable = Path(get_package_prefix("piper_drawing")) / "lib/piper_drawing/draw_executor"
    assert executable.is_file(), f"找不到已构建的执行器：{executable}"
    backend_log = directory / "backend.log"
    backend = None
    rclpy.init(args=[], domain_id=104)
    node = PlannerCheck()
    try:
        with backend_log.open("w") as log:
            try:
                backend = subprocess.Popen(
                    ["ros2", "launch", "piper_drawing", "bringup.launch.py",
                     "mode:=mock", f"config:={config_path}"],
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                )
                wait_for_backend(node, backend)
                for attempt in range(1, 6):
                    run_case(node, backend, executable, model_parameters, directory,
                             f"plan-{attempt}", job)
                # 只改抬笔缩放，抬笔应变快，绘制／落笔／空移时长保持不变。
                slow_lift = copy.deepcopy(job)
                slow_lift["motion"].update(lift_velocity_scale=0.1,
                                            lift_acceleration_scale=0.1)
                for segment in slow_lift["segments"]:
                    if segment["kind"] == "lift":
                        segment["speed_m_s"] = 1.0
                fast_lift = copy.deepcopy(slow_lift)
                fast_lift["motion"].update(lift_velocity_scale=1.0,
                                            lift_acceleration_scale=1.0)
                slow_report = run_case(node, backend, executable, model_parameters, directory,
                                       "slow-lift", slow_lift)
                fast_report = run_case(node, backend, executable, model_parameters, directory,
                                       "fast-lift", fast_lift)
                slow_times = {(s["stroke"], s["kind"]): s["seconds"]
                              for s in slow_report["segments"]}
                for segment in fast_report["segments"]:
                    if segment["kind"] in ("approach", "initial_lift"):
                        continue
                    original = slow_times[(segment["stroke"], segment["kind"])]
                    if segment["kind"] == "lift":
                        assert segment["seconds"] < original, "独立抬笔缩放未提速"
                    else:
                        assert math.isclose(segment["seconds"], original, rel_tol=1e-5,
                                            abs_tol=1e-5), "抬笔缩放影响了其他动作"
                for key in ("lift_velocity_scale", "lift_acceleration_scale"):
                    invalid_lift = copy.deepcopy(job)
                    invalid_lift["motion"][key] = 1.1
                    run_case(node, backend, executable, model_parameters, directory,
                             f"invalid-{key}", invalid_lift, should_fail=True,
                             expected_error=f"motion.{key}")
                unreachable = copy.deepcopy(job)
                final_draw = [item for item in unreachable["segments"]
                              if item["kind"] == "draw"][-1]
                final_draw["poses"][-1][0] = 2.0
                run_case(node, backend, executable, model_parameters, directory,
                         "unreachable-tail", unreachable, should_fail=True)
                first_stroke = copy.deepcopy(job)
                first_stroke["segments"] = first_stroke["segments"][:4]
                run_case(node, backend, executable, model_parameters, directory,
                         "execute-first-stroke", first_stroke, execute=True)
            finally:
                if backend is not None:
                    stop_process(backend)
    except AssertionError as error:
        raise AssertionError(f"{error}\n后端日志：\n{backend_log.read_text()}") from error
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps({"通过": True, "ROS_DOMAIN_ID": 104, "基础规划次数": 5,
                      "独立抬笔规划对照": True, "抬笔缩放边界拒绝": True,
                      "不可达尾部拒绝": True, "首笔四段执行": True},
                     ensure_ascii=False, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="保留本轮任务、参数、报告和日志的目录")
    args = parser.parse_args()
    # 测试与它启动的所有子进程固定使用独立域，覆盖调用终端的常驻仿真域设置。
    os.environ["ROS_DOMAIN_ID"] = "104"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    if args.output is not None:
        directory = args.output.resolve()
        directory.mkdir(parents=True, exist_ok=True)
        run_checks(directory)
    else:
        with tempfile.TemporaryDirectory(prefix="piper-planner-check-") as directory:
            run_checks(Path(directory))


if __name__ == "__main__":
    main()
