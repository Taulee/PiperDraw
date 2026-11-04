"""用本地假反馈检查 Piper ros2_control 插件，不创建 SDK 或连接 CAN。"""

import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from check_planner import stop_process
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

sys.path.insert(0, str(Path(get_package_share_directory("piper_drawing")) / "launch"))
from model import model


class FakePiper(Node):
    def __init__(self):
        super().__init__("fake_piper")
        self.positions = [0.2, 0.7, -1.0, 0.1, 0.3, -0.2, 0.005, -0.005]
        self.commands = []
        self.stopped = False
        self.publisher = self.create_publisher(
            JointState, "/piper/raw_joint_states", qos_profile_sensor_data
        )
        self.create_subscription(JointState, "/piper/joint_commands", self.commands.append, 10)
        self.create_service(Trigger, "/piper/stop", self.stop)
        self.timer = self.create_timer(0.01, self.publish)

    def publish(self):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [f"joint{i}" for i in range(1, 9)]
        message.position = self.positions
        self.publisher.publish(message)

    def stop(self, request, response):
        self.stopped = True
        response.success = True
        return response


def main():
    os.environ["ROS_DOMAIN_ID"] = "114"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    rclpy.init()
    driver = FakePiper()
    process = None

    def wait_for(condition):
        deadline = time.monotonic() + 10
        while not condition() and time.monotonic() < deadline:
            assert process.poll() is None, "控制器意外退出"
            rclpy.spin_once(driver, timeout_sec=0.02)
        assert condition(), "硬件检查等待超时"

    try:
        root = Path(__file__).resolve().parents[1]
        parameters = model(yaml.safe_load((root / "config/sim.yaml").read_text()), "real").to_dict()
        with tempfile.TemporaryDirectory(prefix="piper-hardware-") as directory:
            path = Path(directory) / "parameters.yaml"
            path.write_text(
                yaml.safe_dump(
                    {
                        "/**": {
                            "ros__parameters": {
                                **parameters,
                                "update_rate": 100,
                            }
                        }
                    }
                )
            )
            with (Path(directory) / "controller.log").open("w") as log:
                process = subprocess.Popen(
                    [
                        "ros2",
                        "run",
                        "controller_manager",
                        "ros2_control_node",
                        "--ros-args",
                        "--params-file",
                        str(path),
                    ],
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                wait_for(lambda: len(driver.commands) >= 10)
                assert driver.commands[0].name == [f"joint{i}" for i in range(1, 7)]
                assert list(driver.commands[0].position) == driver.positions[:6], (
                    "首帧未用真实反馈初始化"
                )
                driver.timer.cancel()
                wait_for(lambda: driver.stopped)
                count = len(driver.commands)
                until = time.monotonic() + 0.3
                while time.monotonic() < until:
                    rclpy.spin_once(driver, timeout_sec=0.02)
                assert len(driver.commands) == count, "反馈断流后仍发送新目标"
                print("通过：首帧与反馈对齐；断流停止发送并请求快速停止")
    finally:
        if process is not None:
            stop_process(process)
        driver.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
