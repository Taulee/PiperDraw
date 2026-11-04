"""ROS 2 与 Piper 官方 SDK 之间的薄桥接；默认只读。"""

import json
from pathlib import Path

import rclpy
from piper_drawing.srv import SetGripper
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from .piper_backend import ARM_NAMES, PiperSession, SDKTransport


class PiperBridge(Node):
    def __init__(self):
        super().__init__("piper_bridge")
        for name in ("can_port", "protocol", "dh_version", "lock_dir"):
            self.declare_parameter(name, Parameter.Type.STRING)
        for name, value in (
            ("firmware_speed_percent", 100),
            ("gripper_present", True),
            ("joint_lower", [-2.618, 0.0, -2.967, -1.745, -1.22, -2.094]),
            ("joint_upper", [2.618, 3.14, 0.0, 1.745, 1.22, 2.094]),
        ):
            self.declare_parameter(name, value)
        parameter = lambda name: self.get_parameter(name).value
        self.transport = SDKTransport(
            parameter("can_port"), parameter("protocol"), parameter("dh_version"),
            lock_dir=Path(parameter("lock_dir")),
            firmware_speed_percent=parameter("firmware_speed_percent"),
        )
        self.session = PiperSession(
            self.transport,
            parameter("joint_lower"),
            parameter("joint_upper"),
            parameter("protocol"),
            parameter("dh_version"),
            gripper_present=parameter("gripper_present"),
        )
        self.publisher = self.create_publisher(
            JointState, "/piper/raw_joint_states", qos_profile_sensor_data
        )
        self.status = self.create_publisher(String, "/piper/status", 1)
        self.create_subscription(JointState, "/piper/joint_commands", self.command, 1)
        self.create_service(SetBool, "/piper/enable", self.enable)
        self.create_service(Trigger, "/piper/enable_for_draw", self.enable_for_draw)
        self.create_service(Trigger, "/piper/stop", self.stop)
        self.create_service(Trigger, "/piper/hold", self.hold)
        self.create_service(Trigger, "/piper/resume_commands", self.resume_commands)
        self.create_service(Trigger, "/piper/preflight", self.preflight)
        self.create_service(Trigger, "/piper/check_feedback", self.check_feedback)
        self.create_service(SetGripper, "/piper/set_gripper", self.gripper)
        self.create_timer(0.01, self.poll)
        self.create_timer(1.0, self.publish_status)
        self.get_logger().info("Piper SDK 已启动，只读反馈；显式 enable 后才发送机械臂目标")
        self.get_logger().info(f"固件速度百分比：{self.transport.firmware_speed_percent}%")

    def command(self, message):
        try:
            if (
                len(message.name) != 6
                or set(message.name) != set(ARM_NAMES)
                or len(message.position) != 6
            ):
                raise ValueError("伺服消息必须按名称提供 joint1–joint6")
            age = (
                self.get_clock().now().nanoseconds
                - (message.header.stamp.sec * 10**9 + message.header.stamp.nanosec)
            ) / 1e9
            if not -0.05 <= age <= 0.25:
                raise ValueError("拒绝过期或未来的伺服消息")
            positions = dict(zip(message.name, message.position))
            self.session.command([positions[name] for name in ARM_NAMES])
        except (ValueError, RuntimeError) as error:
            self.get_logger().warning(str(error), throttle_duration_sec=2.0)
            self.fail(str(error))

    def fail(self, reason):
        if self.session.enabled:
            try:
                self.session.hold(reason)
            except (ValueError, RuntimeError):
                # 失联、失能或控制器故障时无法保证位置保持。
                self.session.stop(reason)
            self.get_logger().error(reason)

    def poll(self):
        try:
            self.session.watchdog()
            feedback = self.session.feedback()
        except (ValueError, RuntimeError) as error:
            self.fail(str(error))
            return
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        half_width = feedback.gripper_width_mm / 2000
        message.name = [*ARM_NAMES, "joint7", "joint8"]
        message.position = [*feedback.positions, half_width, -half_width]
        self.publisher.publish(message)

    def publish_status(self):
        feedback = self.transport.read()
        error = ""
        try:
            self.session.preflight(require_enabled=False)
        except (ValueError, RuntimeError) as failure:
            error = str(failure)
        self.status.publish(
            String(
                data=json.dumps(
                    {
                        "firmware": feedback.firmware,
                        "firmware_speed_percent": self.transport.firmware_speed_percent,
                        "dh_version": self.session.dh_version,
                        "session_enabled": self.session.enabled,
                        "motors_enabled": feedback.enabled,
                        "positions_rad": feedback.positions,
                        "gripper_width_mm": feedback.gripper_width_mm,
                        "arm_status": feedback.arm_status,
                        "error_code": feedback.error_code,
                        "preflight_error": error,
                        "fault": self.session.fault,
                        "holding": self.session.holding,
                        "hold_reason": self.session.hold_reason,
                    },
                    ensure_ascii=False,
                )
            )
        )

    def enable(self, request, response):
        try:
            if request.data:
                self.session.enable()
                response.message = "六轴已使能，目标已与反馈对齐"
            else:
                # 停止保留重力支撑与夹笔，不主动失能电机。
                self.session.stop()
                response.message = "已请求快速停止，保留夹爪夹持"
            response.success = True
        except (ValueError, RuntimeError) as error:
            self.fail(str(error))
            response.success, response.message = False, str(error)
        return response

    def stop(self, request, response):
        self.session.stop()
        response.success, response.message = True, "已请求控制器快速停止；实体状态以反馈为准"
        return response

    def hold(self, request, response):
        try:
            if not self.session.enabled:
                response.success, response.message = True, "本会话未使能，无运动目标需要保持"
                return response
            self.session.hold()
            response.success, response.message = True, "已锁定新鲜反馈位置，持续保持"
        except (ValueError, RuntimeError) as error:
            response.success, response.message = False, str(error)
        return response

    def resume_commands(self, request, response):
        try:
            self.session.resume_commands()
            response.success, response.message = True, "控制器已对齐，恢复轨迹目标"
        except (ValueError, RuntimeError) as error:
            response.success, response.message = False, str(error)
        return response

    def enable_for_draw(self, request, response):
        try:
            self.session.enable(allow_recovery=False)
            response.success, response.message = True, "六轴已使能，目标已与反馈对齐"
        except (ValueError, RuntimeError) as error:
            response.success, response.message = False, str(error)
        return response

    def preflight(self, request, response):
        try:
            self.session.preflight()
            if self.session.holding:
                raise RuntimeError("机械臂处于位置保持，请先对齐并恢复轨迹目标")
            response.success, response.message = True, "Piper 真机预检通过"
        except (ValueError, RuntimeError) as error:
            response.success, response.message = False, str(error)
        return response

    def gripper(self, request, response):
        try:
            self.session.gripper(request.width_mm, request.torque_nm)
            response.success = True
            response.message = "夹爪指令已提交；开口和实际夹持需查看反馈"
        except (ValueError, RuntimeError) as error:
            response.success, response.message = False, str(error)
        return response

    def check_feedback(self, request, response):
        try:
            self.session.preflight(require_enabled=False)
            response.success, response.message = True, "Piper 反馈与固件检查通过"
        except (ValueError, RuntimeError) as error:
            response.success, response.message = False, str(error)
        return response


def main():
    rclpy.init()
    node = None
    try:
        node = PiperBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                if node.session.enabled:
                    node.session.stop("桥接节点退出")
            finally:
                node.transport.close()
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
