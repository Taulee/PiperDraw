"""ROS readiness and stop requests for the shared drawing workflow."""
import json
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.action import MoveGroup
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Trigger


class RobotConnection:
    def __init__(self, domain_id):
        self.context = Context()
        rclpy.init(args=[], context=self.context, domain_id=domain_id)
        self.node = rclpy.create_node("piper_draw_workflow", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.controllers = self.node.create_client(ListControllers, "/controller_manager/list_controllers")
        self.planner = ActionClient(self.node, MoveGroup, "/move_action")
        self.feedback = self.node.create_client(Trigger, "/piper/check_feedback")
        self.parameters = self.node.create_client(GetParameters, "/piper_bridge/get_parameters")
        self.stop_service = self.node.create_client(Trigger, "/piper/stop")
        self.enable_draw_service = self.node.create_client(Trigger, "/piper/enable_for_draw")
        self.last_status = None
        self.node.create_subscription(String, "/piper/status", self.status, 10)

    def status(self, message):
        self.last_status = json.loads(message.data)

    def call(self, service, request, timeout, check=lambda: None):
        if not service.service_is_ready():
            return None
        future = service.call_async(request)
        deadline = time.monotonic() + timeout
        try:
            while not future.done() and time.monotonic() < deadline:
                check()
                self.executor.spin_once(timeout_sec=0.1)
            return future.result() if future.done() else None
        finally:
            if not future.done():
                service.remove_pending_request(future)

    def controller_available(self):
        return self.controllers.wait_for_service(timeout_sec=1.0)

    def wait_ready(self, mode, device, backend, check):
        deadline = time.monotonic() + 35
        while time.monotonic() < deadline:
            check()
            if backend and backend.poll() is not None:
                raise RuntimeError("仿真启动失败，详细原因见连接日志" if mode == "mock" else
                                   "机械臂连接失败，请检查电源和连接线。详细原因见连接日志")
            if not self.planner.server_is_ready():
                self.executor.spin_once(timeout_sec=0.2)
                continue
            result = self.call(self.controllers, ListControllers.Request(), 2.5, check)
            active = {c.name for c in result.controller if c.state == "active"} if result else set()
            if {"joint_state_broadcaster", "arm_controller"} <= active:
                if mode == "mock":
                    if self.feedback.service_is_ready():
                        raise RuntimeError("当前设备正运行实机后端，请停止后再启动仿真")
                    return
                parameters = self.call(
                    self.parameters, GetParameters.Request(names=["can_port", "protocol", "dh_version"]),
                    0.8, check
                )
                if parameters:
                    actual = tuple(value.string_value for value in parameters.values)
                    if actual != (device.can, device.protocol, device.dh_version):
                        raise RuntimeError("连接中的设备与当前配置不同，请停止设备后重试")
                feedback = self.call(self.feedback, Trigger.Request(), 0.8, check)
                if parameters and feedback:
                    if not feedback.success:
                        raise RuntimeError(feedback.message)
                    return
            self.executor.spin_once(timeout_sec=0.2)
        raise RuntimeError("连接机械臂超时，请检查电源、连接线和设备状态")

    def stop(self):
        result = self.call(self.stop_service, Trigger.Request(), 5)
        if not result or not result.success:
            raise RuntimeError("机械臂停止请求未确认")
        # Require fresh device feedback after the stop acknowledgement.
        self.last_status = None
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.15)
            if self.last_status and self.last_status.get("arm_status") == 1:
                return
        raise RuntimeError("未收到机械臂停止反馈")

    def hold_and_cancel(self, check=lambda: None):
        from .idle_ros import hold_and_cancel

        return hold_and_cancel(self, check)

    def return_to_idle(self, plan, check=lambda: None):
        from .idle_ros import return_to_idle

        return return_to_idle(self, plan, check)

    def enable(self, check, on_motion):
        self.last_status = None
        deadline = time.monotonic() + 3
        while self.last_status is None and time.monotonic() < deadline:
            check()
            self.executor.spin_once(timeout_sec=0.1)
        state = self.last_status
        if not state:
            raise RuntimeError("未收到机械臂状态，请检查连接")
        if state.get("preflight_error"):
            raise RuntimeError(state["preflight_error"])
        if state.get("arm_status") != 0 or state.get("fault"):
            raise RuntimeError("机械臂处于停止或故障状态，请现场检查后恢复")
        check()
        if not state.get("session_enabled") and not self.enable_draw_service.service_is_ready():
            raise RuntimeError("机械臂仍由旧连接控制，请断开旧连接后重试")
        on_motion()
        if state.get("session_enabled"):
            if state.get("holding"):
                from .idle_ros import trigger

                trigger(self, "/piper/resume_commands", check)
            return
        result = self.call(self.enable_draw_service, Trigger.Request(), 5, check)
        if not result or not result.success:
            raise RuntimeError(result.message if result else "未确认机械臂使能")

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.try_shutdown()
