"""运行中 ROS 后端的操作接口；返回数据供 CLI 或 Web 后端使用。"""

import json
import time


def command(action, *, width_mm=None, torque_nm=0.3):
    if action not in ("status", "enable", "stop", "gripper", "hold", "resume_commands"):
        raise ValueError(f"未知 ROS 设备操作：{action}")
    import rclpy
    from piper_drawing.srv import SetGripper
    from std_msgs.msg import String
    from std_srvs.srv import SetBool, Trigger

    rclpy.init()
    node = rclpy.create_node("piper_operator")
    try:
        if action == "status":
            messages = []
            node.create_subscription(
                String, "/piper/status", lambda msg: messages.append(msg.data), 1
            )
            deadline = time.monotonic() + 3
            while not messages and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            if not messages:
                raise RuntimeError("3 秒内未收到 Piper 后端状态；先启动 real 后端")
            return json.loads(messages[-1])
        if action == "gripper":
            service, name = SetGripper, "/piper/set_gripper"
            request = service.Request(width_mm=width_mm, torque_nm=torque_nm)
        elif action == "enable":
            service, name = SetBool, "/piper/enable"
            request = service.Request(data=True)
        elif action == "resume_commands":
            service, name = Trigger, "/piper/resume_commands"
            request = service.Request()
        else:
            service, name = Trigger, "/piper/hold"
            request = service.Request()
        client = node.create_client(service, name)
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError(f"服务未就绪：{name}；先启动 real 后端")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done():
            raise RuntimeError(f"{name} 响应超时；操作结果未知，请读取 status，勿盲目重发")
        response = future.result()
        if not response.success:
            raise RuntimeError(response.message)
        return {"status": "submitted", "message": response.message}
    finally:
        node.destroy_node()
        rclpy.shutdown()
