#include "piper_drawing/planner.hpp"
#include "piper_drawing/service.hpp"
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace drawing {
void check_backend(rclcpp::Node::SharedPtr node, const std::string& mode) {
  rcl_interfaces::srv::GetParameters::Request request;
  request.names = {"robot_description"};
  const auto response = call<rcl_interfaces::srv::GetParameters>(
      node, "/controller_manager/get_parameters", request, 2);
  const auto& urdf = response->values.at(0).string_value;
  const std::string plugin = mode == "real" ? "piper_drawing/PiperHardware" : "mock_components/GenericSystem";
  if (urdf.find(plugin) == std::string::npos)
    throw std::runtime_error("运行中的控制器与指定模式不一致：" + mode);
  if (urdf != node->get_parameter("robot_description").as_string())
    throw std::runtime_error("任务模型与当前后端不一致，请按任务配置重启后端");
}

void vendor_preflight(rclcpp::Node::SharedPtr node, const Job&, bool execute) {
  const auto response = call<std_srvs::srv::Trigger>(
      node, execute ? "/piper/preflight" : "/piper/check_feedback", std_srvs::srv::Trigger::Request());
  if (!response->success) throw std::runtime_error("Piper 真机预检失败：" + response->message);
  RCLCPP_INFO(node->get_logger(), "Piper 反馈、固件、使能检查通过；逆解和碰撞由官方 MoveIt/KDL 模型完成");
}
}  // namespace drawing
