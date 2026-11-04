#pragma once
#include <rclcpp/rclcpp.hpp>
#include <chrono>

namespace drawing {
template<class Service>
typename Service::Response::SharedPtr call(rclcpp::Node::SharedPtr node,
    const std::string& name, const typename Service::Request& request,
    unsigned int max_attempts = 1) {
  using namespace std::chrono_literals;
  auto client = node->create_client<Service>(name);
  if (!client->wait_for_service(10s)) throw std::runtime_error("服务未就绪：" + name);
  auto shared_request = std::make_shared<typename Service::Request>(request);
  for (unsigned int attempt = 0; attempt < max_attempts; ++attempt) {
    auto future = client->async_send_request(shared_request);
    if (future.wait_for(15s) == std::future_status::ready) return future.get();
    client->remove_pending_request(future);
    if (attempt + 1 < max_attempts)
      RCLCPP_WARN(node->get_logger(), "服务响应超时：%s，复用客户端重试（第 %u/%u 次调用）",
                  name.c_str(), attempt + 2, max_attempts);
  }
  throw std::runtime_error("服务超时：" + name);
}
}  // namespace drawing
