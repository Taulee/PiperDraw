#include "piper_drawing/service.hpp"
#include <gtest/gtest.h>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <algorithm>
#include <iterator>
#include <mutex>
#include <thread>
#include <vector>

namespace {
using Service = moveit_msgs::srv::ApplyPlanningScene;

struct ReceivedRequest {
  rmw_request_id_t header;
  Service::Request request;
};

class ServiceCallTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    client_node_ = rclcpp::Node::make_shared("service_test_client");
    server_node_ = rclcpp::Node::make_shared("service_test_server");
    executor_.add_node(client_node_);
    executor_.add_node(server_node_);
    request_.scene.name = "drawing_test_scene";
    request_.scene.is_diff = true;
  }

  void TearDown() override {
    executor_.cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
  }

  void start_service(unsigned int first_response_request, bool success = true) {
    service_ = server_node_->create_service<Service>(service_name_,
        [this, first_response_request, success](
            std::shared_ptr<rclcpp::Service<Service>> service,
            std::shared_ptr<rmw_request_id_t> header,
            Service::Request::SharedPtr request) {
          size_t request_number;
          {
            std::lock_guard<std::mutex> lock(requests_mutex_);
            requests_.push_back({*header, *request});
            request_number = requests_.size();
          }
          // 延迟回复回调不会自动发送响应，借此确定性模拟 DDS 丢失回复。
          if (first_response_request == 0) return;
          if (request_number < first_response_request) {
            lost_response_header_ = header;
            return;
          }
          if (lost_response_header_) {
            // 旧请求迟到的失败响应不能被当成重试请求的结果。
            Service::Response late_response;
            late_response.success = false;
            service->send_response(*lost_response_header_, late_response);
            lost_response_header_.reset();
          }
          Service::Response response;
          response.success = success;
          service->send_response(*header, response);
        });
    spin_thread_ = std::thread([this] { executor_.spin(); });
  }

  std::vector<ReceivedRequest> received_requests() {
    std::lock_guard<std::mutex> lock(requests_mutex_);
    return requests_;
  }

  rclcpp::Node::SharedPtr client_node_;
  Service::Request request_;
  const std::string service_name_ = "/test_apply_planning_scene";

private:
  rclcpp::Node::SharedPtr server_node_;
  rclcpp::Service<Service>::SharedPtr service_;
  std::shared_ptr<rmw_request_id_t> lost_response_header_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  std::mutex requests_mutex_;
  std::vector<ReceivedRequest> requests_;
};

TEST_F(ServiceCallTest, RetriesLostResponseWithSameClientAndRequest) {
  start_service(2);
  auto response = drawing::call<Service>(client_node_, service_name_, request_, 2);
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);

  auto requests = received_requests();
  ASSERT_EQ(requests.size(), 2u);
  for (const auto& received : requests) {
    EXPECT_EQ(received.request.scene.name, request_.scene.name);
    EXPECT_EQ(received.request.scene.is_diff, request_.scene.is_diff);
  }
  EXPECT_TRUE(std::equal(std::begin(requests[0].header.writer_guid),
                        std::end(requests[0].header.writer_guid),
                        std::begin(requests[1].header.writer_guid)));
  EXPECT_NE(requests[0].header.sequence_number, requests[1].header.sequence_number);
}

TEST_F(ServiceCallTest, DefaultCallDoesNotRetryLostResponse) {
  start_service(2);
  EXPECT_THROW(drawing::call<Service>(client_node_, service_name_, request_), std::runtime_error);
  EXPECT_EQ(received_requests().size(), 1u);
}

TEST_F(ServiceCallTest, StopsAtRetryLimitWhenAllResponsesAreLost) {
  start_service(0);
  EXPECT_THROW(drawing::call<Service>(client_node_, service_name_, request_, 2), std::runtime_error);
  EXPECT_EQ(received_requests().size(), 2u);
}

TEST_F(ServiceCallTest, ReturnsBusinessFailureWithoutRetry) {
  start_service(1, false);
  auto response = drawing::call<Service>(client_node_, service_name_, request_, 2);
  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success);
  EXPECT_EQ(received_requests().size(), 1u);
}
}  // namespace
