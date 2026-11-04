#include <hardware_interface/system_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>

namespace drawing {
class PiperHardware : public hardware_interface::SystemInterface {
public:
  ~PiperHardware() override {
    if (executor_) executor_->cancel();
    if (thread_.joinable()) thread_.join();
  }

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override {
    if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
      return hardware_interface::CallbackReturn::ERROR;
    if (info_.joints.size() != 8) return hardware_interface::CallbackReturn::ERROR;
    positions_.assign(8, 0.0); velocities_.assign(8, 0.0); feedback_.assign(8, 0.0);
    commands_.assign(6, 0.0);
    node_ = std::make_shared<rclcpp::Node>("piper_hardware");
    publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("/piper/joint_commands", 1);
    stop_ = node_->create_client<std_srvs::srv::Trigger>("/piper/stop");
    subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/piper/raw_joint_states", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          std::vector<double> values;
          for (const auto& joint : info_.joints) {
            const auto found = std::find(msg->name.begin(), msg->name.end(), joint.name);
            const auto index = std::distance(msg->name.begin(), found);
            if (found == msg->name.end() || static_cast<size_t>(index) >= msg->position.size() ||
                !std::isfinite(msg->position[index])) return;
            values.push_back(msg->position[index]);
          }
          std::lock_guard<std::mutex> lock(mutex_);
          feedback_ = std::move(values);
          last_feedback_ = std::chrono::steady_clock::now();
          received_ = true;
        });
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    thread_ = std::thread([this] { executor_->spin(); });
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> states;
    for (size_t i = 0; i < positions_.size(); ++i) {
      states.emplace_back(info_.joints[i].name, "position", &positions_[i]);
      states.emplace_back(info_.joints[i].name, "velocity", &velocities_[i]);
    }
    return states;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> commands;
    for (size_t i = 0; i < commands_.size(); ++i)
      commands.emplace_back(info_.joints[i].name, "position", &commands_[i]);
    return commands;
  }

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (fresh()) {
          positions_ = feedback_;
          std::copy_n(positions_.begin(), 6, commands_.begin());
          return hardware_interface::CallbackReturn::SUCCESS;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    RCLCPP_ERROR(node_->get_logger(), "10 秒内未收到 Piper 完整关节和夹爪反馈");
    return hardware_interface::CallbackReturn::ERROR;
  }

  hardware_interface::return_type read(const rclcpp::Time&, const rclcpp::Duration& period) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!fresh()) {
      request_stop();
      return hardware_interface::return_type::ERROR;
    }
    // 速度由真实位置差分获得，不使用指令位置伪造反馈。
    for (size_t i = 0; i < positions_.size(); ++i)
      velocities_[i] = period.seconds() > 0 ? (feedback_[i] - positions_[i]) / period.seconds() : 0.0;
    positions_ = feedback_;
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration&) override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!fresh()) return hardware_interface::return_type::ERROR;
    }
    sensor_msgs::msg::JointState message;
    message.header.stamp = node_->now();
    for (size_t i = 0; i < commands_.size(); ++i) {
      if (!std::isfinite(commands_[i])) { request_stop(); return hardware_interface::return_type::ERROR; }
      message.name.push_back(info_.joints[i].name);
      message.position.push_back(commands_[i]);
    }
    publisher_->publish(message);
    return hardware_interface::return_type::OK;
  }

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
    request_stop();
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State&) override {
    request_stop();
    return hardware_interface::CallbackReturn::SUCCESS;
  }

private:
  bool fresh() const {
    return received_ && std::chrono::steady_clock::now() - last_feedback_ < std::chrono::milliseconds(500);
  }
  void request_stop() {
    if (stop_->service_is_ready()) {
      auto future = stop_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      stop_->remove_pending_request(future);
    }
  }
  std::vector<double> positions_, velocities_, commands_, feedback_;
  bool received_ = false;
  std::chrono::steady_clock::time_point last_feedback_;
  std::mutex mutex_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread thread_;
};
}  // namespace drawing
PLUGINLIB_EXPORT_CLASS(drawing::PiperHardware, hardware_interface::SystemInterface)
