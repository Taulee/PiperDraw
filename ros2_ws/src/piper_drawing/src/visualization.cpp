#include "piper_drawing/visualization.hpp"
#include "piper_drawing/paper.hpp"

#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace drawing {
namespace {
using SteadyClock = std::chrono::steady_clock;
constexpr double kContactDistance = 0.0005;
constexpr double kInkOffset = 0.0001;
constexpr double kInkWidth = 0.0007;
constexpr double kPointSpacing = 0.0002;
constexpr auto kSampleInterval = std::chrono::milliseconds(10);
constexpr auto kPublishInterval = std::chrono::milliseconds(200);
constexpr auto kFeedbackFreshness = std::chrono::milliseconds(250);
constexpr std::size_t kNoStrip = std::numeric_limits<std::size_t>::max();

geometry_msgs::msg::Point point_msg(const Eigen::Vector3d& point) {
  geometry_msgs::msg::Point result;
  result.x = point.x();
  result.y = point.y();
  result.z = point.z();
  return result;
}
}  // namespace

struct DrawingVisualization::Impl : std::enable_shared_from_this<DrawingVisualization::Impl> {
  Impl(rclcpp::Node::SharedPtr node, const Job& job, moveit::core::RobotModelConstPtr model)
      : node_(std::move(node)), base_(job.base), flange_(job.flange),
        tcp_offset_(job.tcp_offset), origin_(job.origin), normal_(job.normal) {
    const auto* group = model->getJointModelGroup(job.group);
    if (!group || !model->getLinkModel(flange_))
      throw std::runtime_error("笔迹显示的关节组或法兰不存在");
    if (base_ != model->getModelFrame() && !model->getLinkModel(base_))
      throw std::runtime_error("笔迹显示的基坐标不存在于机器人模型");
    if (!tcp_offset_.allFinite() || !origin_.allFinite() || !normal_.allFinite() ||
        normal_.norm() < 1e-12)
      throw std::runtime_error("笔迹显示的笔尖或纸面参数无效");
    normal_.normalize();
    for (const auto* joint : group->getActiveJointModels())
      for (const auto& variable : joint->getVariableNames()) required_variables_.push_back(variable);
    if (required_variables_.empty()) throw std::runtime_error("笔迹显示的关节组没有活动变量");
    state_ = std::make_unique<moveit::core::RobotState>(model);
    state_->setToDefaultValues();

    const auto table = job.config["table"];
    const Eigen::Isometry3d table_pose = transform(table["pose"]);
    Eigen::Vector3d size;
    for (int axis = 0; axis < 3; ++axis) size[axis] = table["size"][axis].as<double>();
    if (!table_pose.matrix().allFinite() || !size.allFinite() || (size.array() <= 0.0).any())
      throw std::runtime_error("笔迹显示的纸面尺寸或姿态无效");
    Eigen::Isometry3d surface = table_pose;
    surface.translation() += table_pose.linear().col(2) * size.z() / 2.0;
    paper_ = paper_marker(surface, size.head<2>(), base_);
    publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/drawing_task_markers", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  }

  void start() {
    // 回调只持有弱引用，执行中的回调会保持对象存活，避免入口退出时悬空。
    const std::weak_ptr<Impl> weak = shared_from_this();
    subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(),
        [weak](sensor_msgs::msg::JointState::ConstSharedPtr message) {
          if (const auto self = weak.lock())
            self->observe("关节反馈", [&] { self->feedback(*message); });
        });
    timer_ = node_->create_wall_timer(kPublishInterval, [weak] {
      if (const auto self = weak.lock()) {
        self->observe("快照发布", [&] {
          std::lock_guard<std::mutex> lock(self->mutex_);
          // 反馈空窗也断开线段，恢复反馈后不会横跨未观测的运动。
          if (!self->fresh(SteadyClock::now())) self->current_strip_ = kNoStrip;
          if (self->dirty_) self->publish();
        });
      }
    });
    std::lock_guard<std::mutex> lock(mutex_);
    publish();
  }

  // 显示回调统一记录失败，避免干扰轨迹执行。
  template<class Action>
  void observe(const char* operation, Action action) {
    try {
      action();
    } catch (const std::exception& error) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                           "笔迹显示%s失败：%s", operation, error.what());
    }
  }

  bool current_stamp(std::int64_t stamp_ns) const {
    return stamp_ns == 0 || std::abs(node_->now().nanoseconds() - stamp_ns) <=
        std::chrono::duration_cast<std::chrono::nanoseconds>(kFeedbackFreshness).count();
  }

  bool fresh(const SteadyClock::time_point now) const {
    return has_feedback_ && now - last_feedback_ <= kFeedbackFreshness &&
           current_stamp(last_stamp_ns_);
  }

  void feedback(const sensor_msgs::msg::JointState& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (message.name.size() != message.position.size() || message.name.empty() ||
        message.header.stamp.nanosec >= 1000000000u || message.header.stamp.sec < 0)
      return;
    std::unordered_map<std::string, double> positions;
    positions.reserve(message.name.size());
    for (std::size_t index = 0; index < message.name.size(); ++index) {
      if (!std::isfinite(message.position[index]) ||
          !positions.emplace(message.name[index], message.position[index]).second)
        return;
    }
    for (const auto& variable : required_variables_)
      if (positions.find(variable) == positions.end()) return;

    const std::int64_t stamp_ns = static_cast<std::int64_t>(message.header.stamp.sec) * 1000000000LL +
                                  message.header.stamp.nanosec;
    const auto now = SteadyClock::now();
    // 同时检查接收时间和消息时间，积压的反馈不能在新笔画上留下旧墨线。
    if (!current_stamp(stamp_ns) || (stamp_ns > 0 && stamp_ns < last_stamp_ns_)) return;
    if (!fresh(now)) current_strip_ = kNoStrip;
    for (const auto& variable : required_variables_)
      state_->setVariablePosition(variable, positions.at(variable));
    state_->update();
    const Eigen::Isometry3d model_to_base = state_->getFrameTransform(base_);
    // MoveIt 正解以模型根为参考，先转入任务基坐标，再施加法兰中的 TCP 偏移。
    const Eigen::Vector3d tip =
        (model_to_base.inverse() * state_->getGlobalLinkTransform(flange_)) * tcp_offset_;
    if (!tip.allFinite()) return;
    latest_tip_ = tip;
    last_feedback_ = now;
    last_stamp_ns_ = stamp_ns;
    has_feedback_ = true;
    if (!drawing_) return;
    const double height = (latest_tip_ - origin_).dot(normal_);
    if (std::abs(height) > kContactDistance) {
      current_strip_ = kNoStrip;
      return;
    }
    if (now - last_sample_ >= kSampleInterval) sample(now, false);
  }

  void sample(const SteadyClock::time_point now, const bool endpoint) {
    if (!drawing_ || !fresh(now)) {
      current_strip_ = kNoStrip;
      return;
    }
    const double height = (latest_tip_ - origin_).dot(normal_);
    if (std::abs(height) > kContactDistance) {
      current_strip_ = kNoStrip;
      return;
    }
    const Eigen::Vector3d point = latest_tip_ + (kInkOffset - height) * normal_;
    if (current_strip_ == kNoStrip) {
      strips_.emplace_back();
      current_strip_ = strips_.size() - 1;
    }
    auto& points = strips_[current_strip_];
    if (!points.empty()) {
      const auto& previous = points.back();
      const double distance = (point - Eigen::Vector3d(previous.x, previous.y, previous.z)).norm();
      if (distance < (endpoint ? 1e-9 : kPointSpacing)) return;
    }
    points.push_back(point_msg(point));
    last_sample_ = now;
    dirty_ = true;
  }

  void begin(const Segment& segment) {
    std::lock_guard<std::mutex> lock(mutex_);
    drawing_ = segment.kind == "draw";
    current_strip_ = kNoStrip;
    last_sample_ = SteadyClock::time_point{};
    if (drawing_) sample(SteadyClock::now(), true);
  }

  void end() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (drawing_) sample(SteadyClock::now(), true);
    drawing_ = false;
    current_strip_ = kNoStrip;
    publish();
  }

  void publish() {
    visualization_msgs::msg::MarkerArray snapshot;
    snapshot.markers.reserve(strips_.size() + 2);
    const auto stamp = node_->now();
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = base_;
    clear.header.stamp = stamp;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    snapshot.markers.push_back(std::move(clear));
    auto paper = paper_;
    paper.header.frame_id = base_;
    paper.header.stamp = stamp;
    snapshot.markers.push_back(std::move(paper));
    for (std::size_t index = 0; index < strips_.size(); ++index) {
      // 单个接触点不能构成线条；保留它，第二个真实采样到来后再显示。
      if (strips_[index].size() < 2) continue;
      visualization_msgs::msg::Marker line;
      line.header.frame_id = base_;
      line.header.stamp = stamp;
      line.ns = "drawing_ink";
      line.id = static_cast<std::int32_t>(index);
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.pose.orientation.w = 1.0;
      line.scale.x = kInkWidth;
      line.color.r = line.color.g = line.color.b = 0.03f;
      line.color.a = 1.0f;
      line.frame_locked = false;
      line.points = strips_[index];
      snapshot.markers.push_back(std::move(line));
    }
    publisher_->publish(snapshot);
    dirty_ = false;
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<moveit::core::RobotState> state_;
  std::string base_, flange_;
  Eigen::Vector3d tcp_offset_, origin_, normal_, latest_tip_ = Eigen::Vector3d::Zero();
  std::vector<std::string> required_variables_;
  visualization_msgs::msg::Marker paper_;
  std::vector<std::vector<geometry_msgs::msg::Point>> strips_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
  std::size_t current_strip_ = kNoStrip;
  SteadyClock::time_point last_feedback_{}, last_sample_{};
  std::int64_t last_stamp_ns_ = 0;
  bool drawing_ = false, has_feedback_ = false, dirty_ = false;
};

DrawingVisualization::DrawingVisualization(rclcpp::Node::SharedPtr node, const Job& job,
                                         moveit::core::RobotModelConstPtr model)
    : impl_(std::make_shared<Impl>(std::move(node), job, std::move(model))) {
  impl_->start();
}

DrawingVisualization::~DrawingVisualization() = default;

void DrawingVisualization::begin(const Segment& segment) {
  impl_->observe("开始笔画", [&] { impl_->begin(segment); });
}

void DrawingVisualization::end() {
  impl_->observe("结束笔画", [&] { impl_->end(); });
}

}  // namespace drawing
