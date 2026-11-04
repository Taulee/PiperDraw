#include "piper_drawing/planner.hpp"
#include "piper_drawing/service.hpp"
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <chrono>
#include <optional>

namespace drawing {
namespace {
class IncompleteCartesianPath : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};
}  // namespace

Planner::Planner(rclcpp::Node::SharedPtr node, const Job& job)
    : node_(node), job_(job), arm_(node, job.group) {
  arm_.setPoseReferenceFrame(job.base);
  arm_.setEndEffectorLink(job.flange);
  arm_.setPlanningTime(10.0);
  arm_.startStateMonitor(10.0);
  moveit_msgs::srv::ApplyPlanningScene::Request update;
  update.scene = job.scene();
  // 场景对象使用固定 ID，重复设置与读取均幂等；仅重发场景请求，不重发机械运动。
  if (!call<moveit_msgs::srv::ApplyPlanningScene>(node, "/apply_planning_scene", update, 2)->success)
    throw std::runtime_error("无法设置纸面和笔杆场景");
  moveit_msgs::srv::GetPlanningScene::Request request;
  request.components.components = 1023;
  auto response = call<moveit_msgs::srv::GetPlanningScene>(node, "/get_planning_scene", request, 2);
  scene_ = std::make_shared<planning_scene::PlanningScene>(arm_.getRobotModel());
  scene_->setPlanningSceneMsg(response->scene);
}

Eigen::Vector3d Planner::tip(const moveit::core::RobotState& state) const {
  return state.getGlobalLinkTransform(job_.flange) * job_.tcp_offset;
}

bool Planner::valid_state(moveit::core::RobotState* state,
                          const moveit::core::JointModelGroup* group,
                          const double* joints) const {
  state->setJointGroupPositions(group, joints);
  state->update();
  return state->satisfiesBounds(group) && !scene_->isStateColliding(*state, job_.group);
}

std::pair<double, double> Planner::joint_scaling(const Segment& segment) const {
  double velocity = job_.motion("velocity_scale");
  double acceleration = job_.motion("acceleration_scale");
  if (segment.kind == "approach" || segment.kind == "transit" ||
      segment.kind == "lift" || segment.kind == "initial_lift") {
    const auto motion = job_.config["motion"];
    // 旧任务没有分动作缩放，继续使用任务中原有的基础缩放。
    if (motion["travel_velocity_scale"]) velocity = motion["travel_velocity_scale"].as<double>();
    if (motion["travel_acceleration_scale"]) acceleration = motion["travel_acceleration_scale"].as<double>();
  }
  if (segment.kind == "lift" || segment.kind == "initial_lift") {
    const auto motion = job_.config["motion"];
    // 抬笔可独立提高加速度；缺少字段时保留旧任务的空移缩放。
    if (motion["lift_velocity_scale"]) velocity = motion["lift_velocity_scale"].as<double>();
    if (motion["lift_acceleration_scale"]) acceleration = motion["lift_acceleration_scale"].as<double>();
  }
  return {velocity, acceleration};
}

Plan Planner::cartesian(const Segment& segment, const moveit::core::RobotState& start) {
  auto state = start;
  const auto* group = state.getJointModelGroup(job_.group);
  std::vector<moveit::core::RobotStatePtr> states;
  auto valid = [this](moveit::core::RobotState* candidate, const moveit::core::JointModelGroup* g,
                     const double* joints) {
    return valid_state(candidate, g, joints);
  };
  const double fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
      &state, group, states, state.getLinkModel(job_.flange), segment.poses, true,
      moveit::core::MaxEEFStep(job_.motion("cartesian_step_mm") / 1000.0),
      moveit::core::JumpThreshold(job_.motion("max_joint_step_rad"), 0.0), valid);
  if (fraction < 1.0 - 1e-9)
    throw IncompleteCartesianPath("笛卡尔路径不完整：第 " + std::to_string(segment.stroke) + " 笔 " +
                                  segment.kind + "，完成比例 " + std::to_string(fraction));
  robot_trajectory::RobotTrajectory trajectory(arm_.getRobotModel(), job_.group);
  // 相邻笔画可能共享端点，去掉零位移采样，避免控制器收到重复时间戳。
  for (const auto& point : states)
    if (trajectory.empty() || trajectory.getLastWayPoint().distance(*point) > 1e-8)
      trajectory.addSuffixWayPoint(*point, 0.0);
  trajectory_processing::IterativeParabolicTimeParameterization timing;
  const auto [velocity, acceleration] = joint_scaling(segment);
  if (trajectory.getWayPointCount() > 1 &&
      !timing.computeTimeStamps(trajectory, velocity, acceleration))
    throw std::runtime_error("轨迹时间参数化失败");
  Plan plan;
  moveit::core::robotStateToRobotStateMsg(start, plan.start_state_);
  trajectory.getRobotTrajectoryMsg(plan.trajectory_);
  return plan;
}

void Planner::limit_speed(Plan& plan, const moveit::core::RobotState& start, double speed) {
  auto state = start;
  Eigen::Vector3d previous = tip(start);
  double previous_time = 0.0, factor = 1.0;
  auto& trajectory = plan.trajectory_.joint_trajectory;
  if (trajectory.points.empty()) throw std::runtime_error("规划返回空轨迹");
  for (const auto& point : trajectory.points) {
    state.setVariablePositions(trajectory.joint_names, point.positions);
    state.update();
    const double t = rclcpp::Duration(point.time_from_start).seconds();
    const Eigen::Vector3d current_tip = tip(state);
    const double distance = (current_tip - previous).norm();
    if (t > previous_time) factor = std::max(factor, distance / (t - previous_time) / speed);
    else if (distance > 1e-6) throw std::runtime_error("轨迹时间未递增");
    previous = current_tip; previous_time = t;
  }
  for (auto& point : trajectory.points) {
    point.time_from_start = rclcpp::Duration::from_seconds(
        rclcpp::Duration(point.time_from_start).seconds() * factor);
    for (auto& v : point.velocities) v /= factor;
    for (auto& a : point.accelerations) a /= factor * factor;
  }
}

std::vector<PlannedSegment> Planner::plan_tail(moveit::core::RobotState state) {
  std::vector<PlannedSegment> result;
  result.reserve(job_.segments.size() - 1);
  for (size_t i = 1; i < job_.segments.size(); ++i) {
    const auto& segment = job_.segments[i];
    auto path = cartesian(segment, state);
    if (path.trajectory_.joint_trajectory.points.size() == 1) continue;
    limit_speed(path, state, segment.speed);
    const auto& trajectory = path.trajectory_.joint_trajectory;
    state.setVariablePositions(trajectory.joint_names, trajectory.points.back().positions);
    state.update();
    result.push_back({segment, std::move(path)});
  }
  return result;
}

std::vector<PlannedSegment> Planner::plan() {
  if (job_.segments.empty() || job_.segments.front().kind != "approach")
    throw std::runtime_error("任务必须以 approach 段开始");
  const auto current = arm_.getCurrentState(10.0);
  if (!current) throw std::runtime_error("未收到机械臂关节状态");
  auto state = *current;
  state.update();
  scene_->getCurrentStateNonConst().update();
  // 让离线 RobotState 同样携带笔杆，供笛卡尔求解回调检查碰撞。
  const auto joints = state.getVariablePositions();
  auto with_tool = scene_->getCurrentState();
  with_tool.setVariablePositions(joints);
  state = with_tool;
  state.update();
  if (scene_->isStateColliding(state, job_.group)) throw std::runtime_error("起始状态发生碰撞");
  const auto seed = state;
  std::vector<PlannedSegment> result;
  result.reserve(job_.segments.size() + 1);
  const double height = (tip(state) - job_.origin).dot(job_.normal);
  const double lift = job_.motion("lift_mm") / 1000.0;
  if (height < lift) {
    Eigen::Isometry3d pose = state.getGlobalLinkTransform(job_.flange);
    pose.translation() += (lift - height) * job_.normal;
    const auto motion = job_.config["motion"];
    const double lift_speed = motion["lift_mm_s"] ? motion["lift_mm_s"].as<double>() : job_.motion("lower_mm_s");
    Segment segment{"initial_lift", -1, lift_speed / 1000.0, {pose}};
    auto path = cartesian(segment, state);
    if (path.trajectory_.joint_trajectory.points.size() > 1) {
      limit_speed(path, state, segment.speed);
      const auto& trajectory = path.trajectory_.joint_trajectory;
      state.setVariablePositions(trajectory.joint_names, trajectory.points.back().positions);
      state.update();
      result.push_back({std::move(segment), std::move(path)});
    }
  }

  const auto& approach = job_.segments.front();
  const auto* group = seed.getJointModelGroup(job_.group);
  auto valid = [this](moveit::core::RobotState* candidate, const moveit::core::JointModelGroup* g,
                     const double* joints) {
    return valid_state(candidate, g, joints);
  };
  std::optional<moveit::core::RobotState> approach_goal;
  std::vector<PlannedSegment> tail;
  std::string last_error;
  constexpr unsigned int max_candidates = 32;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  for (unsigned int attempt = 0; attempt < max_candidates &&
       std::chrono::steady_clock::now() < deadline; ++attempt) {
    auto goal = seed;
    // 先用真实起点作为逆解种子；失败后在模型关节范围内寻找其他候选。
    if (attempt > 0) goal.setToRandomPositions(group);
    if (!goal.setFromIK(group, approach.poses.back(), job_.flange,
                        group->getDefaultIKTimeout(), valid)) {
      last_error = "首点未找到满足关节限位和碰撞约束的精确逆解";
      RCLCPP_WARN(node_->get_logger(), "首笔候选 %u/%u：%s，重选逆解分支",
                  attempt + 1, max_candidates, last_error.c_str());
      continue;
    }
    goal.update();
    try {
      tail = plan_tail(goal);
    } catch (const IncompleteCartesianPath& error) {
      last_error = error.what();
      RCLCPP_WARN(node_->get_logger(), "首笔候选 %u/%u：%s，重选逆解分支",
                  attempt + 1, max_candidates, last_error.c_str());
      continue;
    }
    approach_goal = std::move(goal);
    RCLCPP_INFO(node_->get_logger(), "首笔候选 %u：后续 %zu 段全部规划成功",
                attempt + 1, tail.size());
    break;
  }
  if (!approach_goal)
    throw std::runtime_error("首笔逆解分支筛选失败（最多 32 次，10 秒搜索预算）：" + last_error);

  // 固定已验证的关节目标，避免 OMPL 再选择无法完成后续绘画的逆解分支。
  arm_.setStartState(state);
  if (!arm_.setJointValueTarget(*approach_goal))
    throw std::runtime_error("首笔关节目标超出关节限位");
  Plan path;
  const auto [velocity, acceleration] = joint_scaling(approach);
  arm_.setMaxVelocityScalingFactor(velocity);
  arm_.setMaxAccelerationScalingFactor(acceleration);
  if (arm_.plan(path) != moveit::core::MoveItErrorCode::SUCCESS)
    throw std::runtime_error("无法规划到已验证的首笔关节目标");
  limit_speed(path, state, approach.speed);
  result.push_back({approach, std::move(path)});
  for (auto& segment : tail) result.push_back(std::move(segment));
  for (const auto& item : result)
    RCLCPP_INFO(node_->get_logger(), "第 %d 笔 %s：%zu 个轨迹点", item.segment.stroke,
                item.segment.kind.c_str(), item.plan.trajectory_.joint_trajectory.points.size());
  return result;
}

void Planner::execute(const PlannedSegment& segment) {
  const auto current = arm_.getCurrentState(5.0);
  if (!current) throw std::runtime_error("执行前未收到关节反馈");
  const auto& trajectory = segment.plan.trajectory_.joint_trajectory;
  for (size_t i = 0; i < trajectory.joint_names.size(); ++i)
    if (std::abs(current->getVariablePosition(trajectory.joint_names[i]) - trajectory.points.front().positions[i]) > job_.motion("joint_tolerance_rad"))
      throw std::runtime_error("机械臂当前位置偏离已规划起点，请重新规划");
  if (arm_.execute(segment.plan) != moveit::core::MoveItErrorCode::SUCCESS)
    throw std::runtime_error("轨迹执行失败或被取消");
}
}  // namespace drawing
