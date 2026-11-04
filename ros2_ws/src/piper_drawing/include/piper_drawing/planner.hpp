#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <utility>
#include "piper_drawing/job.hpp"

namespace drawing {
using Plan = moveit::planning_interface::MoveGroupInterface::Plan;
struct PlannedSegment { Segment segment; Plan plan; };

class Planner {
public:
  Planner(rclcpp::Node::SharedPtr node, const Job& job);
  std::vector<PlannedSegment> plan();
  void execute(const PlannedSegment& segment);
  moveit::core::RobotModelConstPtr robot_model() const { return arm_.getRobotModel(); }
  void stop() { arm_.stop(); }
private:
  std::pair<double, double> joint_scaling(const Segment& segment) const;
  bool valid_state(moveit::core::RobotState* state, const moveit::core::JointModelGroup* group,
                   const double* joints) const;
  Plan cartesian(const Segment& segment, const moveit::core::RobotState& start);
  std::vector<PlannedSegment> plan_tail(moveit::core::RobotState state);
  void limit_speed(Plan& plan, const moveit::core::RobotState& start, double speed);
  Eigen::Vector3d tip(const moveit::core::RobotState& state) const;
  rclcpp::Node::SharedPtr node_;
  const Job& job_;
  moveit::planning_interface::MoveGroupInterface arm_;
  planning_scene::PlanningScenePtr scene_;
};

void vendor_preflight(rclcpp::Node::SharedPtr node, const Job& job, bool execute);
void check_backend(rclcpp::Node::SharedPtr node, const std::string& mode);
}  // namespace drawing
