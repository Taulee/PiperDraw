#pragma once

#include <memory>
#include <moveit/robot_model/robot_model.h>
#include <rclcpp/rclcpp.hpp>
#include "piper_drawing/job.hpp"

namespace drawing {

// 只从关节反馈还原笔尖，不参与规划或机械臂控制。
class DrawingVisualization {
public:
  DrawingVisualization(rclcpp::Node::SharedPtr node, const Job& job,
                       moveit::core::RobotModelConstPtr model);
  ~DrawingVisualization();

  void begin(const Segment& segment);
  void end();

private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

}  // namespace drawing
