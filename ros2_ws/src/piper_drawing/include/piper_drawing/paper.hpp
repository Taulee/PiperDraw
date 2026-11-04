#pragma once

#include <Eigen/Geometry>
#include <visualization_msgs/msg/marker.hpp>
#include <stdexcept>
#include <string>

namespace drawing {

// 输入纸张上表面的中心和姿态；启动场景与任务快照共用同一薄纸几何。
inline visualization_msgs::msg::Marker paper_marker(
    const Eigen::Isometry3d& surface, const Eigen::Vector2d& size, const std::string& base) {
  constexpr double thickness = 0.0002;
  if (!surface.matrix().allFinite() || !size.allFinite() || (size.array() <= 0.0).any() ||
      base.empty())
    throw std::runtime_error("纸面尺寸、姿态或参考系无效");
  const Eigen::Vector3d center = surface.translation() - surface.linear().col(2) * thickness / 2.0;
  const Eigen::Quaterniond orientation(surface.linear());
  visualization_msgs::msg::Marker paper;
  paper.header.frame_id = base;
  paper.ns = "drawing_paper";
  paper.id = 0;
  paper.type = visualization_msgs::msg::Marker::CUBE;
  paper.action = visualization_msgs::msg::Marker::ADD;
  paper.pose.position.x = center.x();
  paper.pose.position.y = center.y();
  paper.pose.position.z = center.z();
  paper.pose.orientation.x = orientation.x();
  paper.pose.orientation.y = orientation.y();
  paper.pose.orientation.z = orientation.z();
  paper.pose.orientation.w = orientation.w();
  paper.scale.x = size.x();
  paper.scale.y = size.y();
  paper.scale.z = thickness;
  paper.color.r = paper.color.g = paper.color.b = paper.color.a = 1.0f;
  paper.frame_locked = false;
  return paper;
}

}  // drawing 命名空间
