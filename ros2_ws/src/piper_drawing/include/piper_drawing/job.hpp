#pragma once

#include <Eigen/Geometry>
#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>

namespace drawing {
struct Segment {
  std::string kind;
  int stroke;
  double speed;
  EigenSTL::vector_Isometry3d poses;
};

struct Job {
  explicit Job(const std::string& path);
  YAML::Node config;
  std::string group, base, flange;
  Eigen::Vector3d tcp_offset, origin, normal;
  std::vector<Segment> segments;
  moveit_msgs::msg::PlanningScene scene() const;
  double motion(const std::string& name) const { return config["motion"][name].as<double>(); }
};

Eigen::Isometry3d transform(const YAML::Node& values);
geometry_msgs::msg::Pose pose_msg(const Eigen::Isometry3d& pose);
}  // namespace drawing
