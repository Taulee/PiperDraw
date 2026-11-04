#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include "piper_drawing/job.hpp"
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <cmath>

namespace drawing {
Eigen::Isometry3d transform(const YAML::Node& p) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(p[0].as<double>(), p[1].as<double>(), p[2].as<double>());
  result.linear() = Eigen::Quaterniond(p[6].as<double>(), p[3].as<double>(),
                                      p[4].as<double>(), p[5].as<double>()).normalized().toRotationMatrix();
  return result;
}

geometry_msgs::msg::Pose pose_msg(const Eigen::Isometry3d& p) {
  geometry_msgs::msg::Pose msg;
  msg.position.x = p.translation().x();
  msg.position.y = p.translation().y();
  msg.position.z = p.translation().z();
  const Eigen::Quaterniond q(p.linear());
  msg.orientation.x = q.x(); msg.orientation.y = q.y();
  msg.orientation.z = q.z(); msg.orientation.w = q.w();
  return msg;
}

Job::Job(const std::string& path) : config(YAML::LoadFile(path)) {
  const auto motion = config["motion"];
  // MoveIt 会把无效缩放替换为默认值，必须在任务边界拒绝，避免意外加速。
  for (const auto* name : {"velocity_scale", "acceleration_scale",
                           "travel_velocity_scale", "travel_acceleration_scale",
                           "lift_velocity_scale", "lift_acceleration_scale"}) {
    if (std::string(name) != "velocity_scale" &&
        std::string(name) != "acceleration_scale" && !motion[name]) continue;
    const double value = motion[name].as<double>();
    if (!std::isfinite(value) || value <= 0.0 || value > 1.0)
      throw std::runtime_error(std::string("motion.") + name + " 必须在 (0, 1] 内");
  }
  if (motion["lift_mm_s"]) {
    const double value = motion["lift_mm_s"].as<double>();
    if (!std::isfinite(value) || value <= 0.0)
      throw std::runtime_error("motion.lift_mm_s 必须是有限正数");
  }
  group = config["robot"]["group"].as<std::string>();
  base = config["robot"]["base_link"].as<std::string>();
  flange = config["robot"]["flange_link"].as<std::string>();
  for (int i = 0; i < 3; ++i) {
    tcp_offset[i] = config["tool"]["tcp_offset_mm"][i].as<double>() / 1000.0;
    origin[i] = config["paper_origin"][i].as<double>();
    normal[i] = config["paper_normal"][i].as<double>();
  }
  segments.reserve(config["segments"].size());
  for (const auto& item : config["segments"]) {
    Segment s{item["kind"].as<std::string>(), item["stroke"].as<int>(), item["speed_m_s"].as<double>(), {}};
    if (!std::isfinite(s.speed) || s.speed <= 0.0)
      throw std::runtime_error("运动段 speed_m_s 必须是有限正数");
    s.poses.reserve(item["poses"].size());
    for (const auto& p : item["poses"]) s.poses.push_back(transform(p));
    segments.push_back(std::move(s));
  }
}

moveit_msgs::msg::PlanningScene Job::scene() const {
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.robot_state.is_diff = true;
  auto box = [&](const std::string& id, const YAML::Node& data) {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = base; object.id = id;
    object.operation = object.ADD;
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    for (const auto& size : data["size"]) primitive.dimensions.push_back(size.as<double>());
    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(pose_msg(transform(data["pose"])));
    scene.world.collision_objects.push_back(object);
  };
  box("drawing_table", config["table"]);
  int index = 0;
  for (const auto& obstacle : config["obstacles"]) box("drawing_obstacle_" + std::to_string(index++), obstacle);

  // 笔杆参与碰撞检查；最后一小段笔尖保留给纸面接触。
  const double length = tcp_offset.norm() - config["tool"]["tip_clearance_mm"].as<double>() / 1000.0;
  if (length <= 0) throw std::runtime_error("笔杆长度必须大于笔尖接触余量");
  moveit_msgs::msg::AttachedCollisionObject pen;
  pen.link_name = flange;
  if (config["tool"]["touch_links"])
    for (const auto& link : config["tool"]["touch_links"])
      pen.touch_links.push_back(link.as<std::string>());
  else pen.touch_links = {flange};
  pen.object.header.frame_id = flange; pen.object.id = "drawing_pen";
  pen.object.operation = pen.object.ADD;
  shape_msgs::msg::SolidPrimitive cylinder;
  cylinder.type = cylinder.CYLINDER;
  cylinder.dimensions = {length, config["tool"]["radius_mm"].as<double>() / 1000.0};
  Eigen::Isometry3d placement = Eigen::Isometry3d::Identity();
  placement.linear() = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), tcp_offset).toRotationMatrix();
  placement.translation() = tcp_offset.normalized() * length / 2;
  pen.object.primitives.push_back(cylinder);
  pen.object.primitive_poses.push_back(pose_msg(placement));
  scene.robot_state.attached_collision_objects.push_back(pen);
  return scene;
}
}  // namespace drawing
