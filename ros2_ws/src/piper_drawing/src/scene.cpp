#include "piper_drawing/paper.hpp"

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>
#include <memory>

namespace drawing {

class DrawingScene : public rclcpp::Node {
public:
  DrawingScene() : Node("drawing_scene") {
    const auto config = YAML::LoadFile(declare_parameter<std::string>("config", ""));
    const auto paper = config["paper"];
    auto point = [](const YAML::Node& values) -> Eigen::Vector3d {
      return Eigen::Vector3d(values[0].as<double>(), values[1].as<double>(),
                             values[2].as<double>()) / 1000.0;
    };
    const Eigen::Vector3d origin = point(paper["origin_mm"]);
    Eigen::Vector3d x = point(paper["x_point_mm"]) - origin;
    const Eigen::Vector3d y = point(paper["y_point_mm"]) - origin;
    Eigen::Vector3d normal = x.cross(y);
    if (!origin.allFinite() || !x.allFinite() || !y.allFinite() ||
        x.norm() < 0.001 || normal.norm() < 0.000001)
      throw std::runtime_error("纸面标定点非有限、重合或共线");
    x.normalize();
    normal.normalize();
    Eigen::Isometry3d surface = Eigen::Isometry3d::Identity();
    surface.linear().col(0) = x;
    surface.linear().col(1) = normal.cross(x);
    surface.linear().col(2) = normal;
    const Eigen::Vector2d size(paper["width_mm"].as<double>() / 1000.0,
                                paper["height_mm"].as<double>() / 1000.0);
    surface.translation() = origin + surface.linear().leftCols<2>() * size / 2.0;
    auto marker = paper_marker(surface, size, config["robot"]["base_link"].as<std::string>());
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = marker.header.frame_id;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    visualization_msgs::msg::MarkerArray initial;
    initial.markers = {clear, marker};

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("/drawing_markers", qos);
    subscription_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        "/drawing_task_markers", qos,
        [this](visualization_msgs::msg::MarkerArray::ConstSharedPtr message) {
          // 内部执行器提供完整快照；ROS 的瞬态 QoS 保留最后画面。
          publisher_->publish(*message);
        });
    const auto stamp = now();
    for (auto& item : initial.markers) item.header.stamp = stamp;
    publisher_->publish(initial);
    RCLCPP_INFO(get_logger(), "纸面已发布：%.0f × %.0f mm，参考系 %s",
                size.x() * 1000.0, size.y() * 1000.0, marker.header.frame_id.c_str());
  }

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscription_;
};

}  // drawing 命名空间

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<drawing::DrawingScene>());
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("drawing_scene"), "场景启动失败：%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
