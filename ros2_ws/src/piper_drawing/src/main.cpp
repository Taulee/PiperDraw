#include "piper_drawing/planner.hpp"
#include "piper_drawing/visualization.hpp"
#include <fstream>
#include <thread>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("drawing_executor",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  const auto report_path = node->get_parameter("report").as_string();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin([&] { executor.spin(); });
  YAML::Node report;
  report["completed_segments"] = 0;
  const auto save_report = [&] { std::ofstream(report_path) << report << '\n'; };
  int exit_code = 0;
  std::unique_ptr<drawing::Planner> planner;
  std::unique_ptr<drawing::DrawingVisualization> visualization;
  // 统一在入口记录规划或执行失败，并结束 ROS 线程。
  try {
    const auto path = node->get_parameter("job").as_string();
    const auto mode = node->get_parameter("mode").as_string();
    const bool execute = node->get_parameter("execute").as_bool();
    drawing::Job job(path);
    report["job"] = path;
    report["calibration_id"] = job.config["calibration_id"].as<std::string>();
    report["mode"] = mode;
    if (mode == "real" && !job.config["calibrated"].as<bool>())
      throw std::runtime_error("实机任务需要实测标定并在配置中设置 calibrated: true");
    drawing::check_backend(node, mode);
    if (mode == "real") drawing::vendor_preflight(node, job, execute);
    planner = std::make_unique<drawing::Planner>(node, job);
    visualization = std::make_unique<drawing::DrawingVisualization>(node, job, planner->robot_model());
    const auto segments = planner->plan();
    double duration = 0.0;
    for (const auto& item : segments) {
      YAML::Node segment;
      segment["stroke"] = item.segment.stroke;
      segment["kind"] = item.segment.kind;
      segment["points"] = item.plan.trajectory_.joint_trajectory.points.size();
      const double seconds = rclcpp::Duration(item.plan.trajectory_.joint_trajectory.points.back().time_from_start).seconds();
      segment["seconds"] = seconds; duration += seconds;
      report["segments"].push_back(segment);
    }
    report["estimated_seconds"] = duration;
    report["status"] = "planned";
    save_report();
    if (execute) {
      for (size_t i = 0; i < segments.size(); ++i) {
        report["active_segment"] = i;
        report["status"] = "executing";
        save_report();
        visualization->begin(segments[i].segment);
        planner->execute(segments[i]);
        visualization->end();
        report["completed_segments"] = i + 1;
      }
      report["status"] = "executed";
    }
    visualization->end();
    RCLCPP_INFO(node->get_logger(), "任务完成：%s，预计运动 %.1f 秒", report["status"].as<std::string>().c_str(), duration);
  } catch (const std::exception& error) {
    if (planner && rclcpp::ok()) planner->stop();
    if (visualization && rclcpp::ok()) visualization->end();
    report["status"] = "failed";
    report["error"] = error.what();
    RCLCPP_ERROR(node->get_logger(), "%s", error.what());
    exit_code = 1;
  }
  save_report();
  rclcpp::shutdown();
  spin.join();
  return exit_code;
}
