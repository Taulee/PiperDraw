#!/usr/bin/env bash
set -e
apt=(apt-get)
if (( EUID != 0 )); then apt=(sudo apt-get); fi
"${apt[@]}" update
# 按无头后端所需组件安装，避免全量工具包带入图形界面。
"${apt[@]}" install -y --no-install-recommends \
  ros-humble-moveit-ros-move-group ros-humble-moveit-ros-planning-interface \
  ros-humble-moveit-planners-ompl ros-humble-moveit-kinematics \
  ros-humble-moveit-simple-controller-manager ros-humble-moveit-configs-utils \
  ros-humble-controller-manager ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller ros-humble-ros2controlcli \
  ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-foxglove-bridge ros-humble-rosbag2-storage-mcap \
  ros-humble-rosidl-default-generators \
  python3-colcon-common-extensions libyaml-cpp-dev can-utils ethtool iproute2
