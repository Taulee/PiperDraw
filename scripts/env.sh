#!/usr/bin/env bash
PIPER_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$PIPER_ROOT/ros2_ws/install/setup.bash"
export PYTHONPATH="$PIPER_ROOT/src:$PIPER_ROOT/.venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-106}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$PIPER_ROOT/output/ros_logs}"
mkdir -p "$ROS_LOG_DIR"
