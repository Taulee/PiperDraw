#!/usr/bin/env bash
set -eo pipefail

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  echo "用法：bash scripts/foxglove.sh [play]"
  echo "启动只读 Foxglove Bridge，监听 127.0.0.1:8765；按 Ctrl+C 停止。"
  echo "play 使用 rosbag 发布的 /clock 查看历史录制；默认使用当前系统时间。"
  exit 0
fi
if [[ $# -gt 1 || ( $# -eq 1 && "$1" != "play" ) ]]; then
  echo "参数无效，请查看 --help。" >&2
  exit 2
fi
PIPER_USE_SIM_TIME=false
if [[ "${1:-}" == "play" ]]; then
  PIPER_USE_SIM_TIME=true
fi

PIPER_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$PIPER_ROOT/scripts/env.sh"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-96}"
export ROS_LOCALHOST_ONLY=1

if ! ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
  echo "缺少 foxglove_bridge，请先运行项目的 ROS 依赖安装脚本。" >&2
  exit 1
fi
echo "Foxglove Bridge：ws://127.0.0.1:8765，ROS_DOMAIN_ID=$ROS_DOMAIN_ID，use_sim_time=$PIPER_USE_SIM_TIME"
exec ros2 run foxglove_bridge foxglove_bridge \
  --ros-args --params-file "$PIPER_ROOT/config/foxglove.yaml" \
  -p "use_sim_time:=$PIPER_USE_SIM_TIME"
