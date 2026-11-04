#!/usr/bin/env bash
set -eo pipefail

usage() {
  echo "用法：bash scripts/record_sim.sh record [输出目录]"
  echo "      bash scripts/record_sim.sh play 录制目录 [ros2 bag play 参数]"
  echo "record 录制机械臂状态、纸面和渐进笔迹为 MCAP，按 Ctrl+C 完成并保存。"
  echo "play 经 Foxglove Bridge 回放；请先停止模拟后端，再运行 foxglove.sh play。"
  echo "回放可追加 --loop 或 --rate 0.5；空格键暂停/继续，Ctrl+C 停止。"
}
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi
PIPER_RECORD_ACTION="${1:-}"
if [[ "$PIPER_RECORD_ACTION" != "record" && "$PIPER_RECORD_ACTION" != "play" ]]; then
  usage >&2
  exit 2
fi
shift
if [[ "$PIPER_RECORD_ACTION" == "record" && $# -gt 1 ]] || \
   [[ "$PIPER_RECORD_ACTION" == "play" && $# -lt 1 ]]; then
  usage >&2
  exit 2
fi

PIPER_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$PIPER_ROOT/scripts/env.sh"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-96}"
export ROS_LOCALHOST_ONLY=1
if ! ros2 pkg prefix rosbag2_storage_mcap >/dev/null 2>&1; then
  echo "缺少 rosbag2_storage_mcap，请先运行项目的 ROS 依赖安装脚本。" >&2
  exit 1
fi

PIPER_RECORD_DIR="${1:-$PIPER_ROOT/output/recordings/$(date +%Y%m%d-%H%M%S)}"
if [[ "$PIPER_RECORD_ACTION" == "record" ]]; then
  if [[ -e "$PIPER_RECORD_DIR" ]]; then
    echo "输出目录已存在，请指定新目录：$PIPER_RECORD_DIR" >&2
    exit 1
  fi
  mkdir -p "$(dirname "$PIPER_RECORD_DIR")"
  echo "开始录制，按 Ctrl+C 保存：$PIPER_RECORD_DIR，ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  ros2 bag record --storage mcap --output "$PIPER_RECORD_DIR" \
    --qos-profile-overrides-path "$PIPER_ROOT/config/recording-qos.yaml" \
    /tf /tf_static /joint_states /robot_description /drawing_markers
else
  if [[ ! -f "$PIPER_RECORD_DIR/metadata.yaml" ]]; then
    echo "请指定包含 metadata.yaml 的 MCAP 录制目录：$PIPER_RECORD_DIR" >&2
    exit 1
  fi
  shift
  echo "请确认模拟后端已停止、foxglove.sh play 已启动；回放使用 ROS_DOMAIN_ID=$ROS_DOMAIN_ID。"
  ros2 bag play --storage mcap --clock --delay 2 \
    --qos-profile-overrides-path "$PIPER_ROOT/config/recording-qos.yaml" "$PIPER_RECORD_DIR" "$@"
fi
