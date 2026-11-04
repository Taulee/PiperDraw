#!/usr/bin/env bash
set -euo pipefail
PIPER_CAN="${1:-can0}"
if [[ "$PIPER_CAN" == '--help' ]]; then
  echo '用法：sudo bash scripts/can_up.sh can0；将指定 SocketCAN 接口设为 1 Mbit/s。'
  exit 0
fi
if [[ $# -gt 1 || ! "$PIPER_CAN" =~ ^[a-zA-Z0-9_.:-]+$ ]]; then
  echo '接口名无效。' >&2
  exit 2
fi
if [[ ! -f "/sys/class/net/$PIPER_CAN/type" ]] || [[ "$(cat "/sys/class/net/$PIPER_CAN/type")" != '280' ]]; then
  echo "未找到 CAN 接口：$PIPER_CAN" >&2
  exit 1
fi
ip link set "$PIPER_CAN" down
ip link set "$PIPER_CAN" type can bitrate 1000000
ip link set "$PIPER_CAN" up
ip -details -statistics link show "$PIPER_CAN"
