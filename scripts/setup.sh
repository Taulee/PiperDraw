#!/usr/bin/env bash
set -eo pipefail
PIPER_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PIPER_ROOT"
/usr/bin/python3 scripts/fetch_vendor.py
uv sync --locked --python /usr/bin/python3
source /opt/ros/humble/setup.bash
cd "$PIPER_ROOT/ros2_ws"
colcon build --symlink-install --parallel-workers 2 \
  --base-paths src "$PIPER_ROOT/vendor/piper_ros/src/piper_description" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3
