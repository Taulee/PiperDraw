#!/usr/bin/env bash
set -euo pipefail
PIPER_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PIPER_IMAGE="${PIPER_HARDWARE_IMAGE:-piperdraw/hardware:0.1}"
PIPER_DOCKER=(docker)
if [[ $EUID -ne 0 ]] && ! docker version --format '{{.Server.Version}}' >/dev/null 2>&1; then
  PIPER_DOCKER=(sudo docker)
fi
cd "$PIPER_ROOT"
if [[ $EUID -eq 0 && -n "${SUDO_USER:-}" && "$SUDO_USER" != root ]]; then
  sudo -u "$SUDO_USER" python3 scripts/fetch_vendor.py
else
  python3 scripts/fetch_vendor.py
fi
PIPER_BASE="${PIPER_BASE_IMAGE:-python:3.10-slim-bookworm}"
# 本机已有镜像时直接复用，避免重复下载；不修改共享镜像标签。
if [[ -z "${PIPER_BASE_IMAGE:-}" ]] && ! "${PIPER_DOCKER[@]}" image inspect "$PIPER_BASE" >/dev/null 2>&1; then
  PIPER_CACHED='docker.m.daocloud.io/library/python:3.10-slim-bookworm'
  if "${PIPER_DOCKER[@]}" image inspect "$PIPER_CACHED" >/dev/null 2>&1; then
    PIPER_BASE="$PIPER_CACHED"
  fi
fi
exec "${PIPER_DOCKER[@]}" build --network host \
  --build-arg "PIPER_BASE_IMAGE=$PIPER_BASE" \
  --build-arg "PIPER_PIP_INDEX=${PIPER_PIP_INDEX:-https://pypi.tuna.tsinghua.edu.cn/simple}" \
  --build-arg "http_proxy=${http_proxy:-${HTTP_PROXY:-}}" \
  --build-arg "https_proxy=${https_proxy:-${HTTPS_PROXY:-}}" \
  --build-arg "no_proxy=${no_proxy:-${NO_PROXY:-}}" \
  -f deploy/Dockerfile.hardware -t "$PIPER_IMAGE" .
