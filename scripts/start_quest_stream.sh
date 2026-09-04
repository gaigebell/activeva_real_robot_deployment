#!/usr/bin/env bash
# 只启动 Quest WebXR 服务；cambot 机械臂始终禁用。
set -euo pipefail
CAMBOT_DIR="${CAMBOT_DIR:-/home/ubuntu/cambot}"
if [[ ! -x "${CAMBOT_DIR}/run_teleop.sh" ]]; then
  echo "找不到 ${CAMBOT_DIR}/run_teleop.sh" >&2
  exit 1
fi
export UV_CACHE_DIR="${UV_CACHE_DIR:-/tmp/cambot-uv-cache}"
export CAMBOT_FORWARD_HOST="${CAMBOT_FORWARD_HOST:-127.0.0.1}"
export CAMBOT_FORWARD_PORT="${CAMBOT_FORWARD_PORT:-6001}"
export CAMBOT_ACTIVEVA_CONTROLLER_MODE=1
cd "${CAMBOT_DIR}"
# ActiveVA 只借用 WebXR/手柄入口，cambot 真机已由 --no-robot 禁用；因此
# cambot 的“头显摘下后回 Home”watchdog 没有控制对象，只会干扰 D1 标定日志。
# 0 表示禁用。调用者仍可在后面的参数中显式覆盖。
exec ./run_teleop.sh --no-robot --watchdog-timeout 0 "$@"
