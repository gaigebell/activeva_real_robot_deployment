#!/usr/bin/env bash
# 启动所选 D1 bridge + 对应 Quest 手柄遥操；WebXR 服务需在另一终端启动。
set -euo pipefail
cd "$(dirname "$0")/.."
export UV_CACHE_DIR="${UV_CACHE_DIR:-/tmp/activeva-uv-cache}"
uv run python -c '
from robot.dual_d1_test import _port_is_open
if _port_is_open("127.0.0.1", 6001):
    raise RuntimeError(
        "TCP 6001 已被占用；请先 Ctrl+C 停止 monitor_quest_controllers.sh 或遗留 control")
'
if [[ "${1:-}" != "--yes" ]]; then
  echo "请先确认：周围无人、急停可用，且已 capture-home；脚本会为所选单臂上电、使能并进入 Home。" >&2
  echo "确认后运行: $0 --yes --arm left|right|both --control-mode full|position|orientation --engagement-mode deadman|home" >&2
  exit 2
fi
shift
exec ./scripts/start_robot.sh teleop --startup-home "$@"
