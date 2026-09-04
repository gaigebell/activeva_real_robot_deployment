#!/usr/bin/env bash
# 只读 Quest 手柄监视，不启动 bridge，不连接 D1。
set -euo pipefail
cd "$(dirname "$0")/.."
export UV_CACHE_DIR="${UV_CACHE_DIR:-/tmp/activeva-uv-cache}"
exec uv run python -m robot.vr_input_monitor "$@"
