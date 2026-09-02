#!/usr/bin/env bash
# 服务器端：启动推理服务（监听端口，等待机器人端 obs）
set -euo pipefail
cd "$(dirname "$0")/.."
uv run python server/inference_server.py --config server/config.yaml
