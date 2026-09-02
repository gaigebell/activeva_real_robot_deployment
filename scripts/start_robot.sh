#!/usr/bin/env bash
# 机器人端一键启动：两个 d1_bridge + 控制程序
# 用法: ./scripts/start_robot.sh [teleop|inference|idle]
set -euo pipefail
cd "$(dirname "$0")/.."
MODE="${1:-teleop}"

# 1. D1 驱动桥（双臂两实例；topic/网卡参数按现场多臂方案修改，见 robot/config.yaml）
./robot/d1_bridge/build/d1_bridge \
  --port 5500 --topic rt/arm_Command --feedback-topic current_servo_angle --address 1 &
./robot/d1_bridge/build/d1_bridge \
  --port 5501 --topic rt/arm_Command_1 --feedback-topic current_servo_angle_1 --address 1 &

# 2. cambot 遥操作（头显控制 cambot；在 cambot 目录单独运行）
#    (cd ../cambot && ./run_teleop.sh) &
#    并现场应用 robot/cambot_patch/README.md 的手柄转发补丁

# 3. 机器人端控制程序
uv run python -m robot.control --mode "$MODE" --config robot/config.yaml
