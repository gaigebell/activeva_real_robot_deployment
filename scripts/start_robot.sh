#!/usr/bin/env bash
# 机器人端启动：所选 D1 bridge + 控制程序
# 用法: ./scripts/start_robot.sh [teleop|inference|idle] [--arm left|right|both]
#                                  [--control-mode full|position|orientation]
#                                  [--engagement-mode deadman|home]
set -euo pipefail
cd "$(dirname "$0")/.."
MODE="teleop"
ACTIVE_ARM="${ROBOT_ARM:-both}"
CONTROL_MODE="${D1_CONTROL_MODE:-full}"
ENGAGEMENT_MODE="${D1_ENGAGEMENT_MODE:-deadman}"
STARTUP_HOME=0
if (($#)) && [[ "$1" != "--arm" ]]; then
  MODE="$1"
  shift
fi
while (($#)); do
  case "$1" in
    --arm)
      if (($# < 2)); then
        echo "--arm 需要 left、right 或 both" >&2
        exit 2
      fi
      ACTIVE_ARM="$2"
      shift 2
      ;;
    --control-mode)
      if (($# < 2)); then
        echo "--control-mode 需要 full、position 或 orientation" >&2
        exit 2
      fi
      CONTROL_MODE="$2"
      shift 2
      ;;
    --engagement-mode)
      if (($# < 2)); then
        echo "--engagement-mode 需要 deadman 或 home" >&2
        exit 2
      fi
      ENGAGEMENT_MODE="$2"
      shift 2
      ;;
    --startup-home)
      STARTUP_HOME=1
      shift
      ;;
    *)
      echo "未知参数: $1" >&2
      exit 2
      ;;
  esac
done
case "${MODE}" in teleop|inference|idle) ;; *) echo "未知模式: ${MODE}" >&2; exit 2 ;; esac
case "${ACTIVE_ARM}" in left|right|both) ;; *) echo "--arm 必须是 left、right 或 both" >&2; exit 2 ;; esac
case "${CONTROL_MODE}" in full|position|orientation) ;; *) echo "--control-mode 必须是 full、position 或 orientation" >&2; exit 2 ;; esac
case "${ENGAGEMENT_MODE}" in deadman|home) ;; *) echo "--engagement-mode 必须是 deadman 或 home" >&2; exit 2 ;; esac
CONFIG="${ROBOT_CONFIG:-robot/config.yaml}"
BRIDGE_BIN="./robot/d1_bridge/build/d1_bridge"
export UV_CACHE_DIR="${UV_CACHE_DIR:-/tmp/activeva-uv-cache}"

if [[ ! -x "${BRIDGE_BIN}" ]]; then
  echo "找不到 ${BRIDGE_BIN}，请先编译 d1_bridge。" >&2
  exit 1
fi

# 在创建 DDS 进程前阻止错路由或遗留 bridge 导致的串臂。
uv run python -c '
import sys, yaml
from robot.dual_d1_test import _check_network, _port_is_open
c = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))["robot"]
arms = ("left", "right") if sys.argv[2] == "both" else (sys.argv[2],)
for arm in arms:
    a = c[arm]
    port = int(a["bridge_port"])
    _check_network(a["dds_interface"], a["host_ip"], a["robot_ip"])
    if _port_is_open(a["bridge_host"], port):
        raise RuntimeError(
            f"{arm}: TCP {port} 已被占用，请停止遗留的 d1_bridge/control")
print("[start_robot] " + ",".join(arms) + " 网卡、主机路由和 TCP 端口预检通过")
' "${CONFIG}" "${ACTIVE_ARM}"

bridge_pids=()
cleanup() {
  if ((${#bridge_pids[@]})); then
    kill "${bridge_pids[@]}" 2>/dev/null || true
    wait "${bridge_pids[@]}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

# 1. 从 config.yaml 读取所选臂的网卡/topic，逐臂启动独立 DDS bridge。
while IFS=$'\t' read -r arm port topic feedback interface address; do
  echo "[start_robot] ${arm}: interface=${interface}, tcp=${port}, topic=${topic}"
  "${BRIDGE_BIN}" --port "${port}" --topic "${topic}" \
    --feedback-topic "${feedback}" --interface "${interface}" --address "${address}" &
  bridge_pids+=("$!")
done < <(uv run python -c '
import sys, yaml
c = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))["robot"]
arms = ("left", "right") if sys.argv[2] == "both" else (sys.argv[2],)
for arm in arms:
    a = c[arm]
    print("\t".join(map(str, (arm, a["bridge_port"], a["dds_topic"],
                                a["feedback_topic"], a["dds_interface"], a["address"]))))
' "${CONFIG}" "${ACTIVE_ARM}")

# 2. Quest WebXR 服务由 scripts/start_quest_stream.sh 在另一终端运行。

# 3. 机器人端控制程序
control_args=(--mode "$MODE" --arm "${ACTIVE_ARM}" \
  --control-mode "${CONTROL_MODE}" --engagement-mode "${ENGAGEMENT_MODE}" \
  --config "${CONFIG}")
if ((STARTUP_HOME)); then
  control_args+=(--startup-home)
fi
uv run python -m robot.control "${control_args[@]}"
