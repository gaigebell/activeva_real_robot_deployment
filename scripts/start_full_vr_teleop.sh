#!/usr/bin/env bash
# 一键启动 Quest WebXR + 所选 D1 上电/使能 + 6DoF 遥操，退出时卸力并断电。
set -euo pipefail
cd "$(dirname "$0")/.."

if [[ "${1:-}" != "--yes" ]]; then
  echo "本脚本会自动为所选 D1 上电和使能。" >&2
  echo "确认安装牢固、工作区无人且急停可用后运行: $0 --yes [--arm left|right|both] [Quest服务参数]" >&2
  exit 2
fi
shift

ACTIVE_ARM="both"
CONTROL_MODE="full"
ENGAGEMENT_MODE="deadman"
quest_args=()
while (($#)); do
  if [[ "$1" == "--arm" ]]; then
    if (($# < 2)); then
      echo "--arm 需要 left、right 或 both" >&2
      exit 2
    fi
    ACTIVE_ARM="$2"
    shift 2
  elif [[ "$1" == "--control-mode" ]]; then
    if (($# < 2)); then
      echo "--control-mode 需要 full、position 或 orientation" >&2
      exit 2
    fi
    CONTROL_MODE="$2"
    shift 2
  elif [[ "$1" == "--engagement-mode" ]]; then
    if (($# < 2)); then
      echo "--engagement-mode 需要 deadman 或 home" >&2
      exit 2
    fi
    ENGAGEMENT_MODE="$2"
    shift 2
  else
    quest_args+=("$1")
    shift
  fi
done
case "${ACTIVE_ARM}" in left|right|both) ;; *) echo "--arm 必须是 left、right 或 both" >&2; exit 2 ;; esac
case "${CONTROL_MODE}" in full|position|orientation) ;; *) echo "--control-mode 必须是 full、position 或 orientation" >&2; exit 2 ;; esac
case "${ENGAGEMENT_MODE}" in deadman|home) ;; *) echo "--engagement-mode 必须是 deadman 或 home" >&2; exit 2 ;; esac

quest_pid=""
powered=0
cleaned=0

cleanup() {
  if ((cleaned)); then
    return
  fi
  cleaned=1
  trap - EXIT INT TERM
  set +e
  if [[ -n "${quest_pid}" ]]; then
    kill "${quest_pid}" 2>/dev/null
    wait "${quest_pid}" 2>/dev/null
  fi
  if ((powered)); then
    echo "[full_teleop] 安全收尾：卸力并断电…"
    ./scripts/test_dual_d1.sh action disable --arm "${ACTIVE_ARM}" --yes
    ./scripts/test_dual_d1.sh action power-off --arm "${ACTIVE_ARM}" --yes
  fi
}
trap cleanup EXIT INT TERM

echo "[full_teleop] 启动 Quest WebXR（cambot 机械臂禁用）…"
./scripts/start_quest_stream.sh "${quest_args[@]}" &
quest_pid=$!
sleep 2
if ! kill -0 "${quest_pid}" 2>/dev/null; then
  wait "${quest_pid}" || true
  echo "[full_teleop] Quest WebXR 启动失败，未给 D1 上电。" >&2
  exit 1
fi

echo "[full_teleop] 为 ${ACTIVE_ARM} D1 上电、使能…"
./scripts/test_dual_d1.sh action power-on --arm "${ACTIVE_ARM}" --yes
powered=1
sleep 1
./scripts/test_dual_d1.sh action enable --arm "${ACTIVE_ARM}" --yes

echo "[full_teleop] 启动 ${ACTIVE_ARM} D1 6DoF 遥操。Ctrl+C 将卸力并断电。"
robot_args=(teleop --arm "${ACTIVE_ARM}" --control-mode "${CONTROL_MODE}" \
  --engagement-mode "${ENGAGEMENT_MODE}")
if [[ "${ENGAGEMENT_MODE}" == "home" ]]; then
  robot_args+=(--startup-home)
fi
./scripts/start_robot.sh "${robot_args[@]}"
