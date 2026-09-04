#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "请使用 sudo 运行: sudo $0 [--arm left|right|both]" >&2
  exit 1
fi

ACTIVE_ARM="both"
if (($#)); then
  if [[ "$1" != "--arm" || $# -ne 2 ]]; then
    echo "用法: sudo $0 [--arm left|right|both]" >&2
    exit 2
  fi
  ACTIVE_ARM="$2"
fi
case "${ACTIVE_ARM}" in left|right|both) ;; *) echo "--arm 必须是 left、right 或 both" >&2; exit 2 ;; esac

configure_link() {
  local interface=$1 host_cidr=$2 robot_ip=$3 source_ip=$4
  if [[ ! -e "/sys/class/net/${interface}" ]]; then
    echo "找不到网卡 ${interface}，请用 ip -br link 检查名称。" >&2
    exit 1
  fi
  if command -v nmcli >/dev/null 2>&1; then
    nmcli device set "${interface}" managed no
  fi
  ip addr flush dev "${interface}"
  ip addr add "${host_cidr}" dev "${interface}"
  ip link set "${interface}" up
  ip route replace "${robot_ip}/32" dev "${interface}" src "${source_ip}"
}

if [[ "${ACTIVE_ARM}" == "left" || "${ACTIVE_ARM}" == "both" ]]; then
  configure_link enp5s0 192.168.123.97/24 192.168.123.100 192.168.123.97
fi
if [[ "${ACTIVE_ARM}" == "right" || "${ACTIVE_ARM}" == "both" ]]; then
  configure_link enx00e04c681b82 192.168.123.98/24 192.168.123.99 192.168.123.98
fi

echo "D1 ${ACTIVE_ARM} 网络已初始化。当前路由："
if [[ "${ACTIVE_ARM}" == "left" || "${ACTIVE_ARM}" == "both" ]]; then
  ip route get 192.168.123.100
fi
if [[ "${ACTIVE_ARM}" == "right" || "${ACTIVE_ARM}" == "both" ]]; then
  ip route get 192.168.123.99
fi
