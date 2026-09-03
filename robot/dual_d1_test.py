"""不依赖 cambot/Quest 的 Unitree D1 双臂分层测试接口。

默认操作 status 只读。所有发命令的操作都要求 --yes，且不会自动上电或使能。
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import socket
import subprocess
import sys
import time

import numpy as np
import yaml

from robot.d1_client import D1Client
from robot.d1_ik import D1IK
from robot.home import capture_home


ARMS = ("left", "right")
JOINT_LIMIT_DEG = (135.0, 90.0, 90.0, 135.0, 90.0, 135.0)
MAX_JOG_DEG = 5.0
MAX_GRIPPER_JOG = 5.0


def valid_feedback(state) -> bool:
    if state is None or len(state) != 7:
        return False
    return bool(np.all(np.isfinite(np.asarray(state, dtype=float))))


def _port_is_open(host: str, port: int) -> bool:
    try:
        with socket.create_connection((host, port), timeout=0.2):
            return True
    except OSError:
        return False


def _check_network(interface: str, host_ip: str, robot_ip: str):
    address = subprocess.run(
        ["ip", "-4", "-o", "addr", "show", "dev", interface],
        text=True, capture_output=True, check=False)
    if address.returncode != 0 or f"{host_ip}/24" not in address.stdout:
        raise RuntimeError(
            f"{interface}: 未找到 {host_ip}/24，请先运行 "
            "sudo ./scripts/setup_d1_network.sh")
    route = subprocess.run(
        ["ip", "route", "get", robot_ip], text=True, capture_output=True, check=False)
    expected = (f"dev {interface}", f"src {host_ip}")
    if route.returncode != 0 or not all(item in route.stdout for item in expected):
        raise RuntimeError(
            f"{robot_ip}: 路由未经 {interface} src {host_ip}，请先运行 "
            "sudo ./scripts/setup_d1_network.sh")


class DualD1Rig:
    def __init__(self, config_path: str, arms=ARMS):
        self.config_path = Path(config_path).resolve()
        self.root = Path(__file__).resolve().parent.parent
        full_cfg = yaml.safe_load(self.config_path.read_text(encoding="utf-8"))
        self.cfg = full_cfg["robot"]
        self.teleop_cfg = full_cfg.get("teleop", {})
        self.arms = tuple(arms)
        if not self.arms or any(arm not in ARMS for arm in self.arms):
            raise ValueError(f"arms 必须是 {ARMS} 的非空子集")
        self.processes: list[subprocess.Popen] = []
        self.clients: dict[str, D1Client] = {}

    def __enter__(self):
        bridge = self.root / "robot/d1_bridge/build/d1_bridge"
        if not bridge.is_file():
            raise RuntimeError(f"找不到 {bridge}，请先编译 d1_bridge")
        for arm in self.arms:
            cfg = self.cfg[arm]
            interface = cfg["dds_interface"]
            if not Path(f"/sys/class/net/{interface}").exists():
                raise RuntimeError(f"{arm}: 找不到网卡 {interface}")
            _check_network(interface, cfg["host_ip"], cfg["robot_ip"])
            if _port_is_open(cfg["bridge_host"], int(cfg["bridge_port"])):
                raise RuntimeError(
                    f"{arm}: TCP {cfg['bridge_port']} 已被占用，请先停止遗留的 d1_bridge")

        for arm in self.arms:
            cfg = self.cfg[arm]
            command = [
                str(bridge), "--port", str(cfg["bridge_port"]),
                "--topic", cfg["dds_topic"],
                "--feedback-topic", cfg["feedback_topic"],
                "--interface", cfg["dds_interface"],
                "--address", str(cfg["address"]),
            ]
            print(f"[dual_d1_test] 启动 {arm} bridge: {cfg['dds_interface']} -> "
                  f"{cfg['robot_ip']} (TCP {cfg['bridge_port']})")
            self.processes.append(subprocess.Popen(command))

        try:
            for arm in self.arms:
                cfg = self.cfg[arm]
                deadline = time.monotonic() + 5.0
                while True:
                    try:
                        client = D1Client(cfg["bridge_host"], int(cfg["bridge_port"]), arm)
                        client.connect()
                        self.clients[arm] = client
                        break
                    except OSError:
                        if time.monotonic() >= deadline:
                            raise RuntimeError(f"{arm}: 5 秒内未能连接 d1_bridge")
                        time.sleep(0.1)
            return self
        except Exception:
            self.close()
            raise

    def __exit__(self, *_exc):
        self.close()

    def close(self):
        for client in self.clients.values():
            client.close()
        self.clients.clear()
        for process in self.processes:
            process.terminate()
        for process in self.processes:
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        self.processes.clear()

    def wait_feedback(self, timeout: float) -> dict[str, list[float] | None]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            states = {arm: self.clients[arm].get_state() for arm in self.arms}
            states = {arm: state if valid_feedback(state) else None
                      for arm, state in states.items()}
            if all(state is not None for state in states.values()):
                return states
            time.sleep(0.05)
        states = {arm: self.clients[arm].get_state() for arm in self.arms}
        return {arm: state if valid_feedback(state) else None
                for arm, state in states.items()}


def selected_arms(name: str):
    return ARMS if name == "both" else (name,)


def require_confirmation(args):
    if not args.yes:
        raise RuntimeError("该操作会改变机械臂状态；确认环境安全后追加 --yes")


def run_status(rig: DualD1Rig, args) -> int:
    deadline = time.monotonic() + args.seconds
    seen = {arm: False for arm in rig.arms}
    while time.monotonic() < deadline:
        for arm in rig.arms:
            state = rig.clients[arm].get_state()
            if valid_feedback(state):
                seen[arm] = True
                values = " ".join(f"{value:8.3f}" for value in state)
                print(f"{arm:>5}: [{values}]")
            elif state is not None:
                print(f"{arm:>5}: 收到非法反馈（必须是 7 个有限数）")
            else:
                print(f"{arm:>5}: 等待反馈…")
        if all(seen.values()) and not args.watch:
            break
        time.sleep(0.5)
    missing = [arm for arm, ok in seen.items() if not ok]
    if missing:
        print(f"未收到反馈: {', '.join(missing)}", file=sys.stderr)
        return 2
    print(f"{','.join(rig.arms)} 已收到 7 值关节/夹爪反馈。")
    return 0


def run_action(rig: DualD1Rig, args) -> int:
    require_confirmation(args)
    method_name = args.action.replace("-", "_")
    for arm in selected_arms(args.arm):
        getattr(rig.clients[arm], method_name)()
        print(f"{arm}: 已发送 {args.action}")
    time.sleep(0.3)
    return 0


def run_hold(rig: DualD1Rig, args) -> int:
    require_confirmation(args)
    states = rig.wait_feedback(args.feedback_timeout)
    for arm in selected_arms(args.arm):
        state = states[arm]
        if state is None:
            raise RuntimeError(f"{arm}: 无反馈，不发送 hold")
        rig.clients[arm].set_target(state)
        print(f"{arm}: 已将当前反馈值原样下发（通信保持测试）")
    time.sleep(args.hold_seconds)
    return 0


def run_jog(rig: DualD1Rig, args) -> int:
    require_confirmation(args)
    if not math.isfinite(args.delta):
        raise RuntimeError("delta 必须是有限数值")
    max_delta = MAX_JOG_DEG if args.joint < 6 else MAX_GRIPPER_JOG
    if abs(args.delta) > max_delta:
        unit = "度" if args.joint < 6 else "夹爪单位"
        raise RuntimeError(f"单次微动不得超过 {max_delta:g} {unit}")
    state = rig.wait_feedback(args.feedback_timeout)[args.arm]
    if state is None:
        raise RuntimeError(f"{args.arm}: 无反馈，拒绝微动")
    target = np.asarray(state, dtype=float)
    target[args.joint] += args.delta
    if args.joint < 6:
        limit = JOINT_LIMIT_DEG[args.joint]
        if not -limit <= target[args.joint] <= limit:
            raise RuntimeError(f"目标超出 J{args.joint} 限位 ±{limit:g}°")
    elif not 0.0 <= target[6] <= 65.0:
        raise RuntimeError("夹爪目标必须在 0..65 之间")
    rig.clients[args.arm].set_target(target)
    unit = "°" if args.joint < 6 else ""
    print(f"{args.arm}: J{args.joint} {state[args.joint]:.3f} -> "
          f"{target[args.joint]:.3f}{unit}")
    time.sleep(args.hold_seconds)
    return 0


def run_capture_home(rig: DualD1Rig, args) -> int:
    require_confirmation(args)
    client = rig.clients[args.arm]
    state = rig.wait_feedback(args.feedback_timeout)[args.arm]
    if state is None:
        raise RuntimeError(f"{args.arm}: 无反馈，不能捕获 Home")
    teleop_cfg = getattr(rig, "teleop_cfg", {})
    condition = D1IK().condition_number(
        np.deg2rad(np.asarray(state[:6], dtype=float)),
        float(teleop_cfg.get("calibration_orientation_weight", 0.5)))
    max_condition = float(teleop_cfg.get("max_ik_condition_number", 80.0))
    if not np.isfinite(condition) or condition > max_condition:
        raise RuntimeError(
            f"{args.arm}: 当前姿态接近奇异，condition={condition:.1f} "
            f"（上限 {max_condition:g}），不保存为 Home；"
            "请在卸力状态下弯曲肘部和腕部后重试")
    # 用捕获到的当前反馈建立保持目标，操作者看到提示后即可松手。
    client.power_on()
    time.sleep(0.2)
    client.set_target(state, mode=1)
    client.enable()
    time.sleep(0.3)
    client.set_target(state, mode=1)
    saved = capture_home(client, args.output, args.feedback_timeout)
    values = ", ".join(f"{value:.3f}" for value in saved)
    print(f"{args.arm}: Home 已保持并保存到 {args.output}: [{values}] "
          f"(condition={condition:.1f})")
    return 0


def build_parser():
    parser = argparse.ArgumentParser(description="D1 单臂/双臂安全测试接口（不依赖 cambot）")
    parser.add_argument("--config", default="robot/config.yaml")
    sub = parser.add_subparsers(dest="command", required=True)

    status = sub.add_parser("status", help="只读检查所选机械臂反馈")
    status.add_argument("--arm", choices=("left", "right", "both"), default="both")
    status.add_argument("--seconds", type=float, default=5.0)
    status.add_argument("--watch", action="store_true", help="在 seconds 内持续打印")
    status.set_defaults(run=run_status)

    action = sub.add_parser("action", help="上电/使能/卸力/断电/归零")
    action.add_argument("action", choices=("power-on", "enable", "disable", "power-off", "zero"))
    action.add_argument("--arm", choices=("left", "right", "both"), default="both")
    action.add_argument("--yes", action="store_true")
    action.set_defaults(run=run_action)

    hold = sub.add_parser("hold", help="将反馈值原样下发，不主动改变目标")
    hold.add_argument("--arm", choices=("left", "right", "both"), default="both")
    hold.add_argument("--feedback-timeout", type=float, default=5.0)
    hold.add_argument("--hold-seconds", type=float, default=1.0)
    hold.add_argument("--yes", action="store_true")
    hold.set_defaults(run=run_hold)

    jog = sub.add_parser("jog", help="单臂单关节小幅增量测试")
    jog.add_argument("--arm", choices=ARMS, required=True)
    jog.add_argument("--joint", type=int, choices=range(7), required=True,
                     help="0..5 为关节（度），6 为夹爪")
    jog.add_argument("--delta", type=float, required=True,
                     help="相对当前反馈增量；关节最大 ±5°，夹爪最大 ±5")
    jog.add_argument("--feedback-timeout", type=float, default=5.0)
    jog.add_argument("--hold-seconds", type=float, default=1.0)
    jog.add_argument("--yes", action="store_true")
    jog.set_defaults(run=run_jog)

    capture = sub.add_parser(
        "capture-home", help="保持并保存当前单臂反馈为遥操 Home")
    capture.add_argument("--arm", choices=ARMS, required=True)
    capture.add_argument("--output", default="robot/calibration/d1_home.yaml")
    capture.add_argument("--feedback-timeout", type=float, default=5.0)
    capture.add_argument("--yes", action="store_true")
    capture.set_defaults(run=run_capture_home)
    return parser


def main():
    args = build_parser().parse_args()
    try:
        with DualD1Rig(args.config, selected_arms(args.arm)) as rig:
            return args.run(rig, args)
    except (RuntimeError, OSError, KeyError, ValueError) as exc:
        print(f"错误: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
