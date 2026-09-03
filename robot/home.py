"""D1 现场 Home 的捕获、加载和低速进入流程。"""
from __future__ import annotations

from pathlib import Path
import time

import numpy as np
import yaml


def _valid_state(state) -> bool:
    return bool(
        state is not None and len(state) == 7 and
        np.all(np.isfinite(np.asarray(state, dtype=float))))


def wait_state(client, timeout_s: float):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        state = client.get_state()
        if _valid_state(state):
            return np.asarray(state, dtype=float)
        time.sleep(0.05)
    raise RuntimeError(f"{client.arm}: {timeout_s:g}s 内未收到有效 D1 反馈")


def capture_home(client, path, timeout_s=5.0):
    """保存当前 6 关节和夹爪反馈；保留文件中另一只臂的记录。"""
    state = wait_state(client, timeout_s)
    path = Path(path)
    data = {}
    if path.exists():
        loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
        if isinstance(loaded, dict):
            data = loaded
    data[client.arm] = [float(value) for value in state]
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        yaml.safe_dump(data, allow_unicode=True, sort_keys=True), encoding="utf-8")
    temporary.replace(path)
    return state


def load_home(path, arm):
    path = Path(path)
    if not path.exists():
        raise RuntimeError(
            f"未找到 Home 文件 {path}；请先运行 test_dual_d1.sh capture-home")
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    target = np.asarray(data.get(arm, []), dtype=float)
    if target.shape != (7,) or not np.all(np.isfinite(target)):
        raise RuntimeError(f"Home 文件 {path} 中没有有效的 {arm} 七值姿态")
    return target


def move_single_arm_to_home(client, path, cfg):
    """先保持反馈位置，再用 smoothstep 关节插值低速进入已捕获 Home。"""
    timeout = float(cfg.get("feedback_timeout_s", 5.0))
    current = wait_state(client, timeout)
    target = load_home(path, client.arm)
    joint_delta = target[:6] - current[:6]
    max_initial_delta = float(cfg.get("max_initial_delta_deg", 35.0))
    if np.max(np.abs(joint_delta)) > max_initial_delta:
        joint = int(np.argmax(np.abs(joint_delta)))
        raise RuntimeError(
            f"{client.arm}: 当前姿态离 Home 太远，J{joint} 差 "
            f"{joint_delta[joint]:+.1f}°（上限 {max_initial_delta:g}°）；"
            "拒绝自动规划，请人工检查")

    # 先锁住当前反馈，再使能；即使后续 Home 校验失败也不会先追逐未知目标。
    client.power_on()
    time.sleep(0.2)
    client.set_target(current, mode=1)
    client.enable()
    time.sleep(0.3)
    client.set_target(current, mode=1)

    hz = float(cfg.get("command_hz", 10.0))
    joint_speed = float(cfg.get("joint_speed_deg_s", 5.0))
    gripper_speed = float(cfg.get("gripper_speed_unit_s", 10.0))
    if hz <= 0 or joint_speed <= 0 or gripper_speed <= 0:
        raise ValueError("Home 速度与频率必须大于 0")
    duration = max(
        float(np.max(np.abs(joint_delta))) / joint_speed,
        abs(float(target[6] - current[6])) / gripper_speed,
        0.5,
    )
    steps = max(1, int(np.ceil(duration * hz)))
    print(f"[home:{client.arm}] 低速进入 Home，预计 {duration:.1f}s，{steps} 步")
    for step in range(1, steps + 1):
        ratio = step / steps
        blend = ratio * ratio * (3.0 - 2.0 * ratio)
        command = current + blend * (target - current)
        client.set_target(command, mode=1)
        time.sleep(1.0 / hz)

    settle_timeout = float(cfg.get("settle_timeout_s", 4.0))
    tolerance = float(cfg.get("settle_tolerance_deg", 3.0))
    deadline = time.monotonic() + settle_timeout
    actual = wait_state(client, timeout)
    while time.monotonic() < deadline:
        actual = wait_state(client, timeout)
        if np.max(np.abs(actual[:6] - target[:6])) <= tolerance:
            print(f"[home:{client.arm}] 已到达 Home，进入手柄等待")
            return target
        client.set_target(target, mode=1)
        time.sleep(0.1)
    error = np.abs(actual[:6] - target[:6])
    joint = int(np.argmax(error))
    raise RuntimeError(
        f"{client.arm}: Home 到位超时，J{joint} 误差 {error[joint]:.1f}°；"
        "保持 Home 目标但不开放遥操")
