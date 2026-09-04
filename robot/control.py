"""机器人端主控制程序。

模式：
  teleop      手柄（经 cambot_patch 转发）-> mapping -> IK -> D1；头显 -> cambot（cambot 自理）
  inference   采集 obs -> 服务器推理 -> action -> 三臂执行（D1 经 d1_bridge；cambot 经其控制接口）
  idle        仅维护 D1 连接与反馈，不发指令

TODO(现场):
- cambot 关节读取与下发（走 cambot WS 遥测/命令）
- 延迟统计（头显 -> 各臂）
"""
from __future__ import annotations
import argparse
from collections import deque
import json
import socket
import threading
import time

import numpy as np
import yaml
import zmq

from common import contract
from common.protocol import pack_obs, unpack_action
from robot.camera import create_camera
from robot.collect import Collector
from robot.controller_mapping import ControllerMapping, matrix_to_quat
from robot.d1_client import D1Client
from robot.d1_ik import D1IK
from robot.home import move_single_arm_to_home


TELEOP_CONTROL_MODES = ("full", "position", "orientation")
TELEOP_ENGAGEMENT_MODES = ("deadman", "home")
D1_JOINT_LIMITS_DEG = np.array([134.6, 90.0, 90.0, 134.6, 90.0, 134.6])


class ForwardReceiver:
    """监听 cambot_patch 经本地 TCP 转发的手柄/按钮数据。

    消息（JSON line）: {"type": "controller_pose", "hand": "left"|"right",
                       "q": {x,y,z,w}, "p": {x,y,z}, "trigger": 0-1, "buttons": [...]}
                      {"type": "episode", "event": "toggle"}
    """

    def __init__(self, host, port):
        self.host, self.port = host, port
        self.latest: dict[str, dict] = {}
        self._episode_cb = None
        self._running = False
        self._server = None
        self._lock = threading.Lock()
        self._ready = threading.Event()
        self._error = None
        self._thread = None
        self._rx_times = {hand: deque(maxlen=512) for hand in ("left", "right")}

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        if not self._ready.wait(timeout=2.0):
            raise RuntimeError(f"手柄接收器 {self.host}:{self.port} 启动超时")
        if self._error is not None:
            raise RuntimeError(
                f"手柄接收器无法监听 {self.host}:{self.port}: {self._error}")

    def _loop(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server = server
        try:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((self.host, self.port))
            server.listen(8)
            server.settimeout(1.0)
        except OSError as exc:
            self._error = exc
            self._ready.set()
            server.close()
            return
        self._ready.set()
        print(f"[forward] 监听 {self.host}:{self.port}")
        while self._running:
            try:
                sock, _addr = server.accept()
                sock.settimeout(1.0)
                buf = b""
                while self._running:
                    try:
                        data = sock.recv(4096)
                    except socket.timeout:
                        continue
                    if not data:
                        break
                    buf += data
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        self._handle(json.loads(line))
            except socket.timeout:
                continue
            except (OSError, json.JSONDecodeError) as e:
                if self._running:
                    print(f"[forward] 接收错误: {e}")
            finally:
                try:
                    sock.close()
                except (UnboundLocalError, OSError):
                    pass

    def _handle(self, msg):
        if msg.get("type") == "controller_pose":
            received = time.monotonic()
            msg["_received_monotonic"] = received
            hand = msg.get("hand", "?")
            with self._lock:
                self.latest[hand] = msg
                if hand in self._rx_times:
                    self._rx_times[hand].append(received)
        elif msg.get("type") == "episode" and self._episode_cb:
            self._episode_cb(msg.get("event", "toggle"))

    def on_episode(self, cb):
        self._episode_cb = cb

    def get_latest(self, hand, max_age_s=None):
        with self._lock:
            msg = self.latest.get(hand)
        if msg is None:
            return None
        if max_age_s is not None:
            age = time.monotonic() - msg["_received_monotonic"]
            if age > max_age_s:
                return None
        return msg

    def get_stats(self, hand):
        """返回最近约 2 秒的收包频率、数据年龄和最大相邻间隔。"""
        now = time.monotonic()
        with self._lock:
            msg = self.latest.get(hand)
            times = [stamp for stamp in self._rx_times.get(hand, ())
                     if now - stamp <= 2.0]
        age = None if msg is None else now - msg["_received_monotonic"]
        if len(times) < 2:
            return {"hz": 0.0, "age_s": age, "max_gap_s": None}
        gaps = np.diff(times)
        return {
            "hz": (len(times) - 1) / (times[-1] - times[0]),
            "age_s": age,
            "max_gap_s": float(np.max(gaps)),
        }

    def stop(self):
        self._running = False
        if self._server is not None:
            try:
                self._server.close()
            except OSError:
                pass
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=2.0)


def _xyz(mapping):
    return np.array([mapping[k] for k in ("x", "y", "z")], dtype=float)


def _xyzw(mapping):
    return np.array([mapping[k] for k in ("x", "y", "z", "w")], dtype=float)


class D1Teleop:
    """左/右 Quest 手柄分别驱动左/右 D1。

    deadman: squeeze 是死手开关，按下沿做相对对齐，松开即停发。
    home: 机械臂先进入 Home；手柄放在舒适 Home 后长按 A 对齐并持续遥操，B 暂停。
    """

    def __init__(self, clients, ik, receiver, cfg, control_mode="full",
                 engagement_mode="deadman"):
        self.clients = clients
        self.ik = ik
        self.receiver = receiver
        self.cfg = cfg
        if control_mode not in TELEOP_CONTROL_MODES:
            raise ValueError(f"control_mode 必须是 {TELEOP_CONTROL_MODES} 之一")
        if engagement_mode not in TELEOP_ENGAGEMENT_MODES:
            raise ValueError(f"engagement_mode 必须是 {TELEOP_ENGAGEMENT_MODES} 之一")
        self.control_mode = control_mode
        self.engagement_mode = engagement_mode
        self.arms = tuple(clients)
        self.held = {arm: False for arm in self.arms}
        self.timeout_latched = {arm: False for arm in self.arms}
        self.home_resume_deadline = {arm: None for arm in self.arms}
        self.engage_joint_deg = {arm: None for arm in self.arms}
        # IK 以最后下发目标为连续参考，而不是以可能滞后的 D1 反馈逐帧重新起步。
        # 同时在下发前限制目标领先反馈的角度，兼顾跟手性和真实机械臂跟踪能力。
        self.command_joint_deg = {arm: None for arm in self.arms}
        self.align_pressed_since = {arm: None for arm in self.arms}
        self.align_consumed = {arm: False for arm in self.arms}
        self.pause_button_prev = {arm: False for arm in self.arms}
        self.last_diag_time = {arm: 0.0 for arm in self.arms}
        self.last_ik_diag_time = {arm: 0.0 for arm in self.arms}
        self.last_limit_log_time = {arm: 0.0 for arm in self.arms}
        self.arm_cfg = {
            arm: {**cfg, **cfg.get("arms", {}).get(arm, {})}
            for arm in self.arms
        }
        self.mapping = {}
        for arm in self.arms:
            arm_cfg = self.arm_cfg[arm]
            max_position_delta_m = arm_cfg["max_position_delta_m"]
            if self.control_mode in ("position", "orientation"):
                max_position_delta_m = arm_cfg.get(
                    "calibration_max_position_delta_m", max_position_delta_m)
            self.mapping[arm] = ControllerMapping(
                position_scale=arm_cfg["position_scale"],
                position_axes=arm_cfg["position_axes"],
                position_signs=arm_cfg["position_signs"],
                max_position_delta_m=max_position_delta_m,
                orientation_scale=arm_cfg["orientation_scale"],
                max_orientation_delta_deg=arm_cfg["max_orientation_delta_deg"],
                orientation_axes=arm_cfg.get("orientation_axes"),
                orientation_signs=arm_cfg.get("orientation_signs"),
            )

    def _print_diagnostics(self, arm, state, age_s=None):
        now = time.monotonic()
        if now - self.last_diag_time[arm] < 1.0:
            return
        self.last_diag_time[arm] = now
        if hasattr(self.receiver, "get_stats"):
            stats = self.receiver.get_stats(arm)
            age_s = stats["age_s"]
            age_text = "none" if age_s is None else f"{age_s * 1000:.0f}ms"
            gap = stats["max_gap_s"]
            gap_text = "none" if gap is None else f"{gap * 1000:.0f}ms"
            print(f"[teleop:{arm}] state={state} mode={self.control_mode} "
                  f"engagement={self.engagement_mode} rx={stats['hz']:.1f}Hz "
                  f"age={age_text} max_gap={gap_text}")
        else:
            age_text = "none" if age_s is None else f"{age_s * 1000:.0f}ms"
            print(f"[teleop:{arm}] state={state} mode={self.control_mode} age={age_text}")

    def _clear_engagement(self, arm):
        self.held[arm] = False
        self.engage_joint_deg[arm] = None
        self.command_joint_deg[arm] = None

    @staticmethod
    def _button(msg, index):
        buttons = msg.get("buttons", []) if msg else []
        return bool(index < len(buttons) and buttons[index])

    def _engage(self, arm, msg, feedback_deg, controller_pos, controller_quat):
        q_rad = np.deg2rad(np.asarray(feedback_deg[:6], dtype=float))
        if hasattr(self.ik[arm], "condition_number"):
            orientation_weight = float(
                self.arm_cfg[arm].get("calibration_orientation_weight", 0.5))
            condition = self.ik[arm].condition_number(q_rad, orientation_weight)
            max_condition = float(
                self.arm_cfg[arm].get("max_ik_condition_number", 80.0))
            if not np.isfinite(condition) or condition > max_condition:
                print(f"[teleop:{arm}] 拒绝接合：当前 Home/起始姿态接近奇异 "
                      f"(condition={condition:.1f} > {max_condition:g})；"
                      "请弯曲肘部和腕部后重新 capture-home")
                self._clear_engagement(arm)
                return False
        ee = self.ik[arm].forward_kinematics(q_rad)
        self.mapping[arm].reset(
            controller_pos, controller_quat, ee[:3, 3],
            matrix_to_quat(ee[:3, :3]))
        self.held[arm] = True
        self.engage_joint_deg[arm] = np.asarray(feedback_deg[:6], dtype=float).copy()
        self.command_joint_deg[arm] = self.engage_joint_deg[arm].copy()
        print(f"[teleop:{arm}] 已对齐当前手柄与机械臂位姿 "
              f"(control={self.control_mode}, engagement={self.engagement_mode})")
        joints_text = ", ".join(
            f"{value:.1f}" for value in self.engage_joint_deg[arm])
        tcp_text = ", ".join(f"{value:.3f}" for value in ee[:3, 3])
        print(f"[teleop:{arm}] 接合关节=[{joints_text}]° TCP=[{tcp_text}]m")
        margin = float(self.arm_cfg[arm].get("joint_limit_warning_margin_deg", 5.0))
        remaining = D1_JOINT_LIMITS_DEG - np.abs(self.engage_joint_deg[arm])
        near = np.flatnonzero(remaining < margin)
        if near.size:
            detail = ", ".join(
                f"J{joint} 剩 {remaining[joint]:.1f}°" for joint in near)
            print(f"[teleop:{arm}] 警告：接近关节极限 ({detail})；"
                  "该方向可能不可达，请暂停并调整 Home 位姿")
        return True

    def _home_alignment_requested(self, arm, msg, now):
        """处理 Home 模式的 A 长按对齐与 B 暂停，返回本周期是否应对齐。"""
        align_index = int(self.cfg.get("home_align_button_index", 4))
        pause_index = int(self.cfg.get("home_pause_button_index", 5))
        align_pressed = self._button(msg, align_index)
        pause_pressed = self._button(msg, pause_index)

        if pause_pressed and not self.pause_button_prev[arm]:
            self.home_resume_deadline[arm] = None
            self._clear_engagement(arm)
            print(f"[teleop:{arm}] B 暂停；将手柄放回舒适 Home 后长按 A 重新对齐")
        self.pause_button_prev[arm] = pause_pressed

        if not align_pressed:
            self.align_pressed_since[arm] = None
            self.align_consumed[arm] = False
            return False
        if self.align_consumed[arm]:
            return False
        if self.align_pressed_since[arm] is None:
            self.align_pressed_since[arm] = now
            return False
        hold_s = float(self.cfg.get("home_align_hold_s", 0.8))
        if now - self.align_pressed_since[arm] < hold_s:
            return False
        self.align_consumed[arm] = True
        return True

    def step(self):
        for arm in self.arms:
            arm_cfg = self.arm_cfg[arm]
            msg = self.receiver.get_latest(arm)
            now = time.monotonic()
            received = None if msg is None else msg.get("_received_monotonic", now)
            age_s = None if received is None else now - received
            fresh = age_s is not None and age_s <= self.cfg["controller_timeout_s"]
            if not fresh:
                if self.engagement_mode == "home":
                    if self.held[arm]:
                        rearm_s = float(self.cfg.get("home_rearm_timeout_s", 3.0))
                        self.home_resume_deadline[arm] = (
                            None if received is None else received + rearm_s)
                        print(f"[teleop:{arm}] 手柄追踪中断，Home 模式已暂停；"
                              f"{rearm_s:g} 秒内恢复将自动重新对齐")
                    self._clear_engagement(arm)
                    stale_state = "WAIT_CONTROLLER" if msg is None else "STALE_PAUSED"
                    self._print_diagnostics(arm, stale_state, age_s)
                    continue
                was_pressed = bool(
                    msg and float(msg.get("squeeze", 0.0)) >= self.cfg["deadman_threshold"])
                latch_after_s = float(self.cfg.get(
                    "controller_rearm_timeout_s", self.cfg["controller_timeout_s"]))
                should_latch = (
                    age_s is not None and age_s > latch_after_s and
                    (self.held[arm] or was_pressed))
                if should_latch:
                    if not self.timeout_latched[arm]:
                        age_text = "无数据" if age_s is None else f"{age_s * 1000:.0f}ms"
                        print(f"[teleop:{arm}] 手柄连续断流 ({age_text})，已锁止；"
                              "请松开侧握键后重新按下")
                    self.timeout_latched[arm] = True
                self._clear_engagement(arm)
                if self.timeout_latched[arm]:
                    stale_state = "TIMEOUT_LATCHED"
                elif msg is None:
                    stale_state = "WAIT_CONTROLLER"
                else:
                    stale_state = "STALE_PAUSED"
                self._print_diagnostics(arm, stale_state, age_s)
                continue

            align_requested = False
            if self.engagement_mode == "home":
                align_requested = self._home_alignment_requested(arm, msg, now)
                resume_deadline = self.home_resume_deadline[arm]
                if resume_deadline is not None:
                    self.home_resume_deadline[arm] = None
                    if now <= resume_deadline:
                        align_requested = True
                        print(f"[teleop:{arm}] 手柄短暂断流已恢复，自动重新对齐")
                if align_requested:
                    self.home_resume_deadline[arm] = None
            else:
                pressed = bool(
                    msg and float(msg.get("squeeze", 0.0)) >=
                    self.cfg["deadman_threshold"])
                if self.timeout_latched[arm]:
                    if pressed:
                        self._print_diagnostics(arm, "RELEASE_TO_REARM", age_s)
                        continue
                    self.timeout_latched[arm] = False
                    print(f"[teleop:{arm}] 已检测到侧握键松开，可以重新接合")

                if not pressed:
                    self._clear_engagement(arm)
                    self._print_diagnostics(arm, "READY", age_s)
                    continue

            feedback_deg = self.clients[arm].get_state()
            if (feedback_deg is None or len(feedback_deg) != 7 or
                    not np.all(np.isfinite(np.asarray(feedback_deg, dtype=float)))):
                continue
            q_rad = np.deg2rad(np.asarray(feedback_deg[:6], dtype=float))
            controller_pos, controller_quat = _xyz(msg["p"]), _xyzw(msg["q"])
            if not (np.all(np.isfinite(controller_pos)) and
                    np.all(np.isfinite(controller_quat))):
                continue

            if self.engagement_mode == "home":
                if align_requested:
                    if self._engage(
                            arm, msg, feedback_deg,
                            controller_pos, controller_quat):
                        print(f"[teleop:{arm}] Home 对齐完成；"
                              "无需按侧握键，按 B 暂停")
                if not self.held[arm]:
                    self._print_diagnostics(arm, "WAIT_HOME_ALIGN", age_s)
                    continue
            elif not self.held[arm]:
                self._engage(arm, msg, feedback_deg, controller_pos, controller_quat)
                if not self.held[arm]:
                    continue

            target_pos, target_quat = self.mapping[arm].map(controller_pos, controller_quat)
            neutral_pos, neutral_quat = self.mapping[arm].neutral_ee
            if self.control_mode == "position":
                # 平移标定：使用手柄位置增量，但把末端姿态锁在接合瞬间。
                target_quat = neutral_quat
            elif self.control_mode == "orientation":
                # TCP 标定：使用手柄旋转增量，但把末端位置锁在接合瞬间。
                target_pos = neutral_pos
            elif not arm_cfg["control_orientation"]:
                # 旧配置项 false 也采用锁姿态语义，避免“姿态不约束”时腕部自行旋转。
                target_quat = neutral_quat
            current_deg = np.asarray(feedback_deg[:6], dtype=float)
            reference_deg = self.command_joint_deg[arm]
            max_lead = float(arm_cfg.get("max_command_lead_deg", 6.0))
            if reference_deg is None:
                reference_deg = current_deg.copy()
            elif np.max(np.abs(reference_deg - current_deg)) > max_lead:
                # 反馈已越过/偏离旧命令窗口（例如外力移动或内部轨迹超调），
                # 立即以真实状态重新建立连续解，避免向过期命令跳回。
                reference_deg = current_deg.copy()
                self.command_joint_deg[arm] = reference_deg.copy()
            reference_rad = np.deg2rad(reference_deg)
            try:
                # 所有遥操模式都从最后下发目标做一次阻尼微分 IK。旧 full 模式
                # 每帧调用全局 IK，实测会在等价解支间跳转并把 J1/J4 推到限位，
                # 最终报 initial guess 越界。连续 IK 只沿当前解支追踪一小段误差。
                calibration_mode = self.control_mode in ("position", "orientation")
                position_step_key = (
                    "calibration_position_step_m" if calibration_mode
                    else "teleop_position_step_m")
                orientation_step_key = (
                    "calibration_orientation_step_deg" if calibration_mode
                    else "teleop_orientation_step_deg")
                target_rad = self.ik[arm].solve_continuous_step(
                    target_pos,
                    target_quat,
                    reference_rad,
                    max_position_step_m=float(
                        arm_cfg.get(position_step_key, 0.005)),
                    max_orientation_step_deg=float(
                        arm_cfg.get(orientation_step_key, 3.0)),
                    damping=float(arm_cfg.get("continuous_ik_damping", arm_cfg.get(
                        "calibration_ik_damping", 0.05))),
                    orientation_weight=float(arm_cfg.get(
                        "continuous_orientation_weight", arm_cfg.get(
                            "calibration_orientation_weight", 0.5))),
                )
            except (ValueError, RuntimeError) as exc:
                if now - self.last_limit_log_time[arm] >= 1.0:
                    print(f"[teleop:{arm}] IK 失败，保持上一目标: {exc}")
                    self.last_limit_log_time[arm] = now
                continue
            target_deg = np.rad2deg(target_rad)
            if target_deg.shape != (6,) or not np.all(np.isfinite(target_deg)):
                if now - self.last_limit_log_time[arm] >= 1.0:
                    print(f"[teleop:{arm}] IK 返回非法关节目标，保持上一目标")
                    self.last_limit_log_time[arm] = now
                continue
            raw_step = target_deg - reference_deg
            # 连续 IK 若仍算出大步，整帧拒绝。不能把异常解裁成一个较小
            # 关节目标，因为连续多帧裁剪仍会把机械臂带向错误方向。
            max_continuous_step = float(arm_cfg.get(
                "calibration_max_joint_step_deg" if calibration_mode
                else "teleop_max_joint_step_deg",
                5.0))
            offending = np.flatnonzero(np.abs(raw_step) > max_continuous_step)
            if offending.size:
                if now - self.last_limit_log_time[arm] >= 1.0:
                    detail = ", ".join(
                        f"J{joint} {raw_step[joint]:+.1f}°" for joint in offending)
                    print(f"[teleop:{arm}] 拒绝不连续 IK 解 ({detail}; "
                          f"阈值 {max_continuous_step:g}°)，本周期保持")
                    self.last_limit_log_time[arm] = now
                continue
            total_delta = float(arm_cfg["max_joint_delta_from_engage_deg"])
            if calibration_mode:
                total_delta = float(arm_cfg.get(
                    "calibration_max_joint_delta_from_engage_deg", total_delta))
            engage_deg = self.engage_joint_deg[arm]
            unclipped_deg = target_deg.copy()
            target_deg = np.clip(target_deg, engage_deg - total_delta, engage_deg + total_delta)
            total_limited = not np.allclose(target_deg, unclipped_deg)
            max_step = float(arm_cfg["max_joint_step_deg"])
            if calibration_mode:
                max_step = float(arm_cfg.get("calibration_max_joint_step_deg", max_step))
            before_step = target_deg.copy()
            target_deg = np.clip(
                target_deg, reference_deg - max_step, reference_deg + max_step)
            step_limited = not np.allclose(target_deg, before_step)

            # 命令最多领先真实反馈一个小窗口。这样既能跨过低频反馈造成的台阶，
            # 又不会在机械臂尚未跟上时把几十个 IK 小步堆积成一次大动作。
            before_lead = target_deg.copy()
            target_deg = np.clip(
                target_deg, current_deg - max_lead, current_deg + max_lead)
            lead_limited = not np.allclose(target_deg, before_lead)
            before_hard_limits = target_deg.copy()
            target_deg = np.clip(target_deg, -D1_JOINT_LIMITS_DEG, D1_JOINT_LIMITS_DEG)
            hard_limited = not np.allclose(target_deg, before_hard_limits)

            if ((total_limited or step_limited or lead_limited or hard_limited) and
                    now - self.last_limit_log_time[arm] >= 1.0):
                reasons = []
                if total_limited:
                    reasons.append(f"接合点±{total_delta:g}°")
                if step_limited:
                    reasons.append(f"单周期±{max_step:g}°")
                if lead_limited:
                    reasons.append(f"领先反馈±{max_lead:g}°")
                if hard_limited:
                    reasons.append("D1 物理关节限位")
                print(f"[teleop:{arm}] 关节目标被限幅: {', '.join(reasons)}")
                self.last_limit_log_time[arm] = now

            diag_interval = float(arm_cfg.get(
                "teleop_diagnostics_interval_s",
                arm_cfg.get("calibration_diagnostics_interval_s", 0.25)))
            if now - self.last_ik_diag_time[arm] >= diag_interval:
                current_frame = self.ik[arm].forward_kinematics(q_rad)
                command_frame = self.ik[arm].forward_kinematics(
                    np.deg2rad(target_deg))
                hand_mm = (
                    controller_pos - self.mapping[arm].neutral_controller[0]) * 1000.0
                mapped_mm = (
                    np.asarray(target_pos) - np.asarray(neutral_pos)) * 1000.0
                actual_mm = (
                    current_frame[:3, 3] - np.asarray(neutral_pos)) * 1000.0
                command_mm = (
                    command_frame[:3, 3] - current_frame[:3, 3]) * 1000.0
                lag_deg = np.max(np.abs(target_deg - current_deg))
                hand_text = ",".join(f"{value:+.1f}" for value in hand_mm)
                mapped_text = ",".join(f"{value:+.1f}" for value in mapped_mm)
                actual_text = ",".join(f"{value:+.1f}" for value in actual_mm)
                command_text = ",".join(f"{value:+.1f}" for value in command_mm)
                print(f"[teleop:{arm}] 跟踪 hand_xyz=[{hand_text}]mm "
                      f"mapped_xyz=[{mapped_text}]mm "
                      f"actual_xyz=[{actual_text}]mm "
                      f"command_step_xyz=[{command_text}]mm "
                      f"max_dq={np.max(np.abs(raw_step)):.2f}° "
                      f"command_lag={lag_deg:.2f}°")
                self.last_ik_diag_time[arm] = now
            if self.control_mode in ("position", "orientation"):
                # 标定期间固定夹爪，避免松开的食指扳机同时改变开合。
                gripper = float(feedback_deg[6])
            else:
                trigger = np.clip(float(msg.get("trigger", 0.0)), 0.0, 1.0)
                gripper = arm_cfg["gripper_open"] + trigger * (
                    arm_cfg["gripper_closed"] - arm_cfg["gripper_open"])
                gripper_step = float(arm_cfg["max_gripper_step"])
                gripper = float(np.clip(
                    gripper, feedback_deg[6] - gripper_step, feedback_deg[6] + gripper_step))
            self.clients[arm].set_target(np.r_[target_deg, gripper], mode=contract.D1_CMD_MODE)
            self.command_joint_deg[arm] = target_deg.copy()
            self._print_diagnostics(arm, "ENGAGED", age_s)


def current_state(d1: dict[str, D1Client]) -> np.ndarray:
    s = np.zeros(contract.STATE_DIM, dtype=np.float32)
    for arm, sl in (("left", contract.D1L_SLICE), ("right", contract.D1R_SLICE)):
        client = d1.get(arm)
        q = client.get_state() if client is not None else None
        if q is not None:
            s[sl] = q
    # TODO(现场): cambot 段（CAMBOT_SLICE）经 cambot WS 遥测读取
    return s


def encode_images(images: dict) -> dict[str, bytes]:
    # TODO(现场): ndarray -> jpeg bytes（cv2.imencode）
    return {}


def dispatch(action, d1: dict[str, D1Client]) -> None:
    a = np.asarray(action, dtype=np.float32)
    for arm, sl in (("left", contract.D1L_SLICE), ("right", contract.D1R_SLICE)):
        if arm in d1:
            d1[arm].set_target(a[sl], mode=contract.D1_CMD_MODE)
    # TODO(现场): cambot 段（CAMBOT_SLICE）经其控制接口下发


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="robot/config.yaml")
    ap.add_argument("--mode", default="teleop", choices=["teleop", "inference", "idle"])
    ap.add_argument("--arm", default="both", choices=["left", "right", "both"],
                    help="只连接并控制指定 D1；单臂模式只读取对应 Quest 手柄")
    ap.add_argument("--control-mode", default="full", choices=TELEOP_CONTROL_MODES,
                    help="full=完整6DoF，position=锁姿态只平移，orientation=锁位置只旋转")
    ap.add_argument("--engagement-mode", default="deadman",
                    choices=TELEOP_ENGAGEMENT_MODES,
                    help="deadman=持续按侧握；home=进入Home后长按A对齐、B暂停")
    ap.add_argument("--startup-home", action="store_true",
                    help="连接后先低速进入已捕获的单臂 Home，再开放遥操")
    ap.add_argument("--fps", type=float, default=None)
    args = ap.parse_args()

    cfg = yaml.safe_load(open(args.config, encoding="utf-8"))
    fps = args.fps
    if fps is None:
        fps = cfg["teleop"].get("control_fps", contract.CONTROL_FPS) \
            if args.mode == "teleop" else contract.CONTROL_FPS
    if fps <= 0:
        raise ValueError("fps 必须大于 0")

    # ---- 组件 ----
    active_arms = ("left", "right") if args.arm == "both" else (args.arm,)
    d1 = {
        arm: D1Client(host=cfg["robot"][arm]["bridge_host"],
                      port=cfg["robot"][arm]["bridge_port"], arm=arm)
        for arm in active_arms
    }
    for c in d1.values():
        c.connect()

    if args.startup_home:
        if len(active_arms) != 1:
            raise RuntimeError("自动 Home 当前只允许 --arm left 或 --arm right 单臂测试")
        home_cfg = cfg["teleop"]["startup_home"]
        arm = active_arms[0]
        move_single_arm_to_home(d1[arm], home_cfg["file"], home_cfg)

    ik = {arm: D1IK() for arm in active_arms}
    camera = create_camera(cfg["camera"])
    collector = Collector(cfg["collection"], camera, lambda: current_state(d1))

    forward = ForwardReceiver(
        cfg["robot"]["cambot"]["forward_host"], cfg["robot"]["cambot"]["forward_port"])
    forward.on_episode(lambda _e: collector.toggle_episode())
    forward.start()
    teleop = D1Teleop(
        d1, ik, forward, cfg["teleop"], args.control_mode,
        args.engagement_mode)

    sock = None
    if args.mode == "inference":
        ctx = zmq.Context()
        sock = ctx.socket(zmq.REQ)
        sock.connect(f"tcp://{cfg['server']['host']}:{cfg['server']['port']}")
        print(f"[control] 推理模式: 服务器 {cfg['server']['host']}:{cfg['server']['port']}")

    print(f"[control] 模式 {args.mode}，D1={','.join(active_arms)}，"
          f"control_mode={args.control_mode}，"
          f"engagement_mode={args.engagement_mode}，fps {fps}")
    dt = 1.0 / fps
    try:
        while True:
            t0 = time.monotonic()
            if args.mode == "teleop":
                teleop.step()
            elif args.mode == "inference":
                frame = camera.get_frame()
                obs = pack_obs(current_state(d1), encode_images(frame["images"]), time.time())
                sock.send_multipart(obs)
                action = unpack_action(sock.recv())
                dispatch(action["action"], d1)
            time.sleep(max(0.0, dt - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        print("\n[control] 退出")
    finally:
        if args.mode == "teleop" and cfg["teleop"].get("disable_on_exit", True):
            print(f"[control] 遥操退出：向 {','.join(active_arms)} 发送卸力命令")
            for c in d1.values():
                try:
                    c.disable()
                except OSError:
                    pass
            time.sleep(0.1)
        for c in d1.values():
            c.close()
        camera.close()
        collector.stop()
        forward.stop()
        if sock:
            sock.close()


if __name__ == "__main__":
    main()
