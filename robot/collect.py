"""lerobot 数据采集（多线程实时写，QD5）。

线程模型：
  camera_thread   相机采集（独立，最新帧缓存）
  state_thread    读取 D1 反馈（官方 10 Hz）+ cambot 关节
  control_thread  遥操控制（robot/control.py 内）
  writer_thread   episode 期间每 CONTROL_FPS 拍一帧写入 LeRobotDataset

TODO(现场):
- pip install lerobot；核对 LeRobotDataset v3 API（视频编码/features）
- 数据契约最终键名（common/contract.py）
- episode 开始/结束按钮接线（RButtonOne，经 cambot_patch 转发）
"""
from __future__ import annotations
import threading
import time


class Collector:
    def __init__(self, cfg, camera, state_provider):
        self.cfg = cfg
        self.camera = camera
        self.state_provider = state_provider   # callable -> 20 维 state
        self.episode = None                    # TODO(现场): LeRobotDataset 实例
        self._stop = threading.Event()
        self._recording = False
        self._recording_lock = threading.Lock()

    # ---- episode 控制（由按钮事件触发）----
    def start_episode(self) -> None:
        with self._recording_lock:
            if self._recording:
                return
            # TODO(现场): self.episode = LeRobotDataset.create(repo_id, fps=..., robot_type=..., features=...)
            self._recording = True
        print("[collect] episode 开始")

    def stop_episode(self) -> None:
        with self._recording_lock:
            if not self._recording:
                return
            self._recording = False
            # TODO(现场): self.episode.save_episode()；可选 push_to_hub
        print("[collect] episode 结束")

    def toggle_episode(self) -> None:
        self.stop_episode() if self._recording else self.start_episode()

    # ---- 写盘线程 ----
    def writer_loop(self) -> None:
        dt = 1.0 / self.cfg.get("fps", 30)
        while not self._stop.is_set():
            t0 = time.monotonic()
            with self._recording_lock:
                recording = self._recording
            if recording:
                frame = self.camera.get_frame()
                state = self.state_provider()
                # TODO(现场): self.episode.add_frame({"observation.state": state,
                #   "action": action_used, "observation.images.*": 编码后的帧})
            time.sleep(max(0.0, dt - (time.monotonic() - t0)))

    def stop(self) -> None:
        self._stop.set()
        if self._recording:
            self.stop_episode()
