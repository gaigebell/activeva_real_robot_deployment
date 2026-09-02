"""相机抽象。TODO(现场): NQ2 确认后实现后端。

- backend="dummy": 测试用合成图像（先跑通链路）
- backend="realsense": pyrealsense2（D435，NQ2 候选）
- backend="zed": pyzed（cambot 原装 ZED Mini）
分辨率可能 1280x720；深度是否采集现场定（cfg.with_depth）。
"""
from __future__ import annotations
import time

import numpy as np


class Camera:
    """统一接口：get_frame() -> {"images": {key: HxWx3 ndarray}, "depth": {key: ndarray} | None}"""

    def get_frame(self):
        raise NotImplementedError

    def close(self):
        pass


def create_camera(cfg) -> Camera:
    backend = cfg.get("backend", "dummy")
    if backend == "dummy":
        return DummyCamera(cfg)
    if backend == "realsense":
        return RealsenseCamera(cfg)
    if backend == "zed":
        return ZedCamera(cfg)
    raise ValueError(f"未知相机后端: {backend}")


class DummyCamera(Camera):
    """合成图像：移动色块 + 帧号，用于无相机时验证全链路。"""

    def __init__(self, cfg):
        self.w = cfg.get("width", 1280)
        self.h = cfg.get("height", 720)
        self.fps = cfg.get("fps", 30)
        self._t0 = time.time()
        self._n = 0

    def get_frame(self):
        self._n += 1
        t = time.time() - self._t0
        img = np.full((self.h, self.w, 3), 60, dtype=np.uint8)
        x = int((np.sin(t) * 0.5 + 0.5) * (self.w - 50))
        img[:, x:x + 50] = 200
        cv2_put_text(img, f"frame {self._n} t={t:.1f}s", (20, 40))
        return {"images": {"cam_front": img}, "depth": None}


def cv2_put_text(img, text, org):
    """无 cv2 时的简易文字（TODO(现场): 直接换 cv2.putText）。"""
    # 占位：真实相机路径用 cv2；dummy 用像素条即可
    pass


class RealsenseCamera(Camera):
    def __init__(self, cfg):
        raise NotImplementedError(
            "TODO(现场): pip install pyrealsense2；"
            "配置彩色/深度流（cfg.width/height/fps/with_depth），返回 {'images': {'cam_front': rgb}, 'depth': {...}}"
        )


class ZedCamera(Camera):
    def __init__(self, cfg):
        raise NotImplementedError(
            "TODO(现场): 安装 ZED SDK + pyzed；输出双目/单目帧（键名与 contract.IMAGE_KEYS 对齐）"
        )
