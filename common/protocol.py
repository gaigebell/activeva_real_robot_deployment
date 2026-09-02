"""机器人端 <-> 服务器端 ZMQ 消息封包。

帧格式（multipart）:
  请求: [header_json, img0_bytes, img1_bytes, ...]
    header_json: {"type": "obs", "t": <时间戳>, "state": [20],
                  "images": ["cam_front", ...]}   # 图像键名，与帧顺序一一对应
  响应: [header_json]
    header_json: {"type": "action", "t": <回传obs时间戳>, "action": [20]}

图像按 header 中 "images" 的顺序以编码字节流传输（编码方式见 contract.IMAGE_ENCODING）。
"""
from __future__ import annotations
import json
from typing import Any

import numpy as np


def pack_obs(state, images: dict[str, bytes], t: float) -> list[bytes]:
    """state: (20,) array-like；images: {key: jpeg_bytes}。返回 ZMQ multipart 帧。"""
    state = np.asarray(state, dtype=np.float32).tolist()
    keys = list(images.keys())
    header = {"type": "obs", "t": t, "state": state, "images": keys}
    return [json.dumps(header).encode()] + [bytes(images[k]) for k in keys]


def unpack_obs(frames: list[bytes]) -> dict[str, Any]:
    header = json.loads(frames[0])
    assert len(frames) - 1 == len(header["images"]), "图像帧数与 header 不一致"
    header["images"] = {k: frames[i + 1] for i, k in enumerate(header["images"])}
    header["state"] = np.asarray(header["state"], dtype=np.float32)
    return header


def pack_action(action, t: float) -> bytes:
    action = np.asarray(action, dtype=np.float32).tolist()
    return json.dumps({"type": "action", "t": t, "action": action}).encode()


def unpack_action(payload: bytes) -> dict[str, Any]:
    data = json.loads(payload)
    data["action"] = np.asarray(data["action"], dtype=np.float32)
    return data
