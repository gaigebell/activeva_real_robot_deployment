"""机器人端主控制程序。

模式：
  teleop      手柄（经 cambot_patch 转发）-> mapping -> IK -> D1；头显 -> cambot（cambot 自理）
  inference   采集 obs -> 服务器推理 -> action -> 三臂执行（D1 经 d1_bridge；cambot 经其控制接口）
  idle        仅维护 D1 连接与反馈，不发指令

TODO(现场):
- ForwardSubscriber：订阅 cambot 转发的 controller_pose/episode 消息（本地 TCP 6001）
- cambot 关节读取与下发（走 cambot WS 遥测/命令）
- 安全看门狗：指令超时 -> D1 卸力（funcode 5 mode 0）
- 延迟统计（头显 -> 各臂）
"""
from __future__ import annotations
import argparse
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
from robot.controller_mapping import ControllerMapping
from robot.d1_client import D1Client
from robot.d1_ik import D1IK


class ForwardSubscriber:
    """订阅 cambot_patch 经本地 TCP 转发的手柄/按钮数据。

    消息（JSON line）: {"type": "controller_pose", "hand": "left"|"right",
                       "q": {x,y,z,w}, "p": {x,y,z}, "trigger": 0-1, "buttons": [...]}
                      {"type": "episode", "event": "toggle"}
    """

    def __init__(self, host, port):
        self.host, self.port = host, port
        self.latest: dict[str, dict] = {}
        self._episode_cb = None
        self._running = False

    def start(self):
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self._running:
            try:
                sock = socket.create_connection((self.host, self.port), timeout=3)
                buf = b""
                while self._running:
                    data = sock.recv(4096)
                    if not data:
                        break
                    buf += data
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        self._handle(json.loads(line))
            except (OSError, json.JSONDecodeError) as e:
                time.sleep(1)   # 断线重连
                print(f"[forward] 重连: {e}")
            finally:
                try:
                    sock.close()
                except Exception:
                    pass

    def _handle(self, msg):
        if msg.get("type") == "controller_pose":
            self.latest[msg.get("hand", "?")] = msg
        elif msg.get("type") == "episode" and self._episode_cb:
            self._episode_cb(msg.get("event", "toggle"))

    def on_episode(self, cb):
        self._episode_cb = cb


def current_state(d1: dict[str, D1Client]) -> np.ndarray:
    s = np.zeros(contract.STATE_DIM, dtype=np.float32)
    for arm, sl in (("left", contract.D1L_SLICE), ("right", contract.D1R_SLICE)):
        q = d1[arm].get_state()
        if q is not None:
            s[sl] = q
    # TODO(现场): cambot 段（CAMBOT_SLICE）经 cambot WS 遥测读取
    return s


def encode_images(images: dict) -> dict[str, bytes]:
    # TODO(现场): ndarray -> jpeg bytes（cv2.imencode）
    return {}


def dispatch(action, d1: dict[str, D1Client]) -> None:
    a = np.asarray(action, dtype=np.float32)
    d1["left"].set_target(a[contract.D1L_SLICE], mode=contract.D1_CMD_MODE)
    d1["right"].set_target(a[contract.D1R_SLICE], mode=contract.D1_CMD_MODE)
    # TODO(现场): cambot 段（CAMBOT_SLICE）经其控制接口下发


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="robot/config.yaml")
    ap.add_argument("--mode", default="teleop", choices=["teleop", "inference", "idle"])
    ap.add_argument("--fps", type=float, default=contract.CONTROL_FPS)
    args = ap.parse_args()

    cfg = yaml.safe_load(open(args.config, encoding="utf-8"))

    # ---- 组件 ----
    d1 = {
        arm: D1Client(host=c["bridge_host"], port=c["bridge_port"], arm=arm)
        for arm, c in cfg["robot"].items() if arm in ("left", "right")
    }
    for c in d1.values():
        c.connect()

    ik = {arm: D1IK() for arm in ("left", "right")}
    mapping = {arm: ControllerMapping() for arm in ("left", "right")}
    camera = create_camera(cfg["camera"])
    collector = Collector(cfg["collection"], camera, lambda: current_state(d1))

    forward = ForwardSubscriber(
        cfg["robot"]["cambot"]["forward_host"], cfg["robot"]["cambot"]["forward_port"])
    forward.on_episode(lambda _e: collector.toggle_episode())
    forward.start()

    sock = None
    if args.mode == "inference":
        ctx = zmq.Context()
        sock = ctx.socket(zmq.REQ)
        sock.connect(f"tcp://{cfg['server']['host']}:{cfg['server']['port']}")
        print(f"[control] 推理模式: 服务器 {cfg['server']['host']}:{cfg['server']['port']}")

    print(f"[control] 模式 {args.mode}，fps {args.fps}")
    dt = 1.0 / args.fps
    try:
        while True:
            t0 = time.monotonic()
            if args.mode == "teleop":
                # TODO(现场): 读 forward.latest 手柄位姿 -> mapping.reset/map ->
                #             ik.solve -> d1[arm].set_target(6 关节 + 扳机夹爪值)
                pass
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
        for c in d1.values():
            c.close()
        camera.close()
        collector.stop()
        if sock:
            sock.close()


if __name__ == "__main__":
    main()
