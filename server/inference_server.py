"""服务器端推理服务骨架（ZMQ REP）。

流程：监听端口 → 收机器人端 obs 帧（image + state）→ 模型推理 → 回 action。
模型为占位（原样回传 state，用于打通链路）；真实策略现场接入。

TODO(现场):
- 接入真实策略（ManiFlow/DP3/lerobot policy），按模型 I/O 契约适配（chunk、图像键、归一化）
- 单样本延迟优化（CUDA graph / torch.compile / 流式）
"""
from __future__ import annotations
import argparse
import time

import yaml
import zmq

from common.protocol import unpack_obs, pack_action


class PolicyStub:
    """链路测试占位：输出与输入 state 相同（臂保持不动）。"""

    def __init__(self, cfg):
        self.cfg = cfg

    def infer(self, state, images):
        # TODO(现场): 解码图像（契约见 common/protocol.py）-> 模型前向 -> action
        return list(state)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="server/config.yaml")
    args = ap.parse_args()
    cfg = yaml.safe_load(open(args.config, encoding="utf-8"))

    policy = PolicyStub(cfg["model"])
    ctx = zmq.Context()
    sock = ctx.socket(zmq.REP)
    sock.bind(f"tcp://0.0.0.0:{cfg['server']['port']}")
    print(f"[inference_server] 监听 0.0.0.0:{cfg['server']['port']}（模型: {cfg['model']['type']}）")

    n = 0
    t_sum = 0.0
    try:
        while True:
            frames = sock.recv_multipart()
            t0 = time.monotonic()
            obs = unpack_obs(frames)
            action = policy.infer(obs["state"], obs["images"])
            sock.send(pack_action(action, obs["t"]))
            t_sum += time.monotonic() - t0
            n += 1
            if n % 30 == 0:
                print(f"[inference_server] {n} 帧, 平均处理 {t_sum / n * 1000:.1f} ms")
    except KeyboardInterrupt:
        print("\n[inference_server] 退出")


if __name__ == "__main__":
    main()
