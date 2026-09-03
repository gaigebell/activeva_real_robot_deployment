"""Quest controller forwarding monitor. Does not connect to or command D1 arms."""
from __future__ import annotations

import argparse
import time

import yaml

from robot.control import ForwardReceiver


def main():
    parser = argparse.ArgumentParser(
        description="只读监视 Quest 左右手柄（不连接 D1）")
    parser.add_argument("--config", default="robot/config.yaml")
    parser.add_argument("--seconds", type=float, default=0.0,
                        help="0 表示持续运行，直到 Ctrl+C")
    args = parser.parse_args()

    cfg = yaml.safe_load(open(args.config, encoding="utf-8"))["robot"]["cambot"]
    receiver = ForwardReceiver(cfg["forward_host"], cfg["forward_port"])
    receiver.start()
    print("请在 Quest 中进入 VR，移动左右手柄并测试侧握键/扳机。")
    print("本程序不连接 D1，不会发送机械臂命令。")
    started = time.monotonic()
    try:
        while args.seconds <= 0 or time.monotonic() - started < args.seconds:
            fields = []
            for hand in ("left", "right"):
                msg = receiver.get_latest(hand, max_age_s=1.0)
                if msg is None:
                    fields.append(f"{hand}: WAIT")
                    continue
                p = msg.get("p", {})
                fields.append(
                    f"{hand}: p=({p.get('x', 0):+.3f},{p.get('y', 0):+.3f},"
                    f"{p.get('z', 0):+.3f}) squeeze={float(msg.get('squeeze', 0)):.2f} "
                    f"trigger={float(msg.get('trigger', 0)):.2f}")
            print(" | ".join(fields), flush=True)
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n停止手柄监视。")
    finally:
        receiver.stop()


if __name__ == "__main__":
    main()
