"""d1_bridge 的 Python 客户端（TCP JSON line）。

协议（换行分隔 JSON）：
  客户端 -> 桥:
    {"cmd": "target", "angles": [7], "mode": 0}   # funcode 2 全关节角
    {"cmd": "enable" | "disable"}                 # funcode 5（1 使能 / 0 卸力）
    {"cmd": "power_on" | "power_off"}             # funcode 6（1 上电 / 0 断电）
    {"cmd": "zero"}                               # funcode 7 归零
  桥 -> 客户端:
    {"type": "feedback", "angles": [7], "t": ...} # 臂反馈（官方 10 Hz）转发
"""
from __future__ import annotations
import json
import socket
import threading


class D1Client:
    def __init__(self, host: str = "127.0.0.1", port: int = 5500, arm: str = "left"):
        self.host, self.port, self.arm = host, port, arm
        self._sock: socket.socket | None = None
        self._lock = threading.Lock()
        self._recv_thread: threading.Thread | None = None
        self._running = False
        self.latest_feedback: dict | None = None

    def connect(self) -> None:
        self._sock = socket.create_connection((self.host, self.port), timeout=3)
        self._running = True
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()
        print(f"[d1_client:{self.arm}] 已连接 {self.host}:{self.port}")

    def _recv_loop(self) -> None:
        buf = b""
        while self._running:
            try:
                data = self._sock.recv(4096)
            except OSError:
                break
            if not data:
                break
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    self.latest_feedback = json.loads(line)
                except json.JSONDecodeError:
                    pass

    def _send(self, payload: dict) -> None:
        with self._lock:
            if self._sock is None:
                raise RuntimeError(f"D1Client({self.arm}) 未连接")
            self._sock.sendall((json.dumps(payload) + "\n").encode())

    # ---- 命令 ----
    def set_target(self, angles, mode: int = 0) -> None:
        """angles: 7 值（6 关节 + 夹爪）。mode 同 funcode 2（0 小平滑/1 大平滑）。"""
        self._send({"cmd": "target", "angles": [float(a) for a in angles], "mode": int(mode)})

    def enable(self) -> None:
        self._send({"cmd": "enable"})

    def disable(self) -> None:
        self._send({"cmd": "disable"})

    def power_on(self) -> None:
        self._send({"cmd": "power_on"})

    def power_off(self) -> None:
        self._send({"cmd": "power_off"})

    def zero(self) -> None:
        self._send({"cmd": "zero"})

    # ---- 反馈 ----
    def get_state(self):
        """最新 7 值角度反馈；尚无数据返回 None。"""
        fb = self.latest_feedback
        return fb.get("angles") if fb else None

    def close(self) -> None:
        self._running = False
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
