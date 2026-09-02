"""数据契约：全局共享常量。

现场确认后**只改这个文件**：
- 相机路数与键名（NQ2）
- 夹爪语义（NQ5）
- fps（QD3，默认 30）
- 服务器 IP/端口（NQ4）
"""
from __future__ import annotations

# ---- 臂结构 ----
D1_JOINTS = 6            # D1 本体 6 轴
D1_GRIPPER_IDX = 6       # 夹爪为第 7 个控制量（angle6）
D1_VALUES = 7            # funcode 2 的 angle0..angle6
CAMBOT_JOINTS = 6

# state/action 布局: [D1L(7), D1R(7), cambot(6)] = 20
D1L_SLICE = slice(0, D1_VALUES)
D1R_SLICE = slice(D1_VALUES, 2 * D1_VALUES)
CAMBOT_SLICE = slice(2 * D1_VALUES, 2 * D1_VALUES + CAMBOT_JOINTS)
STATE_DIM = 2 * D1_VALUES + CAMBOT_JOINTS
ARM_NAMES = ("left", "right", "cambot")

# ---- 频率 ----
CONTROL_FPS = 30          # 采集/控制目标 fps（QD3 默认 30，可调）
D1_FEEDBACK_HZ = 10       # 官方：关节角度反馈 10 Hz
D1_CMD_MODE = 0           # funcode 2 的 mode：0 = 10Hz 小平滑（遥操推荐）；1 = 轨迹大平滑

# ---- 图像（TODO(现场): NQ2 相机方案确认后修改）----
IMAGE_KEYS = ("cam_front",)   # 相机键名，按实际路数增减
IMAGE_WIDTH = 1280            # 分辨率（可能 1280x720，现场定）
IMAGE_HEIGHT = 720
IMAGE_ENCODING = "jpeg"       # 传输/存储编码

# ---- 端口与地址 ----
CAMBOT_FORWARD_HOST = "127.0.0.1"   # cambot_patch 转发手柄数据的本地地址
CAMBOT_FORWARD_PORT = 6001          # 转发端口（与 robot/config.yaml 一致）
D1_BRIDGE_PORTS = {"left": 5500, "right": 5501}   # d1_bridge TCP 监听端口
SERVER_HOST = "127.0.0.1"           # TODO(现场): 服务器端 IP
SERVER_ZMQ_PORT = 6000              # 机器人端 <-> 服务器端（ZMQ REP）

# ---- 夹爪语义（TODO(现场): NQ5 确认 angle6 单位与范围）----
GRIPPER_CLOSED = 0.0        # 关闭对应的 angle6 值
GRIPPER_OPEN = 65.0         # 打开对应的值（官方行程 0–65 mm，待现场验证）
