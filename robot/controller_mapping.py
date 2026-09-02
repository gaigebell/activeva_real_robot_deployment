"""手柄 6DOF 增量 -> D1 末端目标位姿（语义同仿真 vr/headset_control.py）。

- 启动/标定时记录手柄位姿与臂当前末端位姿为基准
- 手柄位姿增量施加到基准末端位姿上（相对运动）
- 扳机 = 夹爪（1 - index_trigger，仿真语义）
- TODO(现场): Quest 左手系 -> 机器人右手系转换、臂基座标定、增量限幅（安全）
"""
from __future__ import annotations
import numpy as np


def quat_inv(q: np.ndarray) -> np.ndarray:
    """xyzw 四元数求逆。"""
    return np.array([-q[0], -q[1], -q[2], q[3]]) / np.dot(q, q)


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """xyzw 四元数乘法。"""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    w = aw * bw - ax * bx - ay * by - az * bz
    x = aw * bx + ax * bw + ay * bz - az * by
    y = aw * by - ax * bz + ay * bw + az * bx
    z = aw * bz + ax * by - ay * bx + az * bw
    return np.array([x, y, z, w])


class ControllerMapping:
    def __init__(self):
        self.neutral_controller = None   # (pos3, quat4) 手柄基准
        self.neutral_ee = None           # (pos3, quat4) 臂末端基准
        self.enabled = False

    def reset(self, controller_pos, controller_quat, ee_pos, ee_quat) -> None:
        """记录基准（按下映射按钮时调用；同仿真启动对齐）。"""
        self.neutral_controller = (np.asarray(controller_pos, float), np.asarray(controller_quat, float))
        self.neutral_ee = (np.asarray(ee_pos, float), np.asarray(ee_quat, float))
        self.enabled = True

    def map(self, controller_pos, controller_quat):
        """返回末端目标 (pos3, quat4 xyzw)；未标定时返回 None。"""
        if not self.enabled:
            return None
        dp = np.asarray(controller_pos, float) - self.neutral_controller[0]
        dq = quat_mul(np.asarray(controller_quat, float), quat_inv(self.neutral_controller[1]))
        # TODO(现场): 手柄系 -> 机器人系旋转（参考仿真 TRANSFORM_TO_WORLD / convert_left_to_right_coordinates）
        return self.neutral_ee[0] + dp, quat_mul(dq, self.neutral_ee[1])
