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


def quat_normalize(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        raise ValueError("四元数不能为零")
    return q / norm


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    x, y, z, w = quat_normalize(q)
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def matrix_to_quat(matrix: np.ndarray) -> np.ndarray:
    """3x3 旋转矩阵转 xyzw，并选择 w>=0 的最短路径表示。"""
    m = np.asarray(matrix, dtype=float)
    trace = np.trace(m)
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2
        q = np.array([
            (m[2, 1] - m[1, 2]) / s,
            (m[0, 2] - m[2, 0]) / s,
            (m[1, 0] - m[0, 1]) / s,
            0.25 * s,
        ])
    else:
        i = int(np.argmax(np.diag(m)))
        if i == 0:
            s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
            q = np.array([0.25*s, (m[0, 1]+m[1, 0])/s,
                          (m[0, 2]+m[2, 0])/s, (m[2, 1]-m[1, 2])/s])
        elif i == 1:
            s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
            q = np.array([(m[0, 1]+m[1, 0])/s, 0.25*s,
                          (m[1, 2]+m[2, 1])/s, (m[0, 2]-m[2, 0])/s])
        else:
            s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
            q = np.array([(m[0, 2]+m[2, 0])/s, (m[1, 2]+m[2, 1])/s,
                          0.25*s, (m[1, 0]-m[0, 1])/s])
    q = quat_normalize(q)
    return -q if q[3] < 0 else q


def clamp_quat_angle(q: np.ndarray, max_angle_rad: float | None) -> np.ndarray:
    """将相对旋转限制在单位四元数到 max_angle_rad 之内。"""
    q = quat_normalize(q)
    if q[3] < 0:
        q = -q
    if max_angle_rad is None:
        return q
    angle = 2.0 * np.arccos(np.clip(q[3], -1.0, 1.0))
    if angle <= max_angle_rad or angle < 1e-12:
        return q
    axis_norm = np.linalg.norm(q[:3])
    if axis_norm < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])
    axis = q[:3] / axis_norm
    half = max_angle_rad / 2.0
    return np.r_[axis * np.sin(half), np.cos(half)]


class ControllerMapping:
    def __init__(self, position_scale=1.0, position_axes=(0, 1, 2),
                 position_signs=(1, 1, 1), max_position_delta_m=None,
                 orientation_scale=1.0, max_orientation_delta_deg=None,
                 orientation_axes=None, orientation_signs=None):
        self.neutral_controller = None   # (pos3, quat4) 手柄基准
        self.neutral_ee = None           # (pos3, quat4) 臂末端基准
        self.enabled = False
        self.position_scale = float(position_scale)
        self.position_axes = np.asarray(position_axes, dtype=int)
        self.position_signs = np.asarray(position_signs, dtype=float)
        self.orientation_axes = np.asarray(
            position_axes if orientation_axes is None else orientation_axes,
            dtype=int)
        self.orientation_signs = np.asarray(
            position_signs if orientation_signs is None else orientation_signs,
            dtype=float)
        self.max_position_delta_m = max_position_delta_m
        self.orientation_scale = float(orientation_scale)
        if not 0.0 <= self.orientation_scale <= 1.0:
            raise ValueError("orientation_scale 必须在 0..1 之间")
        self.max_orientation_delta_rad = (
            None if max_orientation_delta_deg is None
            else np.deg2rad(float(max_orientation_delta_deg)))
        if sorted(self.position_axes.tolist()) != [0, 1, 2]:
            raise ValueError("position_axes 必须是 [0,1,2] 的排列")
        if not np.all(np.isin(self.position_signs, (-1, 1))):
            raise ValueError("position_signs 只能包含 -1 或 1")
        if sorted(self.orientation_axes.tolist()) != [0, 1, 2]:
            raise ValueError("orientation_axes 必须是 [0,1,2] 的排列")
        if not np.all(np.isin(self.orientation_signs, (-1, 1))):
            raise ValueError("orientation_signs 只能包含 -1 或 1")
        self.basis = np.zeros((3, 3), dtype=float)
        for robot_axis, controller_axis in enumerate(self.orientation_axes):
            self.basis[robot_axis, controller_axis] = self.orientation_signs[robot_axis]

    def reset(self, controller_pos, controller_quat, ee_pos, ee_quat) -> None:
        """记录基准（按下映射按钮时调用；同仿真启动对齐）。"""
        self.neutral_controller = (
            np.asarray(controller_pos, float), quat_normalize(controller_quat))
        self.neutral_ee = (np.asarray(ee_pos, float), quat_normalize(ee_quat))
        self.enabled = True

    def map(self, controller_pos, controller_quat):
        """返回末端目标 (pos3, quat4 xyzw)；未标定时返回 None。"""
        if not self.enabled:
            return None
        controller_dp = np.asarray(controller_pos, float) - self.neutral_controller[0]
        dp = controller_dp[self.position_axes] * self.position_signs * self.position_scale
        if self.max_position_delta_m is not None:
            norm = np.linalg.norm(dp)
            if norm > self.max_position_delta_m:
                dp *= float(self.max_position_delta_m) / norm
        dq_controller = quat_mul(
            quat_normalize(controller_quat), quat_inv(self.neutral_controller[1]))
        dq_robot_matrix = self.basis @ quat_to_matrix(dq_controller) @ self.basis.T
        dq_robot = matrix_to_quat(dq_robot_matrix)
        angle = 2.0 * np.arccos(np.clip(dq_robot[3], -1.0, 1.0))
        if angle > 1e-12 and self.orientation_scale != 1.0:
            dq_robot = clamp_quat_angle(dq_robot, angle * self.orientation_scale)
        dq_robot = clamp_quat_angle(dq_robot, self.max_orientation_delta_rad)
        target_quat = quat_normalize(quat_mul(dq_robot, self.neutral_ee[1]))
        return self.neutral_ee[0] + dp, target_quat
