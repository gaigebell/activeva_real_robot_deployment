"""D1 运动学：ikpy + 官方 URDF。

- 6 轴 IK：链从 base_link 到 Link6 后追加对称夹爪 TCP；夹爪开合关节不参与 IK
- 手柄 6DOF 增量 -> 末端目标由 controller_mapping.py 给出，本模块只做 IK
- TODO(现场):
  - 标定基座位姿（统一世界系）、中性位及 TCP 沿工具轴偏移
  - 核对 URDF 零位与真机零位（funcode 7）的偏移
  - ikpy 版本 API 差异（>=3.4 用 target_position/target_orientation）
"""
from __future__ import annotations
from pathlib import Path

import numpy as np
import ikpy.chain
from ikpy.link import URDFLink

URDF_DIR = Path(__file__).parent / "urdf"
URDF_670 = URDF_DIR / "d1_description.urdf"       # 670mm 含夹爪版
URDF_550 = URDF_DIR / "d1_550_description.urdf"   # 550mm 版
EE_LINK = "Link6"           # IK 末端（不含夹爪）
GRIPPER_JOINTS = ("Joint7_1", "Joint7_2")
# URDF 中两根夹爪手指相对 Link6 的安装原点中点。先把它作为对称 TCP；
# 夹取点沿工具轴的最终偏移仍需真机旋转中心测试确认。
TCP_OFFSET_FROM_LINK6 = np.array([-0.00562, 0.0, 0.0706])

# 最终链为 [base, J1..J6, 固定 TCP]，只激活 J1..J6。
ACTIVE_MASK = [False, True, True, True, True, True, True, False]


class D1IK:
    def __init__(self, urdf_path: Path = URDF_670, ee_link: str = EE_LINK,
                 tcp_offset=TCP_OFFSET_FROM_LINK6):
        self.ee_link = ee_link
        full_chain = ikpy.chain.Chain.from_urdf_file(
            str(urdf_path),
            base_elements=["base_link"],
            active_links_mask=ACTIVE_MASK,
        )
        joint6_index = next(
            (index for index, link in enumerate(full_chain.links)
             if link.name == "Joint6"), None)
        if joint6_index is None:
            raise RuntimeError("D1 URDF 中找不到 Joint6")
        # from_urdf_file 会沿第一条分支继续走到 Joint7_1。若直接使用，IK
        # 末端会落在单根手指且继承其旋转坐标系，导致平移时腕部无谓补偿。
        tcp_link = URDFLink(
            name="gripper_center_tcp",
            origin_translation=np.asarray(tcp_offset, dtype=float),
            origin_orientation=[0.0, 0.0, 0.0],
            joint_type="fixed",
        )
        links = list(full_chain.links[:joint6_index + 1]) + [tcp_link]
        self.chain = ikpy.chain.Chain(
            links, active_links_mask=ACTIVE_MASK, name="d1_tcp")

    def solve(self, target_position, target_orientation=None, q_init=None) -> np.ndarray:
        """解 6 关节角。

        target_position: (3,) 末端目标位置（机器人/世界系，单位 m）
        target_orientation: (4,) xyzw 四元数，None 则不约束姿态
        返回 (6,) 关节角（rad），顺序对应 angle0..angle5
        """
        initial = q_init if q_init is not None else self.home_full()
        if target_orientation is not None:
            target = np.eye(4)
            target[:3, :3] = quat_to_matrix(target_orientation)
            target[:3, 3] = target_position
            q = self.chain.inverse_kinematics_frame(
                target, initial_position=initial, orientation_mode="all")
        else:
            q = self.chain.inverse_kinematics(
                target_position=target_position, initial_position=initial)
        return np.asarray(q[1:7], dtype=np.float64)   # 去 base 与夹爪

    def solve_continuous_step(
        self,
        target_position,
        target_orientation,
        q_current,
        *,
        max_position_step_m=0.005,
        max_orientation_step_deg=3.0,
        damping=0.05,
        orientation_weight=0.5,
    ) -> np.ndarray:
        """从当前关节解做一次阻尼微分 IK，避免跳到另一支解析解。

        这是 VR 标定模式使用的局部控制器。它每次只追踪一小段笛卡尔误差，
        在奇异位形会降低可达位移，而不是让底座/腕部突然转几十度。
        它只保证解的连续性，并不包含机械臂自碰撞模型。
        """
        q = np.asarray(q_current, dtype=np.float64)
        if q.shape != (6,):
            raise ValueError("q_current 必须是 6 关节角")
        current = self.forward_kinematics(q)
        target_position = np.asarray(target_position, dtype=np.float64)
        target_rotation = quat_to_matrix(target_orientation)

        position_error = target_position - current[:3, 3]
        position_error = _clip_vector_norm(position_error, max_position_step_m)
        rotation_error = _rotation_vector(target_rotation @ current[:3, :3].T)
        rotation_error = _clip_vector_norm(
            rotation_error, np.deg2rad(max_orientation_step_deg))

        jacobian = self.geometric_jacobian(q, current)

        weights = np.diag([1.0, 1.0, 1.0] + [float(orientation_weight)] * 3)
        weighted_jacobian = weights @ jacobian
        weighted_error = weights @ np.r_[position_error, rotation_error]
        normal = (weighted_jacobian @ weighted_jacobian.T +
                  float(damping) ** 2 * np.eye(6))
        delta_q = weighted_jacobian.T @ np.linalg.solve(normal, weighted_error)
        if not np.all(np.isfinite(delta_q)):
            raise RuntimeError("微分 IK 返回非有限值")
        return q + delta_q

    def geometric_jacobian(self, q6, current_frame=None) -> np.ndarray:
        """数值几何 Jacobian：[线速度(m/rad), 角速度(rad/rad)]。"""
        q = np.asarray(q6, dtype=np.float64)
        if q.shape != (6,):
            raise ValueError("q6 必须是 6 关节角")
        current = (self.forward_kinematics(q) if current_frame is None
                   else np.asarray(current_frame, dtype=np.float64))
        epsilon = 1e-5
        jacobian = np.zeros((6, 6), dtype=np.float64)
        for joint in range(6):
            perturbed_q = q.copy()
            perturbed_q[joint] += epsilon
            perturbed = self.forward_kinematics(perturbed_q)
            jacobian[:3, joint] = (
                perturbed[:3, 3] - current[:3, 3]) / epsilon
            jacobian[3:, joint] = _rotation_vector(
                perturbed[:3, :3] @ current[:3, :3].T) / epsilon
        return jacobian

    def condition_number(self, q6, orientation_weight=0.5) -> float:
        """返回与连续 IK 权重一致的条件数；越大越接近奇异位形。"""
        jacobian = self.geometric_jacobian(q6)
        weights = np.diag(
            [1.0, 1.0, 1.0] + [float(orientation_weight)] * 3)
        singular_values = np.linalg.svd(weights @ jacobian, compute_uv=False)
        smallest = float(singular_values[-1])
        if smallest <= 1e-12:
            return float("inf")
        return float(singular_values[0] / smallest)

    def home_full(self, q6=None) -> list[float]:
        """ikpy 全链初值（base + 6 关节 + 固定 TCP）。"""
        joints = [0.0] * 6 if q6 is None else list(q6)
        return [0.0] + joints + [0.0]

    def forward_kinematics(self, q6) -> np.ndarray:
        """q6: (6,) 关节角 -> 末端 4x4（不含夹爪）。"""
        frame = self.chain.forward_kinematics([0.0] + list(q6) + [0.0])
        return np.asarray(frame)


def quat_to_matrix(q) -> np.ndarray:
    """xyzw 四元数转 3x3 旋转矩阵。"""
    x, y, z, w = np.asarray(q, dtype=float)
    norm = np.linalg.norm([x, y, z, w])
    if norm == 0:
        raise ValueError("四元数不能为零")
    x, y, z, w = np.asarray([x, y, z, w]) / norm
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def _clip_vector_norm(vector, max_norm):
    vector = np.asarray(vector, dtype=np.float64)
    norm = float(np.linalg.norm(vector))
    if max_norm <= 0:
        raise ValueError("max_norm 必须大于 0")
    return vector if norm <= max_norm or norm == 0 else vector * (max_norm / norm)


def _rotation_vector(rotation):
    """3x3 旋转矩阵的对数映射，结果为轴角向量(rad)。"""
    rotation = np.asarray(rotation, dtype=np.float64)
    cosine = np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)
    angle = float(np.arccos(cosine))
    skew = np.array([
        rotation[2, 1] - rotation[1, 2],
        rotation[0, 2] - rotation[2, 0],
        rotation[1, 0] - rotation[0, 1],
    ])
    if angle < 1e-7:
        return 0.5 * skew
    if np.pi - angle < 1e-5:
        # 本控制器已把单步姿态误差限制到几度，正常不会进入这里；保留稳定退化。
        eigenvalues, eigenvectors = np.linalg.eig(rotation)
        axis = np.real(eigenvectors[:, np.argmin(np.abs(eigenvalues - 1.0))])
        axis /= np.linalg.norm(axis)
        return angle * axis
    return angle * skew / (2.0 * np.sin(angle))
