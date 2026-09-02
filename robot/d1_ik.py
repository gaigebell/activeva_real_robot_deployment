"""D1 运动学：ikpy + 官方 URDF。

- 6 轴 IK：链从 base_link 到 Link6；夹爪（Joint7_1/7_2 prismatic）不参与 IK，扳机直控
- 手柄 6DOF 增量 -> 末端目标由 controller_mapping.py 给出，本模块只做 IK
- TODO(现场):
  - 标定基座位姿（统一世界系）、中性位
  - 核对 URDF 零位与真机零位（funcode 7）的偏移
  - ikpy 版本 API 差异（>=3.4 用 target_position/target_orientation）
"""
from __future__ import annotations
from pathlib import Path

import numpy as np
import ikpy.chain

URDF_DIR = Path(__file__).parent / "urdf"
URDF_670 = URDF_DIR / "d1_description.urdf"       # 670mm 含夹爪版
URDF_550 = URDF_DIR / "d1_550_description.urdf"   # 550mm 版
EE_LINK = "Link6"           # IK 末端（不含夹爪）
GRIPPER_JOINTS = ("Joint7_1", "Joint7_2")

# 链结构: [base, J1..J6, 夹爪指1, 夹爪指2] -> 只激活 J1..J6
ACTIVE_MASK = [False, True, True, True, True, True, True, False, False]


class D1IK:
    def __init__(self, urdf_path: Path = URDF_670, ee_link: str = EE_LINK):
        self.ee_link = ee_link
        self.chain = ikpy.chain.Chain.from_urdf_file(
            str(urdf_path),
            base_elements=["base_link"],
            active_links_mask=ACTIVE_MASK,
        )

    def solve(self, target_position, target_orientation=None, q_init=None) -> np.ndarray:
        """解 6 关节角。

        target_position: (3,) 末端目标位置（机器人/世界系，单位 m）
        target_orientation: (4,) xyzw 四元数，None 则不约束姿态
        返回 (6,) 关节角（rad），顺序对应 angle0..angle5
        """
        kwargs = {"initial_position": q_init if q_init is not None else self.home_full()}
        if target_orientation is not None:
            # ikpy >= 3.4: 位置 + 姿态四元数；旧版本用 4x4 位姿矩阵，现场按版本适配
            kwargs["target_position"] = target_position
            kwargs["target_orientation"] = target_orientation
        else:
            kwargs["target_position"] = target_position
        q = self.chain.inverse_kinematics(**kwargs)
        return np.asarray(q[1:7], dtype=np.float64)   # 去 base 与夹爪

    def home_full(self) -> list[float]:
        """TODO(现场): 标定零位（6 关节 + 夹爪两指固定 0）。"""
        return [0.0] * 6 + [0.0, 0.0]

    def forward_kinematics(self, q6) -> np.ndarray:
        """q6: (6,) 关节角 -> 末端 4x4（不含夹爪）。"""
        frame = self.chain.forward_kinematics([0.0] + list(q6) + [0.0, 0.0])
        return np.asarray(frame)
