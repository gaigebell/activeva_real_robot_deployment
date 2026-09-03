# ActiveVA 真机部署框架

cambot（主动视觉臂）+ 2× Unitree D1（双操作臂）+ Quest3 VR 遥操作 + lerobot 数据采集 + 服务器端推理闭环。

计划、调研结论与决策记录见 [PLAN.md](PLAN.md)；D1 官方协议见 [docs/protocol_d1.md](docs/protocol_d1.md)；
当前已验证的双网卡直连方案和排障步骤见 [docs/d1_dual_arm_network.md](docs/d1_dual_arm_network.md)。
Quest 3 手柄分层联调与双臂遥操步骤见 [docs/quest_d1_teleop.md](docs/quest_d1_teleop.md)。
当前 Quest 位姿到 D1 关节指令的完整公式、代码索引和已知疑点见
[docs/vr_d1_kinematics_calculation_chain.md](docs/vr_d1_kinematics_calculation_chain.md)。
本轮确认的 Home 对齐、初始工作区与夹爪/坐标/TCP 标定方案见
[docs/d1_teleop_design_and_calibration.md](docs/d1_teleop_design_and_calibration.md)。
完成分层验证后，可用 `./scripts/start_full_vr_teleop.sh --yes --camera-backend realsense`
一键运行完整双臂 6DoF 遥操。

## 目录结构

```
common/          数据契约（contract.py）与通信协议（protocol.py）—— 现场只改契约
robot/           机器人端：d1_bridge(C++ 驱动桥) / d1_client / d1_ik / controller_mapping
                 camera / collect / control（主控制程序）/ cambot_patch（手柄转发补丁）/ urdf
server/          服务器端：inference_server（推理服务骨架）
scripts/         启停脚本
docs/            D1 协议整理
```

## 依赖仓库（现场按需克隆/准备，均不在本仓库内）

| 仓库 | 用途 |
|---|---|
| cambot | 主动视觉臂软件（含 WebXR 遥操作，需现场打 cambot_patch） |
| d1_sdk + unitree_sdk2 | D1 的 DDS 通信（Linux 构建，d1_bridge 依赖） |
| d1_urdf | 官方 URDF（IK 所需的两份已内置在 robot/urdf/） |
| gym_av_aloha_occlusion | 仿真参考（操作语义：扳机=夹爪、RButtonOne=episode） |

## 快速开始（机器人端，Ubuntu 20.04）

```bash
# 1. 安装 uv 并同步 Python 依赖（uv 可自装 Python 3.12）
uv sync

# 2. 构建安装 unitree_sdk2（详见其 README：apt 依赖 + cmake + make + sudo make install）

# 3. 重启后初始化 D1 双网卡（已连线时）
sudo ./scripts/setup_d1_network.sh

# 4. 构建 d1_bridge
cd robot/d1_bridge && mkdir -p build && cd build
cmake .. -DD1_SDK_DIR=/home/ubuntu/unitree_ws/d1_sdk && make

# 5. 不接 cambot/Quest，先按 supervisor 建议只读验证单臂反馈
cd ../../..
sudo ./scripts/setup_d1_network.sh --arm left
./scripts/test_dual_d1.sh status --arm left --watch --seconds 5

# 6. 按 docs/d1_dual_arm_network.md 做独立上电/使能/微动测试。
# 7. 双臂独立测试全部通过后，再启动 Quest 3 遥操。
./scripts/start_robot.sh teleop --arm left
```

服务器端（独立机器，RTX 5880 Ada）：

```bash
uv sync && ./scripts/start_server.sh
```

## 现场检查清单（TODO 汇总）

- [x] D1 网络：D1_A `.100`/`enp5s0` + D1_B `.99`/`enx00e04c681b82`，相同 topic 按网卡隔离
- [ ] D1 上电(6) → 使能(5) → 反馈 7 值连续无 NaN → 归零(7)
- [ ] 相机方案（NQ2：D435/ZED、分辨率、深度）→ `robot/config.yaml` + `robot/camera.py`
- [ ] 夹爪语义（NQ5：angle6 单位/范围）→ `common/contract.py`
- [ ] cambot_patch 应用（手柄 6DOF/扳机/按钮转发）→ `robot/cambot_patch/README.md`
- [ ] 服务器 IP → `robot/config.yaml` 的 `server.host`
- [ ] 数据契约最终键名/fps → `common/contract.py`
- [ ] 物理布局与基座标定（阶段 2）

## 代码内 TODO 标记

所有未定项以 `TODO(现场)` 标注，用 `grep -rn "TODO(现场)" .` 可一次列出全部待办。
