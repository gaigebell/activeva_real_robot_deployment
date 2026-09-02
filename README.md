# ActiveVA 真机部署框架

cambot（主动视觉臂）+ 2× Unitree D1（双操作臂）+ Quest3 VR 遥操作 + lerobot 数据采集 + 服务器端推理闭环。

计划、调研结论与决策记录见 [PLAN.md](PLAN.md)；D1 官方协议见 [docs/protocol_d1.md](docs/protocol_d1.md)。

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

# 3. 构建 d1_bridge
cd robot/d1_bridge && mkdir -p build && cd build
cmake .. -DD1_SDK_DIR=../../d1_sdk && make

# 4. 启动（先确认 robot/config.yaml 与现场一致）
./scripts/start_robot.sh teleop
```

服务器端（独立机器，RTX 5880 Ada）：

```bash
uv sync && ./scripts/start_server.sh
```

## 现场检查清单（TODO 汇总）

- [ ] D1 网络：ping 192.168.123.100；双臂区分方案定（网卡绑定 or 改 topic，docs/protocol_d1.md）
- [ ] D1 上电(6) → 使能(5) → 反馈 7 值连续无 NaN → 归零(7)
- [ ] 相机方案（NQ2：D435/ZED、分辨率、深度）→ `robot/config.yaml` + `robot/camera.py`
- [ ] 夹爪语义（NQ5：angle6 单位/范围）→ `common/contract.py`
- [ ] cambot_patch 应用（手柄 6DOF/扳机/按钮转发）→ `robot/cambot_patch/README.md`
- [ ] 服务器 IP → `robot/config.yaml` 的 `server.host`
- [ ] 数据契约最终键名/fps → `common/contract.py`
- [ ] 物理布局与基座标定（阶段 2）

## 代码内 TODO 标记

所有未定项以 `TODO(现场)` 标注，用 `grep -rn "TODO(现场)" .` 可一次列出全部待办。
