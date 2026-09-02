# "ActiveVA" 真机部署计划（细化迭代稿 v0.3）

> 本文档为迭代稿：含调研结论、细化阶段计划、待澄清问题、决策记录。
> 已决问题见「§5 决策记录」，剩余现场确认项见「§4」。
> 官方协议依据：《宇树科技 文档中心.html》（用户提供，已存本目录）+ d1_urdf/。

## 0. 目的

1. 建立以 cambot 为主动视觉臂，两台 Unitree D1 为双操作臂的机器人系统
   - 连通性测试
   - 机械臂功能性测试
2. 建立数据采集管线
   - 机器人系统 + Quest3 VR
   - 头戴设备控制 cambot，手柄设备控制 D1
   - 操作低延迟
   - 数据存储格式：lerobot
3. 建立模型推理部署
   - 模型部署在服务器端，服务器端启动推理程序等待数据，机器人系统端启动控制程序，向服务器端传输数据（observation），推理程序推理，并将动作输出传给控制程序，控制程序接收动作命令，控制机器人系统执行动作
   - 丝滑推理

---

## 1. 调研结论

### 1.1 cambot（主动视觉臂）—— 软件栈已相当完整

- 6× Feetech STS3215 舵机（1 Mbps 串行总线，Waveshare USB 适配器），ZED Mini 双目相机（NQ2：可能换 D435），Ubuntu Linux PC 运行
- 已实现：100 Hz 控制环、ikpy IK、WebXR 浏览器遥操作（Quest 免装 app）、`/ws?no_video=1` control-only 接口、watchdog/位置球/速度钳制等安全机制
- 需扩展：手柄 6DOF/扳机/按钮转发（补丁位置已定位：`cambot/cambot/teleop/server.py` 的 `websocket_stream` 消息分发 388-393 行 + `client/index.html` 的 `sendHeadPose` 发送循环 1156 行附近）

### 1.2 D1 臂 —— 官方协议（已从文档中心提取，全表见 `docs/protocol_d1.md`）

- **规格**：6 轴 + 1 夹爪（底座 0 … 夹爪 6）；550 mm（不含夹爪）/ 670 mm（含夹爪）；动态负载 500 g / 静态 1 kg；17-bit 编码器
- **关节范围**（URDF 与文档一致）：J0 ±135°、J1 ±90°、J2 ±90°、J3 ±135°、J4 ±90°、J5 ±135°；夹爪行程 0–65 mm
- **协议**：JSON 字符串经 DDS topic `rt/arm_Command` 下发；字段 seq（自增）/ address / funcode / data
  - funcode **1** 单关节角 `{id, angle, delay_ms=0}`；**2** 全关节角 `{mode, angle0..angle6}` —— **mode 0 = 10 Hz 数据的小平滑，mode 1 = 轨迹用的大平滑**；**4** 单关节使能/卸力 `{id, mode}`；**5** 全部关节使能/卸力 `{mode}`（0 卸力/1 使能）；**6** 电机供电 `{power}`（0 断电/1 上电）；**7** 位姿归零
- **反馈**（seq 固定 10，**10 Hz**，address+funcode 区分）：address 2 / funcode 1 = 关节角度 angle0~6；address 2 / funcode 3 = 状态 enable/power/error；address 3 / funcode 1、2 = 接收/执行状态
- **多臂区分（官方两种方案）**：(1) 各臂直连 PC 不同网卡，`ChannelFactory::Init(0, "eth0")` 绑定网卡；(2) 同路由器单网卡：ssh 进臂（默认 IP 192.168.123.100，用户 ubuntu/密码 123）停掉 4 个 marm_* systemd 服务，改臂内 `marm_code/src/marm_communication_node.cpp` 的话题后缀使每臂 topic 唯一。**注意：不靠 address 字段区分双臂**（命令 address=1）
- **URDF**：`d1_urdf/d1_description` = 670 mm 含夹爪版（6 revolute Joint1–Joint6 + 2 prismatic 夹爪手指 Joint7_1/7_2 各 0–0.03 m）；`d1_550_description` = 550 mm 版；官方另提供 URDF zip（有/无夹爪版）
- **开发环境**：d1_sdk（C++17 + unitree_sdk2 + ddscxx + iceoryx 2.0.2），仅 Linux；无 Python 绑定、无 IK —— 由本框架补建（d1_bridge + ikpy IK）

### 1.3 unitree_sdk2（DDS 基础设施）

- CycloneDDS v0.10.2 预编译静态库；Ubuntu 20.04+ / gcc 9.4.0+；无 D1 专用代码；仅为 d1_bridge 提供 DDS 运行时

### 1.4 gym_av_aloha_occlusion（仿真 / 操作语义参考）

- MuJoCo + Gymnasium 25 Hz；obs/action 21 维、6 相机 —— **仅参考，真机自定契约（QD1：只用真机数据训练）**
- 操作语义沿用：扳机 = 夹爪、RButtonOne = 开始/结束 episode、手柄 6DOF 增量 → 末端位姿 → IK

---

## 2. 目标架构

```
Quest3 浏览器(WebXR)
  │ WLAN (HTTPS+WS :8080，头显位姿 + 手柄 6DOF/扳机/按钮)
  ▼
机器人端 PC (Ubuntu 20.04 单机, uv + Python)
 ├ cambot 软件        head_pose → IK → 6 舵机；相机视频 → 头显
 │  └ [cambot_patch]  手柄 6DOF/按钮 → 本地 TCP 转发给控制程序
 ├ d1_bridge ×2 (C++)  TCP 收目标 → DDS funcode 2 → D1；订阅反馈(10Hz)回传
 ├ 控制程序 control.py  遥操: 手柄→IK→D1 目标；推理: obs→服务器→action→三臂
 └ 采集 collect.py     lerobot 实时录制（相机/状态/写盘分线程）
  │ LAN（ZMQ/WebSocket，服务器监听端口）
  ▼
服务器端 (独立机器, RTX 5880 Ada, GPU)
 └ 推理服务 inference_server.py  obs(image+state) → action
```

---

## 3. 分阶段细化计划

### 阶段 0：环境与网络准备（现场）

1. 机器人端（Ubuntu 20.04 已装好）：构建安装 unitree_sdk2 → 编译 d1_sdk 示例 → 编译 d1_bridge（框架内）；uv 装 Python 3.12 + 项目依赖
2. 网络接线：**双臂接线方式现场定**（双网卡直连 vs 单网卡+改 topic 后缀，§1.2 方案一/二）；Quest 与机器人端同 WiFi；服务器端同局域网监听端口
3. 相机确认（NQ2 现场项）：D435 是否替换 ZED、分辨率、深度是否采集；据此完成 `robot/camera.py` 后端
4. 下载/核对 URDF（框架已含 d1_description/d1_550_description 两份 URDF）
5. 服务器端：GPU 环境 + 模型部署框架（推理服务为 stub，模型后接）

**验收**：d1_sdk 示例与 d1_bridge 可编译运行；`run_read_params.sh` 读出 6 舵机

### 阶段 1：连通性测试

1. cambot：ping 6 舵机 → 读位置 → 单关节动 → 断扭矩（现有工具）
2. D1 单臂：上电(6) → 使能(5) → 读反馈 → 单关节角(1) → 全关节角(2, mode 0) → 卸力(5, mode 0) → 归零(7)
3. D1 双臂：按方案一（网卡绑定）或方案二（topic 后缀）验证两臂独立寻址、反馈互不混流
4. 三臂并发：cambot 控制环 + 两个 d1_bridge 同跑

**验收判据（草案）**：cambot 6 舵机 ping 全通；D1 使能后反馈 7 值连续无 NaN、指令→反馈延迟粗测 < 100 ms；两臂互不串扰

### 阶段 2：功能性测试

1. D1 连续指令环实测：**反馈只有 10 Hz，下发频率上限需实测**（mode 0/1 平滑差异）；目标 ≥ 30 Hz（对齐采集 fps，若达不到则以实测为准降 fps）
2. D1 运动学：ikpy + 官方 URDF（6 轴 IK，夹爪由扳机直控）；现场标定基座位姿/中性位
3. VR 全链路（WebXR 统一方案）：应用 cambot_patch（手柄位姿/按钮转发）；按钮语义同仿真（扳机=夹爪、RButtonOne=开始/结束 episode）
4. 端到端延迟测量：头显→臂 < 200 ms 总预算，分解到各链路
5. 安全验证：watchdog、暂停、软件 E-stop（卸力 funcode 5 mode 0 + 断电 funcode 6 power 0）

### 阶段 3：数据采集管线（lerobot）

- **数据契约（v0.3 草案）**：
  - state/action = **20 维**：D1L（6 关节 + 夹爪）+ D1R（6 关节 + 夹爪）+ cambot（6 关节）；夹爪语义（0-65 mm 行程还是归一化）现场定
  - images：相机方案现场定（NQ2）；无 gaze、无 environment_state
  - fps = 30（可调）；episode 按钮触发（同仿真）
  - 多线程实时写 LeRobotDataset（相机采集 / 状态读取 / 写盘分离）；录制后可重放检查
- 训练：只用真机数据（QD1）

### 阶段 4：模型推理部署

1. 模型与 I/O 契约（QI1 暂缓）：输入 image + robot state，输出/chunk 可自定义
2. 通信：ZMQ/WebSocket 候选；服务器监听，机器人端发起连接（NQ4）
3. 推理服务（RTX 5880 Ada，单样本延迟优化）；机器人端执行（下发策略待 D1 实测后定）
4. 安全：软件 E-stop、D1 指令超时看门狗、cambot watchdog 保留；闭环联调按"丝滑"标准量化

---

## 4. 剩余现场确认项

- **相机**（NQ2）：D435 是否替换 cambot 的 ZED Mini？单台相机、分辨率（1280×720？）、深度是否采集——现场确认后填 `robot/config.yaml` 并实现 `robot/camera.py` 对应后端
- **夹爪语义**（NQ5）：angle6 的单位与范围（0–65 mm 行程？归一化 0-1？）、与 6 关节同走 funcode 2 还是独立命令——连通性测试时验证
- **双臂接线**：双网卡直连（方案一，免改臂内代码）还是同路由器改 topic（方案二，需 ssh 改臂内文件）——现场按布线条件定
- **物理布局**（原 QT2）：三臂安装位置与基座标定，安装阶段定，不影响软件先行

---

## 5. 决策记录

**2026-09-02 首轮回答**
- QH1：一台机器 Ubuntu 20.04（已装好，保持）
- QH2：官方文档已提供（本目录 HTML）；协议全表见 §1.2 / docs/protocol_d1.md
- QH3：末端夹爪作为第 7 个控制量（0-1 开合，语义现场定）
- QH4：服务器端独立机器，网络端口监听；NQ4：同局域网，机器人端发 obs 收 action，推理与控制为两个通信节点；**GPU：NVIDIA RTX 5880 Ada**
- QH5：同一局域网，无阻隔
- QV1：全部走 Quest 浏览器 WebXR（不沿用仿真 Unity+Firestore 栈）
- QV2：手柄 → D1 末端位姿 → IK；URDF 官方文件（已入库 d1_urdf/，框架内复制两份 URDF）；6 轴+1 夹爪，底座 0 … 夹爪 6
- QV3：不需要注视数据；left_eye/right_eye 键取消
- QV4：操作语义与仿真一致：扳机=夹取、开始/结束按键同仿真
- QV5：端到端延迟 < 200 ms
- QD1：只用真机数据训练（自定契约，无需与仿真对齐）
- QD2：相机用 RealSense D435（是否替换 ZED 现场确认），腕部无相机
- QD3：fps 可自定义默认 30；episode 按钮触发
- QD4：任务自定义多个；environment_state 搁置（不含该键）
- QD5：多线程实时写（相机/写数据/控制分线程）；录制后可重放检查
- QI1：模型多个待选；输入 image + robot state（可能有变化）；输出/chunk 可自定义
- QI2：ZMQ 和 WebSocket 均可
- QI3/QI4/QT1/QT2：现场/后续定，已拟草案（§3）

**NQ 系列（2026-09-02 二轮回答）**
- NQ1：Ubuntu 20.04 保持，已装好系统
- NQ2：D435 可能替换 ZED（不确定）；一台相机；分辨率可能 1280×720；深度可能采集可能不采集——**现场确认**
- NQ3：URDF 已提供（d1_urdf/ 目录：d1_description 670mm、d1_550_description 550mm）
- NQ4：同局域网；机器人端发送 obs、接收动作命令；推理和控制相当于两个通信节点；GPU RTX 5880 Ada
- NQ5：夹爪语义现场确定

---

## 6. 框架结构（v0.3 新增，已按此搭建）

```
(仓库根 = 本目录，git 管理；依赖仓库 cambot/ d1_sdk/ unitree_sdk2/ d1_urdf/ 被 .gitignore，现场按 README 克隆)
├── PLAN.md                    # 本计划（迭代稿）
├── README.md                  # 部署/构建/运行总览 + 现场检查清单
├── pyproject.toml             # uv 项目（Python 侧统一入口）
├── docs/protocol_d1.md        # D1 官方协议全表（含多臂方案、IP/ssh）
├── common/
│   ├── contract.py            # 数据契约常量（维度/键/fps/端口）—— 现场只改这里
│   └── protocol.py            # 机器人端↔服务器端 ZMQ 消息封包
├── robot/
│   ├── config.yaml            # 机器人端配置（IP/端口/相机/模式）
│   ├── control.py             # 主控制程序（teleop / inference 模式）
│   ├── d1_bridge/             # C++ 驱动桥：TCP ← 目标 → DDS funcode 2；反馈回传
│   ├── d1_client.py           # d1_bridge 的 Python 客户端
│   ├── d1_ik.py               # ikpy + 官方 URDF（6 轴 IK + 夹爪直控）
│   ├── controller_mapping.py  # 手柄 6DOF 增量 → D1 末端目标（同仿真语义）
│   ├── camera.py              # 相机抽象（RealSense/ZED 后端，现场定）
│   ├── collect.py             # lerobot 多线程实时采集
│   ├── cambot_patch/README.md # cambot 扩展补丁说明（手柄位姿转发）
│   └── urdf/                  # d1 URDF（ikpy 用，含关节限位）
├── server/
│   ├── config.yaml
│   └── inference_server.py    # 推理服务骨架（ZMQ REP + 模型 stub）
└── scripts/
    ├── start_robot.sh
    └── start_server.sh
```

**设计要点**：
- D1 全部底层交互收敛到 C++ `d1_bridge`（单臂进程，参数化 topic/网卡/端口，双臂启动两个实例），Python 侧只走 TCP JSON——现场改协议无需动 Python
- 数据契约全部集中在 `common/contract.py`，现场确认相机/夹爪后只改一处
- 所有未定项在代码中以 `TODO(现场)` 标注，README 附现场检查清单

---

## 7. 迭代记录

- **v0.1**（2026-09-02）：四仓库调研 + 21 项待澄清问题。
- **v0.2**（2026-09-02）：首轮回答登记；D1 规格检索佐证（6+1 自由度）；新增 NQ1–NQ5。
- **v0.3**（2026-09-02）：二轮回答登记（NQ1–NQ5）；解析官方文档中心 HTML 获得**完整协议表**（funcode 1/2/4/5/6/7、mode 0/1 平滑、反馈 10 Hz、seq 语义、多臂两方案、默认 IP/ssh、URDF 下载）；读取 d1_description.urdf（6 revolute + 2 prismatic 夹爪手指）；确定框架结构并开始搭建；定位 cambot 补丁位置（server.py 388-393 / index.html 1156）。
