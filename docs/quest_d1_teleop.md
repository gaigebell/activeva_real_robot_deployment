# Quest 3 → 双 Unitree D1 遥操测试手册

本流程只把 cambot 的 HTTPS/WebXR 服务当作 Quest 3 入口。
`start_quest_stream.sh` 始终带 `--no-robot`，不会连接或控制 cambot 机械臂。
该脚本同时设置 `--watchdog-timeout 0`，禁用只服务于 cambot 真机回 Home 的 pose watchdog。
ActiveVA 自己的 D1 手柄数据超时锁止仍然保留。

当前保留两种接合方式：`deadman` 为持续按侧握键的相对对齐方式；`home` 为机械臂先进入
已保存 Home、操作者把手柄置于舒适 Home 后长按 A/X 对齐并持续遥操，B/Y 暂停。

## 控制契约与安全限制

- Quest 左手柄 → `left` / D1_A / `192.168.123.100`。
- Quest 右手柄 → `right` / D1_B / `192.168.123.99`。
- 侧握键 `squeeze` 是指令门控：按下沿以当前手柄和 D1 位姿做相对对齐；
  持续按住才下发新目标。
- 松开侧握键或手柄数据超过 250 ms 后，立即停止下发新目标；3 秒以内恢复时以当前手柄与
  机械臂位姿重新对齐，连续断流超过 3 秒才锁止并要求松键重接合。
  D1 仍保持最后位置，不会因松键立即卸力。
- 食指扳机 `trigger` 控制夹爪：0=开，1=关。
- 完整模式同时控制末端位置和姿态（6DoF）。
- 默认 10 Hz，当前手柄位移缩放为 1.2。
- 三种模式都不用逐帧全局 IK，而是沿最后下发目标做阻尼微分 IK。`full` 每周期最多
  追踪 8 mm / 4°，并拒绝任一关节超过 6°的异常局部步长；这避免旧实现跳到远端解支、
  将关节推到限位后因 IK 初值越界而停止。
- `position` / `orientation` 标定模式的限制更保守：
  相对接合点最多 5 cm / 30°，单次只追踪 5 mm / 3°；若局部解仍要求
  任一关节跳变超过 5°，整帧拒绝，不会再把远端解裁剪后发送。
- 当前没有自碰撞模型；IK 不会主动“绕开自己”。操作者仍须小步单轴测试。
- 遥操程序 Ctrl+C 退出或异常退出时，会尝试向两臂发送 `disable`；不自动断电。

## 第一阶段：只验证 Quest 手柄，D1 不动

终端 1：启动 WebXR 服务。如果要在 Quest 中显示 D435 画面：

```bash
cd /home/ubuntu/activeva_real_robot_deployment
./scripts/start_quest_stream.sh --camera-backend realsense
```

如果此时只测手柄，不需要画面：

```bash
./scripts/start_quest_stream.sh --no-camera --no-webrtc
```

终端会打印类似 `https://<电脑局域网IP>:8080` 的地址。
确保 Quest 3 和电脑在同一 Wi-Fi，用 Quest 浏览器打开该地址，
接受本地 HTTPS 证书后点击 `START AR` 进入混合现实。ActiveVA 页面使用 WebXR
`immersive-ar` 和透明画布，Quest 3 应显示彩色 passthrough，使操作者能直接看到机械臂；
普通 cambot 启动方式仍保持原来的 `START VR` 双目视频页面。
更新 WebXR 代码后应先停止并重新启动此服务，再关闭 Quest 中的旧页面并重新打开。
ActiveVA 模式会同时在浏览器端和服务端禁用“侧握键切换 cambot pause”，避免与 D1
死手开关冲突；新页面还带有禁用缓存响应头。
Quest 摘下或浏览器退出 WebXR 后，浏览器会停止发送手柄位姿；禁用 cambot watchdog
不能让此时的手柄数据继续更新。六方向测试应全程佩戴头显，完成后再查看终端记录。
如果终端打印了 VPN/虚拟网卡 IP，Quest 无法访问，可用
`ip -br addr` 查找与 Quest 同一 Wi-Fi 的电脑 IP，指定后重启：

```bash
CAMBOT_HOST_IP=<Wi-Fi-IP> ./scripts/start_quest_stream.sh --no-camera
```

这样打印地址和自签 HTTPS 证书都会使用该 IP。
服务实际监听 `0.0.0.0:8080`。

终端 2：只读监视手柄。

```bash
cd /home/ubuntu/activeva_real_robot_deployment
./scripts/monitor_quest_controllers.sh
```

成功标志：

- `left` 和 `right` 都从 `WAIT` 变成持续更新的 `p=(x,y,z)`。
- 按住左/右侧握键时，对应 `squeeze` 从 0 变到接近 1。
- 扣左/右扳机时，对应 `trigger` 从 0 连续变到接近 1。

这个监视程序不启动 D1 bridge，不连接 D1。
验证完后按 Ctrl+C 停止监视；否则它会占用 TCP 6001，遥操程序无法启动。

## 第二阶段：完整双臂 6DoF 遥操

保持终端 1 的 Quest WebXR 服务运行。在终端 2 中：

```bash
# 重启后必要时先初始化网络
sudo ./scripts/setup_d1_network.sh

# 上电与使能（每条命令执行后 bridge 会自动退出）
./scripts/test_dual_d1.sh action power-on --arm both --yes
./scripts/test_dual_d1.sh action enable --arm both --yes

# 启动遥操；不会自动上电/使能
./scripts/start_vr_d1_teleop.sh --yes --control-mode full
```

### Supervisor 建议：先做单臂遥操

第一次使用某一只臂时，先由操作者最后一次扶到确认安全的 Home（肘部弯曲、夹爪朝前、远离
底座/桌面），捕获当前反馈。命令会立即以当前反馈建立保持，因此看到“Home 已保持并保存”后即可松手：

```bash
./scripts/test_dual_d1.sh capture-home --arm right --yes
```

捕获时会计算局部 IK 条件数。日志中原先的起始姿态
`[19.2,-13.9,16.4,3.6,2.2,-1.2]°` 条件数约 387，已经确认不能作为遥操 Home，脚本会拒绝保存。
应让肘部明显弯曲，并让腕部倒数第二轴离 0° 有足够距离；条件数不超过 80 才会保存。

左右臂必须分别捕获，不能直接把一侧关节角镜像到另一侧。捕获结果保存在
`robot/calibration/d1_home.yaml`。

以后只接通右臂时：

```bash
sudo ./scripts/setup_d1_network.sh --arm right
./scripts/start_vr_d1_teleop.sh --yes --arm right --control-mode position
```

启动脚本会先读取反馈、检查当前姿态与已捕获 Home 的最大关节差不超过 35°，然后上电、保持当前位置，
以约 5°/s 的 smoothstep 轨迹进入 Home；到位误差不超过 3°才开放手柄等待。当前只允许单臂自动
Home，`--arm both` 会被拒绝。左臂版本把 `right` 改为 `left`，且只读取左手柄。
一侧断线不会阻止另一侧以显式单臂模式启动。测试结束后只对当前臂卸力和断电。

现场标定必须使用隔离模式：

- `--control-mode position`：只采用手柄平移，末端姿态锁定在侧握键接合瞬间；用于坐标轴标定。
- `--control-mode orientation`：只采用手柄旋转，TCP 位置锁定在接合瞬间；用于 TCP/旋转中心检查。
- `--control-mode full`：完整 6DoF；只在前两项标定通过后使用。

接合方式：

- `--engagement-mode deadman`：持续按对应侧握键才运动；松开后下次按下重新对齐。
- `--engagement-mode home`：启动时机械臂先进入保存的 Home。把对应手柄放到舒适中性位置，
  长按 A（左手柄为 X）约 0.8 秒完成对齐；之后无需按侧握键。按 B（左手柄为 Y）暂停，
  再次长按 A/X 才恢复。追踪断流时仍会在 250 ms 立即停发；3 秒内恢复会自动以当前状态
  重新对齐，超过 3 秒则仍需长按 A/X。

按当前镜像安装，WebXR 到 D1 局部坐标的初值为：

| 手柄物理方向 | 左臂局部目标 | 右臂局部目标 |
|---|---|---|
| 向右 | `+X` | `-X` |
| 向前 | `+Y` | `+Y`（右臂现场实测修正） |
| 向上 | `+Z` | `+Z` |

WebXR 为 `+X` 右、`+Y` 上、`-Z` 前；右臂前后符号已按现场单轴运动结果修正。
若现场某一方向相反，只修改 `robot/config.yaml` 对应臂的 `position_signs`，不要再把三轴设为原样传递。
姿态使用独立的 `orientation_axes` / `orientation_signs`；右臂本次前后修正不会顺带改变腕部旋转映射。

连续 IK 使用“最后下发目标”作为下一周期参考，并通过 `max_command_lead_deg` 限制目标领先真实
反馈的距离。诊断中的 `command_step_xyz` 是本周期真正下发目标相对反馈的模型位移，
`command_lag` 是最大关节跟踪差。

`position` 和 `orientation` 两种标定模式都会保持当前夹爪反馈值，食指扳机暂不改变夹爪。
如果接合时有 J1/J2 等关节距极限不足 5°，终端会警告。此时不要继续测试该方向；
松开侧握键，先在监督下把机械臂调整到更居中的 Home 位姿。接近关节极限时，
“移动幅度很小”或局部 IK 拒绝运动是预期的安全退化，不应靠提高缩放解决。

如果手柄数据超过 250 ms，控制会立即停发并保持上一目标；3 秒内恢复会从当前状态重新对齐，
不会沿用旧目标。连续断流超过 3 秒则锁止，恢复后必须先松开侧握键，看到“可以重新接合”再按下。
标定日志约每 250 ms 输出手柄原始位移
`hand_xyz`、映射后的机器人目标 `mapped_xyz`、由关节反馈重建的实际 TCP 累计位移
`actual_xyz`、本周期 IK 预计位移 `ik_step_xyz`；另有收包频率、数据年龄、最大包间隔和限幅原因。

首次建议只测一只手：

1. 两只手都不按侧握键，移动手柄，机械臂不应跟随。
2. 保持手柄在舒适中性位置，按住左侧握键，终端应打印
   `teleop:left ... 对齐当前位姿`。
3. 每次只沿一个轴移动约 1–2 cm，检查左臂方向；松开侧握键。
4. 再用右手以相同方式测试右臂。

如果某个方向反了，先 Ctrl+C 退出，再修改 `robot/config.yaml`：

```yaml
teleop:
  position_axes: [0, 1, 2]
  position_signs: [1, 1, 1]
```

`position_axes` 是 Quest xyz 到 D1 xyz 的轴排列，`position_signs` 用于翻转方向。
当前缩放 1.2 只用于在 5 cm 标定工作区内增强体感；不要同时提高
`calibration_max_position_delta_m` 或两个 calibration 关节阈值。

## 一键完整启动

手柄、坐标方向和双臂小幅 6DoF 都验证后，可用一条命令管理整个生命周期。
它会自动上电、使能、启动 Quest 服务与双臂遥操，Ctrl+C 后卸力并断电；
不会执行归零，cambot 机械臂始终禁用。

```bash
cd /home/ubuntu/activeva_real_robot_deployment

# 使用 D435 视频
./scripts/start_full_vr_teleop.sh --yes --camera-backend realsense

# 或：只用 Quest 手柄，不传视频
./scripts/start_full_vr_teleop.sh --yes --no-camera --no-webrtc
```

多网卡机器上可指定 Quest 访问 IP：

```bash
CAMBOT_HOST_IP=<Wi-Fi-IP> \
  ./scripts/start_full_vr_teleop.sh --yes --camera-backend realsense
```

左右臂需要不同坐标变换时，可在 `teleop.arms.left` / `teleop.arms.right`
中分别覆盖 `position_axes`、`position_signs`、缩放与限幅参数。

## 结束测试

在遥操终端按 Ctrl+C。程序会尝试先卸力再关闭 bridge。
然后确保断电：

```bash
./scripts/test_dual_d1.sh action power-off --arm both --yes
```

如果遥操程序异常退出、终端被强制关闭或网络中断，不要假设自动卸力已成功；
应在安全条件下重新运行 `disable` / `power-off` 命令。
