# D1 机械臂控制协议（整理自宇树官方文档中心）

> 来源：《宇树科技 文档中心.html》（官方文档，本仓库根目录另有存档）+ d1_urdf/。
> 现场调试时以真机实测为准，有出入处更新本文件。

## 基本信息

- 机械臂：**6 轴 + 1 夹爪**，从底座为 0 开始计算，到夹爪为 6
- 规格：550 mm（不含夹爪）/ 670 mm（含夹爪）；动态负载 500 g / 静态负载 1 kg；17-bit 编码器
- 关节范围：J0 ±135°、J1 ±90°、J2 ±90°、J3 ±135°、J4 ±90°、J5 ±135°；**夹爪行程 0–65 mm**
- 通信：DDS（unitree_sdk2，domainId=0）；命令 topic `rt/arm_Command`
- 每台臂出厂默认 IP **192.168.123.100**，可 ssh：`ssh ubuntu@192.168.123.100`（密码 `123`）

## 报文格式

所有数据为 JSON 字符串：`{seq, address, funcode, data}`，data 可嵌套 JSON。

- **seq**：下发命令由调用端自增；反馈固定为 10。臂回复时原样带回该命令的 seq，无需解析处理
- **address**：命令侧为 1（多臂区分**不靠** address，见下文）；反馈侧 2/3 表示数据类别
- **funcode**：指令功能码

## 命令表（PC → 臂，topic `rt/arm_Command`）

| funcode | 功能 | data 字段 | 说明 |
|---|---|---|---|
| 1 | 单关节角度控制 | `{id, angle, delay_ms}` | delay_ms 暂时设为 0 |
| 2 | 全部关节角度控制 | `{mode, angle0..angle6}` | **mode 0 = 10 Hz 数据的小平滑；mode 1 = 轨迹使用的大平滑** |
| 4 | 单关节使能/卸力 | `{id, mode}` | mode 0 卸力 / 1 使能 |
| 5 | 全部关节使能/卸力 | `{mode}` | 0 卸力 / 1 使能 |
| 6 | 电机供电开关 | `{power}` | 0 断电 / 1 上电 |
| 7 | 位姿归零 | 无 | 无 data |

示例（全部关节角度控制）：
```json
{"seq":4,"address":1,"funcode":2,"data":{"mode":1,"angle0":0,"angle1":-60,"angle2":60,"angle3":0,"angle4":30,"angle5":0,"angle6":0}}
```

## 反馈表（臂 → PC，seq 固定 10，需回调监听）

**上传频率 10 Hz**，通过 address + funcode 区分数据：

| address | funcode | 内容 |
|---|---|---|
| 2 | 1 | 关节角度 `angle0..angle6`（**10 Hz**） |
| 2 | 3 | `enable_status`(1 使能/0 卸力)、`power_status`(1 供电/0 断电)、`error_status`(1 正常/0 异常) |
| 3 | 1 | `recv_status` 接收状态 |
| 3 | 2 | `exec_status` 执行状态 |

## 多臂控制（官方两种方案）

**方案一：多臂直连电脑不同网卡** → 初始化时绑定网卡区分：

```cpp
ChannelFactory::Instance()->Init(0, "eth0");   // 控制绑定在 eth0 上的臂
```

**方案二：电脑与多臂接同一路由器（单网卡）** → 修改每台臂内部驱动的话题名：

1. ssh 进臂，停止默认服务：
   ```bash
   sudo systemctl stop marm_controller.service
   sudo systemctl stop marm_control.service
   sudo systemctl stop marm_communication.service
   sudo systemctl stop marm_subscripber.service
   sudo systemctl disable marm_controller.service marm_control.service marm_communication.service marm_subscripber.service
   ```
2. 修改 `marm_code/src/marm_communication_node.cpp` 中的 topic 后缀使其唯一，
   例如 `current_servo_angle` → `current_servo_angle_1`、`set_servo_angle` → `set_servo_angle_1`
3. 每台臂逐一（先只连一条臂）修改，重启生效

> 注意：命令里 address 字段不是多臂寻址手段；两臂需通过网卡或 topic 区分。

## D1-T（官方双臂遥操作模式）

- 两台臂经路由器/交换机通讯；一台末端带**手持夹爪**作数据采集端，另一台执行（主从复现关节数据）
- 网段 192.168.123.x；改臂 IP：ssh 后改 `/etc/network` 的 interface 文件并重启
- 官方遥操作基础示例代码 zip：https://oss-global-cdn.unitree.com/static/82b7daf1ca8d43a3aa0f6b8421272abe.zip

## URDF

- `d1_description`（670 mm 含夹爪版）：Joint1–Joint6 revolute（限位 ±2.35/±1.57/±1.57/±2.35/±1.57/±2.35 rad）+ Joint7_1/Joint7_2 **prismatic** 夹爪两指（各 0–0.03 m）
- `d1_550_description`（550 mm 版）
- 官方 URDF zip：有夹爪版 https://oss-global-cdn.unitree.com/static/90b2525be5d84531ab9814f48b2f86a7.zip / 无夹爪版 https://oss-global-cdn.unitree.com/static/02c95ece8e354143b874a3c963241467.zip
- 本仓库 `robot/urdf/` 已内置两份 URDF 供 ikpy IK 使用

## 官方示例程序（d1_sdk = marm_code）

- `joint_angle_control`：单关节角度控制（funcode 1）
- `multiple_joint_angle_control`：多关节角度控制（funcode 2）
- `joint_enable_control`：使能控制（funcode 5）
- `arm_zero_control`：位姿归零（funcode 7）
- `get_arm_joint_angle`：获取关节角度（订阅 `current_servo_angle`）
