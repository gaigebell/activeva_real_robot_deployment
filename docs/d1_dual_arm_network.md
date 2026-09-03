# Unitree D1 双臂直连网络手册

## 已验证拓扑

| 软件侧 | D1 IP | 电脑网卡 | 电脑 IP | bridge TCP |
|---|---|---|---|---|
| `left` / D1_A | `192.168.123.100` | `enp5s0` | `192.168.123.97/24` | `127.0.0.1:5500` |
| `right` / D1_B | `192.168.123.99` | `enx00e04c681b82` | `192.168.123.98/24` | `127.0.0.1:5501` |

D1_A 用网线直连电脑物理网卡。D1_B 用网线接 USB 扩展坞网卡，再接电脑。
D1_B 的 `/etc/network/interfaces` 已永久改为 `192.168.123.99`。

> 不要将电脑网卡设为 `.99` 或 `.100`；它们已分别属于两台 D1。

## 重启后初始化

在仓库根目录执行：

```bash
sudo ./scripts/setup_d1_network.sh
```

Supervisor 建议的单臂测试中，只初始化实际接通的一条链路；未接的网卡不会被检查：

```bash
sudo ./scripts/setup_d1_network.sh --arm left
# 或
sudo ./scripts/setup_d1_network.sh --arm right
```

该脚本会让 NetworkManager 停止接管两张网卡，重置两个电脑端地址，
并添加两条 `/32` 主机路由。它会清除这两张专用 D1 网卡上的其他 IP，
因此不要在运行时将它们用于其他网络。

手动等价命令：

```bash
sudo nmcli device set enp5s0 managed no
sudo nmcli device set enx00e04c681b82 managed no
sudo ip addr flush dev enp5s0
sudo ip addr add 192.168.123.97/24 dev enp5s0
sudo ip link set enp5s0 up
sudo ip addr flush dev enx00e04c681b82
sudo ip addr add 192.168.123.98/24 dev enx00e04c681b82
sudo ip link set enx00e04c681b82 up
sudo ip route replace 192.168.123.100/32 dev enp5s0 src 192.168.123.97
sudo ip route replace 192.168.123.99/32 dev enx00e04c681b82 src 192.168.123.98
```

## 分层排查

1. 确认网卡名称仍存在：

   ```bash
   ip -br link show enp5s0
   ip -br link show enx00e04c681b82
   ```

2. 确认电脑端地址：

   ```bash
   ip -br -4 addr show enp5s0
   ip -br -4 addr show enx00e04c681b82
   ```

3. 确认精确路由（这一步用于排除同 `/24` 多网卡选路错误）：

   ```bash
   ip route get 192.168.123.100
   # 应包含: dev enp5s0 src 192.168.123.97
   ip route get 192.168.123.99
   # 应包含: dev enx00e04c681b82 src 192.168.123.98
   ```

4. 确认 IP 层：

   ```bash
   ping -I enp5s0 -c 3 192.168.123.100
   ping -I enx00e04c681b82 -c 3 192.168.123.99
   ```

5. 确认 DDS 与 bridge：

   ```bash
   ./scripts/start_robot.sh idle --arm left
   ```

   两个 bridge 日志应分别显示 `interface=enp5s0` 和
   `interface=enx00e04c681b82`，且 Python 客户端应连上 `5500/5501`。

## 常见故障

- 一台可 ping，另一台不通：先查 `/32` 主机路由和 USB 扩展坞网卡名是否变化。
- 重启后 USB 网卡地址消失：重跑 `setup_d1_network.sh`，并检查 `nmcli device status`。
- ping 通但无关节反馈：检查 bridge 是否绑定了正确网卡，两臂的 topic 都应为
  `rt/arm_Command` / `current_servo_angle`，不使用 `_1` 后缀。
- 两臂反馈串线：查看 bridge 启动日志的 `interface=`，并确认没有遗留的旧 bridge 进程。
- `.99` 重启后变回 `.100`：SSH 到 D1_B，复查 `/etc/network/interfaces`，再重启 D1_B。

## 为什么 topic 相同也能区分双臂

两个 `d1_bridge` 是独立进程，分别调用
`ChannelFactory::Init(0, <interface>)`。DDS 流量被物理网卡隔离，因此可继续使用 D1 官方默认 topic。
IP 的不同用于主机路由和运维；D1 命令中的 `address=1` 不是用来区分两台臂的。

## 不接 cambot 的单臂/双臂测试

`scripts/test_dual_d1.sh` 会按 `--arm` 只启动所选 bridge，
运行一项测试后自动关闭 bridge。它不启动 cambot、Quest、相机或数据采集。

首次标定建议完整测试完一只臂并断电后，再接通另一只。以左臂为例：

```bash
sudo ./scripts/setup_d1_network.sh --arm left
./scripts/test_dual_d1.sh status --arm left --watch --seconds 5
./scripts/test_dual_d1.sh action power-on --arm left --yes
./scripts/test_dual_d1.sh action enable --arm left --yes
./scripts/test_dual_d1.sh hold --arm left --yes
./scripts/test_dual_d1.sh action disable --arm left --yes
./scripts/test_dual_d1.sh action power-off --arm left --yes
```

右臂测试将 `left` 全部改为 `right`。只有在两臂均完成独立验证后才使用下面的 `both` 流程。

建议严格按下列顺序，且每次只测一个小动作：

```bash
# 1. 只读：两臂都应显示 7 个反馈值
./scripts/test_dual_d1.sh status --watch --seconds 5

# 2. 上电、使能（不会自动连着做）
./scripts/test_dual_d1.sh action power-on --arm both --yes
./scripts/test_dual_d1.sh action enable --arm both --yes

# 3. 原样回发当前目标，验证命令链路
./scripts/test_dual_d1.sh hold --arm both --yes

# 4. 左臂 J0 +1°，观察只有左臂动
./scripts/test_dual_d1.sh jog --arm left --joint 0 --delta 1 --yes

# 5. 右臂 J0 +1°，观察只有右臂动
./scripts/test_dual_d1.sh jog --arm right --joint 0 --delta 1 --yes

# 6. 回到原位可用 -1°（以实时反馈为基准）
./scripts/test_dual_d1.sh jog --arm left --joint 0 --delta -1 --yes
./scripts/test_dual_d1.sh jog --arm right --joint 0 --delta -1 --yes

# 7. 测完立即卸力、断电
./scripts/test_dual_d1.sh action disable --arm both --yes
./scripts/test_dual_d1.sh action power-off --arm both --yes
```

`jog` 对 J0–J5 强制单次不超过 ±5°，并检查 URDF 关节限位；
J6 是夹爪，单次不超过 ±5，目标限制在 0–65。
`zero` 可能引起明显运动，不建议在首次微动测试中使用。
