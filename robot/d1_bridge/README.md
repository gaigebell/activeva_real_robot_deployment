# d1_bridge：D1 驱动桥（C++）

每臂一个实例：本地 TCP 收目标/命令 → DDS 下发；订阅臂反馈（官方 10 Hz）→ TCP 回传。
Python 侧只走 TCP JSON line（见 `robot/d1_client.py`），改协议无需动 Python。

## 构建（Ubuntu，机器人端）

前置：unitree_sdk2 已安装到 /usr/local（`cmake .. && make && sudo make install`），
并已按 d1_sdk 环境装好 ddscxx / iceoryx。

```bash
cd robot/d1_bridge
mkdir -p build && cd build
cmake .. -DD1_SDK_DIR=../../d1_sdk   # 按实际路径
make
```

## 运行

```bash
# 左臂（默认 topic；方案一绑网卡）
./d1_bridge --port 5500 --topic rt/arm_Command --feedback-topic current_servo_angle --address 1
# 右臂（方案二改 topic 后缀）
./d1_bridge --port 5501 --topic rt/arm_Command_1 --feedback-topic current_servo_angle_1 --address 1
```

## TCP 协议（JSON line）

客户端 → 桥：
```json
{"cmd": "target", "angles": [a0..a6], "mode": 0}   // funcode 2
{"cmd": "enable"} {"cmd": "disable"}                // funcode 5
{"cmd": "power_on"} {"cmd": "power_off"}            // funcode 6
{"cmd": "zero"}                                    // funcode 7
```
桥 → 客户端：
```json
{"type": "feedback", "angles": [7], "t": ...}
```

## TODO(现场)

- [ ] 核对 PubServoInfo_ 字段名与 7 值映射（对照 d1_sdk/src/get_arm_joint_angle.cpp）
- [ ] 夹爪语义（NQ5）
- [ ] 指令频率上限实测（mode 0/1 平滑差异）
- [ ] 安全：是否加"无指令超时自动卸力"看门狗（建议加）
