# Quest 3 → Unitree D1：当前运动学计算链条与审计记录

> 状态：按 2026-09-03 当前代码整理。本文描述“代码现在实际做了什么”，不代表其中所有假设已经通过真机标定。
> 本轮只整理和审计，不修改运动学算法。

## 1. 本文要回答的问题

当前遥操闭环把 Quest 手柄的 6DoF 位姿转换成 D1 的 6 个关节角，并把食指扳机转换成夹爪开合量。完整链条是：

```text
Quest WebXR gripSpace
  └─ p_h(t), q_h(t)，约 90 Hz
       ↓ WebSocket
cambot server（只转发，不换坐标、不做滤波）
       ↓ localhost TCP 6001，JSON line
ForwardReceiver（保存每只手最新一帧）
       ↓ robot/control.py，10 Hz 取样
Home/死手接合与基准记录
       ↓
ControllerMapping
  ├─ 手柄相对平移 → D1 基座系目标 TCP 位置
  └─ 手柄相对旋转 → D1 基座系目标 TCP 姿态
       ↓
D1IK.solve_continuous_step
  ├─ URDF 正运动学
  ├─ 位置/姿态误差
  ├─ 数值 Jacobian
  └─ 阻尼最小二乘 → 6 关节增量
       ↓
连续性、接合点、领先反馈、URDF 关节限位
       ↓
6 个关节目标 + 1 个夹爪目标
       ↓ localhost TCP 5500/5501
d1_bridge
       ↓ DDS rt/arm_Command，funcode=2，mode=0，10 Hz
D1 内部控制器
       ↓ DDS current_servo_angle
最新 7 值反馈 → 下一控制周期
```

涉及的主要代码：

- WebXR 采集：`/home/ubuntu/cambot/cambot/teleop/client/index.html:1188`
- cambot 转发：`/home/ubuntu/cambot/cambot/teleop/server.py:411`
- 手柄接收、接合、控制主循环：`robot/control.py:35`、`robot/control.py:321`
- 坐标与四元数映射：`robot/controller_mapping.py:12`、`robot/controller_mapping.py:93`
- D1 正逆运动学：`robot/d1_ik.py:30`
- Python 到 bridge：`robot/d1_client.py:18`
- bridge 到 DDS：`robot/d1_bridge/main.cpp:46`、`robot/d1_bridge/main.cpp:68`
- 当前参数：`robot/config.yaml:29`

## 2. 坐标、变量和单位

本文使用以下符号：

| 符号 | 含义 | 单位/格式 |
|---|---|---|
| $\mathbf p_h(t)$ | 当前 Quest 手柄 `gripSpace` 位置 | m，WebXR xyz |
| $\mathbf R_h(t)$ / $\mathbf q_h(t)$ | 当前手柄姿态 | 旋转矩阵 / xyzw 四元数 |
| $\mathbf p_{h,0}$, $\mathbf R_{h,0}$ | 按 A/接合时的手柄基准 | m / 旋转 |
| $\mathbf q_f(t)$ | D1 最新反馈的 J1…J6 | 代码接收 deg，进入运动学前转 rad |
| $\mathbf q_c(t)$ | 最后一次发送的 J1…J6 目标 | deg；IK 前转 rad |
| $\mathbf T_0(\mathbf q)$ | URDF 从 `base_link` 到自定义 TCP 的正运动学 | $4\times4$ 齐次矩阵 |
| $\mathbf p_{e,0}$, $\mathbf R_{e,0}$ | 接合时由反馈关节角和 URDF 算出的 TCP 基准 | m / 旋转 |
| $\mathbf p_e^*$, $\mathbf R_e^*$ | 本周期希望到达的 TCP 目标 | m / 旋转 |
| $\mathbf J(\mathbf q)$ | $6\times6$ 数值几何 Jacobian | 上三行 m/rad，下三行 rad/rad |

注意：日志中的 `actual_xyz` 不是外部传感器测得的真实 TCP，而是把关节反馈代入当前 URDF 得到的模型 TCP。

## 3. Quest 端位姿采集

### 3.1 数据来源

浏览器对每个 `XRInputSource` 调用：

```javascript
xrFrame.getPose(source.gripSpace, referenceSpace)
```

发送内容为：

```json
{
  "type": "controller_pose",
  "hand": "right",
  "p": {"x": 0, "y": 0, "z": 0},
  "q": {"x": 0, "y": 0, "z": 0, "w": 1},
  "trigger": 0,
  "squeeze": 0,
  "buttons": [],
  "t": 0
}
```

当前使用的是 `gripSpace`，不是手柄射线 `targetRaySpace`，也不是实际夹爪尖端。`p`、`q` 都相对于 WebXR 当前 `referenceSpace`。

### 3.2 这一段没有做的事情

- 没有对位置或四元数做低通滤波。
- 没有检查相邻帧位姿跳变。
- 没有记录或补偿 Quest tracking origin 重定位。
- 没有把手柄坐标原点平移到操作者手腕旋转中心。
- cambot server 只转发 JSON，不改变坐标、四元数顺序或单位。

## 4. ActiveVA 接收与 10 Hz 取样

`ForwardReceiver` 在 `127.0.0.1:6001` 接收消息，以电脑的 `time.monotonic()` 标记到达时间，并且只保存每只手的最新一帧。D1 控制循环按配置以 10 Hz 调用一次 `D1Teleop.step()`。

因此当前是“Quest 约 90 Hz 发布，D1 10 Hz 读取最新值”，不是对 90 Hz 数据平均，也不是对所有样本积分。

当最新数据年龄超过 250 ms：

- 立即停止生成和下发新关节目标；
- Home 模式在 3 秒内恢复时，以恢复时的手柄位姿和机械臂反馈重新建立基准；
- 超过 3 秒需重新长按 A。

## 5. 接合与 Home 基准

Home 模式启动后，机械臂先按关节空间 smoothstep 轨迹进入 `robot/calibration/d1_home.yaml` 中保存的 7 值姿态。右臂当前保存值为：

$$
\mathbf x_{\mathrm{home,right}}=
\begin{bmatrix}
7.2&-60.9&22.1&2.5&50.4&-21.7&60.5
\end{bmatrix}^{\mathsf T}.
$$

前 6 项是关节角度，最后一项是夹爪值。

操作者长按 A 后，`_engage()` 执行：

1. 从 D1 反馈取得 $\mathbf q_{f,0}$。
2. 用 $\mathbf T_0(\mathbf q_{f,0})$ 算接合时 TCP：$\mathbf p_{e,0}$、$\mathbf R_{e,0}$。
3. 保存当前手柄：$\mathbf p_{h,0}$、$\mathbf R_{h,0}$。
4. 令连续 IK 的初始命令参考 $\mathbf q_{c,0}=\mathbf q_{f,0}$。
5. 接合前只检查该姿态的加权 Jacobian 条件数是否小于 80。

这里的“对齐”不是求 Quest 世界坐标到机器人世界坐标的完整外参，而只是把两者当前位姿分别设为各自的零增量。

## 6. 手柄平移到目标 TCP 位置

WebXR 通常定义 `+X` 向右、`+Y` 向上、`-Z` 向前。代码先计算：

$$
\Delta\mathbf p_h(t)=\mathbf p_h(t)-\mathbf p_{h,0}
$$

当前右臂配置：

```yaml
position_scale: 1.20
position_axes: [0, 2, 1]
position_signs: [-1, -1, 1]
```

所以右臂目标增量为：

$$
\Delta\mathbf p_e
=1.2
\begin{bmatrix}
-1&0&0\\
0&0&-1\\
0&1&0
\end{bmatrix}
\Delta\mathbf p_h
$$

逐分量写为：

$$
\begin{aligned}
\Delta p_{e,x}&=-1.2\,\Delta p_{h,x},\\
\Delta p_{e,y}&=-1.2\,\Delta p_{h,z},\\
\Delta p_{e,z}&=+1.2\,\Delta p_{h,y}.
\end{aligned}
$$

也就是：

| 手柄运动 | WebXR 增量 | 右臂局部目标增量 |
|---|---:|---:|
| 向右 | $\Delta x>0$ | $\Delta x<0$ |
| 向前 | $\Delta z<0$ | $\Delta y>0$ |
| 向上 | $\Delta y>0$ | $\Delta z>0$ |

完整模式把向量范数限制到 0.35 m：

$$
\Delta\bar{\mathbf p}_e=
\begin{cases}
\Delta\mathbf p_e,
&\lVert\Delta\mathbf p_e\rVert_2\leq 0.35,\\[4pt]
0.35\dfrac{\Delta\mathbf p_e}{\lVert\Delta\mathbf p_e\rVert_2},
&\lVert\Delta\mathbf p_e\rVert_2>0.35,
\end{cases}
$$

$$
\mathbf p_e^*=\mathbf p_{e,0}+\Delta\bar{\mathbf p}_e.
$$

`position` / `orientation` 标定模式改用 0.05 m 总范围。

## 7. 手柄旋转到目标 TCP 姿态

### 7.1 当前相对旋转公式

代码把 xyzw 四元数正规化后计算：

$$
\mathbf q_{\Delta h}(t)
=\mathbf q_h(t)\otimes\mathbf q_{h,0}^{-1},
\qquad
\mathbf R_{\Delta h}(t)=\mathcal R\!\left(\mathbf q_{\Delta h}(t)\right),
$$

其中 $\otimes$ 表示 xyzw 四元数乘法，$\mathcal R(\cdot)$ 表示四元数到旋转矩阵的转换。

这是把手柄从接合姿态转到当前姿态的“参考空间/左乘”旋转增量。

右臂当前姿态变换配置与平移配置分开：

```yaml
orientation_axes: [0, 2, 1]
orientation_signs: [-1, 1, 1]
```

由此构造：

$$
\mathbf B=
\begin{bmatrix}
-1&0&0\\
0&0&1\\
0&1&0
\end{bmatrix}.
$$

$\det(\mathbf B)=+1$，所以它是正规旋转基变换，不是镜像。手柄相对旋转被转换为：

$$
\mathbf R_{\Delta e}
=\mathbf B\mathbf R_{\Delta h}\mathbf B^{\mathsf T}.
$$

然后限制相对旋转总角度不超过 45°，并左乘接合时 TCP 姿态：

$$
\mathbf R_e^*=\mathbf R_{\Delta e}\mathbf R_{e,0}.
$$

### 7.2 这里尚未得到真机确认的假设

1. WebXR `gripSpace` 的轴方向是否恰好适合作为工具坐标方向，尚未做逐轴旋转标定。
2. 使用空间旋转增量 $\mathbf R_h\mathbf R_{h,0}^{\mathsf T}$ 并左乘 $\mathbf R_{e,0}$ 是否符合期望手感，尚未和“手柄局部旋转增量 $\mathbf R_{h,0}^{\mathsf T}\mathbf R_h$ 后右乘工具姿态”的方案对比。
3. 手柄舒适 Home 的握持角与 D1 夹爪朝向之间没有固定外参，只靠接合瞬间相对对齐。
4. 当前日志没有输出手柄旋转增量、目标 TCP 旋转误差或各关节目标，无法从现有日志验证究竟是哪一旋转轴首先出错。

## 8. D1 URDF 和 TCP

### 8.1 运动链

使用 `robot/urdf/d1_description.urdf` 的 670 mm 版本，从：

```text
base_link → Joint1 → … → Joint6 → Link6 → fixed gripper_center_tcp
```

只激活 J1…J6。夹爪关节不进入 IK。

### 8.2 当前 TCP 假设

代码没有把任意一根夹指当 TCP，而是在 `Link6` 后加固定点：

$$
{}^{L6}\mathbf p_{\mathrm{TCP}}
=\begin{bmatrix}-0.00562&0&0.0706\end{bmatrix}^{\mathsf T}\ \mathrm m.
$$

它来自两根夹指安装原点的几何中点，但尚未通过真机“原地转腕、观察夹取点是否绕固定点旋转”确认。因此 TCP 偏差仍可能造成：请求纯旋转时，真实夹爪中心发生平移；IK 为维持模型 TCP，又让其他关节补偿。

### 8.3 关节范围

URDF 给出的范围近似为：

$$
\begin{aligned}
q_1&\in[-134.6^\circ,\,134.6^\circ],&
q_2&\in[-90^\circ,\,90^\circ],\\
q_3&\in[-90^\circ,\,90^\circ],&
q_4&\in[-134.6^\circ,\,134.6^\circ],\\
q_5&\in[-90^\circ,\,90^\circ],&
q_6&\in[-134.6^\circ,\,134.6^\circ].
\end{aligned}
$$

当前代码采用相同的对称硬限位。但 URDF 零位、轴正方向和真机 `angle0…angle5` 是否完全一致，尚未通过每轴正负微动和 FK 实测完全闭环确认。

## 9. 连续阻尼 IK

### 9.1 使用哪个关节状态作为线性化点

每周期优先使用最后一次发送的目标 $\mathbf q_c(t-1)$，而不是总从滞后的 10 Hz 反馈 $\mathbf q_f(t)$ 重算。如果命令与反馈任一关节相差超过 $6^\circ$，则重新令：

$$
\left\lVert\mathbf q_c(t-1)-\mathbf q_f(t)\right\rVert_\infty>6^\circ
\quad\Longrightarrow\quad
\mathbf q_c(t-1)\leftarrow\mathbf q_f(t).
$$

正运动学在线性化点得到：

$$
\mathbf T_c=\mathbf T_0\!\left(\mathbf q_c\right)
=
\begin{bmatrix}
\mathbf R_c&\mathbf p_c\\
\mathbf 0^{\mathsf T}&1
\end{bmatrix}.
$$

### 9.2 本周期笛卡尔误差

$$
\mathbf e_p=\mathbf p_e^*-\mathbf p_c,
\qquad
\mathbf e_R=\operatorname{Log}\!\left(
\mathbf R_e^*\mathbf R_c^{\mathsf T}
\right)^\vee.
$$

其中 $\operatorname{Log}(\cdot)^\vee$ 返回在世界/基座坐标中表达的三维轴角向量。

完整模式每周期分别截断：

$$
\lVert\mathbf e_p\rVert_2\leq0.008\ \mathrm m,
\qquad
\lVert\mathbf e_R\rVert_2\leq4^\circ.
$$

注意这只是每个 100 ms 周期追踪的最大误差，不是总工作范围。

### 9.3 数值 Jacobian

对每个关节使用 $\varepsilon=10^{-5}\ \mathrm{rad}$ 前向差分。令 $\mathbf e_i$ 为第 $i$ 个标准基向量：

$$
\mathbf J_v[:,i]
=\frac{\mathbf p(\mathbf q+\varepsilon\mathbf e_i)-\mathbf p(\mathbf q)}{\varepsilon},
$$

$$
\mathbf J_\omega[:,i]
=\frac{
\operatorname{Log}\!\left(
\mathbf R(\mathbf q+\varepsilon\mathbf e_i)\mathbf R(\mathbf q)^{\mathsf T}
\right)^\vee
}{\varepsilon},
$$

$$
\mathbf J(\mathbf q)=
\begin{bmatrix}
\mathbf J_v(\mathbf q)\\
\mathbf J_\omega(\mathbf q)
\end{bmatrix}\in\mathbb R^{6\times6}.
$$

### 9.4 阻尼最小二乘

当前权重与阻尼：

$$
\mathbf W=\operatorname{diag}(1,1,1,0.5,0.5,0.5),
\qquad \lambda=0.05.
$$

求解：

$$
\mathbf J_w=\mathbf W\mathbf J,
\qquad
\mathbf e_w=\mathbf W
\begin{bmatrix}
\mathbf e_p\\
\mathbf e_R
\end{bmatrix},
$$

$$
\Delta\mathbf q
=\mathbf J_w^{\mathsf T}
\left(
\mathbf J_w\mathbf J_w^{\mathsf T}+\lambda^2\mathbf I_6
\right)^{-1}
\mathbf e_w,
$$

$$
\mathbf q_{\mathrm{raw}}=\mathbf q_c+\Delta\mathbf q.
$$

这里位置单位是 m，旋转单位是 rad。虽然旋转行乘了 0.5，但没有进行基于机械臂尺度的无量纲化。以当前单周期上限比较，位置误差最多 0.008，而旋转误差最多约 0.0698；加权后旋转仍可达约 0.0349，数值上比位置项大。因此完整 6DoF 中姿态误差可能明显主导解算。这是“平移停住后腕部仍继续找姿态”的重要嫌疑点。

## 10. IK 后的关节安全处理

$\mathbf q_{\mathrm{raw}}$ 依次经过：

1. **连续性拒绝**：若 $\lVert\Delta\mathbf q\rVert_\infty>6^\circ$，整帧不发送。
2. **接合点范围**：完整模式将每个关节限制在接合关节 $\pm90^\circ$。
3. **单周期硬限幅**：配置为 $\pm30^\circ$，正常情况下先被 $6^\circ$ 连续性检查约束，因此很少实际触发。
4. **领先反馈限制**：目标满足 $\lVert\mathbf q_{\mathrm{sent}}-\mathbf q_f\rVert_\infty\leq6^\circ$。
5. **D1/URDF 对称硬限位**：逐关节裁到第 8.3 节范围。

这些限制都是“解算完成后的逐关节裁剪”。连续 IK 本身没有：

- joint-limit avoidance/null-space objective；
- 关节限位方向的速度投影；
- 自碰撞或环境碰撞约束；
- 不可达目标检测和 anti-windup；
- 任务优先级（例如平移优先、姿态次优先）。

因此一旦某关节触限，裁剪后的关节向量一般已不再满足原 TCP 位置/姿态。下一周期仍会追同一个不可达目标，可能表现为腕部持续转、其他关节代偿、反复命中限幅，直至看起来“死住”。

## 11. 夹爪命令与“夹爪旋转”的区别

完整模式中夹爪开合量完全不经过 IK：

$$
g^*=65+u_{\mathrm{trigger}}(0-65)
=65\left(1-u_{\mathrm{trigger}}\right),
\qquad u_{\mathrm{trigger}}\in[0,1].
$$

然后相对夹爪反馈限制为每周期最多 $\pm20$，作为第七项 `angle6` 发送，即

$$
g_{\mathrm{sent}}=
\operatorname{clip}\!\left(
g^*,\ g_f-20,\ g_f+20
\right).
$$

所以需要区分：

- 两根夹指开合：由 `trigger → angle6` 引起；
- 整个夹爪/末端绕轴旋转：由 J1…J6 的姿态 IK 引起，通常主要表现为腕部关节运动。

当前日志没有输出 `trigger`、`angle6 feedback`、`angle6 target` 和 J1…J6 目标，暂时不能只凭“夹爪在转”判断属于哪一种。

## 12. 下发到真机与反馈返回

`D1Client.set_target()` 发送：

```json
{"cmd":"target", "angles":[J1,J2,J3,J4,J5,J6,gripper], "mode":0}
```

`d1_bridge` 转成：

```json
{
  "address":1,
  "funcode":2,
  "data":{"mode":0,"angle0":...,"angle1":...,"angle6":...}
}
```

通过该臂独占网卡上的 `rt/arm_Command` 发布。`mode=0` 是当前采用的 10 Hz 小平滑模式。

反馈从 `current_servo_angle` 进入 bridge。当前 bridge 每 100 ms 广播一次内存中的“最新值”，但消息中没有：

- D1 原始反馈时间戳；
- DDS 序号；
- 本次 100 ms 内是否真的收到过新反馈。

所以 Python 端看到 TCP 数据持续到达，也无法判断收到的是新 DDS 反馈还是 bridge 重播的旧值。这一点与本轮日志中 `actual_xyz` 长时间逐字不变直接相关，必须在后续诊断中补足。

## 13. 当前诊断日志的精确定义

| 字段 | 实际含义 |
|---|---|
| `hand_xyz` | $\mathbf p_h(t)-\mathbf p_{h,0}$，WebXR 原始相对平移，未缩放，mm |
| `mapped_xyz` | $\mathbf p_e^*-\mathbf p_{e,0}$，坐标换轴、符号、1.2 倍缩放及总范围裁剪后的目标位移，mm |
| `actual_xyz` | $\operatorname{FK}(\mathbf q_f)-\mathbf p_{e,0}$，由最新关节反馈和 URDF 重建的模型位移，mm |
| `command_step_xyz` | $\operatorname{FK}(\mathbf q_{\mathrm{target}})-\operatorname{FK}(\mathbf q_f)$，发送目标相对反馈的模型位移，mm |
| `max_dq` | $\lVert\mathbf q_{\mathrm{raw}}-\mathbf q_c\rVert_\infty$，本次连续 IK 原始输出相对命令参考的最大关节增量，deg |
| `command_lag` | $\lVert\mathbf q_{\mathrm{sent}}-\mathbf q_f\rVert_\infty$，最终发送目标相对最新反馈的最大关节差，deg |
| `rx` | 最近约 2 秒 Quest 消息到达频率，不是 D1 反馈频率 |
| `age` | 最新 Quest 消息年龄，不是 D1 反馈年龄 |
| `max_gap` | Quest 消息最大相邻间隔，不是 DDS 间隔 |

`command_step_xyz` 这个名字容易误解：它不是 IK 单步相对上一命令的位移，而是 $\operatorname{FK}(\mathbf q_{\mathrm{sent}})-\operatorname{FK}(\mathbf q_f)$。

## 14. 本轮日志如何沿链条解释

### 14.1 已确认正常的部分

- Quest 转发长期约 90 Hz，常见 `age=0–20 ms`，方向问题不是由通信时延造成。
- 接合时目标与反馈一致：`hand/mapped/actual=0`，且 $\lVert\Delta\mathbf q\rVert_\infty=0$。
- 当前右臂平移映射符合现场反馈，前后左右方向基本正常。
- 新连续 IK 没再出现旧全局 IK 的 `Initial guess is outside of provided bounds`。

### 14.2 停住后末端继续旋转

第一段中手柄平移已经基本稳定，例如：

```text
hand_xyz ≈ [-109.7, +6.2, -8.3] mm
mapped_xyz ≈ [+131.6, +9.9, +7.4] mm
```

但 `actual_xyz.y` 仍从约 $+44\ \mathrm{mm}$ 逐步走到约 $+12\ \mathrm{mm}$，同时命令多次达到 $6^\circ$ 领先反馈限制。这说明控制器仍在追踪尚未完成的 6DoF 目标。因为当前日志完全没有姿态误差字段，不能证明继续运动来自位置残差还是姿态残差；结合“夹爪旋转”的真机观察，姿态任务持续驱动是首要嫌疑。

### 14.3 触限后“死掉”

第一段很快反复出现：

```text
D1 物理关节限位
接合点±90°
领先反馈±6°
```

之后目标和实际位置严重不一致。这符合“局部 IK 没有关节限位规避，解完后再硬裁剪”的失效模式。

第二段中另一个关键现象是：

```text
actual_xyz=[-1.5,-7.7,+17.0]mm
```

在大量连续行中完全不变，而 `hand_xyz`、`mapped_xyz` 和 `command_step_xyz` 仍在变化，`command_lag` 固定到 $6^\circ$。可能性包括：

1. 真机因触限、内部保护或命令语义而没有继续执行；
2. DDS 反馈停止更新，但 bridge 在重复广播缓存反馈；
3. 最终目标持续被领先反馈窗口裁在真机无法执行的位置；
4. URDF/FK 与真机关节语义不一致，使模型判断失真。

现有日志无法区分 1 和 2，因为没有 D1 反馈新鲜度或关节目标明细。

### 14.4 “小角度旋转”期间出现的大平移

日志里相对手柄位移从几十毫米扩大到：

```text
hand_xyz ≈ [-67, +78, -278] mm
随后约 [-84, +54, -345] mm
```

另一段达到约 400 mm，并触发 0.35 m 总范围裁剪。如果操作者实际只做了小角度旋转，这不是正常的小幅旋转伴随位移，需要检查：

- `gripSpace` 原点是否随手臂摆动而实际移动；
- Quest 是否发生 tracking origin 重定位或短暂丢失前的位姿跳变；
- Home 对齐后手柄是否离开了对齐位置很远；
- 是否需要用手腕旋转中心而不是控制器 `gripSpace` 作为遥操参考点。

最后的停止仍是明确的 Quest 真实断流：`age` 从 562 ms 增加到 8.5 s；这与前面的运动学饱和是两个不同问题。

## 15. 当前最需要审查和验证的假设

按优先级排列：

1. **反馈是否真的每周期更新**：bridge 必须携带 DDS 接收时间/序号，不能把缓存重播误当新反馈。
2. **完整模式的位置/姿态任务权重**：当前 m 与 rad 直接混合，姿态项可能主导。
3. **关节限位处理**：应进入 IK 求解或速度投影，而不是在 IK 后逐关节硬裁。
4. **姿态映射约定**：需要逐轴验证空间左乘方案，必要时与工具局部右乘方案比较。
5. **Quest 位姿跳变**：需要记录 $\Delta\mathbf p$、相对旋转轴角和相邻帧速度，并对 tracking reset 单独处理。
6. **TCP 偏移**：当前为 URDF 几何中点估计，未做真实旋转中心标定。
7. **URDF 与真机角度一致性**：零位、轴方向、关节顺序和限位需要逐轴闭环确认。

## 16. 下一轮为了定位而应增加的观测量

在继续修改控制律前，建议让每条 10 Hz 诊断同时输出：

```text
hand_dp_xyz
hand_delta_angle_deg + hand_delta_axis
target_position_error_xyz
target_rotation_error_deg + target_rotation_axis
q_feedback[6]
q_reference[6]
q_raw[6]
q_sent[6]
joint_limit_margin[6]
trigger / gripper_feedback / gripper_target
DDS_feedback_seq / DDS_feedback_age_ms
```

这样能用一次短测试明确区分：

- 手柄位置跳变；
- 手柄姿态映射错误；
- 姿态任务权重过大；
- TCP/URDF 错误；
- IK 触限；
- 真机拒绝命令；
- DDS 反馈冻结。

## 17. 当前审计结论

平移坐标符号已经基本通过现场验证，但完整 6DoF 链条还不能视为正确。现有证据指向三个结构性问题：

1. 姿态映射和 TCP 尚未真机标定，且姿态误差可能在 DLS 中数值占优；
2. IK 不感知关节限位，事后裁剪会破坏笛卡尔任务并造成持续代偿；
3. bridge 重播缓存反馈而不报告新鲜度，导致控制器无法识别反馈冻结。

因此下一步不应继续盲调 `position_scale`、阻尼或滤波系数。应先补齐第 16 节的观测量，然后分别运行 `position`、`orientation`，最后才重新进入 `full`，用日志确定错误首先出现在哪一层。
