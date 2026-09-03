# cambot Quest 手柄转发（已应用）

已对 `/home/ubuntu/cambot` 做了两处增量修改，不改变原有头显控制、
相机视频或 cambot 安全逻辑：

- `cambot/teleop/client/index.html`：在 WebXR 循环中以最高约 120 Hz 发送
  左/右 `gripSpace` 位姿、食指扳机、侧握键和按键列表。
- `cambot/teleop/server.py`：将 `controller_pose` / `episode` 经持久 TCP
  连接转发到 ActiveVA，默认 `127.0.0.1:6001`。ActiveVA 未启动时
  每秒最多重连一次，不影响 WebXR 服务。
- `CAMBOT_ACTIVEVA_CONTROLLER_MODE=1` 时，网页不再让 A/X、B/Y、左右侧握键
  同时触发 cambot 的校准、位置、HUD 或 pause；按键原始状态仍转发给 D1。
  `scripts/start_quest_stream.sh` 会自动设置该变量。

可通过环境变量覆盖转发目标：

```bash
export CAMBOT_FORWARD_HOST=127.0.0.1
export CAMBOT_FORWARD_PORT=6001
```

## 消息契约

```json
{"type":"controller_pose","hand":"left","q":{"x":0,"y":0,"z":0,"w":1},"p":{"x":0,"y":0,"z":0},"trigger":0,"squeeze":0,"buttons":[],"t":0}
```

- `hand`: `left` / `right`，分别对应 ActiveVA 左/右 D1。
- `squeeze`: 侧握键，ActiveVA 的 `deadman` 接合模式用它做指令门控。
- `buttons[4]` / `buttons[5]`: A/X 对齐与 B/Y 暂停，供 `home` 接合模式使用。
- `trigger`: 食指扳机，线性映射夹爪（0=开，1=关）。
- `t`: Quest 页面的 `performance.now()`；ActiveVA 超时判定使用电脑本地收包时间。

完整联调步骤见 `docs/quest_d1_teleop.md`。
