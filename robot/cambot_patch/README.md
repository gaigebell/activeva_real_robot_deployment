# cambot 手柄转发补丁（现场实施）

目标：Quest 浏览器 WebXR 页面在发送头显位姿的同时，发送**手柄 6DOF 位姿 / 扳机 / 按钮**；
cambot server 收到后经本地 TCP（默认 127.0.0.1:6001）转发给机器人端控制程序（`robot/control.py` 的 ForwardSubscriber）。

cambot 的内核逻辑（head_pose 控制环、安全机制、视频推流）**全部不动**，只新增一个消息类型与转发。

## 修改点 1：`cambot/cambot/teleop/client/index.html`

在 WebXR rAF 循环中（现有 `sendHeadPose()` 调用处，约 1156 行）追加：

```js
// 逐手柄发送 gripSpace 位姿与扳机/按钮
for (const source of session.inputSources) {
  const pose = xrFrame.getPose(source.gripSpace, xrRefSpace);
  if (pose) {
    const p = pose.transform.position, q = pose.transform.orientation;
    ws.send(JSON.stringify({
      type: "controller_pose",
      hand: source.handedness,            // "left" | "right"
      q: {x: q.x, y: q.y, z: q.z, w: q.w},
      p: {x: p.x, y: p.y, z: p.z},
      trigger: source.gamepad?.buttons[0]?.value ?? 0,     // 扳机 0-1
      squeeze: source.gamepad?.buttons[1]?.value ?? 0,
      buttons: (source.gamepad?.buttons ?? []).map(b => b.pressed),  // 含 RButtonOne
      t: performance.now()
    }));
  }
}
// 开始/结束 episode：RButtonOne 按下沿 -> toggle
//   （按钮编号以 Quest WebXR xr-standard 映射为准，现场打印核对后确定）
```

注意：手柄按钮现有映射（A/X=Home 等）保持不变，新消息与现有按钮逻辑共存。

## 修改点 2：`cambot/cambot/teleop/server.py`

`websocket_stream` 的消息分发处（约 388–393 行 `msg_type == "head_pose"` 分支旁）新增：

```python
elif msg_type in ("controller_pose", "episode"):
    # 转发给机器人端控制程序（本地 TCP，JSON line）
    await forward_to_local(data)
```

模块级新增转发函数（裸 socket，独立线程或 asyncio to_thread 均可）：

```python
FORWARD_HOST = os.environ.get("CAMBOT_FORWARD_HOST", "127.0.0.1")
FORWARD_PORT = int(os.environ.get("CAMBOT_FORWARD_PORT", "6001"))

async def forward_to_local(data: dict):
    import asyncio
    try:
        reader, writer = await asyncio.open_connection(FORWARD_HOST, FORWARD_PORT)
        writer.write((json.dumps(data) + "\n").encode())
        await writer.drain()
        writer.close()
    except OSError:
        pass    # 控制程序未启动时静默丢弃
```

（每次消息短连接即可：~120 Hz 手柄消息，本地开销可忽略；如追求更低延迟可改为常驻连接。）

## 消息契约（与 robot/control.py ForwardSubscriber 对齐）

- `{"type": "controller_pose", "hand": "left"|"right", "q": {x,y,z,w}, "p": {x,y,z}, "trigger": 0-1, "squeeze": 0-1, "buttons": [bool...], "t": ms}`
- `{"type": "episode", "event": "toggle"}` —— RButtonOne 按下沿触发

## 联调验证

1. 启动 `robot/control.py --mode teleop`（或先跑一个打印工具），确认能收到手柄 JSON
2. Quest 浏览器打开 cambot 页面，动左右手柄，观察控制程序日志
