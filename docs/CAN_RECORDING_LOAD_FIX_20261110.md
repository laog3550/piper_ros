# 录制负载下的原始 CAN 丢失与修复（2026-11-10）

## 结论

`episode_000002` 的 0 个合格帧不是主臂松手造成的，也不是 100 ms 门限过严，而是**录制负载下原始 CAN 通路吞吐崩塌**：

- 录制时 `/piper_direct/*` 约 800 帧/秒的话题只记录到约 6–8%，其余帧的源 Header 到 bag 接收延迟从约 1.5 ms 累积到 1.5–2.6 秒；
- 向量节点 65–89% 的 `stale` 是该延迟在 50 Hz 发布端的下游表现；
- 同一 bag 中 50 Hz 向量话题和三路图像始终准时完整，说明不是整机或 rosbag 全面过载，而是高频 CAN 被挤掉。

原始数据保持只读，`episode_000002` 继续作为诊断材料，不授予 `training_ready`。

## 只读实测

| 检查 | 结果 |
| --- | --- |
| `can_left` 总线 | 3040–3180 帧/秒，100 ms 桶内约 319 帧；`ERROR-ACTIVE`，RX error/dropped/bus-off 均为 0 |
| 空闲 ROS 链路（不录制） | `/piper_direct/left/slave_feedback` 800.6 帧/秒，Header 年龄 2.4 ms；向量 50/秒，年龄约 3.25 ms |
| 录制中原始 CAN | 主目标 759 条、反馈 973 条（按 200 Hz×4 ID×15 s 应约 12000 条），约 6–8% |
| 原始 CAN Header→bag | 主目标 p50 196 ms、p95 2302 ms、max 2615 ms；逐秒中位数 25.7→283→1994→约 2358 ms |
| 每 ID 源 Header | p50 5 ms（200 Hz），但每个 ID 有 6–9 次 1.7–2.1 秒空洞 |
| 向量话题 | 左主目标 89.0% stale，Header 年龄 p50 226 ms；左反馈 65.0%，p50 194.5 ms；四帧 skew p50 0.37 ms |
| 三路图像 | 稳定 30 fps、间隔 33.3 ms、Header 年龄 26–40 ms、无倒退 |
| bag 逐秒总量 | 约 400 条/秒：向量 200/s + 图像/内参 180/s + 原始 CAN 仅约 20/s |

## 根因链

1. rosbag2 0.15.17 默认对每个话题使用 `KEEP_LAST, depth=10` 订阅，不继承发布端的 depth 1000；对 800–1600 帧/秒的 CAN 只有 6–12 ms 缓冲余量，回调里又同步写盘。
2. 三路 640×480 rgb8 以 30 fps 录制，合计约 83 MB/s、90 次写入/秒，单线程录制器大部分时间在写图像；原始 CAN 订阅队列在两次服务之间溢出。
3. 原始 CAN 发布端为 RELIABLE + depth 1000，慢读者形成回压，直连 CAN 节点的 `bus.recv()` 循环被节流；向量节点作为另一订阅者收到 120 ms–2 s 前的旧帧，`now - stamp > 100 ms` 判 stale。
4. 内核 CAN socket 默认接收缓冲 `rmem_max=212992` 字节，按 1600 帧/秒只有约 160 ms；任何超过该时长的停顿都会变成永久丢帧。
5. 结果：原始 CAN 最后约 4 秒缺失，向量源帧引用只有 54/3040 能在原始 CAN 记录中匹配；完成帧与最后有效主目标相差 1.7 秒，16 步窗口不成立。

## 本次程序调整

### 1. 相机采集降到 15 fps

`src/piper/launch/start_cameras.launch.py` 新增 `camera_fps` 参数，默认 `15`：

```bash
ros2 launch piper start_cameras.launch.py camera_fps:=15
```

D455 在 640×480 下没有 10 fps 档位（支持 6/15/30/60/90），Orbbec Gemini 305 支持 15 fps。三路统一 15 fps 后图像写入从约 83 MB/s 降到约 42 MB/s；SFT 转换仍按 `--fps 10` 采样，15 fps 源提供足够的最近帧余量（最大约 66.7 ms，小于 100 ms 门限）。

如果只启动腕部相机，可以用 `camera:=left_wrist camera_fps:=10`；同时启动顶部 D455 时程序会拒绝 10 fps 并提示改用 15 fps。

### 2. 原始 CAN 订阅加深读取队列

`tools/episode_recorder/episode_recorder.py` 在录制启动前生成 `recording_qos.yaml`，对 `/piper_direct/*` 和 `/piper_vector/*` 使用：

```yaml
/topic:
  history: keep_last
  depth: 5000
  reliability: reliable
  durability: volatile
```

通过 `ros2 bag record --qos-profile-overrides-path` 传入。5000 帧约等于 3 秒缓冲，用于吸收录制启动和图像写入的短时抖动；不修改原始 Header、`stale` 或任何数值。配置项 `raw_can_qos_depth` 可调整，`0` 表示关闭覆盖。

### 3. 直连 CAN socket 接收缓冲

`src/piper/piper/piper_direct_can_node.py` 新增 `receive_buffer_bytes` 参数（默认 16 MiB），在打开 SocketCAN 后设置 `SO_RCVBUF` 并读取生效值。若内核上限较小，节点会打印警告。

宿主机需同步提高上限；这是采集前置条件：

```bash
sudo sysctl -w net.core.rmem_max=16777216
# 持久化：写入 /etc/sysctl.d/99-piper-can.conf 后执行 sudo sysctl --system
```

当前实测 `net.core.rmem_max=212992`，只够约 160 ms 的 CAN 流；不提高时，任何超过 160 ms 的停顿仍会丢帧。

### 4. 配置

- `config/episode_recorder.pi05.json`：新的采集配置，`expected_fps` 为 15，`raw_can_qos_depth=5000`，hold 策略为 `max_hold_sec=10`、`anchor_max_gap_sec=2`。
- `config/episode_recorder.example.json`、`config/episode_recorder.camera_only.json`：同步新增字段和 15 fps 预期。

## 重采前的录制负载验收

建议机器人静止、操作员不碰主臂，录制 30–60 秒，分别测试“只录图像”“只录 CAN/向量”“图像+CAN/向量全开”。通过标准：

- 原始 CAN 每 ID 接收率 ≥99%，无超过 1.5 个源周期的 Header 空洞；
- 原始 CAN Header→bag 接收 p95 < 100 ms；
- 向量 `stale` 比例接近 0，四帧 skew ≤30 ms；
- 图像 15 fps，无超过 100 ms 的 Header 缺口；
- 录制器与 `piper_direct_can` 的 CPU、接口 dropped 计数、`/proc/net/can/stats` 无异常增长。

三项都通过后，再按“主臂持续操作到香蕉释放、从臂响应完成”的约定采集，并跑现有 10 fps / 30 ms / 16 步审计。未通过前不重采训练数据。
