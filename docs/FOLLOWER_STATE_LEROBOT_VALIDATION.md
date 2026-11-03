# 从臂关节状态到 LeRobot v3 的只读验收

## 结论与证据

2026-11-03 的复验 Episode 位于 `/tmp/piper_full_live_split_probe_20261103/episode_000001`。左右从臂的 `/piper_vector/{left,right}/slave_feedback` 在 40 秒录制中持续发布；两侧六个关节角使用 `rad`，来源是各侧 `0x2A5`、`0x2A6`、`0x2A7` 的三组原始时间戳。夹爪映射尚未核验，因此不把夹爪维度纳入本次导出。

原先的双侧旁路共用两个 Python 进程，动作时从臂来源 Header 到 ROS 接收延迟约 200～355 ms，100 ms 新鲜度检查会拒绝多数运动状态。现用 `start_readonly_can_collection.launch.py` 为左右两侧各启动独立的 CAN 监听和向量组装进程，四个进程均只读。在用户按原有固件直连流程分别小幅遥操作时，30 秒在线探针得到左右从臂各约 1500 条向量，关节来源全部新鲜；最大 Header 年龄分别约 9.6/9.4 ms。左右关节 1 的变化范围分别约 0.253/0.279 rad。

使用 `export_follower_state_lerobot.py` 从复验 Episode 导出真正的 `LeRobotDataset v3.0`，并重新打开核对：10 fps、**1 条连续 Episode、402 帧、12 维 `observation.state`**。左右主臂动作窗口在该数据集中分别覆盖 24 帧、12 帧；对应从臂关节 1 的变化约 0.252/0.275 rad，所有 36 帧都能与 bag 内原始 CAN 关节字节核对。导出目录是 `/tmp/piper_follower_state_split_lerobot_20261103`，`conversion_manifest.json` 记录来源 SHA-256、阈值、拒绝原因及帧数，`source_timestamps.jsonl` 记录每帧两臂来源 Header 与原始 CAN 核对标记。

独立逐级核对中，402 帧 LeRobot 状态均可回读；其中 **394 帧**的两臂六关节数值还能与保存的三组原始 CAN 字节逐值对应，最大浮点误差约 `1.42e-8 rad`。另 8 帧的从臂向量已记录，但 bag 内缺少至少一条对应原始 CAN 关节帧；这些帧保留在只读状态验收产物中，并在逐帧清单标为 `raw_can_frames_verified=false`。这不影响“从臂状态可记录并转换”的结论，但说明原始帧来源覆盖率尚非 100%。

此数据集**不含 `action`、不含夹爪、`training_eligible=false`**，不能直接用于 π0.5 训练。训练转换器 `to_lerobot.py` 仍须在示教动作、夹爪映射、标定、技能标注与时间门禁通过后单独执行。

## 复现命令

在装有 `rosbags`、`numpy` 和 `lerobot==0.4.4` 的 Python 环境中运行：

```bash
ros2 launch piper start_readonly_can_collection.launch.py
# 在另一个终端对已保存的 Episode 执行：
python3 tools/episode_recorder/export_follower_state_lerobot.py \
  /tmp/piper_full_live_split_probe_20261103/episode_000001 \
  /tmp/piper_follower_state_split_lerobot_20261103 \
  --repo-id local/piper_follower_state_split_validation --fps 10
```

CAN 接口须先按现场流程配置并启用；`ros2 launch` 仅监听已启用的 CAN，不发送控制帧。输出目录必须不存在；再次导出时改用新的输出路径。导出命令只读取已保存的 rosbag2，不连接或控制机械臂。
