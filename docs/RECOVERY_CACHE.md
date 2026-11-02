# 可选恢复回溯缓存

`tools/episode_recorder/recovery_cache.py` 是独立的只读 ROS 订阅进程。它默认关闭，不会发送机械臂、夹爪或 CAN 控制指令。启用前复制 `config/recovery_cache.example.json` 到工作目录，将 `enabled` 改为 `true`，并核对话题类型、输出目录、磁盘空间和场景的实际数据率。应在开始 Episode 前启动，以便收到 `/collection/events` 的 `started` 事件并填满前录窗口。

```bash
source /opt/ros/humble/setup.bash
source /home/mips/piper_ros/install/setup.bash
python3 tools/episode_recorder/recovery_cache.py --config /path/to/recovery_cache.json
```

关闭时使用 `Ctrl+C`。配置仍为 `enabled=false` 时，进程直接退出，不订阅话题，也不创建输出。示例配置缓存三路 RGB-D 原始图像及双臂 `PiperArmVector` 目标/反馈，前录 3 秒、后录 2 秒、环形缓存上限 512 MiB。`pre_roll_sec` 按单调接收时间淘汰旧消息，`max_buffer_bytes` 按 CDR 序列化大小加固定管理开销淘汰最旧消息；两项同时生效。写线程的后录待写队列另以同一字节上限约束。触发后，前录快照、继续运行的环形缓存和后录待写队列可同时持有序列化数据，合计理论上可接近三倍配置上限；ROS 中间件和 SQLite 的占用还未计入。部署前应按实际相机帧大小测量峰值。

录制器在 `recovery_trigger` 标注中提供 `episode_id`、当前 `skill` 和唯一 `recovery_id`。缓存只接受属于当前活动 Episode、具备固定技能的触发；同一时刻只写一个片段。每个触发对应 `recovery_<recovery_id>/manifest.json`，成功时另有标准 rosbag2 SQLite `bag/`。重复或并发触发不会覆盖既有片段；并发触发写入 `failed` 清单。进程意外退出后，下次启动会把仍标记 `recording` 的清单改为 `failed`。

bag 写入使用消息到达本进程时的 Unix 纳秒时间作为 rosbag2 接收时间；消息中的真实 `Header.stamp` 保留在原始 CDR 内，并在清单中汇总。清单包含源 Episode、技能、恢复 ID、触发时间、配置窗口、各话题前后消息数与实际覆盖时长、Header 间隙估算、时间戳倒退、缓存淘汰数、超大消息丢弃数、后录队列溢出数和写入前后磁盘余量。超过 90% 配置窗口且每路话题前后均有消息、无明确写入丢失、无推断缺帧或 Header 倒退时才标为 `complete`；其他情况为 `failed`。`complete` 仅表示片段落盘和基本连续性通过，不代表动作真实性、跨设备同步、标定或训练资格，清单始终设 `training_approved=false`。Header 缺帧数是按配置帧率推断，无法发现没有 Header 或未被订阅端接收到的全部底层丢帧。

合成验收命令：

```bash
ROS_DOMAIN_ID=225 python3 -m pytest -q tools/episode_recorder/test_recovery_cache.py
```

该测试构造合成 ROS 图像与状态流，验证时间/字节双上限、恢复 ID 关联、并发触发失败清单、完整片段的标准 rosbag2 回读与真实 Header 保留。它不替代三机双臂现场吞吐、峰值内存、长时稳定性和人工恢复质量验收；未经现场审核的片段不得回流训练。

2026-11-02 另用同一 Python 进程发布六路 `640×480` 合成 RGB-D，配置 `30 fps`、前后各 `0.8 s`、上限 `128 MiB`。最终复验输出 bag 序列化负载约 `169.0 MB`，触发时环形缓存约 `110.6 MB`，后录待写队列峰值约 `46.1 MB`，前后实际覆盖各约 `0.797 s`。六路各写入 `36–37` 帧，按 Header 估计每路缺 `6–7` 帧，清单正确标为 `failed`。此负载由单 Python 进程同时发布与订阅，不能据此声称已稳定达到 `30 fps`，也不能单凭本次试验定位丢帧发生在发布、DDS 或订阅端。证据临时留在 `/tmp/piper_recovery_highrate_final_20261102` 与 `/tmp/piper_recovery_highrate_final_20261102.log`。
