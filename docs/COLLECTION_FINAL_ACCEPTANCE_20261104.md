# 数据采集程序最终验收记录（2026-11-04）

> 2026-11-05 更正：用户确认的项目夹爪行程上限为 `100 mm`；此前文档中的 `80 mm` 为误记。以下保留 11 月 4 日采集与验收结论，行程数值已按更正修订。

## 验收结论

**双臂 CAN、六路 RGB-D、定时停止、指定路径保存和只读 LeRobot v3 状态导出已在真机联录中通过；训练数据质量验收仍未通过。** 用户找到并接回脱落的 `can_left` CAN 线后，两侧各以约 3040 帧/秒收帧，5 秒只读体检均无 CAN 错误帧。原先左侧断流的直接原因是电脑侧监听线脱落；在未接回时，开始门禁正确拒绝了缺少左从臂反馈的请求。

恢复后的 60 秒真机 Episode `/tmp/piper_final_recovered_saved_20261104/episode_000001` 按时自动停止，使用界面指定目录保存：左、右主臂目标原始帧分别 4555、4879 条，从臂原始反馈分别 46178、47708 条；六路 RGB-D 各 1810～1815 帧，图像 Header 无倒退且估算缺帧为 0。从臂状态导出为 603 帧、14 维 LeRobot v3，左右关节和夹爪均有变化，但只有 524/603 帧有完整原始 CAN 字节旁证。

针对录制期间的原始帧缺口，将 rosbag2 缓存从 0 改为 256 MiB。相同传感器负载的约 25 秒对照中，左右 `0x2A5`～`0x2A8` 四个 ID 均约 200 帧/秒，没有超过 10 ms 的间隔。缓存版真机 Episode `/tmp/piper_final_recovered_saved_20261104/episode_000002` 按 45 秒时限自动停止并保存，左右主臂目标各 3488 条，从臂反馈分别 36265、36780 条，导出 445 帧 LeRobot v3，**445/445 帧**具有完整原始 CAN 旁证。但这条 Episode 录制期间系统时钟发生调整，顶部相机 Header 有 21 次非单调；它不能通过时间同步质量验收。录制器现记录系统时钟跳变，界面提示异常，离线来源审计拒绝 Header 非单调。两条 Episode 的 `training_eligible` 均为 `false`。

2026-11-03 的真机 40 秒 Episode `/tmp/piper_full_live_split_probe_20261103/episode_000001` 已完成自动停止、保存和六路 RGB-D、双臂 CAN 同录。保存的左右从臂反馈已导出并重新打开为 `LeRobotDataset v3.0`：1 条 Episode、402 帧、10 fps、14 维 `observation.state`，每侧六个关节加夹爪；393 帧有完整原始 CAN 字节旁证。导出目录为 `/tmp/piper_follower_state_with_gripper_lerobot_20261103`。夹爪在这次动作中没有开合，零点和方向未经实物核验；该产物没有 `action`，清单明确标记 `training_eligible=false`。详情见[从臂状态导出验收](FOLLOWER_STATE_LEROBOT_VALIDATION.md)。

## 本次软件检查

| 检查项 | 结果 |
| --- | --- |
| ROS/CAN/录制器测试 | `29 passed`，日志 `/tmp/piper_final_system_tests_20261104.log` |
| 转换、质量门禁与 VLA 测试 | `60 passed, 5 subtests passed`，日志 `/tmp/piper_final_conversion_tests_20261104.log` |
| `colcon build --packages-select piper piper_msgs --symlink-install` | 2 包构建通过；仅有既存的 setuptools `tests_require` 提示 |
| 三技能 Web 合成联调 | 开始、标注、停止、保存、丢弃、重启读取通过；手动保存目录和 60 秒时限写入元数据，日志 `/tmp/piper_final_skill_ui_after_fix_20261104.log` |
| VLM Web 合成联调 | 规划、结果、重启读取与候选导出通过，日志 `/tmp/piper_final_web_ui_20261104.log` |
| 本地桌面合成联调 | 三路预览约 29.3 fps；关窗后继续录制、保存、丢弃与跨目录重启读取通过，日志 `/tmp/piper_final_desktop_ui_20261104.log` |
| 历史真机 LeRobot 回读 | 1 条 Episode、402 帧、14 维状态；抽查首、中、末帧，左右夹爪数值可读 |
| 恢复后真机联录 | 60 秒与 45 秒 Episode 均自动停止并保存至手动指定目录；缓存版 445/445 帧有原始 CAN 旁证，顶部相机时间戳异常被识别 |
| 缓存和时间门禁回归 | 录制器单元测试 5 项、原子技能审计 12 项与三技能 Web、VLM Web、桌面界面合成联调通过 |

验收中补齐了三技能 Web 采集界面的每条 Episode 固定时长与保存目录输入，并在服务端限制时长为 1～86400 秒、保存目录为绝对路径。到时自动停止，操作员仍需选择结果并保存。桌面界面此前已有这两个输入。本次改动位于 `tools/episode_recorder/skill_web_console.py`、`skill_web_console.html` 和对应集成测试。

## 七模块状态

| 模块 | 验收状态与未完成项 |
| --- | --- |
| 1 硬件接入与时钟同步 | 脱落的左侧监听线已接回，双臂 CAN 与六路 RGB-D 真机同录通过。系统时间服务多次调整时钟，缓存版 Episode 顶部图像 Header 非单调；时钟稳定性未通过。 |
| 2 技能状态机录制 | 固定三技能、原子 Episode、扰动编号和起始关节快照的合成联调通过；现场技能执行与场景扰动未验收。 |
| 3 即时轻量标注 | 技能边界、完成帧、结果和恢复触发的合成联调通过；真实任务的标注一致性尚无人工复核。 |
| 4 恢复触发 | 环形缓存与清单有合成测试；真实高负载恢复片段和训练回流未验收。 |
| 5 质量审计 | 保存时检查话题与 SQLite；新增系统时钟跳变记录与界面提示，离线审计拒绝 Header 非单调。两条真机 Episode 的训练合格帧仍为 0。 |
| 6 LeRobot v3 与 π0.5 | 真机从臂七维每侧状态导出及回读通过，缓存版 445/445 帧有原始 CAN 旁证；夹爪物理映射、标定、单技能标注、训练门禁及固定 openpi 读取器兼容均未验收，不能启动真实 SFT。 |
| 7 运行时轨迹与 Qwen | 轨迹、图像来源、人工审核和候选导出的合成检查通过；真实部署轨迹与 Qwen LoRA 训练集未验收。 |

## 现场补验入口

保持 `can_left`、`can_right` 为 1 Mbit/s `LISTEN-ONLY`。只读旁路与相机可按现有启动流程运行；不需要 ROS 控制机械臂。开始前确认双臂从臂反馈有效、三路彩色预览在线，并确认系统时间服务稳定。接着录制一条短 Episode，核对自动停止、六路 RGB-D 与双臂 CAN 话题计数、Header 单调性、关节和夹爪字段，再执行只读 LeRobot 导出与来源审计。当前 NTP 状态曾显示大幅抖动及频率校正达到上限，现场出现多次 `Clock change detected`；需要先解决时间源稳定性，再验收用于训练的同步数据。

界面开始门禁在左侧缺帧时返回 `400` 和“left 臂没有可用的实时关节反馈”，这是预期的质量保护。仅把 `mapping_verified` 或 `gripper_mapping_verified` 改为 `true` 不构成实物验收。

## `can_left` 断流定位与恢复

用户确认左臂固件直连遥操作正常。电脑侧 `can_left` 对应 USB 端口 `1-13:1.0`、序列号 `004B00395443570F20393433`，与配置和 udev 命名规则一致；设备保持 USB `active`，没有自动休眠。它与右侧同为 `gs_usb`、1 Mbit/s、`LISTEN-ONLY`、`ERROR-ACTIVE`，接收错误和丢帧计数均为零。3 秒被动计数中，左侧 RX 增量为 0，右侧增加 9136 帧；原始 `candump` 只见右侧帧。两侧无采集进程占用；重设 `can_left` 的 SocketCAN 接口后，左侧仍无帧。内核日志没有同日左侧 USB 断连记录。

这些证据当时把故障定位在左侧 USB-CAN 的 CAN 端监听支路或适配器收发器。主从臂遥操作正常只能证明它们之间的控制总线在工作，不能证明电脑侧监听适配器实际收到信号。用户随后发现监听线的 CAN_L 已脱落并重新接好，左侧立即恢复约 3040 帧/秒；这给出了实际故障原因。

此前经用户授权执行 `sudo usbreset 001/045`，内核重新枚举左适配器并保留 `can_left` 命名。重新配置为 1 Mbit/s `LISTEN-ONLY` 后，左侧 RX 仍为 0。再在只听模式下分别以 1 Mbit/s、500 kbit/s、250 kbit/s 被动计数 2 秒，每档新增帧数均为 0；退出时已恢复 1 Mbit/s `LISTEN-ONLY`。右侧在同期间继续收帧。这些软件排查未恢复接收，和后来发现的 CAN_L 脱落一致。

与前一日对照：2026-11-03 14:46，`can_left` 以相同的 1 Mbit/s `LISTEN-ONLY` 配置启用；14:57 和 14:59 的原始抓帧日志明确记录左接口 `0x2A1`～`0x2A8`。15:13～15:14 的真机 Episode 记录 `/piper_direct/left/slave_feedback` 32331 条，同时右侧为 32345 条。两天使用同一左适配器序列号、USB 端口 `1-13:1.0`、`gs_usb` 驱动和未改动的 udev 命名规则。成功录制结束至次日诊断前，内核日志未记录左适配器 USB 断连或重枚举；这只能排除有日志的 USB 事件，不能排除 CAN 端信号或收发器故障。用户再次给机械臂上电后，左接口连续 5 秒仍为 0 帧，右接口约 3040 帧/秒。差异发生在原始 SocketCAN 收帧之前，和 ROS 旁路、相机或 Episode 转换代码无关。

继续使用项目内 `piper_can_quality` 的显式错误帧过滤器被动监听 5 秒：左侧普通帧和错误帧均为 0，右侧普通帧 15200 条（3040/s）、错误帧 0。排查时发现该工具原先会把 0 帧、0 错误误判为 `CLEAN`，并把没有低速反馈样本误写成“全部 disabled”；现已改为 `NO_SIGNAL`、使能状态未知，退出码为 3。对应单元测试 10 项通过，现场左接口复测正确报告 `NO_SIGNAL`。该修复仅改善诊断，不会让没有原始 CAN 帧的适配器恢复收帧。

## 缓存与时钟质量

原录制器使用 `ros2 bag record --max-cache-size 0`，每条消息都直接写盘。60 秒首次联录的 603 帧 LeRobot 状态中，只有 524 帧能与 bag 内四组原始 CAN 字节完整对应。设置 256 MiB 缓存后，约 25 秒全话题对照中左右 `0x2A5`～`0x2A8` 每个 ID 均约 200 帧/秒，最大来源间隔约 5.2 ms；缓存版 45 秒真机 Episode 导出 445 帧，445 帧全部有原始 CAN 旁证。新增 `rosbag_cache_bytes` 配置和 Episode 元数据，默认值 `268435456`。

约 65 秒的缓存对照中，CAN 和三路相机在同一时刻出现约 0.4 秒时间戳间隔，系统日志同时记录 `Clock change detected`。缓存版真机 Episode 的 13:13:57 又有一次时钟调整：左腕 RGB Header 一次约 700 ms 倒退，顶部 RGB 与对齐深度各 21 次约 701 ms 倒退；顶部对齐深度还在开始时出现一个大间隔。此时 `systemd-timesyncd` 报告较大的 NTP 抖动，频率校正曾达到 `+500 ppm`。不能把这些同步间隔全部解释为录包丢帧，也不能把有倒退的图像 Header 用于训练对齐。

录制器现在每约 0.2 秒对比 `CLOCK_REALTIME` 与 `CLOCK_MONOTONIC` 的偏移，偏移单步变化达到 50 ms 时记录 `clock_adjustments`；采集界面显示时间同步异常，离线审计拒绝系统时钟跳变或任一路图像 Header 非单调。已保存的缓存版 Episode 虽没有新录制器的 `clock_adjustments` 字段，离线审计仍根据其 Header 统计明确拒绝。时间源稳定前，真实训练数据验收保持未通过。

## 训练数据复验（2026-11-04 晚）

对恢复后的两条真机 Episode 重新执行只读来源审计，并显式设定 RGB、从臂状态与主臂目标的总体 Header 跨度上限为 `30 ms`。复验命令为 `/tmp/piper-collection-audit-venv/bin/python tools/episode_recorder/audit_real_data.py /tmp/piper_final_recovered_saved_20261104 --episode-root /tmp/piper_final_recovered_saved_20261104 --max-source-skew-ms 30 --output /tmp/piper_training_acceptance_audit_20261104.json`。机器可读结果：来源 2 条、合格 Episode 0 条、合格训练帧 0 帧，场景组切分尚不可用。两条均为 `task` 模式，未分割成独立 `pick_banana`、`place_banana_in_box` 技能片段；`task_metadata` 缺少 `arm_mode`、`time_domain`、`action_space`、`action_provenance`、`calibration_version` 和 `normalization_version`。第二条另有左腕 RGB、顶部 RGB 与顶部对齐深度的 Header 倒退。这里的 `outcome=success` 不能代替技能完成帧与真实任务成功复核。

| 训练用途 | 当前结果 | 依据与下一步 |
| --- | --- | --- |
| π0.5 原子技能 SFT | **不通过** | 现有 LeRobot v3 导出仅含 14 维从臂 `observation.state`，无已核准的 `action`；场景组、技能边界、成功标注、标定和夹爪实物映射尚无证据。须重新录制原子技能 Episode，完成标注和来源审计，再按场景组切分。 |
| 固定 OpenPI 读取器 | **不通过** | 固定 commit `215abfb217dbac7d5f1273282331b9b1866c0479` 的本地环境读取缓存版 445 帧数据集时，`load_info` 识别 `codebase_version=v3.0`，但 `load_episodes` 要求 `meta/episodes.jsonl` 并抛出 `FileNotFoundError`；该文件不是当前 v3 导出的组成部分。训练启动门禁维持关闭，须完成固定读取器与 v3 的兼容改造及端到端回读。 |
| Qwen 规划器 LoRA | **不通过** | 这两条是采集 Episode，没有真实运行时规划、执行和验证轨迹，也没有经人工审核的规划标签。现有候选导出只通过合成联调。 |

夹爪已作为左右各第七维保存，且主目标与从反馈在动作中同步变化；这证明记录链路包含夹爪，尚不能证明当前设备的闭合零点。此前对首条 Episode 做 50 ms 内配对，左右主目标与从反馈相关系数分别约 `0.959` 和 `0.969`；从反馈仍大量为负值（左 `461/504`、右 `503/554` 个配对样本）。按用户确认采用 `100 mm` 行程上限；在闭合与张开端点、零位和正方向完成只读核对前，不得把 `gripper_mapping_verified` 改为 `true`，也不能把这些值直接用于训练。

复验时 `timedatectl timesync-status` 显示 NTP 偏移约 `+21.755 ms`、抖动约 `14.778 ms`，19:00 后日志没有新的 `Clock change detected`。这是复验时刻的时间服务状态，不能倒推第二条 Episode 的时间戳合格，也不能替代下一条完整技能 Episode 的同步审计。训练入口还要求每种技能至少 10 条训练、3 条测试 Episode，且训练与测试的 `perturbation_id` 场景组不重叠；目前上述四种计数均为 0。

## 夹爪协议依据与设备差异

根据用户确认，本项目左右夹爪均按 `0～100 mm` 行程处理，无须用直尺复测协议单位。已核对本机 `piper_sdk 0.6.2`：V2 `0x2A8` 反馈前四字节为有符号 `int32`、单位 `0.001 mm`，与本仓库按 `1e-6 m` 解码一致。11 月 4 日源码中的遥操作上限为 `0.08 m`；按 11 月 5 日更正，源码上限调整为 `0.10 m`。本机 SDK 的软件默认上限为 `0.07 m`，本项目不以该默认值覆盖用户指定的 `0.10 m`；如以后改用 SDK 限位，需显式配置为本项目上限。官方来源：[双臂说明](https://github.com/agilexrobotics/piper_sdk/blob/master/asserts/double_piper.MD)、[SDK 协议更新记录](https://github.com/agilexrobotics/piper_sdk/blob/master/CHANGELOG.MD)、[反馈消息定义](https://github.com/agilexrobotics/piper_sdk/blob/master/piper_sdk/piper_msgs/msg_v2/feedback/arm_feedback_gripper.py)。

现有录制的负值保留了官方有符号字段，不能直接解释为反向运动；主从反馈相关也不足以确定闭合零点。后续在正常直连遥操作中只读记录左右夹爪的闭合、张开端点与画面对应关系，以核对 `100 mm` 配置下的零点和正方向。此处 `100 mm` 是项目配置上限，不是已观察到的最大反馈值。在此之前不修改历史原始值，也不启用 `gripper_mapping_verified`。
