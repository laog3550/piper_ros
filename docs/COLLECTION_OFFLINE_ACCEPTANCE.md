# 无真实机械臂采集链路验收记录

## 范围与环境

- 验收基于 `/home/mips/piper_ros` 的 `humble` 分支、`62e31af`。持续负载测于 2026-10-20；功能复验完成于 2026-10-29。
- 不使用机械臂或 CAN。相机输入为已有被动 RGB-D rosbag 的隔离域循环重放；关节状态与动作话题为合成 `JointState`，只验证采集链路，不表示真实示教动作。
- 压力测试使用 `ROS_DOMAIN_ID=219`，原始相机录制 640×480、六路图像约 30 Hz，另有六路 CameraInfo、两路合成状态/动作及采集事件。Web 与 Qt 桌面预览同时运行。主机为 ThinkStation P920，40 逻辑 CPU，62 GiB 内存，NVMe。

## 10 秒烟测与 60 秒持续负载

| 指标 | 10 秒烟测 | 60 秒持续负载 |
| --- | ---: | ---: |
| 实录时长 | 10.445 s | 59.666 s |
| bag 数据量 | 1.484 GB | 8.065 GB（7.511 GiB） |
| 六路图像录制帧率 | 30.45–31.31 fps | 腕部 28.79–28.81 fps；顶部 30.20–30.23 fps |
| Web 后端新彩色/深度画面 | 均 2.39 fps | 2.28 / 2.30 fps |
| Web 预览年龄 | 中位 0.30 s，p95 0.40 s | 中位 0.20 s，p95 0.40 s，最大 0.80 s |
| Qt 三路彩色显示帧率中位 | 25.4–28.3 fps | 26.4–29.3 fps |
| Qt 三路接收年龄 p95 | 31–57 ms | 54–88 ms |
| stop / save 用时 | 1.51 / 1.05 s | 6.28 / 5.67 s |
| Web 错误 | 0 | 0 |

60 秒试验中合成状态与动作各发布 3580 条。六路 bag 图像各 1718–1804 帧。五路按 Header 统计无超过 1.5 周期的正向间隙；顶部对齐深度有 1 处，估计缺 2 帧。每路有 3 次 Header 时间戳回退，是源 bag 循环播放造成；因此上述帧率、间隙统计只适用于此重放负载，不能推断真实设备长时不丢帧或真实时钟单调。Web 后端有 0.4 s 节流，浏览器页面刷新周期约 1.5 s；后端 2.3 fps 不等于浏览器实际显示帧率，也不等于录制帧率。

60 秒负载中进程 CPU 中位按单核百分比：桌面与合成发布 243%，Web 164%，bag 录制 61%，回放 38%，录制服务 1%；RSS 中位约 253、221、92、480、61 MB。原始数据约 8 GB/min；按 stop 时剩余 842 GB 粗略估算，容量只能支撑约 104 分钟此类负载，实际还取决于其他文件、文件系统保留空间和场景内容。需据此设置采集时长与容量监控。

本次 60 秒试验 stop HTTP 成功，未复现先前单进程 Python 发布烟测出现的 409。单进程版本受 Python GIL 与 GUI 同进程负载影响，不能作为真实录制架构性能结论；一次未复现也不能证明 409 不会发生。长 bag 的停止、保存耗时明显增加，后续大于 60 秒的场景还需继续观察超时。

## 功能与数据契约

- `test_web_console_integration.py` 通过：合成 ROS 彩色/深度流、固定时长停止、停止后保存、手动保存目录、丢弃、旧 Episode 列表、VLM success/failure/unknown、规划校验、Web 重启读取与导出。
- `test_desktop_console_integration.py` 通过：三路预览约 29–30 fps、关闭窗口后继续录制、固定时长停止、跨磁盘自选目录保存、重复 Episode 目录拒绝、丢弃、录制器重启后读取。
- `test_preview_depth.py` 通过；`test_to_lerobot.py` 14 项通过。这 14 项中的转换测试使用 `FakeDataset` / stub，主要验证输入契约、字段映射、时间对齐与切分。默认 Python 环境未安装 `lerobot`；本轮没有在该环境创建并重新读取真实 `LeRobotDataset`，不能据此宣称实体数据集端到端转换通过。
- 已有被动相机 Episode 含 RGB-D，但缺少显式动作来源、实测关节反馈和示教动作。`audit_real_data.py` 标为 `eligible=false`，`to_lerobot.py` 以缺少 arm_mode/time_domain/action_space/action_provenance 明确拒绝，未把被动相机帧误当示教动作。

## 现场相机与复现边界

2026-10-20 两台 Gemini 305 分别单独启动时，驱动都报告帧数据长度与期望 614400 字节不符，回调约 1 fps，约 20 秒内未获得有效 ROS 深度帧。相机进程随后正常停止。此次现场异常无法靠深度滤镜调参判定效果，也不能用重放通过替代真实相机连接与画质验收；需要在物理 USB/设备链路恢复后做固定场景 A/B 对照。未启动机械臂或 CAN。验收结束后，相机驱动、bag 回放、录制器、Web 和测试进程均已退出。测试结束时主仓库工作区干净；本轮未发现需要立即修复且有独立复现证据的采集程序缺陷。验收过程未修改采集代码，也未推送。

## 证据与复现

- 完整数值：`/tmp/piper_collection_replay_smoke/report.json`、`/tmp/piper_collection_replay_60s_20261020/report.json`；测试数据分别保留在同名目录，总计约 1.4 GB 与 7.6 GB。源被动相机数据保留于 `/tmp/piper_passive_episodes_20261015`，约 2.2 GB。
- 功能日志：`/tmp/piper_accept_web_20261020.log`、`/tmp/piper_accept_desktop_20261029.log`、`/tmp/piper_accept_lerobot_20261029.log`、`/tmp/piper_accept_passive_audit_20261029.json`、`/tmp/piper_accept_passive_reject_20261029.log`。
- 压力复现脚本仅保留在 `/tmp/piper_collection_replay_stress.py`，未纳入仓库。复现压力测试：`source /opt/ros/humble/setup.bash && source /home/mips/piper_ros/install/setup.bash && cd /home/mips/piper_ros && PIPER_STRESS_ROOT=/tmp/piper_collection_replay_repeat python3 /tmp/piper_collection_replay_stress.py 60`。它读取上述源 bag，在隔离 ROS 域重放并生成 `report.json`；需为每轮使用新的输出目录并预留至少约 9 GB 空间。
