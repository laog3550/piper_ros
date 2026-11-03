# 数据采集稳定性与训练数据流核对（2026-11-03）

## 当前结论

采集软件的合成 ROS 闭环、保存/丢弃、自动停止、重启读取、基础审计与离线转换测试通过；这只证明软件流程在合成输入下工作，不等于现场相机、双臂 CAN、主从遥操作或训练数据质量已验收。初检时本机 `can_left`、`can_right` 均为 `DOWN`；默认 ROS 图只见 `/parameter_events` 和 `/rosout`，没有真实相机或机械臂发布者。后续两路 CAN 已恢复接收，见下文；跨设备时间偏差、长时间同录稳定性与夹爪物理映射仍未测量。

同日 CAN 复验：两路接口均为 `UP`、`ERROR-ACTIVE`、1 Mbit/s、`LISTEN-ONLY`，左右适配器身份与映射一致。2 秒被动计数中左侧新增 6087 帧、右侧新增 6084 帧；两侧发送计数与接收错误增量均为 0。抽样可见两侧 `0x2A5`～`0x2A8` 从臂反馈。临时启动只读 `piper_direct_can` 与 `piper_arm_vector` 后，4 秒内四路向量话题各发布约 200 条；左右从臂反馈均具有完整四源 Header，`missing/stale/time_inconsistent` 均为 `false`。主臂目标在静止窗口没有控制帧，四源 `missing=true`；夹爪映射尚未核验，故向量 `valid=false`，符合门禁预期。旁路节点随后正常退出。接口已启动时重复执行 `ip link set ... type can bitrate ...` 会得到 `Device or resource busy`，这不表示接收故障。没有在动作窗口核验主臂控制帧或与相机同录，因此这些项目仍待验收。重启后接口状态须重新检查。

随后在用户按原有固件直连方式分别小幅遥操作左右主臂的窗口中，仅被动启动 ROS 旁路和三机相机。60 秒在线订阅中，左右从臂反馈约 800 原始帧/秒，三路彩色图像约 29.5～29.8 fps；左主臂四种目标 ID 均出现。一次 35 秒的**相机、CAN 同录 Episode**按时限自动停止、SQLite 检查并保存至 `/tmp/piper_full_live_probe_20261103/episode_000001`，约 4.5 GB。六路 RGB-D 各 1032～1034 帧，Header 无倒退、无超过 1.5 倍正常周期的缺口；左右主臂目标帧分别为 979、1072 条，左右从臂原始反馈分别为 27317、26504 条。左右主臂目标至对应从臂向量最近 Header 差中位数为 4.165/3.124 ms，至对应腕部 RGB 为 8.925/8.729 ms；左右腕 RGB-Depth 最近 Header 差最大为 0.061/0.137 ms，顶部对齐深度与 RGB 同戳。来源数据见 `/tmp/piper_full_live_analysis_20261103.json`。

**质量阻断仍在**：左右约 3 秒动作窗口内，50 Hz 目标向量中四源齐全、新鲜且时间一致的分别只有 25/160、11/140 条；原始四种主臂目标 ID 数量不均。临时原始 CAN 与 ROS 旁路对照窗口没有捕获新的主臂动作，尚不能判定缺口发生于总线、SocketCAN 接收还是 ROS 发布/记录。`piper_direct_can` 与 `piper_arm_vector` 在现场分别长期占用约 94% 和 83% 的一个 CPU 核，需进一步压测与优化。夹爪映射未核验，所有向量 `valid=false`。该验收 Episode 的离线来源审计为合格 Episode 0、合格帧 0，原因包括缺少动作契约、标定、归一化版本及原子技能标注，见 `/tmp/piper_full_live_audit_20261103.json`；不能把同录成功解释成可训练。

针对从臂状态进一步定位到动作负载下只读 ROS 旁路积压：原双侧进程中从臂来源 Header 到 ROS 接收延迟约 200～355 ms，运动区间无法稳定通过 100 ms 新鲜度门禁。左右各自运行独立 CAN 监听与向量组装进程后，真机动作时在线最大 Header 年龄分别约 9.6/9.4 ms。40 秒新 Episode 自动保存，导出并回读为 10 fps、1 条连续 LeRobot v3 Episode、402 帧、左右各六关节加夹爪的 14 维状态；左右动作窗口分别覆盖 24、12 帧，均能与保存的原始 CAN 字节逐值核对。全段 393/402 帧具有完整原始 CAN 旁证。该产物仅有 `observation.state`，不含 `action`；夹爪零点及方向未实物核验，也不授予训练资格。详见[从臂状态导出验收](FOLLOWER_STATE_LEROBOT_VALIDATION.md)。

本次只读运行 `audit_real_data.py /home/mips/piper_tjp/data`：8 段来源、合格 Episode 0、合格训练帧 0、`split_ready=false`。旧包缺完整 Episode 契约、示教动作/反馈的核验或原子技能标签；不能把已有 RGB-D 包直接作为 π0.5 或 Qwen 的训练集。审计结果在 `/tmp/piper_collection_real_audit_20261103.json`。

随后仅启动三台相机的 ROS 驱动，不启动机械臂控制或修改 CAN。12 秒六路被动订阅得到 28.5–29.8 fps，所有收到的 Header 单调；右腕深度在六路 Python 探针中出现一次最长约 200 ms 间隔，单路 15 秒复测为 29.85 fps、最长约 66.6 ms，因此不能把前者直接归因于相机驱动。另用**真正的 Episode 录制器**完成一次 12 秒相机专用录制、自动停止、SQLite 核验和保存：六路各 341–343 帧，左右腕 RGB/Depth 最近 Header 差最大分别为 0.154/0.132 ms，顶部 RGB/对齐深度有一帧缺口，最近差最大约 33.373 ms；顶部与左右腕 RGB 最近差最大约 20.213/20.754 ms。这是相机专用短测，不包含 CAN 或主从向量，不能视为模块 1 全链路同步合格。证据为 `/tmp/piper_framework_camera_bag_probe_20261103.json`、`/tmp/piper_camera_pairing_probe_20261103.json` 和 `/tmp/piper_framework_camera_probe_20261103/episode_000001`。相机节点已正常停止。

将这条相机专用 Episode 交给只读来源审计时，得到 1 段来源、合格 Episode 0、合格帧 0；明确列出缺少执行臂、时间域、动作空间/来源、标定/归一化版本、关节反馈与示教目标，且 `task` 尚未拆成原子技能。该结果证明录制成功没有绕过 π0.5 数据门禁，见 `/tmp/piper_camera_only_training_audit_20261103.json`。

## 七模块对照

| 模块 | 软件状态 | 现场或训练状态 |
| --- | --- | --- |
| 1 硬件与时钟 | 左右 USB-CAN 序列号与 `config/pi05_can_map.json` 一致；两台 Orbbec 和顶部 D455 均被 USB 枚举。主从臂按每侧同一 CAN 总线固件直连，`piper_direct_can` 与 `piper_arm_vector` 仅作只读 ROS 旁路。 | 两路 CAN 后续恢复为 1 Mbit/s 只听模式，均有从臂反馈，短窗口无接收错误；从臂 ROS 向量四源完整。尚未核验主臂动作帧或与相机同录的同步和连续性。 |
| 2 技能状态机 | `task` 固定抓取、入盒、双臂翻盖顺序；另有三种 `skill` 原子 Episode。开始时保存从臂六关节起点及原始时间；技能片段要求 `perturbation_id`。三路 RGB 预览与所需从臂反馈未就绪时，开始请求会被拒绝。 | 扰动编号是人工场景分组，不证明物体、盒子和翻盖真实位姿；需现场记录对应场景。 |
| 3 轻量标注 | 录制中写技能开始/完成、结果、恢复触发，关联相机 Header；无法引用时明确待离线核对。预览限频时，完成帧引用现在与实际编码的预览帧绑定。 | 浏览器刷新可能落后于服务器画面，完成边界仍须在 bag 中人工复核。 |
| 4 恢复触发 | 独立 `recovery_cache.py` 默认关闭，有时间和字节双上限，写标准 rosbag2 与 `complete/failed` 清单。 | 合成高负载出现缺帧；真实相机负载未验收。恢复包没有自动回流训练。 |
| 5 质量审计 | 录制停机检查 SQLite 与话题覆盖，离线审计再检查 Header、动作/反馈有效性、相机配对、标注和场景组。 | 停机时可保存不等于训练合格；阈值与真实跨设备偏差仍待同录测量。 |
| 6 LeRobot v3.0 与 π0.5 | 合成 Episode 可经 `to_lerobot.py` 导出并回读；成功原子技能按扰动组切分，π0.5 有独立训练门禁。 | 真实合格帧为 0；固定 openpi 读取器的 v3 兼容、实测夹爪映射与标定尚未验收，当前不得启动真实 SFT。 |
| 7 运行轨迹与 Qwen | `runtime_trace.py` 记录规划、预检、执行、验证、失败和恢复关联，默认 `training_eligible=false`。`qwen_lora_candidates.py` 只读核对真实轨迹、Episode、图像 bag 和人工审核记录，可导出 `candidate_only` 候选；外部恢复 ID 还须绑定完整恢复清单。 | 尚无真实完整轨迹与审核记录；候选导出不是 Qwen LoRA 训练器，也不授予训练资格。合成轨迹不等于真机训练样本。 |

只读复核时发现两个数据契约问题：旧门禁对来源审计版本和技能标签的解释与 v2 Episode 不完全一致；三技能 Web 预览限频曾让标注 Header 超前于被编码的 JPEG。两处均已修复。针对性离线测试验证了 v2 审计、逐段任务文本和跨源时间约束；真实样本、现场同步阈值及固定 openpi 读取器的 LeRobot v3 兼容性仍未验收，因此模块 6 的真机训练闭环尚未通过。

## 数据从采集到训练的路径

实际遥操作是每侧主臂与从臂共用一条 CAN 总线、由固件直连联动；采集程序不介入控制。`piper_direct_can` 只读监听 `can_left`/`can_right` 并发布逐帧 ROS 观测，`piper_arm_vector` 再将四帧合成为七维主目标与从反馈。`episode_recorder.py` 录制这些旁路 ROS 话题和相机话题。物理 CAN 接入与 ROS 旁路出流是不同的验收项：只有 USB 设备在位、但 CAN 接口或旁路节点未运行时，不能采集可转换的示教 Episode。

| 入口与产物 | π0.5（Pi）用途 | Qwen 用途 |
| --- | --- | --- |
| `skill_web_console.py`：三技能 Episode，录制配置中的主臂目标、从臂反馈、三路 RGB-D、任务文字、技能边界、结果与恢复事件；`episode.json` 加 rosbag2 | 单臂抓取/入盒成功示教的候选。`to_lerobot.py --success-sft` 按原始 Header 对齐主目标、从反馈、顶部及对应腕部 RGB，按 `perturbation_id` 场景组切分；独立来源审计、标定、夹爪映射、外部标签与训练门禁仍须通过。当前 π0.5 训练入口只接收单臂；双臂翻盖可录制但不能直接送入该入口。 | 任务、技能、结果和恢复事件只提供人工复核线索；本入口不生成 Qwen 规划监督样本。 |
| `web_console.py`：人工长任务规划、技能上下文、结果/失败原因/下一决策；VLM JSON 导出计划和带 bag 路径的转移索引 | 此入口不填写动作来源、执行臂与标定版本；其 Episode 不能直接转为 π0.5 示教集。 | 规划与结果判断的候选。需按 Header 提取真实 RGB 帧，核对物体身份、状态变化与真实轨迹；导出的 JSON 不是可直接训练的 Qwen LoRA 数据。 |
| `desktop_console.py`：本地 RGB-D 预览与普通任务 Episode | 原始资料和质量审计；缺少三技能元数据时不能直接进入动作训练。 | 原始资料和质量审计；不自动生成规划或结果监督。 |
| 深度、相机信息、原始 CAN、恢复缓存 | 用于同步、标定、故障及边界审计；训练转换器当前主要读取顶部与对应腕部 RGB、动作和反馈。 | 可作人工复核依据；需独立提取与标注才能成为候选。 |

采集配置中的 `teleoperation.mapping_verified` 和 `gripper_mapping_verified` 默认均为 `false`。界面仅在配置明确声明已核对主从映射，且对应主目标、从反馈话题列入录制范围时，写入 `human_demonstration` 等动作契约字段。这个配置声明不能替代现场证据；转换器仍要求逐条向量 `valid=true`、来源和时间一致。界面保存成功也不代表训练资格。

## 现场只读复验入口

主从臂上电即可由固件直连联动，不需要 ROS 控制节点。以下命令仅用于**电脑在 CAN 接口已经由现场流程启用后**观察并发布旁路数据；不执行 `ip link set`、启用机械臂、发送 CAN 帧或动作目标。所有终端应使用相同的 `ROS_DOMAIN_ID`。

```bash
source /opt/ros/humble/setup.bash
source /home/mips/piper_ros/install/setup.bash
ip -details -statistics link show can_left
ip -details -statistics link show can_right
ros2 run piper piper_direct_can
# 另一个终端运行：
ros2 run piper piper_arm_vector --ros-args -p gripper_mapping_verified:=false
# 再核对 /piper_direct/* 和 /piper_vector/* 的 Header、valid、missing、stale。
```

相机驱动可单独用 `ros2 launch piper start_cameras.launch.py` 启动。`gripper_mapping_verified=false` 是未验收时的正确状态，此时七维向量 `valid=false`，只能用于观察，不能作为训练动作。完成现场夹爪实测和版本证据后，才按模块 1 检查表调整该参数与采集配置；仅修改布尔值不构成验收。

## 本次稳定性检查

- 桌面合成 ROS 测试：三路预览各约 29 fps，关闭窗口后录制继续，保存/丢弃与跨磁盘重启读取通过。
- 三技能 Web 合成测试：旧任务请求、实时关节初始值、技能契约、未验证映射拒绝训练声明、标注、重启与保存通过。
- VLM Web 合成测试：旧 Episode、三种结果、规划保存、重启读取与 VLM 导出通过。
- `test_to_lerobot.py` 与 `test_atomic_skill_export.py`：28 项通过；其余录制器标注、VLM 标注、深度预览与恢复缓存测试：12 项通过。
- 修复三技能 Web 和桌面客户端对录制服务响应只调用一次 `recv` 的问题；响应超过一次读取大小时，现在持续读取至连接结束。合成测试覆盖超过 1 MB 的响应。

现场复验还需在真实三路 RGB-D 与双臂遥操作运行时记录持续帧率、Header 单调性和跨源偏差、缺帧/积压、磁盘余量、标定版本及夹爪实测映射，并用真实 Episode 重新运行来源审计和训练门禁。没有这些证据时，界面中的训练用途只表示候选数据流。
