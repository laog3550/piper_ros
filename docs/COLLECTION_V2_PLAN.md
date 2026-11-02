# 香蕉入盒与双臂翻盖采集系统改造方案

## 当前基线

现有录制器可从主端遥操作与从端话题录制一个完整 `task` 或 `skill` Episode，保存 rosbag2 SQLite、相机 Header 时间统计和人工结果。三路 RGB-D、双臂 CAN 与 `PiperArmVector` 已列入示例配置。Web 控制台支持预览、开始、停止、保存与丢弃。转换器已有双臂真实动作/反馈契约和合成 LeRobot 回读测试。当前真实数据仍没有通过该契约的训练 Episode；校准版本与夹爪映射须现场证明。

## 采集单元与技能边界

完整任务保留一条 `mode=task` Episode，顺序为 `pick_banana`、`place_banana_in_box`、`close_hinged_lid_bimanual`。另分别采集三个 `mode=skill` Episode，避免只从成功长序列里截取训练片段。翻盖按一个双臂协同技能录制：一臂稳定盒体，另一臂沿真实铰链转动盖子，记录两臂连续的真实动作与反馈；不要把单臂运动复制成另一臂标签。

起始位姿扰动由人员在安全范围内摆放后记录在 `task_metadata.initial_conditions`，包括香蕉、盒体、盖子角度、两臂初始状态和扰动编号。采集器只记录，不自动移动机械臂。每个扰动编号应覆盖成功与失败/恢复实例；训练/测试按完整 Episode 和场景组切分，防止同一重复动作泄漏。

成功 SFT 导出要求每段原子技能 Episode 明确记录非空字符串 `task_metadata.initial_conditions.perturbation_id`。同一编号在所有技能中视为同一场景组，完整划入 train 或 test；缺少编号的 Episode 拒绝导出，合格数据不足两个不同场景组时不能建立训练/测试切分。旧版未标注转换保持按 `episode_id` 切分。

## v2 标注契约

新 Episode 的 `episode.json` 使用 `schema_version=2`，保留 v1 字段，新增 `active_skill` 与 `annotations`。录制中经 socket `annotate` 写入：

```json
{"action":"annotate","kind":"skill_complete","value":"close_hinged_lid_bimanual","source_topic":"/camera_top/color/image_raw","header_stamp_ns":1234567890000000000}
```

`kind` 为 `skill_start`、`skill_complete`、`outcome`、`recovery_trigger`；每条标注由录制器补 `recorded_at_unix_ns` 并同步写入元数据和 `/collection/events`。恢复触发还生成唯一 `recovery_id` 并记录当前 `skill`。`skill_complete` 对应当前技能；`task` Episode 强制抓取、放置、翻盖顺序。相机来源只接受配置中的 `frame_topics`。如果点击时无法取得真实图像 Header，允许保存待对齐标注并置 `frame_resolution_required=true`；质量审计必须将其挡在成功 SFT 导出之外。录制器接收时间不能冒充完成帧。旧 v1 Episode 仍可读取，但不自动补造 v2 标签。

恢复缓存设计预算：三路 640×480 RGB-D 原始帧、30 fps 约 138 MB/s；前录 3 秒约 415 MB，前 3 秒加后录 2 秒约 691 MB，双份拷贝可能超过 1 GB。实现须同时限制 `pre_roll_sec` 和 `max_buffer_bytes`，满时淘汰最旧消息并记录实际回溯时长、淘汰数和丢帧数。独立恢复 bag 的 manifest 必须关联原 Episode、技能和 `recovery_id`，记录各话题计数、前后窗口、磁盘余量与完整/失败状态。只有通过人工审核的完整片段才能回流；当前 v2 仅提供触发事件，尚无在线回溯缓存。

## 七模块实施与验收门槛

| 模块 | 改动 | 可验收结果 |
| --- | --- | --- |
| 1 硬件与时钟 | 检查双臂主从端话题、三路 RGB-D、Header 时钟域、标定及夹爪映射 | 现场只读清单和同步偏差记录；缺证据时拒绝训练资格 |
| 2 技能状态机 | 固定三技能、顺序检查、起始条件元数据与原子 Episode | 非法技能/乱序/重复完成被拒绝，旧任务录制正常 |
| 3 轻量标注 | Web 点击即写技能、结果、完成帧和恢复事件 | 重启后标签仍在元数据及 rosbag 事件中 |
| 4 恢复触发 | 可选有界预录缓存与触发后片段；先定义事件和关联 ID，再单独验收缓存 | 回溯时长、缓存占用与丢帧可测；未启用时不宣称有回溯视频 |
| 5 质量审计 | SQLite、话题覆盖、Header 连续性、跨传感器同步、边界及标注覆盖 | 每 Episode 给出通过/拒绝原因，成功训练集只收合格样本 |
| 6 LeRobot 导出 | 按原子技能导出真实双臂动作/反馈、相机与任务文本 | 真实 LeRobot 写入/回读；无合格真实样本时停止 π0.5 SFT |
| 7 运行时轨迹 | 规划、Preflight、执行、Verify、失败及恢复关联的可选导出 | 可追溯的 JSONL；仅经人工核验的真实轨迹进入 Qwen LoRA 候选池 |

部署门槛是独立于采集实现的验收项：当前 Qwen 合成规划、π0.5 官方 DROID smoke 和 VLA 契约测试不能代替 Piper 双臂实机验证。运动规划桥梁和 AgentLoop 须先通过现场快照、动作范围、停止与 Verify 检查，才允许闭环执行。恢复样本只能在人工审核、动作/反馈完整和时间同步合格后回流训练。

## 并行工作边界

当前集成任务框负责录制器后端与合并验收；现有 Web 控制台任务框负责操作界面；现有 Episode 转换任务框负责审计与导出；现有现场只读验收任务框负责模块 1；现有 VLM 评估任务框负责模块 7；现有 π0.5 离线任务框负责训练准入。各任务在独立 worktree 提交，集成时逐个核对接口和测试，不以消息分发视为功能完成。

## 2026-11-02 集成进度

合并到 `humble` 时，原有深度预览/VLM 标注控制台继续使用 `tools/episode_recorder/web_console.py`；香蕉三技能实时标注使用 `tools/episode_recorder/skill_web_console.py`。两个入口共用同一录制器与 Episode 元数据，不应同时作为同一个端口的 HTTP 服务启动。

已合入录制器 v2 标注、Web 实时标注、模块 1 只读检查表、原子技能质量审计与 LeRobot v3 导出、Qwen 规划运行时轨迹钩子、π0.5 原子技能训练准入。集成分支的 Web 合成 ROS 测试通过；LeRobot 转换测试 24 项、VLA 与录制器离线回归 78 项、π0.5 准入测试 7 项、恢复缓存测试 5 项通过。v2 合成 Episode 已走通审计、成功技能转换、外部人工标签与 SFT 数据门禁。这些测试使用合成输入验证软件契约。

Web 单技能采集现在要求显式选择执行臂：抓取与入盒为 `LEFT` 或 `RIGHT`，双臂翻盖为 `BIMANUAL`；并填写非空、非 `UNVERIFIED` 占位的 `calibration_version` 与 `normalization_version`。只有录制配置 `teleoperation` 明确声明 `source=master_slave_teleoperation`、`mapping_verified=true`、`gripper_mapping_verified=true`，且对应臂的主目标和从反馈话题映射与已录话题一致时，Web 才写入 `time_domain=unix_ns`、`action_space=absolute_joint_position_7`、`action_provenance=human_demonstration`。示例配置的验证开关均为 `false`；须在主从端真实遥操作、时间域和夹爪物理映射完成现场核对后才可修改。未验证时界面显示训练未就绪，Episode 元数据也不伪造这三项。即使元数据齐全，仍须通过逐帧来源审计和转换器检查；旧 `task` 请求不要求新增字段。

成功 SFT 按 `task_metadata.initial_conditions.perturbation_id` 整组切分，缺组标识或少于两个不同扰动组时拒绝导出；旧转换模式保持原切分行为。π0.5 门禁按 v2 顶层技能、逐 Episode 任务文本和 v2 审计核验，并继续要求独立人工标签、来源摘要、左右臂角色与同步证据。

只读审计 `/home/mips/piper_tjp/data` 的 8 段真实来源：合格 Episode 为 0，合格帧为 0。现场两路 CAN 当前为 DOWN，ROS 相机与机械臂节点未运行；本轮无法重新测量在线同步。标定版本、夹爪物理映射、同场双臂动作和反馈仍需现场核验。固定 openpi 版本的 LeRobot 读取器与 v3 导出格式还没有完成兼容验收，训练入口保持拒绝。

可选恢复缓存已作为独立组件合入，默认关闭；合成 ROS 测试 5 项通过，验证标准 rosbag2 回读、触发边界不重复写帧和磁盘下限失败标记。六路合成 RGB-D 短时压测仍检测到缺帧，清单标为 `failed`，`training_approved=false`。真实相机与双臂同时采集的性能、同步及恢复片段回流仍需现场验收。
