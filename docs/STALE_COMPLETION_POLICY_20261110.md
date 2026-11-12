# 按末尾 stale 前图像标记原子技能完成（2026-11-10）

## 采集约定与行为

操作员明确约定：本采集流程中，原子技能已完成后才停止操作主臂；设定时长只是录制上限，完成后的等待数据不必进入 SFT。单臂原子技能到时自动停录时，按此约定从末尾持续 `stale` 之前选取原始图像作为**完成候选**。候选不自动确认；操作员在保存前点击「确认候选完成帧」或独立补标后才写入 `skill_complete`。

本次规则用于 `LEFT`、`RIGHT` 原子技能的时长到期自动标注。双臂及完整任务沿用已有完成标注流程；已经手动标记的完成帧不被自动覆盖。

1. 只读扫描实际保存的主端向量，按接收时间排序，查找末尾的有效到 `stale` 转变。最初没有新目标时的 `stale` 不算完成；中途 `stale` 后又恢复有效的情况也不算最终完成。
2. 前面须存在原始 `valid=true`、`gripper_mapping_verified=true` 且没有缺失或时间异常的主目标。末尾须至少有两条连续 `stale` 消息，持续观察到停止录制请求前 100 ms 内；相邻向量消息不能有超过 100 ms 的接收缺口。尾段目标 Header 若仍向前更新，也不解释为已停发。缺失、时间不一致或不可追溯来源不会被当作正常完成。
3. 以进入末尾 stale 段之前最后一条有效主目标的原始 Header 为边界，在顶部与执行侧腕部彩色相机中按时间从近到远检查 **Header 严格早于该边界、且距边界不超过 100 ms 的真实图像**。选择最新且通过因果图像组校验的真实相机帧，不固定优先顶部 RGB；记录实际话题、Header 与图像组来源。完整 CAN 原子技能的候选须通过与转换器一致的同步、时效及连续 16 步检查，且候选与结束边界之间不能跨来源中断。没有合格候选时要求复核，不回退到等待段或更远的旧技能阶段。
4. `stale` 通常比最后目标晚约 100 ms 才出现，所以不取 stale 消息接收时刻之前的最近画面。选帧按图像 Header 进行，不按图像接收顺序；接收延迟不会将终点推到等待期。
5. 没有可靠的末尾 stale 边界时，完整 CAN 原子技能在停止请求前 100 ms 内按同样规则筛选；失败标为待复核。仅图像采集沿用原有末帧存在性检查，不声明 CAN 来源有效。

原始图像、向量、Header、质量标志与 bag 不改写。本策略**不从目标停发推断保持意图**；需要复用最后绝对目标的场景必须由操作员显式记录 `hold_segments`，见[保持意图的采集与 SFT 支持](HOLD_INTENT_SFT_20261110.md)。启用 `auto_trim_stale_tail` 时，末尾持续 stale 达到宽限期后自动停止，逻辑终点裁切到最后有效主目标；原始 bag 尾部保留。候选仍须确认，不自动授予成功或训练资格。

## 完成候选与确认

`stop()` 不再自动追加 `skill_complete`。到时自动停止时写入 `auto_completion_candidate`，其中包含：

- `source_topic`、`header_stamp_ns`、`matched_offset_sec`：真实录制图像及候选时刻；
- `candidate_origin=duration_limit_before_terminal_stale` 或 `duration_limit_last_frame`；
- `source_validation` 或 `source_validation_error`：与转换器共享的同步、时效、连续片段和 16 步检查结果；
- `contiguous_frames`、`window_count`、`segment_index`、`tail_gap_ms` 和来源警告。

桌面控制台先按精确 Header 显示已录制的候选画面；加载后，操作员点击「确认候选完成帧」，写入唯一 `skill_complete`，`annotation_origin=operator_confirmed_candidate`，完成时刻固定；之后的动作保留为 bag 尾部数据，不进入 SFT 导出。录制中点击「标记当前技能成功」或停止后补标仍是独立的完成事件。没有确认完成帧时，Episode 仍可按“任务成功、数据待复核”保存，`training_eligibility` 记录 `completion_pending`/`sft_frame_gate_failed`，SFT 导出继续严格拒绝。

## 可追溯字段与导出

确认后的完成标注使用 `annotation_origin=operator_confirmed_candidate`（或原有的 `post_recording_review`），其 `source_topic` 与 `header_stamp_ns` 指向真实录制图像。`verified_in_bag=true` 仅表示该图像在 bag 中存在，不能替代训练资格验收。

Episode 同时记录：

- `auto_completion_boundary_policy=operator_terminal_stale_convention`：边界来自上述操作员采集约定。
- `auto_completion_stale_boundary`：末尾 stale 段的起止接收时间、消息数量及此前最后有效主目标的 Header 和接收时间。
- 既有 `auto_completion_tail_gap_ms`、`auto_completion_master_header_ns` 与来源警告，便于复核。

`to_lerobot.py` 识别 `operator_confirmed_candidate` 及原有标注来源，仍按所选图像 Header 截止，并继续检查来源同步、完成帧所属的连续有效片段及 16 步窗口。末尾等待帧不进入选中片段。完成前存在中途缺口，或所选完成图像不属于有效片段时，不能跨缺口拼接，也不能将完成标记当作 SFT 通过证明。原始保存不再调用 SFT 专用的 `require_verified_skill_completion`；该函数只用于 SFT 门禁。

已有保存数据不自动修改；新规则随更新后的录制服务启动后用于新 Episode。2026-11-11 的来源有效性筛选与复验见[时间对齐与完成帧有效性筛选](TIME_ALIGNMENT_COMPLETION_20261111.md)。

## 验证

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /tmp/piper-atomic-venv/bin/python -m pytest -q \
  tools/episode_recorder/test_recorder_annotations.py \
  tools/episode_recorder/test_to_lerobot.py \
  tools/episode_recorder/test_atomic_skill_export.py
```

结果为 `74 passed in 57.16s`。测试覆盖初始 stale、中途恢复、末尾连续 stale、向量接收中断、未验收映射、缺失或时间异常、图像接收延迟、边界前无近时图像、真实停止流程，以及按新完成帧截断的 SFT 片段；原始 stale 不转为有效，16 步要求保持。

对 `/home/mips/pi05_datas/banana_task_episodes/episode_000001` 只读试算：最后有效主目标 Header 为 `1794290236326212406`，末尾 stale 从接收时间 `1794290236439960363` 开始；新规则选到顶部 RGB Header `1794290236310075439`，即 `ready_at_unix_ns` 后约 `10.782061 s`。此帧仍不属于当时 30 ms 来源阈值下的连续有效片段，成功 SFT 选择继续拒绝；本次未将计算结果回填 Episode。

历史 `pick_banana_into_box_left/episode_000004` 的原始主端映射标志未验收，不能据其构造这一“有效到 stale”边界。两条来源的 `episode.json`、bag 元数据及 `.db3` 大小和修改时间在本次只读试算前后均一致。

## 2026-11-12 末尾零动作自动裁切

原子技能录制新增配置：

- `auto_trim_stale_tail=true`：启用末尾 stale 自动停止与逻辑裁切。
- `stale_stop_grace_sec=1.0`（0.2～10 秒）：主目标进入持续 stale 后等待多久确认操作员已经停止操作。

录制中订阅所选执行臂的 `/piper_vector/{arm}/master_target`，按与 `terminal_stale_master_target` 相同的质量条件分类：

1. `valid=true` 且四路 `stale=false`：刷新最后有效时刻。
2. `valid=false` 且四路 `stale=true`：累计 stale 消息数。
3. 缺失、时间不一致、夹爪映射未核验或来源不符：重置，不据此裁切。

同时满足「至少 2 条 stale 消息」「最后一条 stale 在 100 ms 内」「距最后有效目标超过 `stale_stop_grace_sec`」「没有未结束的显式保持段」时，录制器以 `stop_reason=stale_tail` 自动停止。停止后重新扫描 bag，确认末尾 stale 边界：

- `stopped_at_unix_ns` 设为最后一个有效主目标的原始 Header；
- `recorded_duration_sec` 按裁切后的终点重算；
- 写入 `trim_boundary_ns`、`raw_stopped_at_unix_ns`、`trimmed_tail_ns` 和 `auto_trim_source=terminal_stale_master_target`；
- 完成候选的 `candidate_origin=stale_tail_before_terminal_stale`，仍须操作员确认；
- 自动完成边界策略写为 `auto_trim_terminal_stale_convention`。

原始 bag 不重写：它仍保留 `stale_stop_grace_sec` 的等待窗口，但 Episode 的逻辑终点和 SFT 导出按 `trim_boundary_ns` 截断，尾部零动作不进入训练。若需要物理裁切 bag，应另建来源并重新计算摘要，不能覆盖原始数据。

`tools/rerun_bridge.py replay` 默认读取相邻的 `episode.json`，按 `ready_at_unix_ns` 到 `trim_boundary_ns`（无裁切时用 `stopped_at_unix_ns`）过滤消息；因此 rerun 视图与逻辑 Episode 一致。需要查看原始 bag（包括等待窗口）时显式加 `--full`。过滤按消息 Header 时间执行，缺少 Header 的事件使用 bag 接收时间。

双臂技能要求左右两臂都出现确认的末尾 stale，取较早的 `last_valid_header_ns` 作为裁切终点；完成候选仍按原有双臂流程人工复核。若停止后找不到可确认的边界，写入 `auto_trim_error`，不修改终点，由操作员复核。

该功能只处理「主目标持续 stale、从臂保持」的末尾等待，不把 stale 当作保持意图；显式保持段仍按 `hold_segments` 处理，处于未结束保持段时不会自动裁切。配置示例见 [`episode_recorder.pi05.json`](../config/episode_recorder.pi05.json)；通用示例保持 `auto_trim_stale_tail=false`。

2026-11-12 的选帧、实际数据启动门禁、WAL 存储和数据流检查，见[相机完成选帧与采集数据流优化](COLLECTION_CAMERA_TIMING_PIPELINE_20261112.md)。
