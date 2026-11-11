# 按末尾 stale 前图像标记原子技能完成（2026-11-10）

## 采集约定与行为

操作员明确约定：本采集流程中，原子技能已完成后才停止操作主臂；设定时长只是录制上限，完成后的等待数据不必进入 SFT。单臂原子技能到时自动停录时，按此约定从末尾持续 `stale` 之前选取原始图像作为完成帧。仍由操作员在保存时选择整条 Episode 的成功或失败；程序不做视觉任务成功判断。

本次规则用于 `LEFT`、`RIGHT` 原子技能的时长到期自动标注。双臂及完整任务沿用已有完成标注流程；已经手动标记的完成帧不被自动覆盖。

1. 只读扫描实际保存的主端向量，按接收时间排序，查找末尾的有效到 `stale` 转变。最初没有新目标时的 `stale` 不算完成；中途 `stale` 后又恢复有效的情况也不算最终完成。
2. 前面须存在原始 `valid=true`、`gripper_mapping_verified=true` 且没有缺失或时间异常的主目标。末尾须至少有两条连续 `stale` 消息，持续观察到停止录制请求前 100 ms 内；相邻向量消息不能有超过 100 ms 的接收缺口。尾段目标 Header 若仍向前更新，也不解释为已停发。缺失、时间不一致或不可追溯来源不会被当作正常完成。
3. 以进入末尾 stale 段之前最后一条有效主目标的原始 Header 为边界，在所选彩色相机中选取 **Header 严格早于该边界的最后一帧**。优先顶部 RGB；未配置顶部时使用其他配置的彩色相机。图像距边界不能超过 100 ms，否则记录失败并要求录制后补标，不回退到等待段的图像。
4. `stale` 通常比最后目标晚约 100 ms 才出现，所以不取 stale 消息接收时刻之前的最近画面。选帧按图像 Header 进行，不按图像接收顺序；接收延迟不会将终点推到等待期。
5. 没有可靠的末尾 stale 边界时，沿用原有到时末帧逻辑及来源警告，仍由来源审计判断其资格。该后备行为不代表未验收主目标变为有效。

原始图像、向量、Header、质量标志与 bag 不改写。本策略**不从目标停发推断保持意图**；需要复用最后绝对目标的场景必须由操作员显式记录 `hold_segments`，见[保持意图的采集与 SFT 支持](HOLD_INTENT_SFT_20261110.md)。录制继续保留至原定时长，只有完成标注和后续导出边界前移。

## 可追溯字段与导出

完成标注使用 `annotation_origin=duration_limit_before_terminal_stale`，其 `source_topic` 与 `header_stamp_ns` 指向真实录制图像。`verified_in_bag=true` 仅表示该图像在 bag 中存在，不能替代训练资格验收。

Episode 同时记录：

- `auto_completion_boundary_policy=operator_terminal_stale_convention`：边界来自上述操作员采集约定。
- `auto_completion_stale_boundary`：末尾 stale 段的起止接收时间、消息数量及此前最后有效主目标的 Header 和接收时间。
- 既有 `auto_completion_tail_gap_ms`、`auto_completion_master_header_ns` 与来源警告，便于复核。

`to_lerobot.py` 识别新的标注来源，仍按所选图像 Header 截止，并继续检查来源同步、完成帧所属的连续有效片段及 16 步窗口。末尾等待帧不进入选中片段。完成前存在中途缺口，或所选完成图像不属于有效片段时，不能跨缺口拼接，也不能将完成标记当作 SFT 通过证明。

已有保存数据不自动修改；新规则随更新后的录制服务启动后用于新 Episode。本次没有重启运行中的采集服务。

## 验证

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /tmp/piper-atomic-venv/bin/python -m pytest -q \
  tools/episode_recorder/test_recorder_annotations.py \
  tools/episode_recorder/test_to_lerobot.py \
  tools/episode_recorder/test_atomic_skill_export.py
```

结果为 `74 passed in 57.16s`。测试覆盖初始 stale、中途恢复、末尾连续 stale、向量接收中断、未验收映射、缺失或时间异常、图像接收延迟、边界前无近时图像、真实停止流程，以及按新完成帧截断的 SFT 片段；原始 stale 不转为有效，16 步要求保持。

对 `/home/mips/pi05_datas/banana_task_episodes/episode_000001` 只读试算：最后有效主目标 Header 为 `1794290236326212406`，末尾 stale 从接收时间 `1794290236439960363` 开始；新规则选到顶部 RGB Header `1794290236310075439`，即 `ready_at_unix_ns` 后约 `10.782061 s`。此帧仍不属于当前 30 ms 来源阈值下的连续有效片段，成功 SFT 选择继续拒绝；本次未将计算结果回填 Episode。

历史 `pick_banana_into_box_left/episode_000004` 的原始主端映射标志未验收，不能据其构造这一“有效到 stale”边界。两条来源的 `episode.json`、bag 元数据及 `.db3` 大小和修改时间在本次只读试算前后均一致。
