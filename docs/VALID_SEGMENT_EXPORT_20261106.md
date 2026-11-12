# 停发后的连续有效片段提取与只读复验（2026-11-06）

## 行为约定

主端停发后，向量节点可能重复发布原来的 Header，并将 `stale=true`、`valid=false`。这些消息用于报告质量，不能更新 Header 来假装主端仍在发帧。`to_lerobot.py` 和 `audit_real_data.py` 新增显式的 `--extract-valid-segments`；不启用时仍保持整段严格拒绝。

片段提取只适用于 `can-vector`，并必须显式提供 `--max-source-skew-ms`。它按以下规则工作：

1. 先核验原始 Header、CAN 来源话题、接口、ID、关节维度与数值，再建立每路来源的连续覆盖区间。`missing`、`stale`、`time_inconsistent`、不明原因的 `valid=false`、CAN 时间超窗、来源越界及来源缺口都会截断区间。相机缺口也会截断。来源身份错误、相同 Header 的冲突载荷或 Header 倒退仍拒绝整个来源，不通过过滤掩盖。
2. 同一 Header 的完全相同重复向量可去重；如果其中任何副本存在无效质量标志，则保守排除这个 Header 的所有副本。旧 Header 无法说明质量变化的精确发生时间，因此不使用 bag 接收时间重新给它定位。
3. 各路连续区间取交集后再按指定 FPS 网格采样。`can-vector` 先找网格之前的同步 RGB 图像组，再按图像组的实际参考 Header 匹配此前的动作与状态；来源相对网格的年龄、整体同步阈值和因果顺序继续检查。同一图像组不重复计帧。两个采样点之间出现的无效来源同样产生边界；采样后的 RGB 同步、跨源同步或超龄失败也产生边界。不能删除失败行后把前后帧连接成连续序列。详见[时间对齐与完成帧有效性筛选](TIME_ALIGNMENT_COMPLETION_20261111.md)。
4. 每个导出片段至少包含 `16` 帧。长度为 `N` 的片段最多提供 `N-15` 个完整 16 步窗口；不能用另一个片段补齐。LeRobot 中每个片段分别调用 `save_episode()`，相对时间与 `frame_index` 在各自 Episode 内从零开始。
5. `--success-sft` 只选择完成帧实际所属的连续片段，并截止到完成时刻。完成帧须在原 bag 的指定相机消息中找到，且在该时刻重新通过来源对齐。完成帧在停顿、超龄或同步失败区间内时，其他有效片段不能继承成功标注；完成前不足 16 帧也拒绝。其他片段只保留为来源候选。
6. 未标注导出使用 `源 episode_id__segment_NNN` 标识各片段，同一源 Episode 的所有片段保持在同一 train/test 划分。成功 SFT 只导出一个完成片段，保留源 `episode_id`，仍按完整 `perturbation_id` 场景组划分。

`conversion_manifest.json` 保留源 Episode、源摘要、片段边界、源片段帧数、实际导出帧数及窗口数。`source_timestamps.jsonl` 保留源 Episode、片段号、LeRobot Episode 序号、采样目标时间、原始各路 Header、原始四帧 CAN 时间与原始质量标志。合成的采样目标时间单独记录，不替换原始 Header。审计的 `quality.rejected_frames` 与 `quality.rejected_vector_messages` 列出拒绝时刻、原始 Header、原因及重复消息数；短片段也保留在 `quality.segments`，不会混入满足 16 步条件的计数。

既有 `--gripper-calibration` 的四路映射和零点校正仍可显式使用，但仅能解释发布者同时标记 `gripper_mapping_verified=false` 的映射未验收状态；不能解释映射已经核实后的 `valid=false`，也不能豁免 `missing/stale/time_inconsistent` 或来源校验。每个被采用向量的原始 `valid`、映射标志和校正是否应用均写入来源记录，不把原始无效标志改成有效。

## 候选与训练资格

审计中的 `aligned_frame_count` 包含所有通过对齐的帧，包括不足 16 帧的短片段。`candidate_segment_count` 和 `candidate_frame_count` 只统计至少 16 帧的片段。候选数量不授予训练资格；`eligible_frame_count` 仍需同时满足技能标注、完成帧归属及源数据契约。`eligibility_scope=source_sft_export` 明确此处的 `eligible` 只表示源数据可进入导出流程；报告与导出清单的 `training_ready=false` 不会自动解除。

源数据契约要求 `data_convention_version` 说明原始字段、顺序和单位。训练 `normalization_version` 属于选定 train 划分后计算的统计资产，不要求回写源 Episode；未知时允许缺省，导出清单保留 `null`，并记录 `training_normalization.status=not_computed`、`verified=false` 与后续 train 统计及 checkpoint 绑定要求。已有源声明也仅记为 `source_declared_unverified`。`piper_raw_*` 或与字段约定相同的版本不能充当训练归一化版本。缺少字段约定或标定版本、存在未解决的来源阻断项、相机或映射仍未验收、时钟跳变或已记录的 Header 非单调时，导出保持拒绝。采集时的待审声明与下游训练待办按下述规则分别处理。只读候选审计可以诊断这些来源，但不修改其元数据，也不将暂定来源升级为 `eligible=true`。归一化资产的独立验收属于后续训练门禁。

## 新采集格式的源导出门禁与训练门禁分层

新采集格式始终保留 `collection_contract_status=metadata_ready_pending_data_audit`，以及 `training_normalization.status=not_computed`、`independent_audit.status=pending`。这些快照不要求回写原 Episode：

- `time_domain`、`teleoperation_mapping`、`action_space`、`action_provenance`、`data_convention` 的 `declared_pending_audit` 可以进入本次源审计，但必须继续通过显式字段、原始 CAN 来源、向量质量、Header 连续性、逐帧对齐与完成帧检查。元数据声明本身不会令来源合格。
- `training_normalization` 与 `independent_audit` 属于后续正式训练门禁，其原始检查记录保留在 `deferred_training_checks`。它们的待办说明不阻止源数据导出；导出依然是 `training_ready=false`，不能代替独立训练审计、train 统计及 checkpoint 绑定。
- 相机的 `provisional` 或 `operator_supplied_pending_audit`、未核验映射、未知待审检查，以及无法关联到已解决检查的自由文本阻断项，仍然阻止源导出。不会按 `pending` 单词统一放行，也不会统一拒绝所有待办。

采集端不会自行授予独立相机验收。来源项已在采集后完成独立验收时，可为转换与审计同时提供 `--source-acceptance /绝对路径/source_acceptance.json`。这是只读的源导出证据文件，不修改 `episode.json`，也不授予训练资格。格式示例中的摘要与版本均须替换为真实验收值；不得为当前未验收来源生成通过记录：

```json
{
  "schema_version": "piper_source_export_acceptance/1",
  "episodes": {
    "待验收的原 episode_id": {
      "source_sha256": "原 Episode 的 source_digest 组合 SHA-256",
      "arm_mode": "LEFT",
      "calibration_version": "与原 Episode 一致的标定版本",
      "data_convention_version": "与原 Episode 一致的原始字段版本",
      "reviewed_by": "独立复核人",
      "reviewed_at_unix_ns": 1800000000000000000,
      "checks": {
        "calibration": {
          "status": "verified",
          "evidence": [
            {"path": "camera_acceptance_report.json", "sha256": "验收材料文件的 SHA-256"}
          ]
        }
      }
    }
  }
}
```

`source_digest` 覆盖原 `episode.json`、`bag/metadata.yaml` 与全部 `.db3`；检查时重新计算，不依赖调用方自报的来源摘要。验收记录还须绑定原执行臂、标定版本、字段版本、非空复核人以及不早于停止录制的复核时间。材料路径相对验收文件目录解析，也可使用绝对路径；材料必须存在且文件 SHA-256 匹配。仅允许验收明确的来源检查项，不能用该文件将训练归一化或正式训练门禁标为已通过。缺少对应 Episode、来源摘要不匹配、版本变化或材料被改动时，不会解除来源拒绝。采集时缺失的显式动作来源与动作空间字段也不能通过这份文件补造。

导出清单与审计行的 `source_export_acceptance` 保存验收文件路径、文件摘要和对应记录；`deferred_training_checks` 原样保留下游待办。需要正式训练时，仍由后续训练门禁核对独立训练证据、分组划分、归一化资产及 checkpoint 绑定。

## 真实来源复验

使用 `10 FPS`，`max-age=100 ms`、`max-can-skew=30 ms`、`max-camera-gap=100 ms`、`max-camera-skew=30 ms`、`max-source-skew=30 ms`，显式传入既有 `config/gripper_collection_calibration.20261105.json`。该配置用于离线映射校正，不代表原始采集已经完成映射验收。

| 来源 | 对齐帧数 | 全部连续片段长度 | 至少 16 帧的候选 | 合格训练帧 |
| --- | ---: | --- | --- | ---: |
| 当前 `pick_banana_into_box_left/episode_000004` | 101 | 64、3、29、3、1、1 | 2 段，93 帧 | 0 |
| 回收站 `episode_000001` | 102 | 1、1、1、15、11、21、1、46、1、4 | 2 段，67 帧 | 0 |
| 回收站 `episode_000002` | 141 | 50、9、26、1、4、7、39、5 | 3 段，115 帧 | 0 |
| 回收站 `episode_000003` | 158 | 61、2、4、33、56、1、1 | 3 段，150 帧 | 0 |
| `/tmp/piper_final_recovered_saved_20261104/episode_000001` | 0 | 未建立来源时基 | 0 | 0 |
| `/tmp/piper_final_recovered_saved_20261104/episode_000002` | 0 | 未建立来源时基，另有 Header 非单调 | 0 | 0 |

`episode_000004` 的完成帧为顶部 RGB Header `1793952343651478271`，不属于连续有效片段。其原始字段还缺显式动作来源、动作空间和时间域，原始字段版本仍写在旧 `normalization_version` 中，采集契约含未验收项。因此严格审计与片段审计均为不合格；直接运行成功 SFT 转换也拒绝，并且没有创建输出数据集。不能给前面的 64 帧或 29 帧候选补上成功标签。单独诊断传感器时，严格读取仍因主端 `stale` 拒绝，片段提取则保留上述候选。

回收站路径为 `/home/mips/.local/share/Trash/files/episode_000001`～`episode_000003`。本次只读所有来源的 `episode.json`、`bag/metadata.yaml` 与 `.db3`，逐来源计算复验前后组合 SHA-256，六个来源均一致；没有删除、恢复或改写 Episode。完整报告位于本机 `/tmp/piper_task4_real_validation_20261106.json`，包含各来源前后摘要与拒绝原因；临时路径不视为长期归档。

## 复现命令与测试

在 `/home/mips/piper_ros` 执行以下只读审计；输出写入临时目录，原始来源保持不变：

```bash
/tmp/piper-atomic-venv/bin/python tools/episode_recorder/audit_real_data.py \
  /home/mips/pi05_datas/banana_task_episodes/pick_banana_into_box_left \
  --episode-root /home/mips/pi05_datas/banana_task_episodes/pick_banana_into_box_left \
  --fps 10 --max-age-ms 100 --max-can-skew-ms 30 \
  --max-camera-gap-ms 100 --max-camera-skew-ms 30 --max-source-skew-ms 30 \
  --gripper-calibration config/gripper_collection_calibration.20261105.json \
  --extract-valid-segments --output /tmp/piper_valid_segments_audit.json
```

对满足真实训练契约的来源，可在原有 `to_lerobot.py` 命令中同时添加 `--extract-valid-segments --max-source-skew-ms 30`；成功 SFT 仍需 `--success-sft` 及至少两个独立场景组。不能通过编辑旧 Episode 来使上述真实来源通过。

本次结果：`43 passed in 47.45s`。验证命令：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /tmp/piper-atomic-venv/bin/python -m pytest -q \
  tools/episode_recorder/test_to_lerobot.py \
  tools/episode_recorder/test_atomic_skill_export.py
```

测试覆盖主端停发并重复旧 Header、采样点之间的无效向量、相机中断、冲突或错误来源、完成帧落入停顿区间、完成前不足 16 步、暂定契约的候选拒绝，以及真实 `LeRobotDataset v3.0` 的写入、`finalize()` 和回读。回读核对每个片段独立的 Episode 边界、连续帧号、原始时间戳和完整来源划分。

本轮补充门禁分层验证：转换与审计回归为 `53 passed in 56.21s`，下游归一化与 Piper 适配门禁回归为 `21 passed in 0.53s`。新用例覆盖新采集元数据、下游待办保留、未验收映射和未知阻断、验收材料篡改、源摘要或版本不匹配、复核时间早于录制停止，以及声明虽待审但实际跨源时间差超限时继续拒绝。

另直接调用当前 `skill_web_console.start_request()` 生成两条合成 Episode 的完整采集元数据，再运行审计和真实 LeRobot v3 转换：不提供源验收时为 `false/false`，提供绑定合成证据的独立来源验收后为 `true/true`，成功导出 `2` 条、共 `32` 帧；`training_ready` 仍为 `false`，`normalization_version` 为 `null`，原始合成来源摘要保持一致。此处图像、关节与验收材料都是测试数据，不是现场相机验收。完整记录为 `/tmp/piper_task4_new_collection_gate_20261106.json`。

当前真实 `episode_000004` 在本轮改动后再次只读复验：仍为 `2` 段、`93` 帧候选，`eligible=false`、合格帧数为 `0`，原始组合 SHA-256 前后仍相同。记录为 `/tmp/piper_task4_source_gate_real_20261106.json`；本轮没有为任何真实来源生成通过的源验收文件。
