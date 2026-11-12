# episode_000007 的 40 ms 来源验收（2026-11-12）

## 结论

**在 40 ms 相机/总体同步阈值下，episode_000007 通过源导出审计，`eligible=true`。** 它仍是 `scene_001` 的单条来源，不能单独构成 π0.5 训练集；数据集仍需至少 10 条训练、3 条测试原始 Episode 和至少两个场景组。

30 ms 严格模式下的原始录制元数据仍保留 `completion_validation=sft_frame_gate_failed`；本结论来自 40 ms 的独立来源验收和重新审计，不是回写或修改原始 Episode。

## 数据与方法

- 来源：`/home/mips/pi05_datas/banana_task_episodes/episode_000007`
- Episode ID：`9af4ebf647b04c86ba5a4f130bd5d819`
- 动作来源：`follower_next_state`；state 取从臂反馈 `t`，action 取从臂反馈 `t+1`（10 Hz，100 ms 前瞻）。
- 状态与反馈：643 条向量 / 9504 条原始反馈，0 stale、0 invalid、0 个超过 100 ms 的断档。
- 自动裁切：`stop_reason=stale_tail`，`trimmed_tail_ns=2.147 s`；逻辑终点为最后一个有效主目标。
- 阈值：`max_camera_skew_ms=40`、`max_source_skew_ms=40`；`max_can_skew_ms=30`；10 fps、最大来源年龄 100 ms、最大相机缺口 100 ms。

## 审计结果

- 对齐：99 帧，1 个连续段。
- 完成帧：在段内；完成前选中 98 帧、83 个 16 步窗口。
- 审计阈值记录：`max_camera_skew_ms=40.0`、`max_source_skew_ms=40.0`、`max_can_skew_ms=30`。
- 拒绝原因：空。

证据：

- 相机绑定：`/home/mips/pi05_datas/validation/episode_9af4ebf6_camera_binding_20261112.json`
- 从臂动作绑定：`/home/mips/pi05_datas/validation/episode_9af4ebf6_follower_action_binding_20261112.json`
- 源导出验收：`/home/mips/pi05_datas/validation/episode_9af4ebf6_source_acceptance_20261112.json`
- 40 ms 审计：`/home/mips/pi05_datas/validation/episode_9af4ebf6_source_audit_follower_20261112.json`

## 边界

- 30 fps 实测顶部—左腕相位差中位 13.33 ms、p95 24.33 ms，仍有少量离群帧；40 ms 是数据层折中，不修复相机曝光相位。不同 Episode 的相位仍可能不一致，模型可能学到轻微的多模态时序偏差。
- 如果后续完成帧仍被 40 ms 拒绝，应优先做 60 fps 或无线/硬件触发，不要继续放宽阈值。
- 源导出验收的复核人是 ZCode 只读复核；若流程要求具名人工复核人，须由操作者补签。
- 该结论只覆盖源数据；训练归一化统计、checkpoint 绑定、场景组 train/test 划分另行验收。
