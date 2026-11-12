# 采集系统 7 项问题的修改记录（2026-11-11）

按图片中的 P0/P1 顺序修改。核心原则不变：原始数据不改写、不补帧、不放宽 100 ms/30 ms/16 步门限；任务结果、原始保存和 SFT 资格分开。

## P0-1 主目标输入不连续

**主端需要配合。** 采集端不能靠重复旧值或刷新时间戳补齐。本次在采集侧完成：

- `piper_direct_can` 继续按内核接收时间发布原始 CAN 帧，不做时间重写；
- 诊断话题 `/piper_direct/{side}/diagnostics` 记录接收、入队、发布、溢出、发布失败和退出丢帧计数；
- `to_lerobot.py` 要求录制区间内存在诊断，且任一 `overflow`/`publish_errors`/`shutdown_discarded` 非零时拒绝该来源；
- `audit_real_data.py` 的 `latency.raw_can_bag` 给出原始 CAN Header→bag 接收延迟，按本条录制区间统计，不使用节点启动以来的累计最大值。

主端仍须周期性上报真实采样（静止时也发送），并在协议中增加采样序号或心跳，才能把“正常停发”和“通信异常”彻底分开。当前只能通过 per-ID 时间间隔、接收计数和诊断计数共同判断。

## P0-2 后续 stale 追溯否定早先有效帧

`to_lerobot.py` 的 Header 去重改为：同一 Header 先有效、后 stale 时保留**当时有效**的观测；过期与否由采样帧相对该观测原始 Header 的 `max_age` 判断。真实缺口仍会切段。原始 Header、`stale`、`valid` 均不改写。

测试 `test_stale_duplicate_headers_split_without_rewriting_sources` 已更新：有效片段从 19 帧恢复为 20 帧（0–1.9 s），stale 段仍独立成段。

## P0-3 完成候选与确认

`episode_recorder.stop()` 在到时自动停止时不再直接写入 `skill_complete` 标注，而是写入：

```json
"auto_completion_candidate": {
  "source_topic": "...", "header_stamp_ns": 0, "matched_offset_sec": 0.0,
  "candidate_origin": "duration_limit_before_terminal_stale",
  "source_validation": {...} 或 null,
  "source_validation_error": "...",
  "contiguous_frames": 0, "window_count": 0
}
```

- stale 只生成候选，不自动确认；
- 桌面控制台新增「确认候选完成帧」，确认后写入 `annotation_origin=operator_confirmed_candidate` 的 `skill_complete`；
- 录制中点击「标记当前技能成功」或停止后补标仍是独立的完成事件；
- 确认后的完成时刻固定，之后的动作保留在 bag 尾部数据中，不参与 SFT 导出。

## P0-4 原始保存与 SFT 资格解耦

`save()` 不再因完成帧缺失或连续帧不足而拒绝保存，改为记录：

```json
"training_eligibility": {
  "eligible": false,
  "status": "completion_pending | sft_frame_gate_failed | sft_frame_gate_passed",
  "reasons": ["..."]
}
```

- 任务成功但完成帧/连续性未通过时，可保存为“成功待复核”；
- `to_lerobot.py` 的 `sft_label`、连续来源检查、完成帧所属片段和 16 步窗口仍严格拒绝；
- `require_verified_skill_completion` 保留为 SFT 专用门禁，不再被原始保存调用。

## P1-5 界面提示

`desktop_console.py` 新增 `completion_status_text()`，并让 `skill_completion_issue()` 区分：

- 缺少已确认完成帧；
- 完成帧未写入 rosbag；
- 完成帧已确认但连续片段不足 16 帧；
- 来源校验错误或总体同步警告。

完成帧区域显示候选完成时间、连续帧数、所需 16 帧和具体失败原因；「确认候选完成帧」按钮直接使用候选的原始 Header，不需要反复补标试错。原始保存按钮在“成功待复核”状态下保持可用。

## P1-6 延迟诊断

- `piper_direct_can` 每个诊断周期统计 `interval_max_kernel_to_app_ms`、`interval_max_queue_wait_ms`、`interval_max_publish_call_ms`，与累计最大值分开；
- `to_lerobot.py` 聚合每个录制区间内的诊断峰值到 `_quality.transport_diagnostics[*].peaks`；
- `audit_real_data.py` 新增 `latency.raw_can_bag`：每个原始 CAN 话题的 Header→bag 接收延迟 `median/p95/max`，只统计本条 bag；
- 诊断中的 `received/enqueued/published/overflow/publish_errors/shutdown_discarded` 用于区分主端未发、接收后队列溢出和发布失败。

## P1-7 启动环境

新增 `tools/episode_recorder/start_collection.sh` 和 `preflight.py`：

```bash
# 录制器：使用带 rosbags 的兼容虚拟环境
tools/episode_recorder/start_collection.sh recorder /tmp/piper_episode_config.json

# 控制台：系统 Python，忽略用户 site-packages，避免 NumPy 2 与系统 OpenCV 冲突
tools/episode_recorder/start_collection.sh console /tmp/piper_episode_config.json
```

- 控制台自检结果：`numpy=1.21.5 opencv=4.5.4`；
- 录制器自检结果：`numpy=2.2.6 rosbags=ok`；
- 录制器解释器可用 `PIPER_RECORDER_PYTHON` 覆盖，默认 `/tmp/piper-atomic-venv/bin/python`；
- 自检失败会明确报告缺失模块，不再直接抛出 OpenCV/NumPy 不兼容堆栈。

## 验证

- `tools/episode_recorder` 与 `tools/vla_contract` 核心回归：**106 passed**（含因果对齐、诊断、候选确认）。
- 桌面控制台集成脚本：通过（候选确认、成功待复核保存、手动补标、重启列表）。
- Web 控制台集成脚本：通过。
- 原始 CAN 管线测试需在未设置 `PYTHONNOUSERSITE=1` 的环境运行（`piper_sdk` 位于用户 site）；采集启动脚本只对录制器/控制台设置该变量，CAN 节点由原 launch 启动。

## 仍需现场完成

- 主端持续上报与采样序号/心跳（P0-1 的主端部分）；
- 重采前 30–60 秒录制负载测试：原始 CAN 每 ID 接收率 ≥99%、Header→bag p95 <100 ms、向量 stale ≈0、四帧 skew ≤30 ms、图像 15 fps 无 >100 ms 缺口；
- 标定、夹爪映射、场景组和独立来源审计；
- `episode_000001/000002/000004` 仍不能追溯转为合格 SFT。
