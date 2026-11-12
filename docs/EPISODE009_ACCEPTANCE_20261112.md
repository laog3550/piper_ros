# episode_000009 的 π0.5 SFT 数据检查（2026-11-12）

## 结论

**作为单条来源 Episode，episode_000009 已通过 40 ms 的源导出审计，`eligible=true`，可以作为 π0.5 SFT 的来源样本。** 它覆盖了从香蕉在盒外、抓取、移入盒内到松手后的完整连续轨迹，完成帧门禁通过，三路 CameraInfo 与当前已验收的相机批次一致，左从臂反馈全程连续。

**但它还不能单独构成 π0.5 训练数据。** 它是 `scene_001` 的单条 Episode，当前没有第二个场景组，也没有 LeRobot 转换、`piper_atomic_skill_labels/1` 独立标注和训练归一化统计；转换器要求至少两个 `perturbation_id` 场景组，SFT 门禁还要求每个单臂技能至少 10 条训练和 3 条测试原始 Episode。因此本条只能计入候选来源，不能据此宣称 π0.5 SFT 数据集已合格。

| 判定对象 | 结果 |
| --- | --- |
| 单条来源 SFT 导出 | **合格**，`eligible=true`、拒绝原因为空 |
| 单条完整技能轨迹 | **可用**，从盒外抓取到入盒松手连续；含约 2.1 s 零动作尾部 |
| π0.5 SFT 训练集 | **不合格/不足**，只有 1 条 `scene_001`，未转换、未独立标注、未计算归一化 |
| 训练资格 | **未授予**，`training_ready=false` |

## 来源与方法

- 来源：`/home/mips/pi05_datas/banana_task_episodes/episode_000009`
- Episode ID：`fef43f490ac2422bbcf1deb48b7678ab`
- 来源摘要：`474f24a1549b73cd2dae4fe3eba869fbedd89633a92a64015f56f8d78d944422`
- 执行臂与技能：`LEFT`，`pick_and_place_banana_in_box`，任务文本“香蕉入盒”
- 动作来源：`follower_next_state`，即 `state_t = 左从臂反馈(t)`、`action = 左从臂反馈(t+100 ms)`
- 阈值：10 fps、最大来源年龄 100 ms、CAN 偏差 30 ms、相机偏差 40 ms、总体来源偏差 40 ms
- 夹爪：显式使用 `config/gripper_collection_calibration.20261112.json`，设置哈希 `37dcd497…` 与 Episode 绑定一致

审计为只读操作，没有修改原始 bag、Header、stale 标志、元数据或标注。新增的仅为独立证据文件。

## 通过项

| 检查项 | 结果 |
| --- | --- |
| Episode 状态与结果 | `saved`；`outcome=success`；`skill_complete` 1 条；标注在 bag 中可找到 |
| 录制时长 | 15.105 s；`stop_reason=duration_limit` |
| 完成帧门禁 | `completion_validation.frame_gate_passed=true`，`sft_frame_gate_passed` |
| 完成帧 | 顶部 RGB Header `1794463490207695313`，相对 ready 约 **13.429 s**；在连续段内 |
| 相机来源 | 左腕 230 条、右腕 230 条、顶部 271 条 CameraInfo；三路均只有 1 个内参 profile，与 v7 证据采集及 `camera_info_snapshot_20261112_v1` 逐项一致 |
| 左从臂反馈 | 814 条向量、12260 条原始反馈；0 stale、0 invalid、0 个超过 100 ms 的断档 |
| next-state 对齐 | 163 个 10 Hz 网格点；153 个对齐通过；1 个连续段（153 帧、138 个 16 步窗口） |
| 完成前选中 | 134 帧、119 个 16 步窗口；`segment_index=0` |
| 动作时序 | `state→action` Header 差 79.975～119.998 ms，全部严格晚于 state；动作目标偏移 37.894～99.123 ms |
| 最大选中同步偏差 | 相机 39.037 ms，总体来源 39.037 ms；均低于 40 ms |
| 官方 40 ms 审计 | `eligible=true`，拒绝原因为空；`eligible_frame_count=134` |

## 质量项与边界

### 1. 末尾有约 2.1 秒零动作保持

按对齐后的 134 个选中帧统计，最后 **22 帧（约 11.19～13.29 s）** 的 `action` 与 `state` 完全相同，`dq=0`。轨迹在约 10.5～11.2 s 已经松手，香蕉留在盒内；完成标记在 13.429 s，位于松手后的静止保持段。

这不构成当前 SFT 门禁的硬失败，样本仍满足连续 16 步窗口；但如果技能定义到松手为止，完成点应改到松手帧，避免把约 2.1 秒静止保持计入技能。该修改属于重新标注，本次没有回写原始 Episode。

### 2. 自动 stale 裁切仍未触发

`stop_reason=duration_limit`，不是 `stale_tail`。主目标最后有效 Header 在约 13.46 s、最后有效接收约 13.56 s，之后持续 stale 至 15.10 s，尾部约 **1.64 s** 没有有效主目标；录制持续到 15.105 s 的时长上限，没有写入 `trimmed_tail_ns`。完成帧和完成前选中帧都在 stale 边界之前，且 `follower_next_state` 的动作来源是连续的从臂反馈，因此 stale 尾部没有进入本条已选 SFT 帧；但采集流程的自动裁切仍未按配置触发。

原因已定位：`serve()` 的 socket `accept()` 超时原为 **0.2 s**，而 `_terminal_stale_elapsed_ns()` 要求最后一次 stale 回调距 `status()` 不超过 **100 ms**。`rclpy.spin_once()` 和 `status()` 在同一个循环里，超时路径上的 `status()` 总是看到约 0.2 s 前的回调，100 ms 条件恒为 false；只有外部 `status` 请求恰好落在回调后 100 ms 内才可能触发。episode 9 的 stale 持续约 1.54 s，减去 1.0 s grace 只剩约 0.54 s 窗口，因此未命中。

已调整 `tools/episode_recorder/episode_recorder.py`：

- 将 socket 轮询周期从 0.2 s 收紧到 `RECORDER_STATUS_POLL_SEC = 0.05`，使 `status()` 在回调后约 50 ms 运行，低于 100 ms 新鲜度窗口。
- 将 100 ms 新鲜度提为 `LIVE_STALE_FRESHNESS_NS` 常量，并在测试中锁定“轮询周期必须小于新鲜度窗口”的不变量。
- 新增边界测试 `test_live_terminal_stale_requires_status_within_freshness_window` 和 `test_status_poll_period_is_shorter_than_live_stale_freshness`。
- `stop()` 新增 `auto_trim_live_monitor` 诊断字段，记录每臂的 `last_valid_age_ns`、`last_stale_age_ns`、`stale_count`，以及轮询周期、新鲜度窗口、grace 和实时计算的 `elapsed_ns`，便于下一条 Episode 直接确认触发条件。

录制器相关无 ROS 测试 100 项通过。录制服务已于 14:21 重启，新 PID `693934`，当前状态 `idle`，日志为 `/home/mips/pi05_datas/banana_task_episodes/recorder.20261112_142155.log`。episode 9 的原始 bag 不回写；下一次录制应确认 `stop_reason=stale_tail`、`trimmed_tail_ns>0`，且逻辑终点为最后一个有效主目标。

### 3. 顶部 D455 的 16.59 fps 已修正

本 Episode 记录时，顶部 D455 的中位帧间隔为 **60.286 ms（约 16.59 fps）**，左/右腕为 66.63 ms（约 15.01 fps）。当前选中帧的门限内最大跨相机偏差为 **39.037 ms**（p95 34.374 ms），非常接近 40 ms；更长 Episode 可能因顶部与腕部相位漂移超过门限。

原因已定位：D455 在 `640x480@15`、固定曝光、**standalone 且关闭深度**时，驱动虽然打开 15 fps profile，实际稳定输出 16.59 fps。对比实验：

| 顶部配置 | 实测 |
| --- | ---: |
| depth 关、standalone、固定曝光 80 µs | 16.59 fps |
| depth 开、standalone、固定曝光 80 µs | 16.59 fps |
| depth 关、主触发模式 | 16.59 fps |
| **depth 开、主触发模式（`inter_cam_sync_mode=1`、`output_trigger_enabled=true`、全局时间）** | **14.99 fps** |
| depth 开、standalone、自动曝光 | 14.99 fps |

因此仅打开深度或仅开自动曝光都不够，必须让 D455 以深度帧时钟作为触发主时钟。

`src/piper/launch/start_cameras.launch.py` 已新增 `top_fps_lock` 参数（默认 `true`）：

- 在 `camera_fps:=15` 时，顶部 D455 强制启用深度流，**不启用 `align_depth`**，并设置 `depth_module.inter_cam_sync_mode=1`、`depth_module.output_trigger_enabled=true`、`depth_module.global_time_enabled=true`、`rgb_camera.global_time_enabled=true`。
- 顶部深度只参与帧时钟，SFT 录制仍只录彩色；腕部参数不变。
- `enable_depth:=false` 仍可保持腕部无深度，但顶部会因 `top_fps_lock` 单独启用深度。需要完全关闭顶部深度时显式传 `top_fps_lock:=false`，此时顶部会回到 16.59 fps。
- `camera_sync:=external_trigger` 时外部触发参数会覆盖顶部主模式；三路仍需按硬件同步文档接线。

现场验证（2026-11-12 14:39 最终配置，日志 `cameras.20261112_143851.log`）：

- 顶部 **14.988 fps**，左腕 15.008 fps，右腕 15.008 fps；顶部稳态无超过 100 ms 的缺口。
- 15 秒重叠区间相位差：顶部—左腕中位 28.32 ms、p95 32.80 ms、最大 33.29 ms（210 帧中 0 帧 >40 ms）；顶部—右腕中位 8.41 ms、p95 17.37 ms、最大 18.46 ms。
- 当前相机 launch PID `698701`，顶部相机节点 PID `698823`。

**新的启动批次带来新的顶部内参。** 顶部 K 从 v1 的 `fx=388.6639, fy=388.0994` 变为 `fx=388.4529, fy=387.8886`（约 0.211 px），左/右腕不变。相机证据已按新批次重建：

- 快照：`config/collection_camera_info_snapshot.20261112_v2.json`（`camera_info_snapshot_20261112_v2`，SHA-256 `167305…`）
- 契约：`config/camera_training_contract.v4.20261112.json`（SHA-256 `4b58c4…`）
- 证据包：`/home/mips/pi05_datas/validation/camera_evidence_20261112_v8/`（顶部 14.988 fps、左腕/右腕 15.008 fps，三路 K 与快照逐项一致）
- 安装复核：`/home/mips/pi05_datas/validation/camera_mount_review_20261112_v8.json`
- 验收：`/home/mips/pi05_datas/validation/camera_training_acceptance_20261112_v8.json`（`training_camera_evidence_ready=true`）
- 状态：`config/camera_calibration_status.20261112_v8.json`
- 控制台默认 `calibration_version` 已切到 `camera_info_snapshot_20261112_v2`；桌面控制台已重启（PID `699245`）。

episode 9 本身仍绑定 v1/v7 证据，是在 16.59 fps 配置下录制的原始数据，不做追溯修改；它的 40 ms 审计结论不变。下一次录制应使用 v2 快照和 v8 证据，并在开始前确认顶部 topic 稳定在 15.0 fps。

## 复现

```bash
cd /home/mips/piper_ros/tools/episode_recorder

/tmp/piper-atomic-venv/bin/python audit_real_data.py \
  /home/mips/pi05_datas/banana_task_episodes \
  --episode-root /home/mips/pi05_datas/banana_task_episodes/episode_000009 \
  --gripper-calibration /home/mips/piper_ros/config/gripper_collection_calibration.20261112.json \
  --source-acceptance /home/mips/pi05_datas/validation/episode_fef43f49_source_acceptance_20261112.json \
  --action-source follower_next_state \
  --max-source-skew-ms 40 --extract-valid-segments \
  --output /home/mips/pi05_datas/validation/episode_fef43f49_source_audit_follower_20261112.json
```

`--extract-valid-segments` 只读提取 ready 后的连续有效段，不回写原始 Episode。去掉该选项时，开始前的 10 个网格点因相机尚未产生图像而被判为“采样网格前缺帧”，整条审计会失败；这 10 个点不属于本条技能轨迹。

## 数据集层面的缺口

- 全部已录制 Episode 的 `perturbation_id` 都是 `scene_001`；转换器 `group_split()` 在少于两个场景组时直接拒绝划分。
- 当前通过源审计的 Episode 只有 4 条（episode 5、6、7、9），全部是 `scene_001`，仍低于每个单臂技能至少 10 条训练、3 条测试原始 Episode 的要求。
- 没有生成 LeRobot v3 数据集，因此没有 `conversion_manifest.json`、`source_timestamps.jsonl` 和回读检查。
- 没有 `piper_atomic_skill_labels/1` 独立人工标注文件；`atomic_skill_sft.py` 还要求 `success`、`verified`、`atomic_episode`、`calibration_verified`、`units_verified`、`gripper_mapping_verified` 等字段。
- `training_normalization` 仍为 `not_computed`；训练统计必须从通过独立审计的 train 划分计算并绑定 checkpoint。
- `training_eligibility.eligible=false`，`training_ready=false`，`eligibility_scope=source_sft_export`。这些字段表示“源数据可进入导出流程”，不是训练资格。

## 证据文件

新增的 Episode 9 独立证据（原始 Episode 未修改）：

- 相机绑定：`/home/mips/pi05_datas/validation/episode_fef43f49_camera_binding_20261112.json`
- 从臂动作绑定：`/home/mips/pi05_datas/validation/episode_fef43f49_follower_action_binding_20261112.json`
- 源导出验收：`/home/mips/pi05_datas/validation/episode_fef43f49_source_acceptance_20261112.json`
- 40 ms 源审计：`/home/mips/pi05_datas/validation/episode_fef43f49_source_audit_follower_20261112.json`

相机侧证据：

- `/home/mips/pi05_datas/validation/camera_evidence_20261112_v7/`
- `/home/mips/pi05_datas/validation/camera_training_acceptance_20261112_v7.json`
- `/home/mips/pi05_datas/validation/camera_mount_review_20261112_v7.json`
- `config/camera_calibration_status.20261112_v7.json`
- `config/collection_camera_info_snapshot.20261112.json`
- `config/gripper_collection_calibration.20261112.json`

源导出验收的复核人为 ZCode 只读复核，不是新的现场人工确认。如果流程要求具名人工复核人，须由操作者补签。

## 下一步

1. 继续采集至少两个不同 `perturbation_id` 场景组的数据，并凑齐每个单臂技能 10 条训练、3 条测试原始 Episode。
2. 将 Episode 9 与其他合格 Episode 一起转换为 LeRobot v3，生成转换清单和逐帧来源时间戳。
3. 生成覆盖全部转换 Episode 的 `piper_atomic_skill_labels/1` 独立标注，再运行 `atomic_skill_sft.py` 门禁。
4. 若要减少零动作样本，优先在采集/标注阶段把完成点放在松手帧；不要回写已保存的原始 Episode。
5. 现场复验 stale 自动裁切：轮询周期已从 0.2 s 收紧到 0.05 s，下一条录制应出现 `stop_reason=stale_tail`。顶部 D455 已通过 `top_fps_lock` 固定在 15.0 fps；录制前确认顶部 topic 稳定在 15.0 fps，再采集下一条 SFT Episode。
