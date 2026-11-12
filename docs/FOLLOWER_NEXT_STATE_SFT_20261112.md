# 从臂反馈作为 SFT 动作（2026-11-12）

## 目的与定义

主从遥操作中，主臂给出绝对关节目标，从臂实际执行并回报反馈。此前转换器把 **主目标** 作为 `action`、把 **从臂反馈** 作为 `observation.state`。这会让动作标签与相机画面之间存在控制环的自然超前：相机看到的是从臂当前姿态，主目标可能对应之后才到达的位置。

本次新增独立动作来源 `follower_next_state`：

```text
observation.state = 从臂反馈(t)
action            = 从臂反馈(t+100ms)   # 10 Hz 下一时刻
observation.image = 相机(t)
```

模型学习 `s_t → s_{t+1}` 的状态转移。整段轨迹提供连续的转移序列，运动意图在序列中学习，不依赖主目标。`action` 仍为 7 维绝对关节位置（joint1～6 + gripper），夹爪使用 `/piper_vector/{arm}/slave_feedback` 的零点流。

这个模式和原有 `master_target` 模式并存，原路径行为不变。

## 代码改动

### 转换器

`to_lerobot.py` 新增：

```bash
--action-source follower_next_state
```

- `action_source=follower_next_state` 时，state 和 action 都取 `/piper_vector/{arm}/slave_feedback`；state 在网格时刻 `t`，action 在 `t + 1/fps`（10 Hz 为 100 ms）。
- 动作必须严格晚于同帧 state 的来源 Header，不能用同一反馈样本充当下一时刻动作。
- RGB、state 仍按原有 100 ms 时效和 40 ms 跨源偏差检查；action 单独按动作参考时刻检查时效，不参与同刻 RGB/state 的 40 ms 总体偏差（它本来就有 100 ms 前瞻）。
- 对齐策略写入 `causal_follower_next_state_v1`。
- 转换清单写入 `action_source`、`action_provenance=teleoperation_follower_next_state`、`action_offset_ns`。
- `source_timestamps.jsonl` 每帧新增 `action_target_timestamp_ns`、`action_source_header_timestamp_ns`、`action_source_can_stamps_ns` 和 `action_source_vector_quality`。
- 该模式不接受 `hold_segments`；从臂反馈本身就是连续动作来源。

### 审计

`audit_real_data.py` 新增同名 `--action-source follower_next_state`，并把 `action_source` 写入每条来源记录。审计的连续段、16 步窗口和完成帧选择都使用同一套 next-state 对齐。

### 录制器与桌面控制台

- 采集配置新增 `action_source`，可选 `master_target`（原有）或 `follower_next_state`。
- `follower_next_state` 时元数据写入 `action_provenance=teleoperation_follower_next_state` 和 `action_source=follower_next_state`。
- 完成帧校验读取 `task_metadata.action_source`；从臂反馈模式下按 action 的下一时刻 Header 过滤完成前帧，并使用 `causal_follower_next_state_v1` 对齐策略。
- `config/episode_recorder.pi05.json` 已设为 `follower_next_state`；示例配置保持 `master_target`。

### π0.5 训练门禁

`piper_pi05_openpi.py` 按转换清单的 `action_source` 选择期望来源：

- `master_target` → `human_demonstration`
- `follower_next_state` → `teleoperation_follower_next_state`

同时核对转换清单、独立来源证据、Episode 元数据和来源审计行中的来源一致。

## episode_000006 只读验收

Episode 6 的主目标在 ready 后 10.28～12.31 秒有 2.03 秒空档，按主目标定义没有有效完成段。从臂反馈全程连续：

- 768 条向量、12282 条原始反馈，0 stale、0 invalid、0 个超过 100 ms 的断档；
- next-state 对齐：154 个网格点、153 个有效帧、1 个连续段、138 个 16 步窗口；
- 当前完成标记 12.284 秒前选中 122 帧、107 个窗口，官方审计 `eligible=true`。

证据文件：

- 相机绑定：`/home/mips/pi05_datas/validation/episode_17ae4647_camera_binding_20261112.json`
- 从臂动作绑定：`/home/mips/pi05_datas/validation/episode_17ae4647_follower_action_binding_20261112.json`
- 源导出验收：`/home/mips/pi05_datas/validation/episode_17ae4647_source_acceptance_20261112.json`
- 从臂动作审计：`/home/mips/pi05_datas/validation/episode_17ae4647_source_audit_follower_20261112.json`

需要注意：12.284 秒的完成标记落在约 2.5 秒静止保持段内，此时 state 与下一时刻反馈完全相等。若技能在松手时结束，完成点应改到约 9.7 秒的松手帧；若保持段也是示教的一部分，则可以保留 12.284 秒，但样本会包含零动作保持。

## 复现命令

```bash
cd /home/mips/piper_ros/tools/episode_recorder

# 只读审计
/tmp/piper-atomic-venv/bin/python audit_real_data.py \
  /home/mips/pi05_datas/banana_task_episodes \
  --episode-root /home/mips/pi05_datas/banana_task_episodes/episode_000006 \
  --gripper-calibration /home/mips/piper_ros/config/gripper_collection_calibration.20261112.json \
  --source-acceptance /home/mips/pi05_datas/validation/episode_17ae4647_source_acceptance_20261112.json \
  --action-source follower_next_state \
  --max-source-skew-ms 40 --extract-valid-segments

# 转换（至少需要两个合格 Episode 才能划分 train/test）
/tmp/piper-atomic-venv/bin/python to_lerobot.py \
  /home/mips/pi05_datas/banana_task_episodes /path/to/output \
  --repo-id local/piper_follower --arm-mode LEFT --input-format can-vector \
  --action-source follower_next_state --gripper-calibration \
  /home/mips/piper_ros/config/gripper_collection_calibration.20261112.json \
  --fps 10 --max-age-ms 100 --max-camera-skew-ms 40 --max-source-skew-ms 40 \
  --success-sft --extract-valid-segments
```

## 边界

- 从臂反馈包含这台从臂和当前控制器的跟踪动态。若推理时更换控制器或负载，需要重新评估；它学习的是从臂实际运动，不是主臂指令意图。
- 仍须满足 π0.5 数据集门槛：每个单臂技能至少 10 条训练和 3 条测试原始示教 Episode，至少两个场景组。
- 相机来源验收只覆盖原始 RGB + 关节绝对位置路径；内参几何精度、手眼、外参和运动规划几何仍未验收。
- 本模式不改变原始 bag、Header、stale 标志或数值；所有新增信息只写入转换清单、来源时间戳和独立验收文件。
