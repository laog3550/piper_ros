# 保持意图的采集与 SFT 支持（2026-11-10）

## 目的

操作员完成抓取放置后常会先松开主臂，主端目标停止发送，但从臂仍需要保持最后目标一段时间。只延长录制超时或让程序自动进入“保持”状态，会把**传输故障**和**操作员意图**混为一谈。本方案要求操作员显式记录保持意图，并让转换器、审计和 16 步窗口只在这种证据存在时复用最后绝对目标。

**不能用 `stale` 推断保持。** `stale` 是来源时效标志；本次 `episode_000002` 的 89% stale 发生在操作员仍在操作的前几秒，是 CAN 录制链路故障，不是保持意图。把 stale 当保持会把坏数据洗成合格样本。

## 操作流程

1. 原子技能录制中，操作员准备停止移动主臂、但希望从臂继续保持最后目标时，点击桌面控制台的 **“开始保持（重复最后目标）”**。
2. 从臂反馈和三路相机必须继续有效；保持期间不要松开从臂使能或断开反馈。
3. 结束保持、准备松手或进入下一阶段时，点击 **“结束保持”**。
4. 单段保持默认上限 10 秒（`hold_max_sec`）。超时后转换器按上限截断，之后的帧不再使用保持证据。
5. 录制到时自动停止时，未结束的保持段会以 `end_reason=recording_stopped` 关闭。

按钮只在“录制中 + 当前技能非空”时可用；保持段属于当前技能。

## 录制字段

`episode.json` 新增：

```json
"hold_policy": {
  "source": "operator_annotation",
  "max_hold_sec": 10.0,
  "anchor_max_gap_sec": 2.0
},
"hold_segments": [
  {
    "hold_id": "…",
    "start_unix_ns": 1794299668000000000,
    "end_unix_ns": 1794299669500000000,
    "duration_ns": 1500000000,
    "source": "operator_annotation",
    "skill": "pick_and_place_banana_in_box",
    "intent": "operator_keep_last_target",
    "end_reason": "operator_release"
  }
]
```

同时保留对应的 `hold_start` / `hold_end` 标注记录。原始 bag、Header、`stale`、`valid` 和数值不改写。

## 转换器行为

`to_lerobot.py` 读取 `hold_segments` 后：

1. 仅当 `source=operator_annotation`、起止时间有效、段与段不重叠时才接受保持段。
2. 保持段内的动作帧使用**保持开始前最后一条有效主目标**作为动作标签；该锚点必须存在，且距保持开始不超过 `anchor_max_gap_sec`。
3. 保持段内过时向量的位置必须与锚点一致；不一致、缺失或时间异常时拒绝该帧，不伪造目标。
4. 从臂反馈、三路图像仍按原 100 ms 最大年龄和 30 ms 同步阈值校验；保持期间它们必须有效。
5. 原始 `stale` 标志保留在 `hold.stale_flags`，锚点 Header 写入 `source_timestamps.jsonl` 的 `header_timestamp_ns`，过时重复帧的 Header 写入 `hold.stale_header_ns`。
6. 保持动作不参与 RGB/状态/动作的总体来源偏差计算；非保持动作仍参与。左右臂状态同步仍须满足阈值。
7. 连续来源区间在存在有效锚点且未超 `max_hold_sec` 时包含保持段；超时或证据中断即切段，不能跨段拼接。
8. 16 步窗口把有证据的保持帧计入连续帧数；窗口可以跨越保持段，但不能跨越无证据的 stale 区。

导出清单和 `source_timestamps.jsonl` 每帧新增 `hold` 字段，未保持帧为 `null`。

## 审计与门禁

`audit_real_data.py` 使用同一 `read_episode` 路径，保持段的接受/拒绝与转换器一致；`_quality.hold_segments` 记录每段的 `aligned_frames`、`truncated` 和来源。

`atomic_skill_sft.py` 的来源时间门禁：

- 保持帧必须带 `hold_id`、`source=operator_annotation`、`arms` 和 `anchor_header_ns`；
- 导出动作 Header 必须等于锚点 Header，且不晚于帧目标时间；
- 被保持的臂从动作年龄、动作跨臂同步和总体来源偏差检查中排除；
- 从臂状态、RGB、原始 CAN 四帧内部偏差仍按原阈值检查；
- 缺少证据、锚点不一致或保持段外的旧动作仍按原规则拒绝。

## 不追溯

`episode_000001`、`episode_000004` 等历史数据没有显式 `hold_segments`，不能因为程序新增了保持支持就自动转为合格。它们仍按原 stale 规则审计。

## 仍须满足的条件

保持只解决“操作员希望继续保持最后目标”的动作标签问题，不解决：

- 原始 CAN 录制不足（见[录制负载修复](CAN_RECORDING_LOAD_FIX_20261110.md)）；
- 标定、夹爪映射、场景组划分和独立来源审计；
- 16 步窗口所需的连续有效帧数；
- 操作员松手前主端目标已经出现传输级空洞的情况。

只有在来源时效、连续性和保持证据同时满足时，数据才可能进入 π0.5 SFT。
