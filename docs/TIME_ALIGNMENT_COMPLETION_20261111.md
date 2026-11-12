# 时间对齐与完成帧有效性筛选（2026-11-11）

## 对齐规则

`can-vector` 使用 `causal_rgb_bundle_v1`。导出仍保留固定 10 fps 采样网格，原始消息 Header、CAN 来源时间、数值和质量标志均不改写。

1. 在每个网格时刻之前寻找最近的同步 RGB 图像组；只使用已经发生的图像，组内 Header 偏差不超过指定相机同步阈值。
2. 以图像组中最晚的真实 Header 为参考时刻，为每侧动作和状态选择该参考时刻之前最近的向量。
3. RGB、状态和非保持动作的 Header 总体偏差仍不超过 30 ms；每个非保持来源相对原采样网格的年龄仍不超过 100 ms。向量内部四帧 CAN 偏差继续检查。延后或前移采样参考不能豁免这些检查。
4. 图像组参考与网格须属于同一个连续有效来源区间。原始 stale、missing、时间异常或来源缺口仍切段；不能退回中断前的旧图像组掩盖中断。
5. 图像组参考必须逐帧递增。同一旧图像组不能重复计入连续示教，也不能靠重复帧凑满 16 步。
6. 显式保持段仍按原 `hold_segments` 证据处理，保持锚点不得晚于图像组参考；普通 stale 不转为保持。

这一规则把真实同步的观察与动作组合重新采样到固定网格。各路真实 Header 可以呈现 15 fps 的间隔；固定 10 fps 指网格间隔，不代表原始图像或动作 Header 被改成 100 ms 间隔。

`source_timestamps.jsonl` 每帧新增：

```json
"alignment": {
  "policy": "causal_rgb_bundle_v1",
  "reference_timestamp_ns": 1794379530000000000,
  "grid_lag_ns": 40000000
}
```

`target_timestamp_ns` 保留原网格时刻；`header_timestamp_ns` 和 `source_can_stamps_ns` 保留被采用消息的原始来源时间。`conversion_manifest.json` 记录 `alignment_policy`，审计质量记录 `max_grid_lag_ns`。训练门禁同时核对图像组参考、实际来源的因果顺序、网格年龄与总体偏差；声明此策略却缺失对齐依据时拒绝。

## 完成帧筛选

完整 CAN 向量采集的原子技能在停止后通过 `completion_validation.py` 复用转换器的同一对齐与切段实现。录制端用原生 ROS 解码器只读读取已关闭的 SQLite bag，不依赖安装 LeRobot 或 rosbags；该检查不授予独立标定或训练资格。

- 可靠末尾 stale 边界存在时，只考虑最后有效主目标 Header 之前、距离边界不超过 100 ms 的真实图像；没有此边界时，检查停止请求之前 100 ms 的候选。
- 候选完成图像必须实际参与该时刻的 RGB 对齐，不能改用更早的另一张图像来通过检查。
- 候选到结束边界之间不能跨越来源中断；候选必须属于一个有效连续片段，且完成前该片段至少有 16 个有效网格帧。
- 自动候选不合格时写入 `completion_review_required=true` 和 `auto_completion_error`，不自动写入成功完成标记，也不回退到遥远的旧片段。
- 录制中手动点击的标记在停止后执行同样的检查；失败记录保留在 `unverified_completion_attempts`。停止后的人工补标也检查所选精确图像，不能绕过来源时效和 16 步要求。
- 合格标注附带 `source_validation`，记录候选 Header、选择边界、回退距离、对齐策略、片段号、连续帧数和窗口数。成功保存核对该证明与标注的图像引用一致。只有彩色图像、没有完整 CAN 向量的采集保持原图像存在性检查，不声明已通过此来源校验。

程序只筛选数据来源有效性，任务结果仍由操作员确认。原子技能的连续数据不足时，不通过拼接其他技能阶段或延用过时目标构造成功 SFT。

## episode_000004 的只读复验

来源为 `/home/mips/pi05_datas/banana_task_episodes/episode_000004`，Episode ID 为 `323565ad227f40b0942095620ec6de25`。

| 项目 | 修复前 | 修复后 |
| --- | --- | --- |
| 10 fps 网格点 | 164 | 164 |
| 对齐通过 | 18 | 123 |
| 最长连续片段 | 1 帧 | 44 帧 |
| 至少 16 步的候选片段 | 0 | 3 |
| 完整 16 步候选窗口 | 0 | 40 |
| 原完成标记可用于成功 SFT | 否 | 否 |

改动后最大网格到图像组参考延迟约 66.65 ms；所有采用来源仍受 100 ms 年龄和 30 ms 偏差限制。原完成图像所在位置存在真实目标中断，自动筛选和人工精确补标均拒绝，其他有效片段不继承成功标签。该 Episode 仍待独立相机验收绑定；本次没有生成通过的验收记录。

检查前后原始来源组合 SHA-256 均为 `d2da1ce152e1ce49daf4f068c7b495c6f3ab3090857ed8a3d8ff90d7a1636961`。没有修改历史 Episode。复验输出位于 `/tmp/piper_time_alignment_fix_20261111/`，临时输出不视为长期归档。

## 验证与应用

转换、真实 LeRobot 写入回读、录制标注、异相位采样、来源中断与训练门禁共 `101` 项回归通过。合成用例同时覆盖正常尾部停顿可保留合格完成帧、真实空洞拒绝自动标记、人工补标不能绕过检查、重复图像组不计帧、参考时刻晚于来源的因果约束及网格年龄超窗拒绝。

```bash
source /opt/ros/humble/setup.bash
source /home/mips/piper_ros/install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /tmp/piper-atomic-venv/bin/python -m pytest -q \
  tools/episode_recorder/test_to_lerobot.py \
  tools/episode_recorder/test_atomic_skill_export.py \
  tools/episode_recorder/test_temporal_alignment.py \
  tools/episode_recorder/test_recorder_annotations.py \
  tools/vla_contract/test_atomic_skill_sft.py
```

当前录制服务已在空闲、无待处理 Episode 时重启并生效；CAN 节点和相机没有重启。系统 Python 在未安装 rosbags 的环境中完成真实 bag 的来源检查，拒绝原因与离线复验一致。
