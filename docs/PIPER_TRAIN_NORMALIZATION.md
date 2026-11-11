# Piper 训练统计的来源与 checkpoint 绑定

原始 Episode 的 `data_convention_version` 表示字段、顺序和单位；训练专用的 `normalization_version` 表示实际计算的 `state/actions` 统计资产。新采集的 Episode 可以没有 `normalization_version`，转换清单对应值可以为 `null`。OpenPI 门禁只用 `data_convention_version` 核对原始版本，不要求把统计结果倒填进 Episode 或转换清单，也不使用原始数据约定代替训练统计。

该流程由 `tools/vla_contract/piper_pi05_openpi.py` 执行，只读来源 Episode、转换目录和既有证据；生成物写入新的工作目录。不会因为采集声明齐全、转换完成或 `eligible=true` 就自动批准训练。`eligible=true` 必须来自绑定当前来源的独立审计，正式训练另须独立标定、示教来源、夹爪映射和统计核验证据。

## 1. 核准来源数据

来源核验文件须包含：

- `status=verified_for_piper_pi05_source_data`。
- `conversion_manifest_sha256`、`source_audit_sha256`，绑定当前转换清单和审计报告。
- `arm_mode`、`action_space=absolute_joint_position_7`、`action_provenance=human_demonstration`。
- `calibration_verified=true`、`gripper_mapping_verified=true`，及与所有来源匹配的 `calibration_version`、`data_convention_version`。这些声明须有独立验收依据，不能仅修改布尔值。
- `episode_skills`，逐导出 Episode 标注 `banana_in_box`。

审计必须核准全部来源，训练/测试场景组隔离，来源文件 SHA-256 与转换清单一致。每段至少 16 帧，单臂入口最低要求 10 条训练和 3 条测试**原始示教 Episode**。多个连续片段仍按原始来源计数，不能用同一次示教拆出的片段凑数量。导出片段通过 `source_episode_id` 关联原始 Episode 和审计行，`episode_skills` 仍以导出片段 ID 为键。

## 2. 仅从 train 计算统计

```bash
PYTHONPATH=/path/to/lerobot-v3/lib/python3.11/site-packages:tools/vla_contract \
  /path/to/openpi/.venv/bin/python tools/vla_contract/piper_pi05_openpi.py \
  --openpi-root /path/to/openpi --base-checkpoint /path/to/pi05_droid \
  --dataset /path/to/validated_lerobot \
  --evidence /path/to/source_evidence.json --audit-report /path/to/source_audit.json \
  --lerobot-v3-site-packages /path/to/lerobot-v3/lib/python3.11/site-packages \
  --work-dir /path/to/new_normalization_run \
  --prepare-normalization --normalization-version piper_train_stats_v1
```

工作目录必须为空，并位于转换目录和所有原始 Episode 之外。程序仅把已审计数据的 `train` 目录映射为 OpenPI 的 `local/piper_train`，固定统计批量为 `1`，使用固定上游 commit 的统计脚本；不把 `test` 或 `validation` 交给统计读取器，也不传入截断帧数。上游统计格式是 `norm_stats.json` 中的 `state/actions`，各含 `mean/std/q01/q99`；Piper 入口要求每项为 7 维有限数，并检查标准差及分位数范围。格式依据固定版本的[统计脚本](https://github.com/Physical-Intelligence/openpi/blob/215abfb217dbac7d5f1273282331b9b1866c0479/scripts/compute_norm_stats.py)及[统计序列化实现](https://github.com/Physical-Intelligence/openpi/blob/215abfb217dbac7d5f1273282331b9b1866c0479/src/openpi/shared/normalize.py)。

输出包含：

- `assets/pi05_piper_absolute_7/local/piper_train/norm_stats.json`：实际统计资产。
- `normalization_provenance.json`：`schema_version=piper_train_normalization/1`、统计版本和 SHA-256、`source_split=train`、固定 OpenPI commit、原始数据约定版本、转换及审计 SHA-256、`train` 目录逐文件摘要汇总、精确的训练 Episode/片段列表及其来源 SHA-256、帧数与场景组。
- 终端状态 `normalization_computed_pending_independent_review`：计算完成，仍待独立核验；不产生 checkpoint 或训练记录。

计算完成后会重新核对来源和证据。计算期间数据、清单或审计发生变化即拒绝出具统计来源记录。

## 3. 独立核验统计，再训练

核验者检查统计来源、数值、训练划分及资产内容后，在独立证据文件中保留上述来源核验字段，并补充以下字段。示例中的 SHA-256 必须替换为实际计算值：

```json
{
  "status": "verified_for_piper_pi05_training",
  "normalization_version": "piper_train_stats_v1",
  "normalization_verified": true,
  "normalization": {
    "path": "/path/to/new_normalization_run/assets/pi05_piper_absolute_7/local/piper_train/norm_stats.json",
    "sha256": "<统计资产的实际 SHA-256>"
  },
  "normalization_provenance": {
    "path": "/path/to/new_normalization_run/normalization_provenance.json",
    "sha256": "<统计来源记录的实际 SHA-256>"
  }
}
```

默认预检和 `--train` 都要求完整的独立统计证据。只批准来源数据的 `verified_for_piper_pi05_source_data` 不能用于训练。原始版本冒充统计版本、统计文件缺失、7 维统计无效、任一 SHA-256 不符、来源列表夹入测试 Episode、训练目录或审计发生变化，均拒绝。

训练命令沿用准备命令的环境和数据路径，改用核准后的证据文件、另一个空工作目录，并将 `--prepare-normalization --normalization-version ...` 替换为 `--train`。正式训练复制并校验已核准统计资产，**不重新计算**统计。两种模式互斥，统计版本只在准备阶段指定。

训练完成后核对 checkpoint 的 `assets/local/piper_train/norm_stats.json`，要求其统计数值与已核准资产一致；允许上游 JSON 重新序列化造成字节格式差异。`training_record.json` 同时保存统计版本、原资产与来源记录的 SHA-256、训练划分摘要、checkpoint 内统计路径及 SHA-256、全部 checkpoint 文件摘要。上游 checkpoint 资产路径依据固定版本的[保存实现](https://github.com/Physical-Intelligence/openpi/blob/215abfb217dbac7d5f1273282331b9b1866c0479/src/openpi/training/checkpoints.py)。

训练记录状态仍为 `trained_unvalidated`。推理加载须通过独立离线评估，并重新核对 checkpoint 全部文件摘要及统计绑定；缺少这条证据链的旧训练记录不会被自动放行。该离线门禁不代表真机执行许可。

## 合成回归

```bash
python3 -m unittest discover -s tools/vla_contract -p 'test_piper*.py' -v
```

测试仅用合成来源、统计和 checkpoint 文件验证门禁与阶段调用；不会运行真实模型微调，也不证明当前真实数据已达到训练条件。
