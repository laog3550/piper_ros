# 真实 Episode 逐帧来源诊断（2026-11-06）

## 方法

使用 [`audit_frame_timeline.py`](../tools/episode_recorder/audit_frame_timeline.py) 只读扫描 rosbag2，在主目标与录制区间的交集上按 `10 Hz` 建立诊断时刻。每个时刻选择不晚于该时刻的顶部 RGB、执行臂腕部 RGB、主目标与从反馈，逐项记录 Header 龄、RGB 格式、CAN ID 与来源话题、四个 CAN 源时间、`missing/stale/time_inconsistent/valid` 和跨源时间差。默认阈值为单源最大龄 `100 ms`、跨源 Header 最大跨度 `30 ms`、向量内部 CAN 源最大跨度 `30 ms`。

诊断输出 `summary.json` 与逐帧 `frames.jsonl`；`sensor_alignment_pass` 只表示这些传感器检查通过，**不授予训练资格**。标注、相机标定、夹爪物理映射、训练归一化和 OpenPI 读取器另行验收。报告绑定 `episode.json` 与 `bag/metadata.yaml` 的 SHA-256；未计算大体积 `.db3` 全文件摘要，保管原始 bag 时应另外做完整归档校验。

## 结果

| 来源 | 10 Hz 诊断时刻 | 传感器检查通过 | 主要阻断 |
| --- | ---: | ---: | --- |
| `/tmp/piper_final_recovered_saved_20261104/episode_000001` | 413 | 0 | 当时夹爪映射标志未启用；主目标在 330 个时刻过期；329 个时刻跨源偏差超限。 |
| `/tmp/piper_final_recovered_saved_20261104/episode_000002` | 257 | 0 | 当时夹爪映射标志未启用；主目标在 209 个时刻过期；194 个时刻跨源偏差超限，顶部 RGB 在 bag 顺序中有 21 次 Header 非递增。 |
| 回收站中的 `episode_000003` | 193 | 162 | 31 个时刻主目标 `stale`；25 个时刻目标超龄或越界；19 个时刻跨源偏差超限。两路 RGB 各 581 帧，均无 Header 非递增，最大间隔约 `33.4 ms`。 |

回收站来源为 `/home/mips/.local/share/Trash/files/episode_000003`，本次仅只读分析，未恢复或修改。用户提供的 `/home/pi05_datas/banana_task_episodes` 在本机不存在；`/home/mips/pi05_datas/banana_task_episodes` 当前没有 `episode.json`。如有其他挂载目录，需用实际绝对路径重跑。

`episode_000003` 的 162 个传感器合格时刻不能视为 162 帧训练样本：原始 Episode 使用暂定的相机与数据约定版本，缺少经核验的完成标注及动作来源声明；正式来源审计仍给出 0 条合格 Episode、0 帧合格训练数据。停顿期目标帧重复发布并标 `stale` 是向量节点的正常质量信号，不能把旧目标 Header 改写为当前时间，也不能简单删除失败行后宣称序列连续。后续转换器若支持有效连续片段，必须保留片段边界、原始 Header 和训练窗口长度约束。

## 复现

```bash
cd /home/mips/piper_ros
/tmp/piper-collection-audit-venv/bin/python tools/episode_recorder/audit_frame_timeline.py \
  /tmp/piper_final_recovered_saved_20261104/episode_000002 \
  --arm-mode LEFT --fps 10 --max-age-ms 100 --max-source-skew-ms 30 \
  --max-can-skew-ms 30 --output-dir /tmp/piper_frame_audit_recheck_000002
```

输出目录须为空。逐帧报告通过 `reasons` 指明失败来源；无需修改原始 Episode。对回收站数据进行复验时，也应保持只读，不将其混入当前有效采集目录。
