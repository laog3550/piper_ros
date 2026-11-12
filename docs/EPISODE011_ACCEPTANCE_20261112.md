# episode_000011 的 SFT 数据检查（2026-11-12）

## 结论

> 2026-11-12 补充更正：后续在 Episode 12 排查到完成确认时间域混用错误，Episode 11 也能复现。它的候选图像 Header 在逻辑终点前约 24.857 ms，但接收晚于终点约 10.919 ms，旧确认逻辑会误判图像不存在。因此本文“缺少确认”仅指保存文件没有落盘标注，不能据此断言操作者没点击。故障与修复见 [Episode 12 检查报告](EPISODE012_ACCEPTANCE_20261112.md)。

**采集轨迹与自动完成候选的技术检查通过，但当前仍不满足成功 SFT 源导出资格。** 本次没有再出现 episode_000010 的自动候选 RGB 同步失败；主要人工缺口是没有确认完成候选。元数据只有 `outcome=success`，没有 `skill_complete`。另缺本条与独立相机来源验收之间的正式绑定，不能仅凭内参版本一致自动授予资格。

本条可以保留，不需要因为相机时序问题直接重采。后续应回放核实实际完成状态，进行有来源记录的补标和逐条相机来源绑定，再重跑审计。它已经保存，本次没有将其他 Episode 的验收或成功标注移植过来，也没有回写原始元数据。

## 来源和方法

- 路径：`/home/mips/pi05_datas/banana_task_episodes/episode_000011`
- Episode ID：`8bf436f6ff454f5194a79d3cb65a624a`
- 技能：`pick_and_place_banana_in_box`，执行臂 `LEFT`，场景 `scene_001`
- 动作来源：`follower_next_state`，从臂当前反馈为 state，下一时刻反馈为 action。
- 规则：10 fps；来源年龄 100 ms；CAN 偏差 30 ms；RGB 与总体来源偏差 40 ms；允许提取连续片段。
- 夹爪：显式加载录制时绑定的 `gripper_collection_calibration.20261112.json`，设置哈希匹配。
- 只读复核采用实际 `audit_native`、`read_episode`、`select_sft_frames`，另独立核对 CAN 六关节字节映射与精确来源引用。

## 通过项

| 项目 | 结果 |
| --- | --- |
| 启动覆盖 | 新启动门禁有 7 路实际新鲜来源和首批落盘证据；三路 RGB 均在声明 ready 前已有图像 |
| ready 时刻 | `1794468907330185573` |
| 自动停止 | `stop_reason=stale_tail`，逻辑时长约 11.190 s |
| 尾部裁切 | 1.498887668 s；原始 bag 尾部保留 |
| 整段对齐 | 113 帧，1 个连续段，98 个 16 步窗口 |
| 自动候选之前的可选数据 | 112 帧，97 个完整窗口；仅为候选检查，未取得人工成功标注 |
| 候选实际 Header | 顶部 RGB `1794468918495501709`，相对 ready **11.165316136 s** |
| 界面候选偏移 | 11.201 s，来自接收时刻，不能替代图像真实 Header 时刻 |
| 候选图像组偏差 | 28.008709 ms，小于 40 ms |
| 整段最大 RGB / 总体来源偏差 | 39.877904 ms，小于 40 ms |
| 候选所属连续段 | 第 0 段，候选与结束边界之间没有来源中断 |
| 左从臂反馈向量 | 680 条，0 六关节 stale，0 Header 倒退，2040/2040 六关节来源引用匹配 |
| 112 帧状态与下一状态来源 | 去重后 860/860 个原始 CAN 引用匹配，无缺失 |
| state → action Header 差 | 99.892139～100.189448 ms，动作严格晚于当前状态 |
| 独立六关节解码 | 无数值/顺序/单位解码错误，来源键无重复或冲突 |
| 左侧逻辑录制区间诊断 | 62 条，未标记采集失败；队列溢出、发布失败、退出丢帧均为 0 |
| 三路 RGB | 左腕 189、右腕 190、顶部 205；记录中无 Header 倒退或估计缺帧 |
| 三路 CameraInfo | 左腕 189、右腕 190、顶部 204；各只有一个 profile，与 v2 快照逐项一致 |
| 存储 | WAL；SQLite `quick_check=ok` |

采样网格共 124 点，前 11 个拒绝点位于 ready 之前的启动覆盖区间，不能解释为技能中途断流。113 帧有效段包括一个 ready 前约 17 ms 的预录制网格点；112 帧候选数据网格范围为 ready 后 -0.017040317～11.082959683 s。预录制仍使用真实来源，不填造数据。

右主目标为非执行侧，本条没有原始新目标且其向量 stale，不用于单左臂动作。左主目标全包有 182 条六关节 stale，包含起始静止和末尾停发；本条使用连续从臂反馈作为动作来源，不能把该全包主目标计数当作训练动作断流。

`zero_action_tail_frames=0` 只表示最后没有 action 与 state 全部逐元素完全相等的连续尾段；真实反馈有微小变化，不能据此断言不存在视觉静止保持。

## 未通过项

### 完成候选尚未人工确认

`annotations` 只有一条保存时的成功结果：

```json
{"kind":"outcome","value":"success","annotation_origin":"save_selection"}
```

没有 `skill_complete`，`completion_review_required=true`，`completion_validation.status=completion_pending`。新程序已经生成合格候选，`source_validation_error=null`、`contiguous_frames=112`；问题在于候选没有被确认，保存“成功”结果不会代替完成帧确认。

正式审计拒绝原因包括“缺少唯一且匹配 skill 的完成标注”。本次仅使用候选进行只读技术核对，不将它改写成已确认标注。

### 本条相机来源尚未完成独立验收绑定

元数据内 `calibration` 仍为 `provisional`。本次三路 CameraInfo 与 `camera_info_snapshot_20261112_v2` 的一致性检查通过，但尚未提供覆盖本条摘要、发布者身份、设备与安装批次的逐条来源验收记录。因此正式审计另报“calibration 尚未验收”及相应契约阻塞。

这不是要求再次改写相机内参，而是补齐独立证据与本条原始数据之间的绑定。不能将其他 Episode 的 source acceptance 直接拿来使用。

### 40 ms 门限余量较小

整段最大偏差 39.877904 ms，距门限约 0.122 ms。本条通过该项，但其他相位或更长录制仍可能越界。本次不放宽门限；应继续监测图像组偏差，而不是认为新候选选择消除了真实拍摄相位差。

## 画面复核

真实起始顶部图像显示香蕉位于盒外；候选顶部和左腕画面显示香蕉已位于盒内。画面支持本次轨迹覆盖入盒过程。夹爪仍靠近香蕉，操作者仍须按技能定义确认是否完成松手以及是否要求撤离，不由技术校验或 stale 自动代替语义确认。

提取的原图位于证据目录：`start_top.png`、`grasp_top.png`、`middle_top.png`、`placing_top.png`、`last_training_top.png`、`unconfirmed_completion_top.png`、`unconfirmed_completion_wrist.png`。

## 训练集边界与证据

当前审计 `eligible=false`、`training_ready=false`。即便补齐本条来源验收，它也只是 `scene_001` 的一条来源 Episode；数据集数量、独立场景划分、转换标注和 train 归一化仍须另行验收。

只读检查前后来源组合摘要一致：

```text
bd48d4ffc7663c40dabce54e9b5089dc14df694a31a47cabce82ac8328cfd930
```

证据目录：`/home/mips/pi05_datas/validation/episode_000011_check_20261112/`。包含 `check_episode.py`、`audit.json`、`details.json`、`check_sources.py`、`sources.json`、`joint_mapping.json` 和原图。没有修改原始 bag、Header、stale 标志、完成帧或任务结果。
