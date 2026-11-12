# episode_000013 的 SFT 来源验收（2026-11-12）

## 结论

**单条来源数据合格，可作为 SFT 的来源样本：带独立证据绑定的源审计 `eligible=true`、拒绝原因为空，可选 104 帧、89 个 16 步窗口。** 本条完成确认已经实际落盘，先前的确认时间域错误本次没有再阻止标注。

该结论适用于原始 RGB 与从臂关节绝对位置、`follower_next_state`、当前 40 ms 来源同步规则。它不代表整个训练集已经合格，也不授予几何用途或训练 checkpoint 资格。独立验收文件保存在原始 Episode 外，原始元数据的训练待审声明未改写。

## 来源与方法

- 路径：`/home/mips/pi05_datas/banana_task_episodes/episode_000013`
- Episode ID：`7f12b24dccfc44e6b16e83b8e1745cdd`
- 技能：`pick_and_place_banana_in_box`，执行臂 `LEFT`，场景 `scene_001`
- 动作：当前从臂反馈为 state，约 100 ms 后从臂反馈为 action。
- 规则：10 fps，来源年龄 100 ms，CAN 偏差 30 ms，RGB/总体来源偏差 40 ms，至少 16 步连续窗口。
- 夹爪：显式使用录制时绑定的 `gripper_collection_calibration.20261112.json`，校验设置哈希。
- 工具：实际 `audit_native`、`read_episode`、`select_sft_frames`，另以独立有符号 CAN 解码检查关节值、顺序及精确来源。

## 通过项

| 检查项 | 结果 |
| --- | --- |
| 保存结果与完成标注 | `success`；唯一技能匹配 `skill_complete`；`operator_confirmed_candidate`，`verified_in_bag=true` |
| 完成帧门禁 | `sft_frame_gate_passed`、`frame_gate_passed=true` |
| 完成图像 | 顶部 RGB Header `1794470735170586670`，实际存在于 bag |
| 完成图像实际时间 | ready 后 10.310147566 s；界面 10.346 s 为接收时刻偏移，两者分开保留 |
| 逻辑任务时长 | 约 10.339 s，`stop_reason=stale_tail` |
| 尾部裁切 | 1.479158973 s，原始 bag 尾部保留 |
| 全部有效轨迹 | 105 帧、1 个连续段、90 个完整窗口 |
| 确认完成前选中 | **104 帧、89 个完整窗口** |
| 完成图像组 RGB 偏差 | 0.697670 ms |
| 完成图像组总体来源偏差 | 18.779124 ms |
| 整段最大 RGB 偏差 | 13.438100 ms，低于 40 ms |
| 整段最大总体来源偏差 | 19.943872 ms，低于 40 ms |
| 左从臂反馈 | 637 条向量，0 六关节 stale，0 Header 倒退；1911/1911 六关节来源引用匹配 |
| 所选状态和下一状态原始 CAN 引用 | 去重后 **832/832 匹配**，无缺失 |
| 下一状态时间差 | action 晚于 state 99.886417～100.105285 ms |
| 独立 CAN 字节审计 | 无解码、顺序或数值错误；来源键无重复或冲突 |
| 左侧逻辑录制区间诊断 | 57 条，无采集失败；溢出、发布失败、退出丢帧均为 0 |
| 三路 RGB | 左腕 176、右腕 176、顶部 191；无 Header 倒退或估计缺帧 |
| 三路 CameraInfo | 左腕 176、右腕 176、顶部 190；各 1 个稳定 profile，与 v2 快照和 v8 验收采集逐项一致 |
| 存储 | WAL，SQLite `quick_check=ok` |
| 最终独立源审计 | `eligible=true`，`eligible_frame_count=104`，`training_ready=false` |

115 个原网格点中，启动覆盖前的 10 点被排除；105 个有效点中有少量真实预录制。所选 104 个网格范围为 ready 后 -0.129149241～10.170850759 s，不填造帧。主目标全包的起始静止、末尾停发 stale 不作为本条从臂反馈动作的断流；右主目标是非执行侧，不用于动作。

## 相机独立证据绑定

首次不传独立验收文件的审计仍报告相机 `calibration` 待验收。随后完成逐项核对，并新增本条独立绑定，没有修改原始 Episode，也没有仅凭版本号消除门禁：

1. 使用 `readonly_camera_evidence.load_package` 校验 v8 证据包所有文件哈希、冻结契约、内参基线、采集器和原图，重新计算已有安装复核对应的相机验收；结果与已验收文件一致。
2. 三路只读设备信息服务返回序列号：左腕 `CV27561000MR`、右腕 `CV275610002H`、顶部 `338122301303`；当前 USB 身份与冻结映射一致。
3. 当前 RGB/CameraInfo 发布者唯一，GID 与 Episode ready 时记录一致；三路驱动进程均早于 v8 证据采集启动，进程连续性成立。
4. 逐条比较本 Episode 全部 CameraInfo，包括尺寸、frame_id、畸变、K/R/P、binning 和 ROI，与 v8 已验收采集及 v2 冻结快照一致。
5. 新增的绑定明确标注为 **Codex 只读自动复核，非新的现场人工安装确认**；沿用既有 v8 安装批次和原始 RGB/关节来源范围。不声明重新测得几何标定、手眼或跨相机外参精度。

新增资产：

- `/home/mips/pi05_datas/validation/episode_7f12b24d_camera_binding_20261112.json`
- `/home/mips/pi05_datas/validation/episode_7f12b24d_source_acceptance_20261112.json`
- `/home/mips/pi05_datas/validation/episode_000013_check_20261112/live_camera_identity.json`
- `/home/mips/pi05_datas/validation/episode_000013_check_20261112/audit_with_source_evidence.json`

源验收只覆盖已核对的本条摘要、执行臂和版本，不能直接用于其他 Episode。如果部署流程另外要求具名现场复核人，本自动复核不能代签。

## 画面与技能范围

实际起始顶部图像显示香蕉在盒外；候选顶部和左腕画面显示香蕉位于盒底，夹爪仍在附近。确认标注来自操作者，且与本次选中图像相同，连续轨迹覆盖起始至入盒过程。它支持本条“抓起香蕉并放入盒中”的采集结果；若未来技能定义另外要求夹爪撤离，应单独确定终态定义，不能自动扩展本次标注含义。

已提取 `start_top.png`、`grasp_top.png`、`middle_top.png`、`placing_top.png`、`confirmed_completion_top.png`、`confirmed_completion_wrist.png` 和三路首帧供回放核对。

## 训练集边界

本条可以进入来源样本池。它仍是 `scene_001` 的单条 Episode，不能单独满足训练/测试数量与场景组划分。LeRobot 转换、独立训练标注、仅从合格 train 划分计算的归一化及 checkpoint 绑定仍须在数据集阶段验收。源审计的 `training_ready=false` 与本条来源 `eligible=true` 不冲突。

## 复现与完整性

```bash
cd /home/mips/piper_ros
/tmp/piper-atomic-venv/bin/python tools/episode_recorder/audit_real_data.py \
  /home/mips/pi05_datas/banana_task_episodes \
  --episode-root /home/mips/pi05_datas/banana_task_episodes/episode_000013 \
  --gripper-calibration /home/mips/piper_ros/config/gripper_collection_calibration.20261112.json \
  --source-acceptance /home/mips/pi05_datas/validation/episode_7f12b24d_source_acceptance_20261112.json \
  --action-source follower_next_state --max-camera-skew-ms 40 \
  --max-source-skew-ms 40 --extract-valid-segments \
  --output /tmp/episode013_source_audit_recheck.json
```

只读检查前后来源组合摘要均为：

```text
e8f6c9bec5b26a3b0c919527f9136e0b27b07ca443a73bd2c5fe934ce620ae63
```

完整证据位于 `/home/mips/pi05_datas/validation/episode_000013_check_20261112/`，含原始门禁审计、独立 CAN 审计、来源引用、原图、身份核对、绑定脚本与最终带证据审计。原始 bag、Header、任务结果及已确认标注均保持不变；未重启采集服务或更改设备参数。
