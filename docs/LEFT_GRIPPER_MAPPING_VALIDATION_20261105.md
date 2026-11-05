# 左侧夹爪采集映射核对（2026-11-05）

本次核对只监听 `can_left` 和顶部 RGB，不发送机械臂控制帧。操作员按原有 CAN 直连遥操作流程执行左夹爪闭合、张开及左臂小幅运动。原始目标帧与抽样图像保存在 `/home/mips/pi05_datas/validation/left_gripper_mapping_20261105/`；`target_can.jsonl` 的 SHA-256 为 `39426067ba0cf8ca2c42c41db7a1b4c42932110f7a3e7b088b12f79f3b80f026`。

| CAN ID | 关节/夹爪 | 帧数 | 到达间隔中位数 | 时间戳倒退 |
| --- | --- | ---: | ---: | ---: |
| `0x155` | 关节 1、2 | 3169 | 5.00 ms | 0 |
| `0x156` | 关节 3、4 | 3169 | 5.00 ms | 0 |
| `0x157` | 关节 5、6 | 3168 | 5.00 ms | 0 |
| `0x159` | 夹爪目标 | 3168 | 5.00 ms | 0 |

四类帧在停顿时同步停止发送，最长一次间隔为 `13.325 s`。因此向量组装器在没有新目标帧时标记 `stale` 是预期行为；不能将旧目标的 Header 时间冒充为当前采样时刻。后续训练导出必须显式处理这类停顿，不能仅放宽时间阈值。

`0x159` 按本机 Piper V2 协议以有符号 `int32 × 1e-6 m` 解码。该次操作的闭合附近原始值约 `-0.0027 m`，张开时最高 `0.0658 m`；开口增加对应数值增加。已保存的左臂香蕉入盒 Episode 中，在排除过期与时间不一致向量后，以 30 ms 内最近帧配对得到 824 组主目标/从反馈夹爪值，去除各自零点后相关系数为 `0.9732`，绝对差中位数为 `2.0 mm`。这些证据支持左侧主目标、从反馈的夹爪零点与正方向映射。`0.10 m` 是用户确认的项目上限，不代表本次操作实测到了 100 mm 开口。

只为左侧向量节点启用 `gripper_mapping_verified`；右侧本次未复验，保持 `false`。采集配置中的整体 `teleoperation.mapping_verified` 及 `gripper_mapping_verified` 仍保持 `false`，因为完整训练契约还缺少手眼标定、训练归一化、逐帧同步和成功技能标注。节点标志仅表示左侧夹爪采集映射已核对，不宣称 Episode 可用于 π0.5 训练。

启动命令：

```bash
ros2 launch piper start_readonly_can_collection.launch.py left_gripper_mapping_verified:=true
```

## 重录复审

左侧参数启用后，操作员重录并保存了 `/home/mips/pi05_datas/banana_task_episodes/pick_banana_into_box_left/episode_000002/`。该 Episode 在 `20.070 s` 自动停止，三路 RGB 的 Header 无倒退或估算缺帧；左侧四类主目标 CAN ID 各有 `2616` 帧，时间戳无倒退，中位间隔 `5.00 ms`。四类帧仍会在停顿期同步暂停，最长间隔约 `212 ms`。

左侧从臂反馈向量 `1062/1062` 条 `valid=true`；主臂目标在录制时间范围内 `963` 条，其中 `761` 条 `valid=true`、`199` 条标记 `stale`、`4` 条标记 `time_inconsistent`。这表明映射参数已经生效，剩余无效值主要来自无新目标帧时的重复发布。按 10 Hz、源年龄不超过 `100 ms`、RGB/目标/反馈 Header 总体偏差不超过 `30 ms` 的只读试算，`156` 个候选时刻中约 `120` 个可对齐，最长连续段 `52` 帧；本结果不是正式 LeRobot 导出。

本条 `annotations=[]`。操作员确认录制中未点击技能完成与成功标记；保存时的 `outcome=success` 不能代替带相机 Header 引用的完成帧。`task_metadata` 仍使用原始采集版本，未声明经验收的标定、训练归一化或示教动作来源。只读审计结果为 `eligible=false`、合格训练帧 `0`，不得用于 π0.5 微调。复审文件保存于 `/home/mips/pi05_datas/validation/episode_000002_audit_20261105.json`。
