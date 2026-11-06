# 右侧夹爪映射现场验收（2026-11-06）

## 来源与结论

操作员使用原有主从直连遥操作完成右夹爪闭合、张开、回闭合；ROS 只读旁路同录右侧主目标、从反馈、向量、右腕 RGB、顶部 RGB 与顶部 `CameraInfo`。原始 bag 位于 `/home/mips/pi05_datas/validation/right_gripper_mapping_20261106/`，时长 `31.379 s`，未发送 ROS 机械臂控制命令。本次只读解码、近时配对及画面复核支持**右侧夹爪采集映射通过**：`0x159` 是右主端夹爪目标，`0x2A8` 是右从端夹爪反馈，开口增大时两者数值均增大，回闭合后回到各自零点附近。

`100 mm` 是操作员确认的项目行程上限。图像用于确认开合方向与实体归属，不能据此量出精确开口；原始目标峰值约 `103.7 mm`，反馈峰值约 `99.6 mm`，均为未校零的协议读数，不应把目标峰值当作新的软件限位，也不应直接对训练动作做静默钳制。

| 核对项 | 结果 |
| --- | --- |
| 原始 bag | `metadata.yaml` SHA-256 `bdebc79cd74edc27f9f294dfdccdccaff68743f538ce9875e2984b463748d134`；`.db3` SHA-256 `edf40296509aac6b3cce0b8f1e3194202b82a0b0ce3d0401c2bca107a8c388f0`。 |
| 消息覆盖 | 右主目标 `6728` 条、从反馈 `25105` 条、主/从向量各 `1569` 条、右腕 RGB `941` 帧、顶部 RGB 与 `CameraInfo` 各 `940` 帧。主目标 `0x155/0x156/0x157/0x159` 各 `1682` 条；从反馈 `0x2A5/0x2A6/0x2A7` 各 `6276` 条，`0x2A8` 为 `6277` 条。 |
| 原始解码 | `0x159` 和 `0x2A8` 前四字节按有符号大端 `int32 × 1e-6 m` 解码，与 ROS 消息 `gripper_position_m` 逐条一致。目标范围 `-0.0016～0.1037 m`，反馈范围 `-0.0011～0.0996 m`。 |
| 零点和方向 | 录制开始及结束的稳定闭合读数分别约为目标 `-0.0014 m`、反馈 `-0.0011 m`；张开窗口反馈稳定约 `0.0996 m`。约第 `15/17/20/23 s` 的右腕 RGB 分别显示闭合、张开、张开、回闭合，变化方向与两路数值一致。 |
| 主从配对 | 每条 `0x159` 在 `30 ms` 内配最近 `0x2A8`，共 `1682` 对；相关系数 `0.9877`，时间差中位数 `2.44 ms`。扣除上述各自闭合基线后，绝对差中位数约 `0 mm`；该中位数受大量稳定端点样本影响，不代表运动瞬态误差为零。 |
| 时间连续性 | 四个相关原始话题和两路 RGB 的 Header 均无倒退；两路 RGB 最大 Header 间隔约 `33.4 ms`。主目标在静止期可停止发送，最长间隔约 `780 ms`，对应向量 `stale` 信号，不应通过改写 Header 消除。 |

## 生效边界

本 bag 是**映射验证数据**，不是技能 Episode：没有 `episode.json`、技能完成帧、任务结果、场景组和训练切分，不能直接导出为 π0.5 训练数据。录制时右侧向量节点的 `gripper_mapping_verified` 与 `valid` 均为 `false`，这是验收前配置，不应修改历史 bag。当前运行节点复核仍为 `false`；下次启动只读采集节点时，可在保留原始证据的前提下使用：

```bash
ros2 launch piper start_readonly_can_collection.launch.py \
  left_gripper_mapping_verified:=true right_gripper_mapping_verified:=true
```

此参数只表示左右夹爪的采集映射已有现场证据；相机手眼与跨相机外参、训练归一化、逐帧同步、技能完成帧及整条 Episode 的动作来源仍须独立验收。采集配置中的整体 `teleoperation.mapping_verified` 应按完整主从来源核验结果单独设置，不能由本报告自动推导为 `true`。

## 复核方式

在已加载 ROS 2 Humble 的终端运行 `ros2 bag info /home/mips/pi05_datas/validation/right_gripper_mapping_20261106`，核对消息覆盖；以 `sha256sum` 核对上述源文件。用 `rosbags` 从 `/piper_direct/right/master_target` 的 `0x159` 和 `/piper_direct/right/slave_feedback` 的 `0x2A8` 解码前四字节，再按 Header 对齐右腕 RGB 复核开合。任何从本 bag 派生的图表或截帧都应保留原始 Header 和源 `.db3` 摘要。
