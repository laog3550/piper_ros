# 两 CAN 上位机遥操作操作说明

当前入口是 `piper_two_can_teleop`。每侧主从臂共享一条 CAN，主臂保持
`0xFC`、反馈/控制偏移均为 `0x20`，从臂保持默认地址。实体示教由人按按钮进入和退出。
完整依据见 [实现方案](TWO_CAN_HOST_TELEOP_IMPLEMENTATION.md)。
从手动进入实体示教到自动对齐、摇操、停流、回到起始姿态和整侧失能的实现架构、状态机、
数据通路及安全约束见[流程技术资料](TWO_CAN_TELEOP_WORKFLOW.md)。

## 构建与只读检查

```bash
cd ~/piper_ros
source /opt/ros/humble/setup.bash
colcon build --packages-select piper_msgs piper --symlink-install
source install/setup.bash
ros2 run piper piper_two_can_teleop --side left --no-gripper
```

默认不发送任何帧。检查两套反馈、六关节失能状态和实际对齐轨迹。
`recoverable` 是默认错误策略：运行时用最近 5 s 的错误总数除以 5，超过 20 Hz
才停止；bus-off、error-passive、总线错误类别和控制源竞争仍立即停止。
预检按实际观察时间计算平均错误率，严重错误无条件拒绝。

## 先验收姿态对齐

```bash
ros2 run piper piper_two_can_teleop --side left \
  --send --workspace-clear --align-only --no-gripper
```

1. 确认打印的对齐计划，按回车提交启动。
2. 程序发送一次整侧使能并读回；按提示用实体按钮进入示教，托稳主臂后确认。
3. 从臂先保持起点，再以 minimum-jerk 轨迹对齐。默认最大姿态差 `30°`、
   峰值 `5°/s`，至少 `2 s`，姿态差较大时自动延长。
4. 对齐完成或中止后停止目标流、从臂待机。关闭实体示教并确认后，程序整侧失能并读回。

`--align-only` 不进入持续跟随。当前夹爪零点读数为负，先用 `--no-gripper` 验收关节。

## 正常遥操作

```bash
ros2 run piper piper_two_can_teleop --side left \
  --send --workspace-clear --no-gripper --rate 50
```

启动流程与上面相同，对齐完成后进入 `ACTIVE`。Ctrl-C 停止目标流，再按提示关闭实体
示教、确认整侧失能。`--duration 20` 可限定跟随阶段时长。

左侧已验收 `--rate 50` 的 30 秒关节跟随；命令上限默认仍为 `200 Hz`，该频率尚未
真机验收。同一份主臂快照只发送一次。`--max-delta-deg`、`--master-drift-deg` 等参数可显式调整，拒绝时会打印
实际差值。不能用放宽参数掩盖地址冲突、失效反馈或持续失能。

故障锁存后停止目标，不会自动恢复。没有实体示教退出确认时不会广播失能；退出前应确认
现场状态。共享总线上的 `0x471` 同时影响主从臂，不能单独控制某一台臂的使能。

## 双侧服务与只读观察

```bash
ros2 launch piper start_two_can_teleop.launch.py
ros2 launch piper start_two_can_teleop.launch.py send:=true
```

使用 `/teleop/<side>/start`、`teach_engaged`、`stop`、`teach_released` 服务；两侧地址
均配置完成后才可用 `/teleop/start`。主从观测可以独立运行：

```bash
ros2 run piper piper_joint_watch --arm can_left:主@0x20 --arm can_left:从
```

旧的 `piper_teleop`、`piper_teleop_fast`、`start_two_masters.launch.py`、
`start_fast_two_teleop.launch.py` 已删除。现场记录见 [实现方案](TWO_CAN_HOST_TELEOP_IMPLEMENTATION.md)。


## 停流后先回到本轮起点

从启动时打印的“从臂起点”记录六个角度。停止跟随后，暂不确认失能，保持整侧使能。
用独立工具先只读规划回程，再发送；不要把示例角度套用到另一次会话。

本轮（2026-10-02 左侧）记录的起点与已验收命令为：

```bash
ros2 run piper piper_two_can_align --side left \
  --return-to-deg -2.653 -1.814 1.799 -1.021 20.521 2.977 \
  --max-delta-deg 90 --max-goal-clamp-deg 0
# 核对打印的固定目标、位移和时长后，使用同样参数加 --send --workspace-clear。
```

`--return-to-deg` 只将从臂移到已记录的固定关节角；保留反馈、使能、外部控制和错误监测，
回程不追随主臂。默认峰值 `5°/s`、`50 Hz`、速度百分比 `5%`，完成后从臂待机。
回程姿态差可能大于启动对齐门禁，因此应按实际打印位移设置 `--max-delta-deg`；
关节限位仍生效。程序不根据关节角证明末端路径无障碍，现场空间须保持清空。

回位完成后关闭实体示教、托住主臂，再确认整侧失能；回位失败时停止目标流并检查原因，
不自动重新发起回程。
