# 受控使能测试操作手册

本手册描述如何把一台 Piper 从臂从已验证的 **失能** 状态带到 **使能** 状态，
全程不下发任何运动指令，并在事后加以证明。手册刻意拆成「只读通电前检查」
和「唯一一次通电动作」两段，避免两者被混淆。

本仓库中没有任何逻辑会自动使能机械臂。`auto_enable` 始终为 `false`；
使能永远是一个显式的、有人在场的动作。

## 门禁：以下条件全部成立前不要开始

| # | 前置条件 | 确认方式 |
| --- | --- | --- |
| 1 | 机械臂工作空间内无人，且急停可达 | 人工确认 |
| 2 | 机械臂处于空载状态，或负载已固定足以抵抗小幅沉降 | 人工确认 |
| 3 | 机械臂在当前姿态下已有支撑，关节变硬后不会失稳 | 人工确认 |
| 4 | 两臂均报 `DISABLED`，且六个关节的反馈都是新鲜的 | `piper_enable_check --expect disabled` |
| 5 | 四路 CAN 均为 `ERROR-ACTIVE`，无总线错误 | `ip -details -statistics link show dev can_fr can_ml` |
| 6 | 没有控制节点在发布运动，且 `/pos_cmd_*`、`/joint_ctrl_cmd_*` 都没有发布者 | `ros2 topic info /pos_cmd_left` 显示 `Publisher count: 0` |

使能关节会让它进入力矩保持状态、锁住当前位置。它不应产生位移，但轻微的
沉降动作是正常的；而承担重力的关节变硬时会顶住而不会下垂。这就是第 3 条
重要的原因。

## 第 1 步 —— 只读通电前检查（不改变任何状态）

按检测阶段完全相同的方式拉起接口和从臂节点。此步骤每个接口只发送 13 帧
启动查询（`0x472` 共 12 帧、`0x4AF` 共 1 帧）；实测节点此后空闲期间的发送
增量为零。

```bash
cd /home/mips/piper_ros
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select piper_msgs piper
source install/setup.bash

ros2 launch piper start_two_piper.launch.py \
  auto_enable:=false \
  gripper_exist:=false
```

在第二个终端里取一份不依赖节点的基线。该工具自行解码驱动器的原始反馈帧，
以接收-only 方式打开套接字，并已验证不发送任何帧：

```bash
# 基线：两臂必须报 DISABLED，且退出码为 0。
ros2 run piper piper_enable_check --port can_fr --port can_ml \
  --duration 3 --timeout 0.5 --expect disabled
```

记录发送计数器，以便后续把通电动作的流量归属清楚：

```bash
for i in can_fr can_ml; do echo "$i tx=$(cat /sys/class/net/$i/statistics/tx_packets)"; done
```

## 第 2 步 —— 只使能左臂

只使能一条臂，不要同时使能两条，这样一条臂出现意外时不会和另一条叠加。
左臂是 `can_fr`。

```bash
# 使能：这是本手册中第一条也是唯一一条改变状态的命令。
ros2 service call /enable_srv_left piper_msgs/srv/Enable "{enable_request: true}"
```

预期：节点观察到六个关节全部使能后，`/enable_srv_left` 返回
`enable_response: true`。若返回 `enable_response: false`，表示节点等待
「六分之六」超时。

## 第 3 步 —— 独立确认使能结果

```bash
# 预期 ENABLED，且六个关节都新鲜。
ros2 run piper piper_enable_check --port can_fr --timeout 0.5 --expect enabled

# 预期 state 为 ENABLED、all_enabled 为 True、can_port 为 can_fr。
ros2 topic echo /arm_enable_status_left --once

# err_code 必须保持为 0。
ros2 topic echo /arm_status_left --once
```

成功要求**独立原始帧工具**与**节点话题**两者一致给出 `ENABLED`。若两者不
一致，立即停止，并在证伪之前一直把该臂当作已使能处理，然后将其失能。

### `ctrl_mode` 不能佐证使能位

2026-09-23 实测：使能成功后，左臂和右臂的 `ctrl_mode` 都仍为 `0`（STANDBY）。
`driver_enable_status` 与 `ctrl_mode` 描述的是两件不同的事——前者说明驱动器
已通电并保持位置，后者说明机械臂已切换到接受运动指令的模式。单纯的使能不会
切换模式，因为尚未下发 `MotionCtrl_2`。因此使能后 `ctrl_mode` 为 `0` 是预期
行为而非故障，且 `ctrl_mode` 不可用作「机械臂是否已通电」的证据。

关节确实保持了位置：在实测的使能/失能周期内，左臂六个关节的最大偏差为
0.000000000 rad，因此「关节变硬但没有移动」是本场景下的正常结果。

## 第 4 步 —— 失能并验证

```bash
ros2 service call /enable_srv_left piper_msgs/srv/Enable "{enable_request: false}"
ros2 run piper piper_enable_check --port can_fr --expect disabled
```

## 回滚

将机械臂失能，并确认其处于非使能状态：

```bash
ros2 service call /enable_srv_left piper_msgs/srv/Enable "{enable_request: false}"
ros2 run piper piper_enable_check --port can_fr --expect disabled
```

响应字段是按请求命名的，而不是按结果状态命名的：调用**失能**同样会返回
`enable_response: true`，它表示请求的操作已完成，**并不**表示机械臂已使能。
要判断机械臂是否带电，请读 `piper_enable_check` 的结果，而不是读响应字段。

若服务无响应，停止节点（`Ctrl-C`）并对机械臂断电重启。节点退出并不会让
机械臂失能，因此在认定机械臂已松懈之前，务必先用 `piper_enable_check` 验证。

## 总线上实际发生了什么

以下为实测发送增量，它界定了本手册能做到的范围：

| 事件 | 每接口帧数 |
| --- | --- |
| 节点启动（`PiperInit`） | +13（`0x472` 查询 12 帧、`0x4AF` 1 帧） |
| 节点空闲且 `auto_enable:=false` | 每 15 秒 +0 |
| `piper_enable_check` | +0 |
| 左臂一次使能加一次失能 | `can_fr` +8，`can_ml` +0 |

使能/失能流量被限制在 `can_fr` 上，因此右臂所在总线根本没有看到任何使能帧。
使能路径是节点既有行为，它在每次迭代中下发 `EnableArm(7)` 以及一条夹爪使能
指令，直到六个关节全部报使能为止，这就是帧数不是 1 的原因。

`EnableArm(7)` 覆盖 7 号电机即夹爪，而 `handle_enable_service` 中的
`GripperCtrl` 调用不受 `gripper_exist` 保护。因此即使传入
`gripper_exist:=false`，使能机械臂的同时也会激活夹爪并把它驱向位置 0。在被测
的左臂上夹爪原始角度为 `-500`（约 -0.5 mm，实际上已处于其闭合零点），所以这
是一次亚毫米级位移；但它确实是一条真实的夹爪指令，属于前置条件的一部分——
**手指之间不能有任何东西**。

## `piper_enable_check` 的退出码

| 退出码 | 含义 |
| --- | --- |
| 0 | 每条臂都符合 `--expect`，且所有关节都新鲜 |
| 1 | 出现部分使能，或判定结果与 `--expect` 不符 |
| 2 | 使能状态不完整：有关节缺失或陈旧 |
| 3 | 初始化失败，例如接口无法打开 |

退出码 1 和 2 是与安全相关的两个。`PARTIAL` 表示六个关节中只有部分使能，
此时绝不能把机械臂当作已使能处理。`UNKNOWN` 表示某个关节的反馈缺失或已旧于
`--timeout`，因此不再采信它最后一次上报的使能位。

## 运动门禁

运动是以**实时读数**为门禁，而不是以锁存的使能标志为门禁。`pos_callback` 与
`joint_callback` 只在 `GetEnableFlag()` 为真**且**该时刻六个关节全部使能时才
转发指令。使能服务只在使能那一刻确认过一次「六分之六」；若不做实时复检，此后
掉线的关节仍会放行运动。

门禁只可能拒绝运动，不可能产生运动，因此它自身出故障时是失败在安全方向。当它
因为「标志位说已使能、但六轴并不同意」而拒绝时，会记录一次
`motion refused, the six joints are not all enabled (<六轴明细>)`，按阻断片段
记录而非每个回调都刷屏。

该门禁尚未在硬件上实测过拒绝路径，因为要触发它必须向已使能的机械臂发布一条
运动指令。其逻辑由覆盖六轴全部状态的单元测试保障；真机路径将在运动测试开始时
首次被走到。

## 已验证记录（2026-09-23）

两条从臂均已按本手册完成一次「使能 → 确认 → 失能 → 确认」的完整往返，
全部判据通过。

| 项目 | 左臂 `can_fr` | 右臂 `can_ml` |
| --- | --- | --- |
| 使能前 | 六轴全 `False` | 六轴全 `False` |
| 使能后 | 六轴全 `True`，`state=ENABLED` | 六轴全 `True`，`state=ENABLED` |
| 失能后 | 六轴全 `False` | 六轴全 `False` |
| 使能后 `ctrl_mode` | 仍为 `0`（STANDBY） | 仍为 `0`（STANDBY） |
| `err_code` | 全程 `0` | 全程 `0` |

**双向隔离均已确认。** 使能左臂时右臂保持 `DISABLED`，使能右臂时左臂保持
`DISABLED`；两次的 `ENABLED` / `DISABLED` 判定都正确归属到各自的接口，话题
`can_port` 字段分别为 `can_fr` 与 `can_ml`。

**零运动。** 左臂往返窗口内 14,255 个采样点、38.7 秒，右臂 11,651 个采样点、
30.8 秒；两臂六关节相对首帧的最大偏差均为 `0.000000000 rad`。关节变硬但未
移动，符合预期。

**发送帧账目。** 节点启动每接口 `+13`（`PiperInit` 查询）；左臂往返在
`can_fr` 上 `+8`、`can_ml` 上 `+0`；右臂往返在 `can_ml` 上 `+8`、`can_fr` 上
`+0`。每次操作只在自己那条总线上产生流量，另一条上一帧使能流量都没有。

**CAN 完整性。** 四路接口计数器在全部往返前后完全一致，无 bus error、
error-passive、bus-off，全程保持 `ERROR-ACTIVE`。节点均 `finished cleanly`，
无残留进程；每次收尾后复检两臂均为 `DISABLED`。

两条独立链路在每次判定上结果一致：节点的 `/arm_enable_status_*` 话题与直接解
原始反馈帧的 `piper_enable_check`。

**尚未验证：** 运动门禁的拒绝路径未在硬件上触发过，运动指令从未下发过。

## 为什么新鲜度检查放在这里而不放在话题里

`/arm_enable_status_*` 在某个关节的反馈帧被收到过之后就把该关节标为有效，这
足以区分「从未上报」与「上报为失能」，但它不限制该帧有多旧。`piper_enable_check`
补上了这个上限，因此一个在上报过之后转为静默的关节无法继续贡献一个陈旧的使能
位。正因如此，通电前检查与使能后检查的权威判据是该工具。
