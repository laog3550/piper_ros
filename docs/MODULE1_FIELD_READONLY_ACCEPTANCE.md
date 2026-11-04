# 模块 1 现场只读验收：硬件、时间同步与采集资格

基线：`1c932dd`。核对日期：2026-11-01（Asia/Shanghai）。本页只记录已有证据和只读检查；没有启动相机、启用 CAN、发送 CAN 帧、移动机械臂或修改运行参数。`通过`只表示所列证据覆盖的范围；设备离线时不把历史结果冒充本次在线验收。

## 现场结论

| 项目 | 本次状态 | 证据与边界 |
| --- | --- | --- |
| 两侧 USB-CAN 身份 | 适配器映射通过，臂在线状态未验证 | `can_left` 为 USB `1-13:1.0`、序列号 `004B00395443570F20393433`；`can_right` 为 `1-4:1.0`、`004C003B5443570F20393433`，与 [`config/pi05_can_map.json`](../config/pi05_can_map.json) 一致。两接口当前均为 `DOWN / STOPPED`，RX/TX 为 `0/0`。无 ROS/CAN 采集进程。不能由适配器在位推断四台臂已上电、已使能或主从角色仍正确。 |
| 主端与双从臂 CAN/七维向量 | 当前未验证 | 历史记录分别证明左、右侧官方 `0xFA` 主臂与 `0xFC` 从臂曾在动作窗口联动：左侧 `0x155`～`0x157`、`0x159` 各 177 帧；右侧各 97 帧，且对应从臂反馈变化，见 [`OFFICIAL_NATIVE_TELEOP.md`](OFFICIAL_NATIVE_TELEOP.md)。这不是 2026-11-01 的在线证明，也不是两侧同时运行、四路 `/piper_vector/*` 完整有效的证明。静止窗口缺主臂控制帧不能单独判为故障。 |
| 三路 RGB-D 身份与链路 | 枚举通过，当前出流未验证 | 左腕 `CV27561000MR`、右腕 `CV275610002H`、顶部 D455 USB 序列号 `250343060799` 均在 USB `5000M` 链路；角色配置的 D455 SDK 序列号为 `338122301303`，见 [`config/camera_map.json`](../config/camera_map.json)。当前无相机 ROS 节点，不能核对今天的帧、`CameraInfo` 或图像质量。 |
| 三小时相机历史复验 | 连续采集与正常停机通过，零异常帧不通过 | 2026-10-16 三机六流各运行 `10800 s`、约 `30 fps`，左/右腕无 `>50 ms` Header 间隔或非递增戳；顶部有一次重复 RGB Header、一次 `66.75 ms` 深度间隔及两张 RGB 未在 `5 ms` 内配到深度。结束进程均为 `0`，无残留，见 [`CAMERA_PAIRING.md`](CAMERA_PAIRING.md)。这未证明三机相互硬同步，也未证明 2026-11-01 的状态。 |
| Header 时钟域与跨源同步 | 软件约定可追溯，现场联合测量未通过 | 直连 CAN 单帧 Header 来自 SocketCAN 接收 Unix 时间；七维向量 Header 取四个源戳的最大值，四个 `source_stamps` 仍需分别检查，见 [`piper_direct_can_node.py`](../src/piper/piper/piper_direct_can_node.py) 与 [`piper_arm_vector.py`](../src/piper/piper/piper_arm_vector.py)。历史三小时相机六流的接收墙钟相对 Header 最大延迟均 `<100 ms`，但没有与同次主/从臂向量同录，因此不能声明跨 CAN 与相机的偏差达标。驱动内部 `enable_sync` 不等于三机外部同步。 |
| 标定版本、夹爪映射 | 阻断 | [`config/vla_observation.example.json`](../config/vla_observation.example.json) 的 `calibration_version`、`normalization_version` 仍为 `UNVERIFIED_*` 占位值；未找到绑定三机内外参、左右手眼、共同坐标系与深度单位现场复核的版本证据。官方 Piper SDK 已定义 `0x2A8` 前四字节为有符号 `int32`、单位 `0.001 mm`，因此 `1e-6 m` 换算有协议依据；本项目的夹爪行程上限由用户确认为 `80 mm`，与遥操作常量一致。闭合零点和正方向仍须由只读开合端点记录核验，不要求尺量。`gripper_mapping_verified` 暂保持 `false`。 |
| 当前真实数据训练资格 | **不通过：0 合格 Episode、0 合格帧** | 只读运行 `audit_real_data.py` 盘点 `/home/mips/piper_tjp/data`：8 段真实来源（4 段静态、4 段左侧直连），均非已保存的本仓库 `episode.json/bag` 契约；静态段无示教动作和实测反馈。直连候选中，a05-002 为 139 帧，新增 a06-001/002 为 137/140 帧，均标记 `training_ready=false`、实体归属与夹爪映射未核实，内部最大缺口分别 `1.235/1.402/0.801 s`。a05 的另一份 139 帧候选是同一来源，不能重复计数。没有独立 `pick`/`place` 逐段标签与可用于本契约的双侧示教证据。审核脚本返回 `source_count=8`、`eligible_episode_count=0`、`eligible_frame_count=0`、`train/test=0`。 |

## 采集前检查表

以下每项须随**同一次采集**保存结果；未通过即不得把该 Episode 标为可训练。

1. **硬件与角色**：按 USB 序列号核对两个适配器和三台相机的左右/顶部位置。确认 `can_left`、`can_right` 的实际上电臂和接口状态、`1 Mbit/s`、`ERROR-ACTIVE`、错误计数；由现场观察与同窗口原始 CAN 帧共同证明每侧主臂 `0x155`～`0x157`、`0x159` 及从臂 `0x2A5`～`0x2A8` 的实体归属。左右分别通过不等于双侧同时通过。只读检查本身不要求人员移动机械臂；没有动作窗口就将主臂目标和联动标为未验证。
2. **六路图像**：三机均绑定固定序列号并经 USB3 枚举；RGB、对齐 Depth、各自 `CameraInfo` 持续出流，尺寸、编码、`frame_id` 与 [`CAMERA_PAIRING.md`](CAMERA_PAIRING.md) 一致。每次 Episode 保存六路实际帧数、Header 帧间隔及异常；顶部历史偶发缺口要求下游按 Header 容差配对，不按到达序号拼接。历史亮度实时调节不构成已持久化的图像参数验收。
3. **同一时钟域**：同录相机 RGB/Depth、四路 CAN 向量与其四个 `source_stamps`，核对都是 Unix 纳秒、没有零值、倒退或未来戳；分别报告每源帧率、延迟、RGB-Depth 最近戳差、三机间差和相机至主目标/从反馈的最近戳差。示例配置的 `max_age_ns=150000000`、`max_cross_camera_skew_ns=50000000`、`max_rgb_depth_skew_ns=10000000` 是待现场确认的门限，不能因配置存在就宣称达标。向量默认四原始帧最大龄 `100 ms`、最大跨度 `30 ms`，逐条须看 `valid`、`missing`、`stale`、`time_inconsistent`。
4. **版本与物理映射**：冻结并记录可追溯的相机内外参、两侧手眼、公共坐标/TF、深度单位及复核日期；按官方协议确认夹爪单位，以用户确认的 `80 mm` 为项目行程上限，在只读录制中标记闭合、张开端点与对应的主目标、从臂反馈及画面，核对零点和正方向。保存配置值、原始帧和记录文件哈希后，才允许启用 `gripper_mapping_verified=true`。尺量可作为补充证据，不是必需步骤；不能用测试或占位版本替代。
5. **Episode 资格**：记录真实人类示教动作来源、从臂实测状态、执行臂及 `LEFT/RIGHT/BIMANUAL`、`time_domain=unix_ns`、`action_space=absolute_joint_position_7`、有效标定/归一化版本，以及逐段任务标签。`can-vector` 路径要求主臂目标和从臂反馈七维都逐条 `valid=true`，无缺失/过期/时间不一致；完整 Episode 再通过转换器逐帧检验，并按 Episode 分离训练/验证。当前历史候选不满足这些条件。

## 最小现场只读命令

这些命令只观察已运行的设备与 ROS 节点；**不负责上电、启用 CAN、启动摇操或发送控制帧**。若接口仍为 `DOWN` 或话题没有发布者，记录“未验证”并停止相应步骤。不要用 `piper_two_can_teleop`、`can_activate.sh`、`ip link set` 或 `ros2 param set` 作为本页验收命令。

```bash
# 1. 当前设备身份与总线状态；不改变接口。
ip -details -statistics link show can_left
ip -details -statistics link show can_right
udevadm info -q property -p /sys/class/net/can_left | rg 'ID_SERIAL_SHORT|ID_PATH'
udevadm info -q property -p /sys/class/net/can_right | rg 'ID_SERIAL_SHORT|ID_PATH'
lsusb -t
ls /dev/v4l/by-id | rg 'CV27561000MR|CV275610002H|250343060799'

# 2. 仅在现场设备及只读节点已由操作者启动后检查话题。
source /opt/ros/humble/setup.bash
cd /home/mips/.codex/worktrees/7062/piper_ros
source /home/mips/.codex/worktrees/7062/piper_ros/install/setup.bash
ros2 topic list -t | rg 'piper_(direct|vector)|camera_(wrist_left|wrist_right|top)'
for topic in \
  /camera_wrist_left/color/image_raw /camera_wrist_left/depth/image_raw \
  /camera_wrist_right/color/image_raw /camera_wrist_right/depth/image_raw \
  /camera_top/D455/color/image_raw /camera_top/D455/aligned_depth_to_color/image_raw; do
  timeout 8 ros2 topic hz "$topic"
  timeout 5 ros2 topic echo --once "$topic" --field header
done
date +%s
timeout 5 ros2 topic echo --once /piper_vector/left/master_target
timeout 5 ros2 topic echo --once /piper_vector/left/slave_feedback
timeout 5 ros2 topic echo --once /piper_vector/right/master_target
timeout 5 ros2 topic echo --once /piper_vector/right/slave_feedback

# 3. 对已保存的真实数据重新做只读资格盘点。
python3 tools/episode_recorder/audit_real_data.py /home/mips/piper_tjp/data
```

上述六个 `hz` 命令及 Header 抽样仅是快速出流和时间域检查；完整采集资格还需要同一 Episode 中六路图像、四路向量及源戳的帧级统计。`ros2 topic echo --once` 没有消息时超时不表示角色故障，尤其静止主臂可能不发目标帧。若集成工作树路径变化，`cd` 和第二个 `source` 改为当前 **`1c932dd` 及其后继已构建版本**的路径。

## 证据与未覆盖范围

- 配置与消息契约：[`PI05_CAN_MAPPING.md`](PI05_CAN_MAPPING.md)、[`OFFICIAL_NATIVE_TELEOP.md`](OFFICIAL_NATIVE_TELEOP.md)、[`VLA_MVP_INTERFACE.md`](VLA_MVP_INTERFACE.md)、[`CAMERA_PAIRING.md`](CAMERA_PAIRING.md)。
- 真实数据候选原始清单：`/home/mips/piper_tjp/data/2026-10-14/banana_into_box/scene-a05/banana-direct-a05-002-candidate-v2/candidate_manifest.json` 与 2026-10-26 `scene-a06` 的两份 `candidate_manifest.json`。本次只读审计没有修改该目录。
- 本次没有同录 CAN 与相机的在线流、没有标定实物/版本验收、没有同时双侧七维有效向量，因此硬件链路在位与历史单侧联动、历史相机长测都不能解除真实数据训练阻断。
