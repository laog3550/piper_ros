# VLM/VLA MVP 接口与离线验收契约（v1）

本契约只定义数据交换与离线检查，不授权将策略输出发给驱动。范围对应部署指南第 8、10、12 节：先做单臂 `pick(object)`、`place(region)`，再用相同结构扩展为左右臂独立执行。双臂协同、模型效果和实机安全验收不在 v1 内。

## 边界与版本

- 顶层对象 `contract_version` 固定为 `piper_vla_mvp/1.0`；不认识该版本必须拒绝。`observation_id` 在一次运行中唯一，动作须原样引用。
- JSON 交换对象分为 `observation`、`skill_goal`、`action_chunk`。文件路径只是离线示例载体；未来 ROS 2 transport 须保持字段语义。额外字段与缺字段均拒绝，变更字段或单位须升级版本。
- `normalization_version`、`model_version`、`calibration_version` 是非空不可隐式推断的标识。动作的 `normalization_version` 必须等于观测值；离线校验不证明这些标识对应的文件正确。
- 所有 `*_timestamp_ns` 是同一 ROS 时间域下的整数纳秒；当前实机采集应使用可比较的 Unix 时间。`rosbag2 messages.timestamp` 是接收时间，不能替代消息 `header.stamp`。使用 `/use_sim_time` 时应另建数据集并声明时间域，不能和实机 Unix 时间混用。`inference_timestamp_ns >= observation_timestamp_ns`，快照各源时间不晚于快照时间；过期阈值由部署配置确定，未标定前禁止执行。

## Observation 与 state

`observation` 必含：`contract_version`、`observation_id`、`observation_timestamp_ns`、`time_domain`（固定 `unix_ns`）、`arm_mode`（`LEFT`、`RIGHT`、`BIMANUAL`）、`calibration_version`、`normalization_version`、`images`、`state`。单臂 MVP 只允许 `LEFT` 或 `RIGHT`，扩展时才允许 `BIMANUAL`。

`images` 包含 `top_rgb` 和当前执行臂的 `left_wrist_rgb` 或 `right_wrist_rgb`。双臂模式包含三个 RGB 槽位。每个槽位为 `{topic, header_timestamp_ns, frame_id, encoding, width, height, uri}`；`uri` 指向不可变的离线帧，`encoding` 固定 `rgb8`，尺寸固定 `640×480`。`frame_id` 来自图像 Header，不从话题名猜测。模型输入只使用 RGB；深度和 `CameraInfo` 留在原始 bag，供几何检查与后验评估使用。若未来把深度放入模型输入，需要新版本。

`state` 是按臂名索引的对象；单臂只包含执行臂，双臂包含 `left`、`right`。每臂恰有 `{source_topic, header_timestamp_ns, joint_names, position}`。`joint_names` 固定顺序 `joint1`…`joint6`, `gripper`；`position` 恰为 7 个有限数。前六个为各关节相对自身零位的 `rad`，第七个为夹爪开口宽度 `m`，`0` 表示闭合。每侧各自的关节坐标由该侧机器人 URDF 定义；向量不是公共笛卡尔坐标系中的位姿。适配层按 `JointState.name` 重排，不按消息中的数组下标盲取；缺名、重复名、非有限数须拒绝。`joint7`、`joint8` 是 URDF 夹爪指表示，不进入 7 维向量。

快照构造时按 Header 时间戳为各源找最近帧；不得按到达顺序拼接。时间窗口、最大源间偏差和缺帧策略在现场测量后配置并记录，超窗或缺源的快照拒绝。`calibration_version` 只是版本引用；未完成三机内外参、两侧手眼和双臂共同坐标系验收前，不得据图像推断世界位姿。涉及几何的后续字段统一使用 `base_link` 到相机 optical frame 的 TF 变换链，具体左右 `base_link` 与公共世界帧命名待标定冻结。

## Skill goal 与 action chunk

`skill_goal` 字段固定为 `{contract_version, skill_id, instruction, arm_mode, target_object_id, target_region_id, timeout_sec, max_retries, observation_id, safety_profile}`。`skill_id` 只允许 `pick` 或 `place`；`pick` 必须提供 `target_object_id` 且 `target_region_id` 为 `null`，`place` 相反。单臂 `arm_mode` 必须与观测一致。`instruction` 是简短任务提示，不能代替结构化目标。`timeout_sec > 0`、`max_retries >= 0`；实际值由实验配置确定。

`action_chunk` 字段固定为 `{contract_version, observation_id, arm_mode, model_version, normalization_version, action_space, dt_ns, horizon, observation_timestamp_ns, inference_timestamp_ns, targets}`。`action_space` 固定为 `absolute_joint_position_7`。`targets` 单臂仅有执行臂，双臂有左右臂；每臂为 `horizon × 7` 有限数数组，顺序和单位与 `state.position` 相同。每行是绝对目标，不是增量、速度、末端位姿或直接 CAN 指令。`dt_ns > 0`，`horizon` 为正整数且与数组长度相同。两臂共用一个时间基准和 `horizon`，但此结构本身不证明两臂可同步执行。

离线验收只检查形状、关联、版本、时间顺序和有限数。接入 `action_supervisor` 前，还必须测定并配置动作块最大年龄、关节与夹爪限位、速度/加速度、碰撞距离、共享区互斥、超时、watchdog、HOLD/STOP/ESTOP 路径；不满足即拒绝。策略不得直连 `/joint_ctrl_cmd_left`、`/joint_ctrl_cmd_right` 或 CAN。

## 与现有采集消息的映射

| 契约字段 | 现有来源 | 转换与限制 |
| --- | --- | --- |
| `images.top_rgb` | `/camera_top/D455/color/image_raw` | `sensor_msgs/Image` 的 Header、`encoding`、尺寸；`uri` 由离线导出器生成。 |
| `images.left_wrist_rgb` / `right_wrist_rgb` | `/camera_wrist_left/color/image_raw` / `/camera_wrist_right/color/image_raw` | 同上；配对依据 Header 时间戳。 |
| 几何旁路 | 三路对齐深度与各自 `CameraInfo`，见 `config/episode_recorder.example.json` | 不放入 v1 模型观测；保留原消息 Header 与内参。 |
| `state.left` / `state.right` | `start_two_piper.launch.py` 的 `/joint_left` / `/joint_right`，`sensor_msgs/JointState` | 反馈消息七个名字 `joint1`…`joint6`, `gripper`；按名字重排。此话题未在当前录制示例中，采集前须加入。不能把 `/joint_states_ctrl_*` 的控制回显当作实测状态。 |
| 替代只读状态 | `/piper_vector/left/slave_feedback`、`/piper_vector/right/slave_feedback` | `piper_arm_vector` 从各侧四类原始反馈帧组装七维向量；仅在 `valid=true` 且夹爪映射经现场核对后可用于 `state.position`。原始 `/piper_direct/*/slave_feedback` 保留供追溯。 |
| 示教目标（训练动作候选） | `/piper_vector/left/master_target`、`/piper_vector/right/master_target` | 仅来自主臂 `0x155`～`0x157`、`0x159`；不得用从臂反馈代替 leader 示教动作。`valid` 与夹爪映射要求相同。 |
| `skill_goal`、`observation_id`、模型与标定版本 | `StartEpisode.metadata_json`、`episode.json`、`/collection/events` 或后续上层节点 | 当前录制器没有这些完整字段；采集时须显式写入，不能从 `task`/`skill` 文本猜测。`episode.json.schema_version=1` 是录制格式版本，不是本契约版本。 |

驱动反馈 `/joint_left`、`/joint_right` 的前六维在节点内由角度换算到 `rad`，夹爪原始计数除以 `1e6`；只读 CAN 解码也以 `1e-6 m` 处理。驱动反馈的 `velocity` 只有六项，因此本契约暂不纳入速度和 effort。机械臂实测前须核对夹爪开口定义、换算方向、时间戳来源及与真实尺量的一致性。

只读适配器运行 `ros2 run piper piper_arm_vector`，订阅四个 `/piper_direct/{left,right}/{master_target,slave_feedback}` 话题，不打开 CAN 接口。输出 `piper_msgs/msg/PiperArmVector` 的 `joint_names` 固定为 `joint1`～`joint6`, `gripper`，`position` 前六维为 `rad`，第七维暂按 `1e-6 m` 换算。`source_can_ids`、`source_stamps`、`source_topic`、`header.frame_id` 保留四帧来源；`header.stamp` 为四帧中最新的原始 SocketCAN Unix 时间，不能视为四维同时采样时间。`missing`、`stale` 按关节对、夹爪四项标识；任一缺失、过期、跨帧时间差超过阈值、时间来自未来或夹爪映射未经确认，`valid=false`。无效向量中的零填充值不得作为物理状态使用。默认最大帧龄 `100 ms`、最大帧间差 `30 ms`，由 `max_age_ms`、`max_skew_ms` 调整，现场测量后应记录配置。`gripper_mapping_verified` 默认 `false`，故发布的向量默认 `valid=false`；只有现场核对夹爪换算、零位和符号后才能显式置 `true`。当前适配器未做图像与状态配对，也不生成契约 `observation` 或 `action_chunk`。

## 离线验收

运行 `python3 tools/vla_contract/validate_contract.py --self-test`：生成内存中的合成单臂 `pick`、`place` 和双臂扩展对象，正例应通过；维度、关节顺序、非有限数、缺图像、时间倒置、引用和版本不匹配等负例应被拒绝。运行 `python3 tools/vla_contract/validate_contract.py sample.json` 可检查自备的三对象 JSON 文件。离线接口验收要求：全部样本通过结构检查、负例全部被拒绝、每个样本的原始 bag/图像/标定/归一化版本可追溯，并单独统计缺源与超窗快照数量。零退出码仅表示接口结构检查通过；当前校验器不读取图像 `uri`，不核对 bag 内容、标定文件或动作物理可行性。验收记录应保存输入文件摘要、程序 commit、输出及版本标识。

离线接口通过后仍需依次完成：S0 控制模式、限位、停机与安全路径；S1 标定、跨源时间误差和数据质量；S2 单臂 `pick`/`place` 隔离测试集与逐级实机验收；S3 左右臂独立技能与共享空间检查。对象数量、类别和桌面范围按指南 MVP 目标准备，但没有数据或机械臂时不得宣称技能成功率、同步精度或安全能力。

## 只读观测快照与 RGB-D 几何旁路

`tools/vla_observation/snapshot.py` 从已保存的 Episode `bag` 或实时 ROS 话题读取三台相机的 RGB、对齐深度、对应 `CameraInfo` 和 `/joint_left`、`/joint_right`。程序仅订阅或读取，不发布控制话题。示例：

```bash
python3 tools/vla_observation/snapshot.py bag /path/to/episode/bag --config config/vla_observation.example.json --arm-mode LEFT --output /tmp/snapshots.jsonl
python3 tools/vla_observation/snapshot.py live --config config/vla_observation.example.json --arm-mode LEFT --output /tmp/live-snapshots.jsonl
python3 -m unittest discover -s tools/vla_observation -p 'test_*.py'
```

每行包含快照锚点、来源 Header 时间、age/stale、相机角色、标定版本、拒绝原因和各相机的 `roi_depth`。先在每台相机内按 Header 容差寻找 RGB-D 对，再从配对帧中选择最接近顶部 RGB 锚点的帧；关节状态也按 Header 时间查找，不按到达顺序拼接。顶部 RGB 锚点后的配对窗口由 `max_cross_camera_skew_ns` 控制；过期阈值和深度单位须在现场测量后修改示例配置。`CameraInfo` 的尺寸和针孔内参、深度编码、数据长度、光学帧名以及 ROI 均接受校验。ROI 摘要只报告相机 optical frame 中的有效像素比例、深度统计和点云中位点；零深度与非有限数不参与计算。

只有 RGB 与对应臂状态均有效时，离线输出才包含符合 v1 形状的 `observation`。`rosbag2://...#message=...` URI 标识原始 bag 中的帧，使用前应保存并固定整个 Episode；实时 `ros://` URI 不是不可变图像，因此实时输出不声称符合离线 v1 契约。深度缺失不会伪造 ROI；相机外参、手眼标定和对应时间的 TF 未在此工具中验收，所有 `world_geometry.valid` 始终为 `false`，不得将相机光学坐标点解释为机器人基座中的安全距离。当前 `config/episode_recorder.example.json` 未录制 `/joint_left`、`/joint_right`；用于完整离线观测时需在录制前加入两话题，并确认发布源为实测反馈。现场还需确认三机深度单位、内参、RGB-D 对齐误差、跨源时钟、最大年龄、手眼与外参、共同坐标系以及 TF 链。

2026-10-15 使用本机 camera-only Episode `/tmp/piper_passive_episodes_20261015/episode_000001` 做只读回归：12 路相机话题均存在，顶部 RGB、对齐深度与对应 CameraInfo 各 518 帧，左右腕各路各 492 帧。以顶部 RGB 为锚点产生 518 个双臂快照；顶部 ROI 全部有效，左右腕各 493 个锚点选到有效 RGB-D 与 CameraInfo，启动阶段各 25 个锚点缺腕部帧。最近帧允许被相邻锚点重复使用，因此有效锚点数可以大于腕部原始帧数。顶部 RGB-D 时间差为 `0 ms`；左腕范围 `0.019–0.060 ms`，右腕范围 `-0.012–0.068 ms`。本次配置下三路均无过期帧；缺帧与 stale 分开统计。三机实测编码为 RGB `rgb8`、深度 `16UC1`，尺寸 `640×480`，相关 CameraInfo 的内参和 optical frame 校验通过。示例配置假定深度比例 `0.001 m/unit`，由此计算的 ROI 距离仅用于检查数值处理，bag 本身不能验证真实距离或比例。此 Episode 无 `/joint_left`、`/joint_right`，全部快照明确标记关节状态缺失，未导出 v1 `observation`；世界几何仍全部无效。输出保存在本机 `/tmp/piper_observation_real_bag_bimanual.jsonl`，不纳入 Git。

## 确定性安全监督与离线仿真

运行 `python3 tools/vla_contract/offline_simulator.py` 演练正常动作、陈旧动作、反馈掉线、双臂共享区冲突、单侧故障联动与显式复位。运行 `python3 -m unittest discover -s tools/vla_contract -p 'test_*.py'` 检查边界条件。`action_supervisor.py` 只接受通过本契约校验的 `action_chunk`，不包含 ROS、CAN 或驱动发送入口；仿真使用虚构参数，不得用于真机。

安全配置必须明确提供两侧各自七维位置、速度、加速度限值，动作、观测及快照各源的最大年龄、反馈最大年龄、watchdog、动作最长时限、调度延迟、反馈位置容差，以及能覆盖整段轨迹的扫掠碰撞检查器和共享区段占用检查器。缺项、未知检查结果、版本不符、时钟倒退和任一越界均拒绝。动作预检包括当前反馈与观测一致性、每步速度和加速度、终点静止、双臂同一时间段的共享区互斥。执行期对新反馈的运动段再次做限位、碰撞和共享区检查；最后一个目标也必须收到对应的新反馈才进入 `HOLD`。反馈掉线、超时、故障或越界进入锁存 `STOP`，两臂同时停止发出新目标；仅在新鲜反馈和静态安全检查通过后允许显式 `reset` 回到 `HOLD`。`HOLD` 和 `STOP` 都不输出目标。

实机接入前仍须现场测定并审核：左右臂各关节与夹爪的物理限位、允许速度/加速度、制动行为及停机距离、反馈时间误差、控制周期与 watchdog 阈值、末端和夹爪几何、环境障碍、连续扫掠碰撞模型、双臂共享区边界与坐标标定。当前仓库没有这些可靠参数，也没有证明 `HOLD`、`STOP` 或 `ESTOP` 在驱动硬件上的效果；因此不得把离线测试通过解释为真机安全许可。

## Episode → LeRobot v3 转换

离线转换器为 `tools/episode_recorder/to_lerobot.py`，必须显式选择 `--input-format`。`joint-state` 模式要求 `/joint_left` 或 `/joint_right` 实测状态，以及 `/demonstration/action_left` 或 `/demonstration/action_right` 的 `sensor_msgs/msg/JointState` 示教动作；动作话题需由采集端提供，录制器不会自动生成。`can-vector` 模式直接读取 `/piper_vector/{left,right}/slave_feedback` 作为状态、`/piper_vector/{left,right}/master_target` 作为主臂示教目标，消息类型为 `piper_msgs/msg/PiperArmVector`。两种模式都要求顶视与执行臂腕视 RGB，七维顺序为 `joint1`…`joint6`, `gripper`，前六维为绝对关节目标 `rad`，夹爪为开口宽度 `m`。原始 CAN 帧、控制回显或下一时刻反馈不能代替示教动作。

`can-vector` 模式逐条要求 `valid=true`、`gripper_mapping_verified=true`、没有 `missing`、`stale` 或 `time_inconsistent`，并核对左右臂、`source_topic`、四个 CAN ID、四个 `source_stamps` 与 Header 的关系。缺失、冲突或未核实的向量会拒绝整段 Episode。适配器默认 `gripper_mapping_verified=false`，因此默认采集不能被转换器认作有效示教；必须先在现场核实开口宽度、零位和符号，再显式启用该标志。四路向量可能以高于训练帧率重复发布；转换器先按 Header 合并完全相同的快照，再按 `--fps` 建立均匀时基，逐点选取不晚于该时刻且未超过 `--max-age-ms` 的最新主臂目标、从臂反馈和 RGB。`--max-can-skew-ms` 约束每个向量内部四个原始 CAN 时间的跨度。

启动 Episode 时，`metadata_json` 应显式提供 `arm_mode`（`LEFT`、`RIGHT` 或 `BIMANUAL`）、`time_domain: "unix_ns"`、`action_space: "absolute_joint_position_7"`、`action_provenance: "human_demonstration"`，以及非空的 `calibration_version`、`normalization_version`。这些字段是采集声明，转换器只能核对其存在与数据形状，不能证明动作确由人示教。现场还须核对夹爪开口方向、位置单位和动作发布时刻。转换器严格按 Header 对齐：状态与 RGB 只能取不晚于训练时刻且在 `--max-age-ms` 内的最近帧；`joint-state` 模式的动作采样周期另须符合 `--fps` 与 `--jitter-ms`。缺帧、超窗、时间戳异常、维度错误及非有限数会让整批转换失败。

安装环境使用 `tools/episode_recorder/requirements-lerobot.lock.txt`（Python 3.10、Linux x86_64、CPU，完整固定间接依赖）；`requirements-lerobot.txt` 是重新解析锁文件的顶层输入。目标是 `lerobot==0.4.4` 所写的 LeRobotDataset v3.0。命令示例：

```bash
python3 -m venv /tmp/piper-lerobot-env
/tmp/piper-lerobot-env/bin/pip install -r tools/episode_recorder/requirements-lerobot.lock.txt
/tmp/piper-lerobot-env/bin/python tools/episode_recorder/to_lerobot.py /path/to/episodes /path/to/output --repo-id local/piper_demo --arm-mode LEFT --input-format can-vector --fps 10 --max-age-ms 100 --max-can-skew-ms 30
```

输入目录须至少有两个 `episode_*`；每个必须是 `state=saved` 的完整 Episode。转换前先核验全部 Episode，再按 `episode_id` 的 SHA-256 排序，将指定比例的完整 Episode 分入 `test`，其余进入 `train`，不会把同一 Episode 的帧分到两边。输出为两个本地 LeRobotDataset、含来源摘要的 `conversion_manifest.json` 以及逐帧原始 Header 时间的 `source_timestamps.jsonl`；`can-vector` 模式还记录所选四帧的原始 CAN 时间。LeRobot 内部 `timestamp` 由 `--fps` 形成均匀训练时基；原始 Header 时间不被它取代。转换器只处理本地文件，不上传数据；`use_videos=False` 保留原始 RGB 为图像，不再进行有损视频编码。生成的数据仍需独立检查标定、动作真实性与物理可行性，不能直接视为可用于实机训练或执行的示教集。

## 单臂任务编排离线基线

运行 `python3 tools/vla_contract/offline_task_demo.py` 可执行合成四步序列：`pick(object-1)` → `place(region-1)` → `pick(object-2)` → `place(region-2)`。运行 `python3 -m unittest discover -s tools/vla_contract -p 'test_*.py'` 覆盖验证失败后重新观测并有限重试、重试耗尽、执行超时、动作预检拒绝、评估 `UNKNOWN`、人工接管、反馈 STOP 后禁止评估，以及取消失败时保持 STOP。所有输入、限位和碰撞检查均为合成数据；成功仅表示离线控制流通过。

`task_orchestrator.py` 使用 `OBSERVE` → `PLAN` → `EXECUTE` → `VERIFY` → `RECOVER` / `DONE` / `SAFE_STOP` 的状态机，`Blackboard` 记录当前臂、技能、尝试次数、观测、目标、动作、截止时间和状态历史。观测器、策略、评估器、执行器和时钟均以接口注入；执行器仅能收到 `ActionSupervisor.tick()` 返回的单个获准目标。每次重试重新观测、重新规划，且再次经过完整安全预检。`SnapshotObserverAdapter` 可接收观测分支 `SnapshotBuilder.build()` 的返回记录，以不晚于当前时间的最新顶部 RGB Header 为锚点，并要求另行提供独立反馈来源；无效快照和实时 `ros://` URI 直接拒绝。人工接管成功取消后进入 `HUMAN_TAKEOVER`；取消失败时锁存 STOP。未知评估结果、碰撞风险、反馈故障及超时也进入锁存 STOP。当前仅接受 `LEFT` 或 `RIGHT` 单臂任务；左右臂可分别构造编排器，尚未提供双臂协同调度。

此离线接口以同步调用模拟时间。未来接入真实策略、相机与执行设备时，仍需为可能阻塞的外部调用提供独立的超时与取消机制，并完成前述标定、安全参数和实机验收；当前代码没有驱动发送入口。

## 官方 π0.5-DROID 离线基线边界

固定上游源码为 [openpi commit `215abfb217dbac7d5f1273282331b9b1866c0479`](https://github.com/Physical-Intelligence/openpi/tree/215abfb217dbac7d5f1273282331b9b1866c0479)，依赖按该 commit 的 `uv.lock` 安装，配置为 `pi05_droid`，权重路径为 `gs://openpi-assets/checkpoints/pi05_droid`。上游代码采用 Apache-2.0，仓库另含 `LICENSE_GEMMA.txt` 中的 Gemma 使用条款；公开权重目录中未见单独的许可文件，因此不能把权重直接概括为 Apache-2.0。本次仅在本地下载并运行官方 smoke，未再分发权重。该 checkpoint 是 DROID 策略，不是 Piper 策略。

该配置的模型内部 action 为 `15×32`，上游 `DroidOutputs` 截取前 8 维，公开 DROID 执行示例把这 8 维解释为 **7 维关节速度加 1 维夹爪位置**；上游示例中的旧 `10×8` 注释与当前 `pi05_droid` 配置的 15 步不一致，以固定 commit 的配置与运行输出为准。DROID 输入需要 7 维关节位置和 1 维夹爪位置，而 Piper 契约状态只有 6 个关节角加夹爪开口。两个机器人的关节数、动作语义和夹爪标度不同，不能直接重排、补零、积分或把原输出标为 `absolute_joint_position_7`。官方 DROID 图像槽位为一张外部 RGB 和一张腕部 RGB，第三槽位用掩码屏蔽；本契约单臂 RGB 可供图像预检查，但不能据此宣称策略适配完成。

`tools/vla_contract/openpi05_offline.py` 提供两个互不混淆的入口：

```bash
python3 tools/vla_contract/openpi05_offline.py --preflight sample_input.json --image-dir /path/to/frames --now-ns 1800000000105000000 --max-age-ns 10000000 --max-skew-ns 10000000
python3 tools/vla_contract/openpi05_offline.py --smoke --openpi-root /path/to/openpi --checkpoint /path/to/pi05_droid
python3 -m unittest discover -s tools/vla_contract -p 'test_openpi05_offline.py' -v
```

`sample_input.json` 顶层只含 `observation` 和 `skill_goal`，图像 `uri` 为 `--image-dir` 内的本地相对路径。预检查要求契约版本、引用、RGB 文件、`640×480`、单臂、源间 Header 偏差与明确的最大观测年龄均通过；结果只返回校验元数据和缺失的语义桥接项，**不会生成 `action_chunk`**。`--smoke` 仅以官方随机 DROID 样例检查完整本地 checkpoint 的输出形状、有限性、模型加载及推理耗时和进程峰值显存；随机输出不可用作 Piper 策略。两种入口都不连接 ROS/CAN。

本次环境检查：Ubuntu 目标机的 RTX 5880 Ada 显存约 48 GB，驱动 595.99.02，磁盘空闲约 822 GB；系统原有 Python 为 3.10.12。已在本独立 worktree 的 `.offline/` 内安装 Python 3.11.17、`uv 0.13.1`，并用固定 `uv.lock` 执行 `GIT_LFS_SKIP_SMUDGE=1 uv sync --frozen --no-dev`。官方 checkpoint 下载到同一 `.offline/` 目录，共 20 个对象、12,429,488,598 字节；每个文件大小和 MD5 均与公开 GCS 元数据一致。`tools/vla_contract/pi05_droid.sha256` 固定各文件的 SHA-256；进入 checkpoint 根目录后运行 `sha256sum -c /path/to/piper_ros/tools/vla_contract/pi05_droid.sha256` 可重新核对。`.offline/` 不纳入 Git。

在 `XLA_PYTHON_CLIENT_PREALLOCATE=false` 下，以官方随机 DROID 样例运行一次完整 smoke，JAX 后端为 `cuda:0`，输出 `15×8` 且全为有限数。该次实测加载 18,191.4 ms，首次推理 23,922.0 ms（含首次编译），后续两次推理为 89.7、98.4 ms；`nvidia-smi` 按进程采样的峰值显存为 8,556 MiB。测试前整卡占用约 1,179 MiB，模型退出后恢复约 1,213 MiB。上述时间仅代表该机器、该 checkpoint、随机输入和这一次 smoke，未包含 Piper 数据适配，也未测长期吞吐或任务成功率。复现命令：

```bash
XLA_PYTHON_CLIENT_PREALLOCATE=false .offline/openpi/.venv/bin/python tools/vla_contract/openpi05_offline.py --smoke --openpi-root .offline/openpi --checkpoint .offline/checkpoint/pi05_droid
```

只读检查另一个工作区 `/home/mips/piper_tjp/data` 的 `metadata.yaml`：其中有 6 次录制，4 次只有相机/事件话题；`scene-a05` 的 2 次录制包含左侧 `/dataset/can_left/action_target`、`feedback_observed` 与三路 JPEG RGB。`banana-direct-a05-002` 的候选清单记录了 139 个对齐样本，但最长内部缺口为 1.235 s、`training_ready=false`，并列出反馈实体归属、夹爪零点/执行映射及最终动作节拍待核对。这些是已有的左侧候选示教材料，不能据此推断双臂或完整 `pick`/`place` 训练集已经具备。`/dataset/...` 话题与本契约话题不同，JPEG 与本契约原始 `rgb8` 帧也不同；本离线工具不会隐式转换。仍缺少经过验证的契约映射、标定版本、Piper 归一化统计与适配 checkpoint。只有这些条件和动作空间转换经验证后，才可能生成符合本契约的真实动作块；离线 smoke 不能替代该验证。上述检查没有修改 `/home/mips/piper_tjp`。
