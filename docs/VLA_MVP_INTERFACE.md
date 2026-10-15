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
| 替代只读状态 | `/piper_direct/left/slave_feedback`、`/piper_direct/right/slave_feedback` | 当前录制示例已有原始 CAN 反馈；要另行实现有时间戳和完整性检查的解码适配，不可直接填充七维数组。 |
| `skill_goal`、`observation_id`、模型与标定版本 | `StartEpisode.metadata_json`、`episode.json`、`/collection/events` 或后续上层节点 | 当前录制器没有这些完整字段；采集时须显式写入，不能从 `task`/`skill` 文本猜测。`episode.json.schema_version=1` 是录制格式版本，不是本契约版本。 |

驱动反馈 `/joint_left`、`/joint_right` 的前六维在节点内由角度换算到 `rad`，夹爪原始计数除以 `1e6`；只读 CAN 解码也以 `1e-6 m` 处理。驱动反馈的 `velocity` 只有六项，因此本契约暂不纳入速度和 effort。机械臂实测前须核对夹爪开口定义、换算方向、时间戳来源及与真实尺量的一致性。

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
