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

## Episode → LeRobot v3 转换

离线转换器为 `tools/episode_recorder/to_lerobot.py`。它要求保存的 Episode bag 中具有 `/joint_left` 或 `/joint_right` 实测状态、三路 RGB 中的顶视与执行臂腕视，以及 `/demonstration/action_left` 或 `/demonstration/action_right` 的示教动作。动作消息使用 `sensor_msgs/msg/JointState`，Header 为 Unix 时间，`name` 恰为 `joint1`…`joint6`, `gripper`（可重排），`position` 前六项为绝对关节目标 `rad`，末项为夹爪开口 `m`。动作话题是采集端需要提供的接口，并非现有录制器自动生成。原始 CAN 帧、控制回显或下一时刻反馈都不能替代它。采集时须将状态与动作话题加入录制配置的 `topics`；旧示例配置本身不满足转换条件。

启动 Episode 时，`metadata_json` 应显式提供 `arm_mode`（`LEFT`、`RIGHT` 或 `BIMANUAL`）、`time_domain: "unix_ns"`、`action_space: "absolute_joint_position_7"`、`action_provenance: "human_demonstration"`，以及非空的 `calibration_version`、`normalization_version`。这些字段是采集声明，转换器只能核对其存在与数据形状，不能证明动作确由人示教。现场还须核对夹爪开口方向、位置单位和动作发布时刻。转换器严格按 Header 对齐：以动作帧为基准，状态与 RGB 只能取不晚于动作且在 `--max-age-ms` 内的最近帧；动作采样周期必须符合 `--fps` 与 `--jitter-ms`。缺帧、超窗、时间戳异常、维度错误及非有限数会让整批转换失败。

安装环境使用 `tools/episode_recorder/requirements-lerobot.lock.txt`（Python 3.10、Linux x86_64、CPU，完整固定间接依赖）；`requirements-lerobot.txt` 是重新解析锁文件的顶层输入。目标是 `lerobot==0.4.4` 所写的 LeRobotDataset v3.0。命令示例：

```bash
python3 -m venv /tmp/piper-lerobot-env
/tmp/piper-lerobot-env/bin/pip install -r tools/episode_recorder/requirements-lerobot.lock.txt
/tmp/piper-lerobot-env/bin/python tools/episode_recorder/to_lerobot.py /path/to/episodes /path/to/output --repo-id local/piper_demo --arm-mode LEFT --fps 10 --max-age-ms 100
```

输入目录须至少有两个 `episode_*`；每个必须是 `state=saved` 的完整 Episode。转换前先核验全部 Episode，再按 `episode_id` 的 SHA-256 排序，将指定比例的完整 Episode 分入 `test`，其余进入 `train`，不会把同一 Episode 的帧分到两边。输出为两个本地 LeRobotDataset、含来源摘要的 `conversion_manifest.json` 以及逐帧原始 Header 时间的 `source_timestamps.jsonl`。LeRobot 内部 `timestamp` 由 `--fps` 形成均匀训练时基；原始 Header 时间不被它取代。转换器只处理本地文件，不上传数据；`use_videos=False` 保留原始 RGB 为图像，不再进行有损视频编码。生成的数据仍需独立检查标定、动作真实性与物理可行性，不能直接视为可用于实机训练或执行的示教集。
