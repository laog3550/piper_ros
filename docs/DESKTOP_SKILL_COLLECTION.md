# 本地技能采集控制台

2026-11-12 更新：人工完成标记保持原始图像 Header，来源或连续性不足单独记录，不由尾部候选替换。原始保存的 `completion_validation` 只表示局部帧检查；`training_eligibility` 在独立验收前保持 `eligible=false`。失败候选分别显示片段总帧数和完成前有效帧数。启动入口、负载优化及实机复测见[验收问题修复与主动复测](COLLECTION_FIX_20261112.md)。

## 启动

在每个终端先进入 `/home/mips/piper_ros`，执行 `source /opt/ros/humble/setup.bash` 和 `source install/setup.bash`。按现场流程启动主从臂后，分别运行以下四个进程。主从臂仍由原有 CAN 直连遥操作，控制台不发送机械臂控制帧。

```bash
ros2 launch piper start_cameras.launch.py camera_map:=$PWD/config/camera_map.json camera_fps:=15
```

```bash
ros2 launch piper start_readonly_can_collection.launch.py
```

```bash
tools/episode_recorder/start_collection.sh recorder /tmp/piper_episode_config.json
```

```bash
tools/episode_recorder/start_collection.sh console /tmp/piper_episode_config.json
```

`start_collection.sh` 固定启动环境：录制器使用带 `rosbags` 的兼容虚拟环境（默认 `/tmp/piper-atomic-venv/bin/python`，可用 `PIPER_RECORDER_PYTHON` 覆盖），控制台使用系统 Python 并设置 `PYTHONNOUSERSITE=1`，避免用户 site-packages 的 NumPy 2 与系统 OpenCV 4.5 冲突。两者启动前都会运行 `preflight.py` 自检；失败时明确报告缺失或不兼容的模块。

采集前应按现场目录修改配置中的 `output_dir`、相机与 CAN 话题。窗口内的「Episode 保存目录」可逐条修改；「本条时长上限」到时自动停止，之后选择结果并保存或丢弃。关闭窗口不会停止录制器。

只读 CAN 与向量节点按当前用户、ROS domain 和左右侧持有进程锁，同一来源不能重复启动；`side=all` 也不能与已运行的分侧实例重叠。进程退出后操作系统释放锁，不要删除仍被占用的锁文件。升级前已启动的旧节点没有此锁，应先停止旧的只读采集 launch，再启动一套新版本。

录制器在创建暂存 Episode 前及 rosbag2 启动就绪前检查所有配置话题。若同一话题存在多个发布者，即使节点名称相同，也会拒绝录制并指出具体话题；可选的非执行臂话题同样不能混流。就绪时各话题的节点名称、命名空间和 GID 写入 `source_publishers_at_ready`，供后续审计。初始全缺失的向量不发布零 Header，等待首个真实 CAN 来源；已有来源的 `stale`、`missing` 与原始 Header 保留。

### 训练采集配置与录制负载

训练数据建议使用 `config/episode_recorder.pi05.json`，把它复制到现场配置路径后让录制器与控制台共用同一份文件。该配置约定：

- 三路相机当前按 15 fps 纯彩色采集（`camera_fps:=15 enable_depth:=false`）。D455 在 640×480 下没有 10 fps 档位；实测 30 fps 深度+彩色会使 Orbbec 发布帧率塌陷到 1–13 fps，30 fps 纯彩色也只能到约 20 fps，因此 30 fps 不作为当前采集配置。15 fps 顶部—左腕 p95 约 33 ms，当前 `max_camera_skew_ms=40`、`max_source_skew_ms=40` 可用；30 ms 保留为严格模式。D455 固定曝光 80 µs、gain 32、白平衡 4000K；Orbbec 固定曝光 100、gain 0、白平衡 4600K，自动曝光与自动白平衡已关闭（`d455_exposure`、`d455_gain`、`d455_white_balance`、`orbbec_color_exposure`、`orbbec_color_gain`、`orbbec_color_white_balance`）。顶部由 `top_fps_lock=true` 锁定 15.0 fps（顶部深度仅用于帧时钟，不进入 bag）；当前相机验收为 v8。独立的 SFT 转换仍按 `--fps 10` 采样。
- `raw_can_qos_depth=5000`。录制器会为 `/piper_direct/*` 和 `/piper_vector/*` 生成 `recording_qos.yaml`，通过 `ros2 bag record --qos-profile-overrides-path` 加深读取队列。rosbag2 默认 depth 10 对 800–1600 帧/秒的原始 CAN 只有 6–12 ms 余量，会在图像写入期间大量丢帧。
- `hold_max_sec=10`、`hold_anchor_max_gap_sec=2`，用于操作员显式记录的保持段。
- `auto_trim_stale_tail=true`、`stale_stop_grace_sec=1.0`：原子技能录制中主目标持续 stale 且无后续动作时自动停止，Episode 逻辑终点裁到最后一个有效主目标；原始 bag 仍保留等待窗口。细节见[末尾 stale 完成标记](STALE_COMPLETION_POLICY_20261110.md)。
- `action_source=follower_next_state`（2026-11-12 新增）：元数据写入 `action_provenance=teleoperation_follower_next_state`，转换器用从臂反馈 `t+1` 作为 action；若仍要主臂目标作为 action，设为 `master_target`。新模式的坐标约定、验收和 episode_000006 结果见[从臂反馈作为 SFT 动作](FOLLOWER_NEXT_STATE_SFT_20261112.md)。
- 当前默认 `calibration_version` 为 `camera_info_snapshot_20261112_v2`，对应 `config/collection_camera_info_snapshot.20261112_v2.json`；`camera_info_snapshot_20261112_v1` 是 16.59 fps 启动批次的历史快照，保留给 episode 8/9。当前相机验收为 v8，见[三相机训练证据与几何标定验收](CAMERA_TRAINING_CALIBRATION_ACCEPTANCE.md) 和[顶部固定 15 fps 锁定](CAMERA_HARDWARE_SYNC_D455_MASTER_20261112.md)。

开始采集前还要检查 SocketCAN 接收缓冲。默认 `net.core.rmem_max=212992` 只够约 160 ms 的 CAN 流，建议在宿主机执行：

```bash
sudo sysctl -w net.core.rmem_max=16777216
```

直连 CAN 节点启动时会打印实际生效的接收缓冲；若仍被内核限制，会给出警告。完整诊断与验收指标见[录制负载下的原始 CAN 丢失与修复](CAN_RECORDING_LOAD_FIX_20261110.md)。

### 只采集彩色图像

若深度图噪声影响观察，可选用 `config/episode_recorder.rgb_only.example.json`。先把它复制到现场配置路径，再按上面的流程让录制器和本地控制台使用同一份配置。例如：

```bash
cp config/episode_recorder.rgb_only.example.json /tmp/piper_episode_config.json
tools/episode_recorder/start_collection.sh recorder /tmp/piper_episode_config.json
```

另开终端启动控制台：

```bash
tools/episode_recorder/start_collection.sh console /tmp/piper_episode_config.json
```

此配置仍录制三路 RGB、相机彩色内参、双臂遥操作目标与反馈；不订阅、不录制深度图及深度内参，控制台也不显示深度区。`start_cameras.launch.py` 的相机驱动仍可能发布深度话题，选择此配置不会改变相机驱动设置。π0.5 原子技能训练使用 RGB、动作与状态，不要求深度；需要深度几何或三维定位的后续流程则要另行采集。更换配置不会把数据自动标为训练就绪，仍需完成映射和时间同步审计。

## 数据与标注

控制台直接订阅配置中的三路 ROS `Image` 彩色帧和可选的对应深度帧，订阅队列深度为 1，只处理最新帧；不经过 JPEG、HTTP 或浏览器轮询。配置深度时，三路彩色图下方显示深度，旁边显示预览帧率、最近帧龄及离线原因。深度预览使用固定量程：左右腕 `0–0.5 m`、顶部 `0–1 m`，灰色表示超过预览量程，黑色表示无效像素；灰色大片不等于传感器数据丢失。左右从臂各显示六关节实时弧度值；开始 Episode 时自动把合格反馈写入 `task_metadata.initial_conditions.joint_positions`。

「完整任务」按固定顺序记录三个技能：单臂抓起香蕉并放入盒子中、双臂翻过盒盖、双臂扣上盒盖。「原子技能」逐条选择其中一个技能及执行臂，并填写标定版本、归一化版本和扰动编号。界面记录技能开始、完成、结果与异常/恢复事件，可选左腕、右腕或顶部图像为标注提供原始 `Header` 时间戳。标注与 Episode 均写入现有录制服务格式。界面只显示映射配置声明状态；标定、夹爪映射、时间同步和逐帧来源审计未通过时，不会把数据宣称为训练就绪。

录制中还可以点击「开始保持（重复最后目标）」/「结束保持」显式记录操作员希望从臂继续保持最后目标的时间段。保持期间从臂反馈和三路相机必须继续有效；保持只复用最后一条有效绝对目标，原始 `Header` 与 `stale` 保留。该记录不能由目标停发或 `stale` 推断，旧 Episode 不会因程序新增该功能而自动转为合格。约定见[保持意图的采集与 SFT 支持](HOLD_INTENT_SFT_20261110.md)。

「本条任务名」和「本条语言指令」描述当前 Episode：原子技能只写当前选中的技能，完整任务才写三个技能的总体目标。桌面与 Web 控制台会按采集类型、技能及执行臂保留文本，保存后继续沿用；切换技能时恢复各自的文本。语言指令框缩小为短指令高度，仍可手动展开 Web 文本框或输入较长说明。

原子技能可以在录制中点击「标记当前技能成功」记录当前彩色相机的完成时刻，并选择任务成功；停止后仍可复核。到时自动停止只生成**完成候选**，不再自动写入 `skill_complete`；点击「确认候选完成帧」后完成时刻固定，之后的动作保留为尾部数据。若候选无效或提前停止未标记，可选择彩色相机、填写完成时刻（录制开始后的秒数），再点击「录制后补标完成帧」；程序从本条 rosbag 中选取该时刻附近 0.5 秒内的真实图像帧。任务结果是成功但完成帧或连续 16 帧不足时，窗口显示具体原因（候选时间、连续帧数、来源校验错误）并允许保存为“成功待复核”；`training_eligibility` 记录 `completion_pending` 或 `sft_frame_gate_failed`，SFT 导出继续严格拒绝。失败结果仍可保存。

到时自动完成候选仍按你的免按键操作流程生成。录制器同时记录该相机帧接收时刻之前最近一条主端 CAN 目标的接收间隔，写入 `auto_completion_master_gap_ms`；若任一执行臂超过 `100 ms` 无主端目标，窗口显示 `auto_completion_source_warning`。该提示不改变任务结果选择，也不填造动作帧；最终训练资格由逐帧 Header、CAN 来源与连续片段审计决定。若主端在动作完成后停发，确认候选时刻即可；不需要在遥操作时按键。主端持续上报的真实采样和采样序号/心跳仍需主端配合，采集端不会用旧值补帧。

完整任务与原子技能都可设置本条 Episode 时长上限，到时自动停止。完整任务开始录制时自动进入第一个技能；每次点击「标记当前技能成功」会记录当时选中的彩色相机 `Header` 为该技能完成帧，并自动进入下一技能。停止后会核对完成帧是否实际写入 rosbag；三个技能均成功时界面预选整条 Episode 成功，保存前仍可人工核对和修改。单臂原子技能达到时长上限时，若主端末尾持续 stale，则按操作员的“停发即任务已完成”采集约定，使用最后有效主目标 Header 之前的真实彩色图像作为完成帧，排除中途停顿后恢复的情况；其他情况沿用停止请求前的末帧。完整 CAN 原子技能从顶部和执行侧腕部相机中选择最新的合格因果图像组；记录实际选中帧，不固定优先顶部。停止后对应窗格显示“已录制完成画面”及 Header 相对时刻，加载成功后才能确认候选。规则及训练边界见[末尾 stale 完成标记](STALE_COMPLETION_POLICY_20261110.md)。自动取帧失败会在元数据中记录原因，可在保存前手动补标。提前手动停止的原子技能仍需手动补标。最终保存结果表示整条 Episode 的成败。

## 当前原始采集版本

本地界面从 [`config/collection_camera_info_snapshot.20261110.json`](../config/collection_camera_info_snapshot.20261110.json) 预填 `calibration_version=camera_info_snapshot_20261110_v1`。该文件绑定 2026-11-10 当前三路在线 `CameraInfo` 和只读证据包，尤其将顶部 D455 相对旧快照的 `K/P` 变化独立版本化。它只说明本次原始 RGB 采集的设备与内参来源；安装角色仍须现场复核，几何精度仍未验收。更换相机启动批次后应先重新核对内参版本，再采集 Episode。

界面从 [`config/collection_data_convention.20261105.json`](../config/collection_data_convention.20261105.json) 预填 `data_convention_version=piper_raw_joint6_gripper_m_20261105_v1`。它冻结六关节 `rad`、夹爪原始有符号 `m`、左右臂拼接次序及 CAN ID。两份文件的 SHA-256 随 Episode 写入 `task_metadata.collection_version_evidence`，证据键分别为 `calibration_version` 与 `data_convention_version`。

采集程序分别记录有依据的原始事实：选用有本地来源的时间域约定时写入 `time_domain=unix_ns`；所选臂的主从话题与夹爪映射在配置中声明已核对时，写入 `action_space=absolute_joint_position_7` 与 `action_provenance=human_demonstration`。相机外参未验收、训练统计尚未生成，都不会抹去这些采集声明。反过来，映射未核验时不写动作声明，时间域证据缺失或版本未知时不写 `time_domain`；每项原因保存在 `collection_contract_checks` 中。

`collection_contract_status=metadata_ready_pending_data_audit` 只表示上述原始事实声明齐全。采集程序始终记录 `training_eligibility={"eligible": false, "status": "pending_independent_evidence_and_audit"}`，标定检查保持 `provisional` 或 `operator_supplied_pending_audit`。正式训练资格仍须独立核对标定、实体归属、夹爪换算、逐帧时间同步、技能完成帧和场景组划分，版本名与采集配置中的布尔声明不能替代核准。

`normalization_version` 专用于训练集 `state/action` 归一化统计，采集界面不要求填写，也不写入原始 Episode。统计只可在独立证据与来源审计通过、场景组划分冻结之后，从合格的 `train` 划分计算；`test`/`validation` 不可参与。统计资产须保存版本、SHA-256、训练划分与审计来源，并在训练记录中绑定实际 checkpoint；不能用原始字段/单位版本替代。采集时的 `training_normalization` 检查为 `not_computed`，不宣称这些统计已存在。

为兼容旧客户端，开始录制接口只会把已知的原始数据约定值从旧 `normalization_version` 参数转存为 `data_convention_version`；其他 `normalization_version` 输入会被拒绝。此兼容只处理新请求，不修改既有 Episode，也不为历史数据倒填时间域、动作来源或训练资格。正式转换与训练仍须通过各自门禁。

## 合成闭环验收

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/episode_recorder/test_desktop_console_integration.py
```

合成三路 RGB-D、双从臂反馈及真实录制服务闭环测得三路彩色预览约 `28～30 fps`，ROS 接收至 Qt 显示延迟的 `p95` 约 `27～29 ms`。测试覆盖任务模式的双臂起点、技能开始与完成、异常与恢复、按时自动停止；原子技能模式的所选臂起点、画面 Header 完成帧、标注、保存、丢弃、窗口重开与跨磁盘保存目录重启读取。这些数值来自合成流和无窗口显示环境，真机负载下仍须复测。

## 2026-11-12 启动与候选画面更新

点击开始后，程序须确认实际收到连续新鲜的图像、CameraInfo、执行臂有效反馈，并核对首批消息已写入本条 bag，才进入“录制中”并开始任务计时。请等待“录制中”后再操作。启动临时订阅会自动销毁；若数据未就绪，窗口会报告具体话题，不能仅凭 ROS 发布者存在就开始。

完成候选采用与导出相同的因果图像组校验，不再固定选择顶部末帧。停止后标为“已录制完成画面”的窗格来自本条 bag 的精确 Header，其他窗格仍是实时预览；应根据该录制画面确认实际松手和任务终态。技术检查通过不自动代表任务成功。实现与验收见[相机完成选帧与采集数据流优化](COLLECTION_CAMERA_TIMING_PIPELINE_20261112.md)。

## 记录表刷新（2026-11-12）

记录表右上方新增「刷新记录」按钮。它直接扫描当前「Episode 保存目录」中的实际 `episode_数字/episode.json`，只显示已保存记录，按目录编号从大到小排序；不读取旧目录索引，也不依赖录制服务连接。删除目录后刷新会移除旧记录，目录内没有已保存 Episode 时表格清空。切换保存目录或修改路径后离开输入框也会刷新，原有每 5 秒自动刷新保留。

刷新在独立后台任务中读取元数据，不解码图像、不更改 Episode 内容或 `.next_id`。个别记录损坏会跳过并提示；目录不存在时清空旧列表并显示原因。界面显示本次读取数量和刷新时间。

本次还处理了清理数据后通信文件丢失的问题：当前运行配置将 `socket_path` 显式设在 `/tmp/piper_episode_recorder_41629463606d.sock`，不再位于测试数据目录。控制台与录制服务使用同一配置；其他自定义配置仍应自行设置目录外、互不冲突的 `socket_path`。不修改默认配置兼容行为，也不重置用户的 Episode 编号。

验证已覆盖数字倒序（10、2、1）、删除后移除、全部删除后清空、切换目录、忽略旧索引、损坏记录提示及录制服务断开时独立刷新。当前服务与控制台已重启，实际保存目录记录数为 0。
