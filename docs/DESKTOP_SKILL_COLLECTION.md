# 本地技能采集控制台

## 启动

在每个终端先进入 `/home/mips/piper_ros`，执行 `source /opt/ros/humble/setup.bash` 和 `source install/setup.bash`。按现场流程启动主从臂后，分别运行以下四个进程。主从臂仍由原有 CAN 直连遥操作，控制台不发送机械臂控制帧。

```bash
ros2 launch piper start_cameras.launch.py camera_map:=$PWD/config/camera_map.json
```

```bash
ros2 launch piper start_readonly_can_collection.launch.py
```

```bash
python3 tools/episode_recorder/episode_recorder.py serve --config /tmp/piper_episode_config.json
```

```bash
python3 tools/episode_recorder/desktop_console.py --config /tmp/piper_episode_config.json
```

采集前应按现场目录修改配置中的 `output_dir`、相机与 CAN 话题。窗口内的「Episode 保存目录」可逐条修改；「本条时长上限」到时自动停止，之后选择结果并保存或丢弃。关闭窗口不会停止录制器。

### 只采集彩色图像

若深度图噪声影响观察，可选用 `config/episode_recorder.rgb_only.example.json`。先把它复制到现场配置路径，再按上面的流程让录制器和本地控制台使用同一份配置。例如：

```bash
cp config/episode_recorder.rgb_only.example.json /tmp/piper_episode_config.json
python3 tools/episode_recorder/episode_recorder.py serve --config /tmp/piper_episode_config.json
```

另开终端启动控制台：

```bash
python3 tools/episode_recorder/desktop_console.py --config /tmp/piper_episode_config.json
```

此配置仍录制三路 RGB、相机彩色内参、双臂遥操作目标与反馈；不订阅、不录制深度图及深度内参，控制台也不显示深度区。`start_cameras.launch.py` 的相机驱动仍可能发布深度话题，选择此配置不会改变相机驱动设置。π0.5 原子技能训练使用 RGB、动作与状态，不要求深度；需要深度几何或三维定位的后续流程则要另行采集。更换配置不会把数据自动标为训练就绪，仍需完成映射和时间同步审计。

## 数据与标注

控制台直接订阅配置中的三路 ROS `Image` 彩色帧和可选的对应深度帧，订阅队列深度为 1，只处理最新帧；不经过 JPEG、HTTP 或浏览器轮询。配置深度时，三路彩色图下方显示深度，旁边显示预览帧率、最近帧龄及离线原因。深度预览使用固定量程：左右腕 `0–0.5 m`、顶部 `0–1 m`，灰色表示超过预览量程，黑色表示无效像素；灰色大片不等于传感器数据丢失。左右从臂各显示六关节实时弧度值；开始 Episode 时自动把合格反馈写入 `task_metadata.initial_conditions.joint_positions`。

「完整任务」按固定顺序记录三个技能：单臂抓起香蕉并放入盒子中、双臂翻过盒盖、双臂扣上盒盖。「原子技能」逐条选择其中一个技能及执行臂，并填写标定版本、归一化版本和扰动编号。界面记录技能开始、完成、结果与异常/恢复事件，可选左腕、右腕或顶部图像为标注提供原始 `Header` 时间戳。标注与 Episode 均写入现有录制服务格式。界面只显示映射配置声明状态；标定、夹爪映射、时间同步和逐帧来源审计未通过时，不会把数据宣称为训练就绪。

「本条任务名」和「本条语言指令」描述当前 Episode：原子技能只写当前选中的技能，完整任务才写三个技能的总体目标。桌面与 Web 控制台会按采集类型、技能及执行臂保留文本，保存后继续沿用；切换技能时恢复各自的文本。语言指令框缩小为短指令高度，仍可手动展开 Web 文本框或输入较长说明。

原子技能到时自动停止并标记末帧。若提前停止或自动取帧失败，可在保存前选择彩色相机、填写完成时刻（录制开始后的秒数），再点击「提前停止后补标完成帧」；程序从本条 rosbag 中选取该时刻附近 0.5 秒内的真实图像帧。随后点击「标记当前技能成功」或「标记本条失败」（也可直接在结果下拉框选择），再保存；程序自动生成来源为 `save_selection` 的结果标注。成功 SFT 仍须唯一、可核验的 `skill_complete` 完成帧。

完整任务与原子技能都可设置本条 Episode 时长上限，到时自动停止。完整任务在每个技能完成时点击「标记当前技能成功」，当时选中的彩色相机 `Header` 即为该技能完成帧；停止后会核对该帧是否实际写入 rosbag。原子技能达到时长上限时自动使用顶部彩色相机在停止请求前的最后一帧作为完成帧；如果没有顶部彩色话题，则使用其他已配置的彩色相机。自动取帧失败会在元数据中记录原因，可在保存前手动补标。提前手动停止的原子技能仍需手动补标。最终选择保存结果仍表示整条 Episode 的成败。

## 当前原始采集版本

本地界面从 [`config/collection_camera_info_snapshot.20261105.json`](../config/collection_camera_info_snapshot.20261105.json) 预填 `calibration_version=camera_info_snapshot_20261105_v1`。该文件是 2026-11-05 从三路在线 `CameraInfo` 读取的内参快照，含相机序列号、源话题、Header 时间、分辨率及 `D/K/R/P`；不含经现场验收的手眼与跨相机外参。因此它只是原始采集来源版本，不是训练标定通过证据。

界面从 [`config/collection_data_convention.20261105.json`](../config/collection_data_convention.20261105.json) 预填 `normalization_version=piper_raw_joint6_gripper_m_20261105_v1`。它冻结六关节 `rad`、夹爪原始有符号 `m`、左右臂拼接次序及 CAN ID；尚未计算训练集 `state/action` 分位数统计。两份文件的 SHA-256 随 Episode 写入 `task_metadata.collection_version_evidence`。用这些版本可录制原子技能原始数据，但 `collection_contract_status` 明确为 `training_not_ready_provisional_collection_versions`，不会写入已验证的示教动作来源声明；训练转换与来源审计仍应拒绝。

## 合成闭环验收

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/episode_recorder/test_desktop_console_integration.py
```

合成三路 RGB-D、双从臂反馈及真实录制服务闭环测得三路彩色预览约 `28～30 fps`，ROS 接收至 Qt 显示延迟的 `p95` 约 `27～29 ms`。测试覆盖任务模式的双臂起点、技能开始与完成、异常与恢复、按时自动停止；原子技能模式的所选臂起点、画面 Header 完成帧、标注、保存、丢弃、窗口重开与跨磁盘保存目录重启读取。这些数值来自合成流和无窗口显示环境，真机负载下仍须复测。
