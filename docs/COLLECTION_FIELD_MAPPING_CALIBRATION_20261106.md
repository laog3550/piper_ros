# 采集映射与相机标定现场证据复核（2026-11-06）

## 复核范围与结论

本记录于 2026-11-06 11:45 CST 对现有文件和运行参数作只读复核；没有移动机械臂、修改 CAN 接口或重启采集节点。**左侧夹爪的原始目标信号、零点附近读数及开口正方向已有局部证据；右侧夹爪映射、三相机几何标定与可训练 Episode 均未完成验收。** `100 mm` 是操作员确认的项目行程上限，不等于本次实测开口。

| 项目 | 已有证据 | 当前判断 |
| --- | --- | --- |
| 左侧主目标 CAN 归属与开口方向 | [`LEFT_GRIPPER_MAPPING_VALIDATION_20261105.md`](LEFT_GRIPPER_MAPPING_VALIDATION_20261105.md) 绑定 `target_can.jsonl` 的 SHA-256。重新解码该文件：`0x155`～`0x157`、`0x159` 分别为 `3169/3169/3168/3168` 帧，时间戳无倒退；`0x159` 有符号原始目标范围 `-0.0031～0.0658 m`。已有报告用左侧 Episode 的 824 组近时目标/反馈记录得到零点校正后相关系数 `0.9732`。 | 可作为**左侧信号映射的历史证据**；未实测到 `100 mm`，也不代替整条 Episode 的动作来源审计。 |
| 右侧主目标与从反馈 | `/tmp/piper_mapping_check_20261105_1027/status.json` 有四路向量计数及一次当前值；`config/gripper_collection_calibration.20261105.json` 含右侧闭合参考值与观测张开值，但该文件尚未提交，来源目录只有状态快照及抽样 JPEG。 | 没有可重放的右侧开合事件、原始帧与视觉端点对应记录；**右侧映射未验收**。不得将右侧或双臂标为 `gripper_mapping_verified=true`。 |
| 相机身份与彩色内参 | [`config/camera_map.json`](../config/camera_map.json) 固定顶部、左右腕序列号；[`config/collection_camera_info_snapshot.20261105.json`](../config/collection_camera_info_snapshot.20261105.json) 保存三路 `640×480` `CameraInfo` 的 `D/K/R/P`、Header、话题与序列号。 | 可追溯采集时使用的设备与内参**快照来源**；没有标定板重投影残差或保留姿态复验，不宣称几何精度已验收。 |
| 手眼与跨相机外参 | 在本仓库配置、`/home/mips/pi05_datas`、`/home/mips/piper_tjp` 及相关临时采集目录中，未找到带坐标系定义、变换矩阵、原始对应点与验收残差的标定产物。 | 左右腕相机到末端、顶部相机到机器人/桌面、跨相机共同坐标系均**待现场标定**。 |
| 运行配置 | 复核时 `/tmp/piper_episode_config.json` 的 `teleoperation.mapping_verified` 和 `gripper_mapping_verified` 均为 `false`；`ros2 param get /piper_arm_vector_{left,right} gripper_mapping_verified` 两侧均为 `false`。 | 当前运行实例没有声明双侧映射验收。历史左侧节点曾启用，不代表当前实例仍启用。 |
| 真实 Episode | 先前 `episode_000001/000002/000003` 现位于 `~/.local/share/Trash/files/`，原采集目录为空；回收站元信息记录前两条于 2026-11-05 18:57、第三条于 2026-11-06 11:40 移入。只读审计这三条为 `0` 合格 Episode、`0` 合格帧；均缺训练契约字段与技能完成标注。 | 这些来源可供历史复核，但不计为当前采集目录中的训练集；没有改动或还原回收站文件。 |

原始证据摘要：`target_can.jsonl` 为 `39426067ba0cf8ca2c42c41db7a1b4c42932110f7a3e7b088b12f79f3b80f026`；相机内参快照为 `9d6fe0112438446057b5a426d2c74b8a05c14797e16a60d6ea359febe3d6203d`；相机身份配置为 `4f6e19311ce7ce1a25259e94e1745fa30e82da6a8155d86def9cd7b56dad3ca7`。本次回收站三条 Episode 的审计结果在 `/tmp/piper_mapping_field_audit_20261106.json`，其 SHA-256 为 `1521f78427a221c4f2cf4ad39b59526838effed79d1eed72cf361443bf540295`。临时路径只用于本机复核，不作为长期不可变存储。

## 最小只读复验命令

进入 `/home/mips/piper_ros` 后执行；命令仅读取已有来源与 ROS 状态：

```bash
sha256sum /home/mips/pi05_datas/validation/left_gripper_mapping_20261105/target_can.jsonl \
  config/collection_camera_info_snapshot.20261105.json config/camera_map.json

source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 param get /piper_arm_vector_left gripper_mapping_verified
ros2 param get /piper_arm_vector_right gripper_mapping_verified
ros2 topic echo --once /camera_wrist_left/color/camera_info sensor_msgs/msg/CameraInfo
ros2 topic echo --once /camera_wrist_right/color/camera_info sensor_msgs/msg/CameraInfo
ros2 topic echo --once /camera_top/D455/color/camera_info sensor_msgs/msg/CameraInfo
```

如果回收站三条 Episode 仍在，可只读重复来源审计：

```bash
/tmp/piper-collection-audit-venv/bin/python tools/episode_recorder/audit_real_data.py \
  /home/mips/.local/share/Trash/files \
  --episode-root /home/mips/.local/share/Trash/files \
  --max-source-skew-ms 30 \
  --output /tmp/piper_mapping_field_audit_20261106.json
```

## 现场仍须补采与验收

1. **右侧夹爪**：由操作员按原有固件直连遥操作流程，在顶部或右腕画面可见夹爪时，依次保持闭合、部分张开、最大安全张开、再次闭合。只读同录 `/piper_direct/right/master_target`、`/piper_direct/right/slave_feedback`、两路 `/piper_vector/right/*`、顶部 RGB 和 `CameraInfo`；保存每段事件时间、原始 CAN ID/字节、Header、画面与文件 SHA-256。核对 `0x159` 与 `0x2A8` 的零点、正方向、主从实体归属以及停止发帧后的 `stale` 标志。左侧可按同一流程补采，使双侧证据形式一致。录制命令不发送控制帧：

   ```bash
   ros2 bag record -o /home/mips/pi05_datas/validation/right_gripper_mapping_new \
     /piper_direct/right/master_target /piper_direct/right/slave_feedback \
     /piper_vector/right/master_target /piper_vector/right/slave_feedback \
     /camera_top/D455/color/image_raw /camera_top/D455/color/camera_info
   ```

   录制前确认目标目录不存在；操作员完成各状态后用 `Ctrl-C` 正常结束。仅凭配置文件中的 `observed_open` 数值或 `mapping_verified=true` 布尔值，不构成端点验收。

2. **相机几何标定**：为两腕相机分别记录固定标定板在机器人基座坐标系中的已知位姿、多个覆盖工作区的静止末端位姿、相应 RGB 原图、`CameraInfo`、关节反馈与时间戳；为顶部相机记录标定板在机器人/桌面坐标系中的已知位置。冻结标定板尺寸、坐标系方向、相机序列号及求解程序版本；求解并保存两侧手眼和顶部外参、跨相机 TF 链、输入文件哈希、重投影误差和未用于求解的姿态复验结果。仓库当前没有可据此直接运行的标定求解命令；在结果文件和验收门限确定前，保持外参状态为未验收。

3. **训练资格**：原始 RGB 和关节向量可继续采集。若训练路径确定只使用 RGB、关节状态及人类目标动作，可单独审定哪些几何标定证据是训练门禁真正必需；不要把模型不直接输入外参解释成外参已完成。训练集归一化统计应在合格 Episode 与训练/测试划分确定后计算并与 checkpoint 绑定。每条 Episode 还需完整技能完成帧、动作来源、逐帧同步与来源审计。上述任务均不应通过修改旧 Episode 元数据或直接翻转布尔值代替。

Piper V2 主端 `0x155`～`0x157`、`0x159` 以及夹爪距离单位可与[官方双臂说明](https://github.com/agilexrobotics/piper_sdk/blob/master/asserts/double_piper.MD)核对；现场结论仍以本机原始帧与视觉对应证据为准。
