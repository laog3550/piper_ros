# 三相机训练证据与几何标定验收

## 适用范围与结论

本文区分原始 RGB + 关节绝对位置训练需要的相机证据，和机器人坐标下视觉定位、深度几何、跨视角融合及运动规划需要的几何标定。**当前单臂 π0.5 输入路径不直接读取内参或外参；相机身份、安装角色、原始图像规格及内参来源仍须可追溯。CameraInfo 快照不能证明几何精度。**

本项目的 [`PiperInputs`](../tools/vla_contract/piper_pi05_adapter.py) 接受顶部和对应腕部的 `640×480` RGB、`joint1..joint6 + gripper` 状态、任务文本及训练动作。另一腕视图填零并设为无效；当前适配器只支持单臂 `left/right`。[`piper_pi05_openpi.py`](../tools/vla_contract/piper_pi05_openpi.py) 绑定 OpenPI commit `215abfb217dbac7d5f1273282331b9b1866c0479`，其[官方 Observation 定义](https://github.com/Physical-Intelligence/openpi/blob/215abfb217dbac7d5f1273282331b9b1866c0479/src/openpi/models/model.py)也没有相机外参字段。这里据实际输入代码判断外参不属于这条训练路径的必需输入；不据此推断模型训练效果或部署安全。三机留档不表示当前代码已支持双臂 π0.5 训练。

| 用途 | 必需证据 | 当前证据能证明什么 |
| --- | --- | --- |
| 原始 RGB + 关节状态/人类绝对目标训练 | 实际设备序列号与角色、安装批次、RGB 规格/颜色/预处理、对应 CameraInfo 来源和版本、逐 Episode 来源关联 | 设备身份和输入规格可检查；不要求为这一输入路径先求解手眼或跨相机外参 |
| 去畸变、像素射线或深度反投影 | 明确畸变模型、对应分辨率/ROI/binning 的内参、原始板图和独立复验误差；深度还需单位、无效值和 RGB–Depth 对齐验证 | 非零 `K`、驱动参数稳定及 `D/K/R/P` 快照均不足以声明几何精度通过 |
| 相机目标转换到机器人/桌面坐标、跨相机融合或视觉驱动的运动规划 | 上述几何内参及手眼、顶部外参、公共参考、FK/URDF 和动态 TF 链、独立保留姿态复验 | 当前缺少这些验收产物；禁止用单位矩阵、猜测位姿或只存在 TF 数值代替 |

每条训练 Episode 还需单独验收时间配对、动作来源、主从/夹爪映射、完成帧、技能与结果，随后按训练划分计算归一化统计。相机证据通过不会自动使 Episode 合格。单臂数据实际只需顶部与所选腕部；本文工具为三相机装置做统一核对，其三机总状态不能替代单臂 Episode 审计。

[`collection_camera_info_snapshot.20261105.json`](../config/collection_camera_info_snapshot.20261105.json) 中旧字段 `missing_for_training` 把外参列为训练缺项，是早期混合用途清单。保留该历史文件及原哈希，避免破坏已有引用；对本文明确的原始 RGB+关节路径，以新建的 [`camera_training_contract.v1.json`](../config/camera_training_contract.v1.json) 说明必需项。若改用深度、去畸变、笛卡尔动作或世界坐标目标，必须重新审定输入契约。

## 2026-11-06 当前装置复核

三机继续使用 [`camera_map.json`](../config/camera_map.json) 的历史绑定。2026-10-04 操作员曾通过画面确认安装角色，详见 [`CAMERA_PAIRING.md`](CAMERA_PAIRING.md)。`position_verified=true` 是该历史声明，不能自动成为今天的安装复验记录。

本次对已运行的驱动订阅 RGB/CameraInfo，调用只读 `GetDeviceInfo` / `DeviceInfo`，读取 USB sysfs；没有重启相机、修改参数、访问 CAN 设备或移动机器人。当前结果与源包哈希写入 [`camera_calibration_status.20261106.json`](../config/camera_calibration_status.20261106.json)。

| 角色 | 驱动实际报告序列号 | USB 序列号 | 当前出流/内参/安装状态 |
| --- | --- | --- | --- |
| 左腕 | `CV27561000MR` | `CV27561000MR` | `640×480 rgb8`，正确光学 `frame_id`；本次参数稳定且与旧快照一致；当前安装复核待补 |
| 右腕 | `CV275610002H` | `CV275610002H` | `640×480 rgb8`，正确光学 `frame_id`；本次参数稳定且与旧快照一致；当前安装复核待补 |
| 顶部 | `338122301303` | `250343060799` | `640×480 rgb8`，`D455_color_optical_frame`；当前 `K/P` 与旧快照不同，版本一致性未通过；当前安装复核待补 |

最终现场采集于 `2026-11-06 16:40:37～16:40:45 +08:00`，订阅端接收左腕 RGB/CameraInfo `236/242`、右腕 `238/241`、顶部 `237/240` 条；三台保存的样本 RGB 与 CameraInfo Header 差均为 `0 ns`。这些计数不等于设备帧率，也不表示整条 Episode 无丢帧。原始来源在[本机最终证据包](/home/mips/pi05_datas/validation/camera_evidence_20261106_task2_final/manifest.json)，清单 SHA-256 为 `7f243c721fa1d1600286294370bafb88cea4f1b819c9e4db85dd443a954b0616`。原图与较大的捕获文件留在数据目录，仓库中的状态文件保存完整 CameraInfo 样本、关键结果和哈希索引。

三台 USB 均为 `5000M`，发布 RGB/CameraInfo 的节点各唯一且符合配置。D455 的只读设备信息同时给出 `serial_number=338122301303`、`firmware_update_id=250343060799`，后者也与 USB 枚举一致。不能把 USB 值填进 RealSense 驱动 `serial_no`。

顶部 `fx` 从 `388.55841064453125` 变为 `388.6639404296875`，`fy` 从 `387.9939880371094` 变为 `388.099365234375`，变化分别为 `0.10552978515625 px` 和 `0.105377197265625 px`。本次捕获区间内参数稳定；现有证据不能解释变化原因，也不能把这个差异当成标定误差。工具以绝对容差 `1e-6` 核对版本一致性，**此门限用于检测配置变化，不是重投影精度指标**。保留旧快照，另存新快照并复核图像/预处理、设备和变化来源后，才建立新契约版本；不得为通过而直接放宽门限。

两台 Orbbec 当前均为 `distortion_model=plumb_bob`、`D[8]`；[ROS Humble 的 CameraInfo 定义](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/CameraInfo.msg)规定该模型为 5 个系数。工具保留驱动原值并报告语义警告，不擅自截断 `D` 或改名为 `rational_polynomial`。只输入原始 RGB 且不做几何变换时，该警告不单独否决来源记录；用于去畸变、深度反投影或求外参前，必须查清驱动/SDK 的系数含义并以板图独立复验。

这里的几何用途指按标定参数执行去畸变、像素射线、反投影或空间坐标变换。模型内部的 resize/pad 和训练增强不要求外参；它们仍须固定预处理版本并保证训练、推理路径一致，不能把 `640×480` 的内参数值直接当成缩放后像素坐标的内参。

本次样本可见左腕附近香蕉与右侧纸箱、右腕前方夹爪及左侧纸箱、顶部双夹爪与桌面，视角与历史角色相容。这只是画面观察，不能代替设备标签/安装实体核对或证明相机刚性安装未变化。本文没有生成已通过的安装复核记录。

当前没有为本次装置提供可重算的标定板数据、独立保留姿态残差、两腕手眼、顶部到机器人/桌面及公共坐标系验收。所有几何通过字段保持 `false`。

## 只读采集与离线验收工具

入口：[`readonly_camera_evidence.py`](../tools/camera_calibration/readonly_camera_evidence.py)。`capture` 只消费 RGB/CameraInfo，并调用契约中允许的两种设备信息接口；没有参数写入、图像发布、机器人控制话题或 CAN 操作。ROS 节点会有 DDS 发现和客户端通信；“只读”指不更改设备与机器人状态。

已有相机驱动运行时，在新目录采集。目录不得已存在；版本名应换成实际日期与序号：

```bash
cd /home/mips/piper_ros
source /opt/ros/humble/setup.bash
/usr/bin/python3 tools/camera_calibration/readonly_camera_evidence.py capture \
  --duration 8 --version camera_evidence_YYYYMMDD_v1 \
  --output /home/mips/pi05_datas/validation/camera_evidence_YYYYMMDD_v1
```

采集退出 `0` 表示证据已完整保存并通过自身离线重算，不表示所有验收项目通过。无出流或不符合契约时也会保留失败项，不能只看退出码。中断/工具异常留下没有完整清单的目录时，保留该目录为未完成来源，用新版本重采；不覆盖或修补旧包。

包内包含：

- `contract.json`、`camera_map.json`、`baseline.json`：按原文件字节复制的冻结输入。
- `collector.py`：本次实际采集工具源文件；`capture.json` 绑定其哈希和仓库 HEAD，脏工作区时以此源文件哈希为准。
- `capture.json`：设备信息原始响应、USB 枚举、发布者节点、全部接收样本的 Header 与规格、完整 CameraInfo（含 ROI/binning）、驱动版本、包描述与驱动可执行文件哈希。
- `*.rgb.bin`、`*.rgb.png`：每台相机一组近时 RGB/CameraInfo 样本的原始图像负载与无缩放、无色彩修改的 PNG；不是完整标定数据集或整段视频。
- `report.json`：冻结的自动检查和未完成项。
- `manifest.json`：上述所有文件的逐文件 SHA-256；清单自身哈希在命令输出及外部索引留存。

独立机器无需 ROS，使用 Python 标准库即可核对包完整性并重算报告：

```bash
python3 tools/camera_calibration/readonly_camera_evidence.py verify \
  /home/mips/pi05_datas/validation/camera_evidence_20261106_task2_final

python3 tools/camera_calibration/readonly_camera_evidence.py audit \
  /home/mips/pi05_datas/validation/camera_evidence_20261106_task2_final
```

`verify` 退出 `0` 仅表示文件集合、哈希、RGB 原始负载与 PNG、冻结报告重算一致；输出明确保留 `geometry_verified=false`。`audit` 在相机证据合格时退出 `0`，未验收时退出 `2`，文件损坏或工具异常退出 `1`。本次预期 `audit` 退出 `2`，因为顶部参数变化及安装复核仍待处理。后续验收算法升级时，保留历史 `collector.py` 作为重放版本；用 `python3 包目录/collector.py verify 包目录` 重放旧包。

### 安装角色复核

生成独立记录模板，不修改已冻结包：

```bash
python3 tools/camera_calibration/readonly_camera_evidence.py mount-review-template \
  /home/mips/pi05_datas/validation/camera_evidence_20261106_task2_final \
  --output /home/mips/pi05_datas/validation/camera_mount_review_YYYYMMDD_v1.json
```

模板预填 `capture_sha256`、实际序列号和本次三张 PNG 哈希，但 `confirmed=false`，复核人、时间、安装批次和观察描述均为空，不能自动通过。

本次已生成[待复核模板](/home/mips/pi05_datas/validation/camera_mount_review_20261106_task2_pending.json)，未替复核人填写确认结果。

复核人应按设备标签/既有物理编号和实时画面确认顶部、左右腕的实体归属、镜头方向、固定件、视野及遮挡，填写 `reviewer`、带时区且不早于采集结束的 `reviewed_at`、不可混用的 `installation_epoch_id`，并逐相机填写 `observation` 后确认。必要时只遮挡镜头以识别角色，不要求机器人运动。相机换位、支架松动、重新安装、焦距/分辨率/裁剪/去畸变流程变化时新建安装批次和来源版本。

```bash
python3 tools/camera_calibration/readonly_camera_evidence.py audit \
  /home/mips/pi05_datas/validation/camera_evidence_20261106_task2_final \
  --mount-review /home/mips/pi05_datas/validation/camera_mount_review_YYYYMMDD_v1.json
```

复核记录不豁免顶部基线变化。即使最终 `training_camera_evidence_ready=true`，工具仍输出 `intrinsic_geometry_verified=false`、`extrinsic_geometry_verified=false`、`motion_planning_geometry_ready=false`、`episode_training_eligible=not_evaluated`。

### 版本、哈希和 Episode 关联规范

1. 原始包只增不改；版本格式建议 `camera_evidence_YYYYMMDD_vN`。不要以改名复用不同内容。配置中的 `version_id`、外部索引中的 `manifest.json` 哈希和文件字节共同识别来源，版本名本身不是证明。
2. 文件哈希按原字节使用 SHA-256。JSON 内容摘要仅在明确标记为对象摘要时按 UTF-8、排序键、无多余空白、拒绝 NaN 的规范编码；不得将其与文件 SHA-256 混用。人工复核的对象摘要和原文件摘要分别输出。
3. 本次新契约固定旧 `camera_map` 与内参快照的原字节哈希。参数变化时另存新快照、新契约及差异说明并重新复核；历史快照仍保留原引用。清单证明完整性，不是签名，也不能替代可信存储和复核人责任。
4. 每条后续 Episode 应在独立来源索引中记录 `episode_id`、原始 Episode/bag 哈希、所需相机角色、安装批次、相机证据清单哈希、安装复核文件哈希、预处理版本；训练转换清单和 checkpoint 再绑定该来源索引哈希。此工具不改已有 Episode 或转换器；主集成流程须显式消费这些关联，不能只把 `calibration_version` 填成任意非空字符串。
5. 短时留档只检查本次来源一致性；逐 Episode 的跨相机、RGB/关节/动作 Header 配对、帧缺口和颜色预处理另验。驱动内部 `enable_sync` / `enable_frame_sync` 不证明三机硬件同步。

## 几何标定待补采清单

[`camera_geometry_checklist.v1.json`](../config/camera_geometry_checklist.v1.json) 是未完成模板，所有矩阵和几何阈值均为 `null`，不是求解结果。当前工具不求解内外参，也不自动验收几何。实际几何用途明确后，先按任务允许误差确定门限与覆盖范围，再采数据和求解；不能以“保存成功”替代验收。

| 阶段 | 可复现来源与验收要求 |
| --- | --- |
| 内参/畸变 | 实物板照片与尺寸/不确定度、图案/字典/角点定义；覆盖画面边缘、距离、倾角的原始 RGB 与同次 CameraInfo；冻结模型、分辨率和预处理；留出未参与求解的图像，报告重投影误差分布及失败样本。可在机器人保持静止时改变板姿态。 |
| 两腕手眼 | 两个相机分别绑定序列号和末端刚性安装批次；提供多组非退化静止姿态、原图/角点、关节反馈 Header、实际末端/基座坐标定义、FK/URDF 哈希和零位来源；独立保留姿态检验平移、旋转及重投影误差。获得不同机器人姿态须另行授权，本任务不执行。 |
| 顶部与公共参考 | 说明板在基座或桌面的参考如何测得、单位/方向/不确定度，绑定顶部固定安装；若使用两侧机器人共同坐标，还需两基座间有来源的变换。不能把视觉上平行的桌面或对称摆放直接当共同坐标。 |
| 跨相机复验 | 用共同参考点和未参与拟合的姿态验证闭环误差、工作区覆盖与时间配对；保存输入文件哈希、求解器/依赖版本、参数、残差和拒绝样本。腕部随关节移动，不能保存一份常量“腕到顶部外参”覆盖所有姿态。 |
| 深度用途 | 另录实际使用的深度、配套 CameraInfo、编码/单位/无效值、RGB–Depth Header 配对及实物距离/对齐误差；本次 RGB 工具不验收深度。 |

变换采用列向量约定 `p_target = T_target_from_source @ p_source`，平移单位 `m`。单侧链应明确为：

```text
T_base_from_wrist_camera(t) = T_base_from_end(q(t)) @ T_end_from_wrist_camera
T_top_from_wrist(t) = inverse(T_base_from_top_camera) @ T_base_from_wrist_camera(t)
```

若左右基座不同，还须显式加入经验证的基座间变换。`CameraInfo.R` 是校正矩阵，`P` 是投影矩阵；其中出现单位阵或零平移不意味着机器人手眼/顶部外参为单位阵。

需要补采标定原图及机器人状态时，可由只读 bag 录制器消费现有话题；以下命令不会触发机器人运动，也不包含求解或验收声明：

```bash
source /opt/ros/humble/setup.bash
source /home/mips/piper_ros/install/setup.bash
ros2 bag record -o /home/mips/pi05_datas/validation/camera_geometry_YYYYMMDD_v1 \
  /camera_wrist_left/color/image_raw /camera_wrist_left/color/camera_info \
  /camera_wrist_right/color/image_raw /camera_wrist_right/color/camera_info \
  /camera_top/D455/color/image_raw /camera_top/D455/color/camera_info \
  /piper_vector/left/slave_feedback /piper_vector/right/slave_feedback
```

目标目录须不存在；正常结束并确认所有录制文件关闭后，按文件排序保存 `metadata.yaml`、全部分片及板/姿态记录的 SHA-256。ROS 接收时间和 Header 时间分开保存，不能以 bag 写入时间代替曝光或关节源时间。单次静止场景留档不足以求解手眼。

## 2026-11-10 当前启动批次验收

三路驱动于 2026-11-10 重新启动。新采集的[只读证据包](/home/mips/pi05_datas/validation/camera_evidence_20261110_v3/manifest.json)核对了设备实际序列号、USB 枚举、唯一发布者、`640×480 rgb8`、CameraInfo 与 RGB 时间配对；三路自动检查均通过。操作者在本对话中确认左腕、右腕和顶部安装角色、固定件、镜头方向及任务视野。确认结果绑定本次三张原图、设备序列号和安装批次，保存在[安装复核记录](/home/mips/pi05_datas/validation/camera_mount_review_20261110_v3.json)。最终[审计结果](/home/mips/pi05_datas/validation/camera_training_acceptance_20261110_v3.json)为 `training_camera_evidence_ready=true`。

顶部 D455 的 `K/P` 与 2026-11-05、2026-11-06 快照不同。本次启动的两份独立采样中值相同，左右腕参数也与旧版一致。已保留旧快照，另建[`collection_camera_info_snapshot.20261110.json`](../config/collection_camera_info_snapshot.20261110.json)与[`camera_training_contract.v2.20261110.json`](../config/camera_training_contract.v2.20261110.json)，将本次实际 CameraInfo 作为新版本；没有推断数值变化的物理原因，也没有放宽比较门限。新启动批次若再变化，须重新采证和版本化，不能沿用本次通过结论。

该结论仅适用于当前**原始 RGB + 关节绝对位置**训练输入的相机来源，见[状态索引](../config/camera_calibration_status.20261110.json)。两台 Orbbec 的 `plumb_bob + D[8]` 警告保留；内参几何精度、手眼、跨相机外参及运动规划几何仍未验收。每条新 Episode 仍须绑定相机来源并接受逐帧审计，不能由这份相机通过记录直接获得训练资格。

## 工具验证

```bash
/usr/bin/python3 -m pytest -q tools/camera_calibration/test_readonly_camera_evidence.py
```

测试覆盖实际设备序列号不符、重复/错误发布者、USB 缺失、参数/ROI/光学坐标改变、时间戳倒退、近时配对缺失、数据损坏、安装复核错绑、非有限数、ROS 数值类型、文件篡改及重新计算的伪报告。合成通过样例仅证明工具逻辑，不是现场标定。现场另以三机短时采集、冻结报告重算和 `audit` 预期不通过验证当前事实。
