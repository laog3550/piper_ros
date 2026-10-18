# 三相机重新配对记录

## 2026-10-16 三小时复验

在 `37d6989` 的相机入口上连续订阅三机 RGB-D `10800 s`，当前集成分支未改动该入口。左腕彩色/深度各 `324239` 帧，右腕分别 `324212/324213` 帧，顶部为 `323611/323609` 帧；各流约 `30 fps`，左右腕无超过 `50 ms` 的 Header 间隔或非递增时间戳，顶部彩色有一次重复 Header 时间戳，顶部深度有一次 `66.75 ms` 间隔。三机 RGB/深度最近 Header 差的 `p99` 均不超过 `0.19 ms`；顶部有两张彩色帧未在 `5 ms` 内找到深度配对。全部监视进程和 launch 以退出码 `0` 完成，未留残余相机进程。该结果证明三机可持续采集及正常停机，但不满足“全程零异常帧”条件；后续采集仍应保存每次 Episode 的六路帧计数和 Header 缺口统计。

## 2026-10-15 Orbbec 停机与重启复验

左腕现已接入 `5000M` USB3 链路，两台 Gemini 305 均为 USB3.2。改动前，
三机 RGB-D 连续运行约 30 分钟：左、右腕约 `30.02 fps`，顶部约
`29.69 fps`，六路图像无时间戳倒退；随后 `SIGINT` 停机，两台 Orbbec
的 `component_container` 均以 `-11` 退出。另一次约 50 秒运行的结果相同。
两次日志均显示 SDK 已完成停流和设备资源释放，D455 正常退出。
退出崩溃发生在采集停止阶段，但不能以此推断录制文件必定完整；应先停止
录制并核对文件，再关闭相机。

驱动原有 Humble 启动文件把 Orbbec 放进 `component_container`。对照中，
相机子进程与 `ros2 launch` 同属一个进程组；从终端按 `Ctrl+C` 后，
子进程会同时收到终端信号和 launch 转发的 `SIGINT`。只向 launch 主进程
发送一次 `SIGINT` 时，两台相机均正常退出。该证据符合重复信号触发
停机竞态，但尚未定位驱动内部的精确缺陷。

现行入口改用驱动提供的独立 `orbbec_camera_node`，并让两台 Orbbec 各自
运行在独立进程组；launch 仍负责在 `Ctrl+C` 时向它们发送一次 `SIGINT`。
保留原序列号、`640x480 @ 30 fps` YUYV/Y16、深度对齐、帧同步和 raw
图像参数，话题契约见下表。两轮三机并发重启试验中，每轮六路图像和六路
CameraInfo 均有帧；第一次 15 秒接收左腕 RGB/Depth `423/423`、
右腕 `451/451`、顶部 `424/417`；第二次 20 秒分别为 `572/572`、
`600/600`、`575/566`。两轮 `Ctrl+C` 后三个相机进程均以退出码 `0`
正常结束，第二轮重新启动后再次出流。上述是订阅端接收数。
后续以现行独立节点入口完成 30 分钟三机 RGB-D 流验收：六路无超过
`50 ms` 的 Header 间隔、时间戳倒退或 USB reset。但停止时，两台
Orbbec 均未在 launch 默认 `5 s` 内退出；launch 升级为 `SIGTERM`
后约 `1.3 s` 才正常退出。约一分钟的重复运行也出现相同停机超时。
这两轮没有复现早前的 `-11`。

现将两台 Orbbec 的 `SIGINT` 等待窗口从默认 `5 s` 延长到 `15 s`，
保留进程隔离及原 RGB-D 参数。更新后一次右腕约 66 秒、两次三机
约 67 秒和约 33 秒运行均正常出流，停止时 Orbbec 约 `1.3 s`
干净退出，未升级 `SIGTERM`。此调整保留为驱动退出的等待余量。

随后三机连续运行约 3 小时，停止时 D455 在 `5 s` 后、两台 Orbbec
在 `15 s` 后才被升级为 `SIGTERM`，三者随后正常退出。检查该次
launch 日志发现，停止时没有向任何相机子进程发送 `SIGINT` 的记录；
SDK 在等待期间仍报告正常出帧。Humble 的交互式 launch 在自身收到
`SIGINT` 时，默认假定终端也已把信号发给同进程组的子进程，因此跳过
逐个转发。相机使用 `setsid` 独立进程组后不满足这个假定；只向 launch
主进程发送 `SIGINT` 时，三台相机也不会收到终端广播。

现行入口在交互式 launch 关闭时显式向三台隔离的相机各发送一次
`SIGINT`；非交互式 launch 保留其自带的信号转发。D455 改为直接启动
`realsense2_camera_node`，并显式关闭红外及运动传感器，维持原序列号、
话题和 RGB-D 参数。短时复验中，终端 `Ctrl+C` 与仅向 launch PID
发送 `SIGINT` 两种路径均有三台子进程收到 `SIGINT` 的记录，三者均
正常退出，未升级 `SIGTERM`。两轮启动的六路图像和六路 CameraInfo
均持续到达；后续三小时运行与停机复验结果见本文首节。

30 分钟订阅记录中，左腕 RGB/Depth 的接收墙钟相对 Header 时间戳
最大延迟为 `368.6/344.4 ms`，虽无 Header 帧间断，仍未通过接收延迟
验收。单次尖峰无法区分驱动、主机调度与订阅端积压，本次不据此更改流参数。后续三小时监视中六路最大接收延迟均低于 `100 ms`；这不抹除早前的单次尖峰。

若后续仍遇到 Orbbec 异常退出或重启后零图像，先确认录制已停止并校验
输出，再确认旧相机进程已全部退出、对应设备仍在 USB3 枚举，之后只启动
一套相机入口并检查六路图像和 CameraInfo 均持续到达。不要让两个驱动
进程同时打开同一序列号；本次并行误启动曾与右腕
`The Color has already been started` 和零图像同时出现，该错误不能据此
归因于原先的停机崩溃。

## 2026-10-14 RGB-D 现场检查

入口现已按原序列号绑定同时启用 RGB、Depth 和驱动内的深度到彩色对齐，
三台均请求 `640x480 @ 30 fps`，关闭红外图像及点云。Orbbec 使用
`depth_registration:=true`、`enable_frame_sync:=true`，D455 使用
`align_depth.enable:=true`、`enable_sync:=true`。这两个帧同步参数只作用于各自
驱动内部；没有三台相机之间的触发线或统一时钟验收。

| 角色 | RGB 图像 / CameraInfo | 对齐深度图像 / CameraInfo | 原生深度图像 / CameraInfo |
| --- | --- | --- | --- |
| 左腕 | `/camera_wrist_left/color/image_raw` / `/camera_wrist_left/color/camera_info` | `/camera_wrist_left/depth/image_raw` / `/camera_wrist_left/depth/camera_info` | `/camera_wrist_left/depth/image_unaligned` / 配套 CameraInfo 未确认 |
| 右腕 | `/camera_wrist_right/color/image_raw` / `/camera_wrist_right/color/camera_info` | `/camera_wrist_right/depth/image_raw` / `/camera_wrist_right/depth/camera_info` | `/camera_wrist_right/depth/image_unaligned` / 配套 CameraInfo 未确认 |
| 顶部 | `/camera_top/D455/color/image_raw` / `/camera_top/D455/color/camera_info` | `/camera_top/D455/aligned_depth_to_color/image_raw` / `/camera_top/D455/aligned_depth_to_color/camera_info` | `/camera_top/D455/depth/image_rect_raw` / `/camera_top/D455/depth/camera_info` |

三台的彩色图像实测为 `rgb8`，深度为 `16UC1`，图像尺寸均为 `640x480`。
Orbbec 对齐深度的 `frame_id` 是各自 `camera_wrist_*_color_optical_frame`，
未对齐深度是 `camera_wrist_*_depth_optical_frame`；D455 对齐深度是
`D455_color_optical_frame`，原生深度是 `D455_depth_optical_frame`。
使用深度时应读取对应 `CameraInfo` 和 Header，不要凭话题名称假定内参相同、
已完成手眼标定或像素级对齐精度已经验收。Orbbec 的
`depth/image_unaligned` 的配套 CameraInfo 尚未确认；采集优先使用上表的
对齐深度。

单机 8 秒订阅检查中，两台 Orbbec 各流约 `30 fps`；D455 单机 5 秒
RGB 与对齐深度 150 对时间戳完全一致。Orbbec 左腕单机 RGB 与对齐深度
没有完全同戳，最近时间差约 `0.15–0.2 ms`；右腕单机时间差中位数
约 `0.001 ms`。三机并发进入稳态后的 15 秒接收数依次为：左腕 RGB/Depth
`400/401`（约 `26.7 fps`）、右腕 `427/428`（约 `28.5 fps`）、顶部
`399/399`（约 `26.6 fps`）。这是 Python 同进程订阅六路图像的接收数，
不等同于设备出帧率；该段驱动日志中两台 Orbbec 自报约 `30 fps`。
并发检查中 D455 的 RGB 与对齐深度接收时间戳匹配 `398` 对；两台
Orbbec 的相近时间戳仍不保证逐帧严格相等。下游应按 Header 时间戳
设容差配对，并记录超时与丢帧，不能按序号直接拼接。

当日左腕 `CV27561000MR` 经 `480M` USB2.1 链路连接。三机启动时左腕
约 36 秒后才出现稳定图像，之前驱动报告帧数据长度不符、丢帧及一次
缓冲区大小异常。之后虽有约 `30 fps` 的驱动日志，仍不能声明该链路
稳定。尝试左腕 `320x240 @ 30 fps` 被驱动拒绝（无对应彩色 profile）；
`640x480 @ 15 fps` 仍出现损坏帧；`640x480 @ 30 fps` 的 MJPG 彩色流
在约 30 秒观察内没有图像。因此入口暂保留本次能进入稳态的
`640x480 @ 30 fps` YUYV 彩色流及 Y16 深度流。当时左腕仍需接入 USB3；
后续已接入并完成持续接收与重启复验，见本文首节和 2026-10-15 记录。物理断开重连、三机同步、RGB–Depth 对齐误差、深度有效值与内外参标定仍
尚待现场验收。

## 2026-10-04 检查结果

USB 控制器重启后已恢复，三台相机均完成独立 ROS 彩色图像验证，实测约 `30 fps`。
用户已根据本次画面确认：`00MR` 对应左腕、`002H` 对应右腕、D455 对应顶部。
相机身份与安装位置配对已完成。
当前映射保存在 [`config/camera_map.json`](../config/camera_map.json)，
已设置 `position_verified: true`。

| 已确认位置 | 型号 | ROS 驱动使用的序列号 | USB 序列号 | 本次枚举连接速度 |
| --- | --- | --- | --- | --- |
| 左腕 | Orbbec Gemini 305 | `CV27561000MR` | `CV27561000MR` | `480M` |
| 右腕 | Orbbec Gemini 305 | `CV275610002H` | `CV275610002H` | `5000M` |
| 顶部 | Intel RealSense D455 | `338122301303` | `250343060799` | `5000M` |

D455 的 USB 序列号与 RealSense SDK 序列号不同。本次驱动日志确认 SDK 枚举值为
`338122301303`；使用 `250343060799` 作为 `serial_no` 会提示找不到设备。
RealSense 启动参数可写为 `serial_no:=_338122301303`，避免纯数字被解析为数值类型。

恢复后的单机检查结果：

- `CV27561000MR`：`640x480`，约 `29.99 fps`；纸箱在画面右侧。
- `CV275610002H`：`640x480`，约 `29.96 fps`；纸箱在画面左侧，香蕉在夹爪前。
- D455：`640x480`，约 `29.73 fps`；桌面顶部视角。

固定配置应按驱动序列号绑定，不应保存临时 `/dev/videoN` 编号。

三路同时运行的 `30 s` 接收检查通过：左腕 `883` 帧、约 `29.48 fps`；
右腕 `881` 帧、约 `29.39 fps`；顶部 `851` 帧、约 `28.41 fps`。
这是订阅端接收速率，不能据此声明三路时间同步或完全无丢帧。
期间未出现新的 USB 控制器故障。
原始统计与三张画面保存在本机 `log/camera_pairing_20261004/`（不纳入 Git）。

停止测试时，D455 正常退出，两台 Orbbec 的 `component_container` 在收到 `SIGINT`
后以 `-11` 退出。该退出阶段异常尚未修复；相机测试进程均已结束，USB 设备仍正常枚举，
未出现控制器再次掉线。不能将本次图像验证视为驱动完整生命周期稳定性验收。

## 启动与位置核对

现行入口发布彩色和深度图像，统一请求 `640x480 @ 30 fps`。
Orbbec 只启用已安装的 raw 图像传输插件，关闭点云。
三台相机间隔 `4 s` 启动，每台驱动的 launch 参数作用域相互隔离。
该入口不会启动机械臂控制节点。

```bash
cd ~/piper_ros
source /opt/ros/humble/setup.bash
colcon build --packages-select piper --symlink-install
source install/setup.bash
ros2 launch piper start_cameras.launch.py
```

仅启动一台时使用以下命令之一，结束后按 `Ctrl+C`，再启动下一台：

```bash
ros2 launch piper start_cameras.launch.py camera:=left_wrist
ros2 launch piper start_cameras.launch.py camera:=right_wrist
ros2 launch piper start_cameras.launch.py camera:=top
```

RGB-D 话题见本页开头的现场检查表。

可用 `camera_map:=/绝对路径/camera_map.json` 或环境变量 `PIPER_CAMERA_CONFIG`
指定其他机器映射。默认读取 `~/piper_ros/config/camera_map.json`，不会回退为自动选择首台设备。
配置中的序列号必须保留为 JSON 字符串，三台的序列号和节点名称不得重复。

本次现场位置确认已完成。以后更换相机或调换安装位置时，先将 `position_verified`
设为 `false`，再通过画面或逐个遮挡镜头重新核对。若交换左右相机，应对调两条腕部配置的
`serial` 与 `usb_serial`，保留角色名称和 `camera_name`，重新验证后再设为 `true`。

2026-10-04 的检查仅验证 RGB 身份绑定；深度流验证结果见上文。

## 首次检查的中断记录

13:56:40，内核报告：

```text
xhci_hcd 0000:00:14.0: xHCI host not responding to stop endpoint command
xhci_hcd 0000:00:14.0: xHCI host controller not responding, assume dead
xhci_hcd 0000:00:14.0: HC died; cleaning up
```

随后相机、CAN 适配器、USB 键鼠等均从该控制器断开，`lsusb` 仅剩根集线器。
故障发生在相机测试期间，具体触发原因尚未确定。
本次启动的相机测试进程已全部停止；未修改机械臂控制配置或发送运动指令。

恢复需要管理员权限；本次环境执行 `sudo -n true` 返回需要密码。
在机械臂停止、相机与 CAN 程序均退出后，可由用户在可用终端尝试重新绑定该控制器：

```bash
sudo sh -c 'echo 0000:00:14.0 > /sys/bus/pci/drivers/xhci_hcd/unbind; echo 0000:00:14.0 > /sys/bus/pci/drivers/xhci_hcd/bind'
lsusb
```

该操作会重新枚举此控制器上的所有 USB 设备，包括 CAN 适配器和键鼠。
若无可用终端或恢复失败，保存工作后重启主机。
恢复后重新检查 CAN 接口配置，不要假设 USB 故障前的接口状态仍然有效。

## 后续验收

1. 同序列号相机停机重启后恢复对应话题已于 2026-10-15 复验；物理插拔后重新枚举仍待验收。
2. 后续启用机械臂前检查 CAN 接口；本次重启后 `can_left`、`can_right` 均为 `DOWN`，
   相机测试没有修改 CAN 接口状态。

这里的配对指相机身份与安装位置绑定；本次尚未执行内参、外参或手眼标定。
