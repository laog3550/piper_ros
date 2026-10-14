# 三相机重新配对记录

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

当前入口仅发布彩色图像，统一使用 `640x480 @ 30 fps`。
Orbbec 关闭深度流、帧同步及点云，只启用已安装的 raw 图像传输插件。
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

| 角色 | 彩色图像话题 |
| --- | --- |
| 左腕 | `/camera_wrist_left/color/image_raw` |
| 右腕 | `/camera_wrist_right/color/image_raw` |
| 顶部 | `/camera_top/D455/color/image_raw` |

可用 `camera_map:=/绝对路径/camera_map.json` 或环境变量 `PIPER_CAMERA_CONFIG`
指定其他机器映射。默认读取 `~/piper_ros/config/camera_map.json`，不会回退为自动选择首台设备。
配置中的序列号必须保留为 JSON 字符串，三台的序列号和节点名称不得重复。

本次现场位置确认已完成。以后更换相机或调换安装位置时，先将 `position_verified`
设为 `false`，再通过画面或逐个遮挡镜头重新核对。若交换左右相机，应对调两条腕部配置的
`serial` 与 `usb_serial`，保留角色名称和 `camera_name`，重新验证后再设为 `true`。

本次仅验证 RGB 身份绑定；深度流、内外参标定、硬件同步不在本次验证范围内。

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

1. 停止相机程序后进行插拔或重启，核对相同序列号仍进入对应话题；本次尚未完成此项验收。
2. 后续启用机械臂前检查 CAN 接口；本次重启后 `can_left`、`can_right` 均为 `DOWN`，
   相机测试没有修改 CAN 接口状态。

这里的配对指相机身份与安装位置绑定；本次尚未执行内参、外参或手眼标定。
