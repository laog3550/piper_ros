# Pi05 CAN 映射

本分支使用稳定的角色名，而不用 Linux `can0` 的枚举顺序：

| 角色 | 接口 | USB 端口 | 波特率 |
| --- | --- | --- | --- |
| `master_left` | `can_fl` | `1-11:1.0` | 1 Mbps |
| `master_right` | `can_mr` | `1-4:1.0` | 1 Mbps |
| `follower_left` | `can_fr` | `1-13:1.0` | 1 Mbps |
| `follower_right` | `can_ml` | `1-2:1.0` | 1 Mbps |

已确认的接口名、USB 端口、适配器序列号与波特率保存在纳入版本管理的
`config/pi05_can_map.json` 中。Python 工具在按角色选定接口之前会校验这四个
字段。

该 Python 工具是只读的。它不会把接口拉起、不会修改波特率、不会使能机械臂，
也不会发送任何 CAN 帧：

```bash
cd /home/mips/piper_ros
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select piper
source install/setup.bash
piper_pi05_can show
piper_pi05_can verify all
piper_pi05_can verify follower_right --require-up
```

激活全部四路已映射的接口（需要本机管理员权限）：

```bash
sudo /usr/bin/python3 /home/mips/piper_ros/src/piper/piper/pi05_can.py \
  --config /home/mips/piper_ros/config/pi05_can_map.json activate --apply
```

这里刻意使用源码文件形式调用：`sudo` 通常会清掉 colcon 的 `PYTHONPATH`，
因此在 root 环境下，符号链接安装的控制台入口可能找不到 `piper` 包的元数据。

该命令同时依据序列号和 USB 端口识别适配器，通过临时接口名化解循环的接口名
冲突，配置 1 Mbps，并把四路 SocketCAN 链路拉起。它不启动任何 ROS 节点，也不
发送任何 Piper 运动指令。

例如，在 `can_ml` 上启动右从臂：

```bash
ros2 launch piper start_single_piper.launch.py \
  can_port:=can_ml auto_enable:=false gripper_exist:=false
```

上表是经操作者确认的实体机械臂映射。上游的 `can_muti_activate.sh` 包含相同的
四组 USB 端口与接口对应关系，而 `start_two_piper.launch.py` 默认使用两个从臂
接口：左侧 `can_fr`、右侧 `can_ml`。若当前 Linux 中接口与序列号的对应关系与
此不符，请先修复持久化的链路命名再进入控制阶段；不要仅凭接口名去选择机械臂。
