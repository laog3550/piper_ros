# Pi05 CAN 映射

本分支使用稳定的角色名，而不用 Linux `can0` 的枚举顺序：

| 角色 | 接口 | USB 端口 | 波特率 |
| --- | --- | --- | --- |
| `master_left` | `can_fl` | `1-11:1.0` | 1 Mbps |
| `master_right` | `can_mr` | `1-4:1.0` | 1 Mbps |
| `follower_left` | `can_fr` | `1-13:1.0` | 1 Mbps |
| `follower_right` | `can_ml` | `1-2:1.0` | 1 Mbps |

## 警告：不要按接口名推断角色

上面四个接口名是本项目自定的（官方 SDK 的脚本用的是 `can_arm1`/`can_arm2`），
仓库里没有任何文档说明它们的构词规则，而且**这些名字极易被误读**。

字母与角色的实际对应关系如下：

| 接口 | 第 1 位 | 第 2 位 | 实际角色 |
| --- | --- | --- | --- |
| `can_fl` | `f` | `l` | `master_left` |
| `can_fr` | `f` | `r` | `follower_left` |
| `can_mr` | `m` | `r` | `master_right` |
| `can_ml` | `m` | `l` | `follower_right` |

第 1 位决定左右（`can_f*` 均为 left，`can_m*` 均为 right）。但第 2 位在左右两组
的含义**相反**：左侧 `l` 为主、`r` 为从，右侧 `r` 为主、`l` 为从。因此这套命名
不是自洽的编码，`l`/`r` 既不代表固定含义的 left/right，也不代表固定含义的
master/follower，只看名字无法判断角色。

按最自然的读法（`f`=follower、`m`=master、`l`=left、`r`=right）去解读，四个名字
里有三个会得出**错误**角色：

| 名字 | 直觉读作 | 实际是 |
| --- | --- | --- |
| `can_fl` | follower_left | **`master_left`** |
| `can_fr` | follower_right | **`follower_left`** |
| `can_ml` | master_left | **`follower_right`** |
| `can_mr` | master_right | `master_right` |

其中 `can_ml` 最危险：它会被读成「master left」，实际却是 `follower_right`，
主从与左右**同时**搞反；`can_fl` 次之，会被当成从臂而实际是主臂。

**因此：一切以本文件与 `config/pi05_can_map.json` 的角色名为准，绝不要从接口名
反推角色。** 在需要选择机械臂的任何脚本或命令行里，先查这张表。

`config/pi05_can_map.json` 是权威来源，其对应关系已通过两种独立手段确认：按
适配器序列号与 USB 端口校验接口身份（`piper_pi05_can verify`），以及逐台拖动
机械臂观察哪条接口读数变化（`piper_joint_watch`）。


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
ros2 run piper piper_pi05_can show
ros2 run piper piper_pi05_can verify all
ros2 run piper piper_pi05_can verify follower_right --require-up
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

## 只读映射确认

`piper_pi05_can verify` 只能证明「适配器没接错」，它校验的是接口名、序列号与
USB 端口三者是否一致，证明不了该接口上真的连着一台健康的 Piper 臂，也证明不了
左右没有颠倒。用 `piper_bus_probe` 补上这一层：

```bash
ros2 run piper piper_bus_probe                      # 默认普查四条接口
ros2 run piper piper_bus_probe --port can_fl --duration 2
```

该工具以接收-only 方式打开套接字，自行解码原始帧、不依赖 `piper_sdk`，代码中
没有发送路径。它报告每条接口上出现的全部 CAN ID、帧率、解码后的关节角度、
母线电压、使能位，以及反馈 ID 是否已被偏移。退出码 0 表示核心反馈帧齐全。

它判定的是一条 Piper 臂应有的 **20 个核心反馈帧**是否全部在流：

| 类别 | CAN ID | 标称帧率 |
| --- | --- | --- |
| 驱动器高速反馈（速度/力矩） | `0x251`–`0x256` | 200 Hz |
| 驱动器低速反馈（电压/温度/使能位） | `0x261`–`0x266` | 40 Hz |
| 机械臂状态 | `0x2A1` | 200 Hz |
| 末端位姿 | `0x2A2`–`0x2A4` | 200 Hz |
| 关节角度（每帧两个关节） | `0x2A5`–`0x2A7` | 200 Hz |
| 夹爪反馈 | `0x2A8` | 200 Hz |

### 2026-09-23 检测结果

四条接口的 20 个核心帧全部齐全，退出码 0。四台臂报出**四个互不相同且数值合理**
的关节角度，说明它们是四台独立的物理设备，而不是同一台被读了四次。

| 接口 | 角色 | 不同 ID 数 | 布局 | j1 角度 | 母线电压 |
| --- | --- | --- | --- | --- | --- |
| `can_fl` | `master_left` | 25 | 常规 | −6.993° | 23.0 V |
| `can_mr` | `master_right` | 25 | 常规 | −6.637° | 23.0 V |
| `can_fr` | `follower_left` | 20 | 常规 | −4.778° | 23–24 V |
| `can_ml` | `follower_right` | 20 | 常规 | −2.921° | 23–24 V |

**结论：映射关系正确。** 四条接口各有完整的 Piper 反馈帧、帧率正常、供电正常、
角度合理，无 ID 冲突。

### 两台 master 臂多出 5 个未定义 ID

`can_fl` 与 `can_mr` 各比从臂多出 5 个 CAN ID，且两条总线上的这 5 个 ID
**完全一致**（ID、频率、载荷都相同）：

| CAN ID | 频率 | 载荷 |
| --- | --- | --- |
| `0x1C0` | 20 Hz | `00 00 00 00 00 00 00 00` |
| `0x1C1` | 40 Hz | `00 00 00 00 00 00 00 00` |
| `0x1C2` | 20 Hz | `00 00 00 00 00 00 00 00` |
| `0x1C3` | 20 Hz | `00 00 00 00 00 00 00 00` |
| `0x212` | 1 Hz | `04 01 0f 02` |

这 5 个 ID **不在 `piper_sdk` 的 CAN ID 定义中**，官方协议文档里也没有。它们的
语义未能确定。判断它们不构成映射问题，依据是：两条 master 总线发出的内容完全
一致（若是误挂的外部设备，不会恰好与另一条总线发出相同 ID、相同频率、相同
载荷），载荷为全零或恒定值，且 master 与从臂的适配器序列号前缀本就不同
（`0040…` 对 `004B…`/`004C…`），提示是不同批次或不同固件。

若要进一步确认，可在允许发送查询帧时读取固件版本比对，本阶段的检测刻意保持
了 master 总线发送计数为 0。

### 四台臂当前都在常规布局

判断依据是**反馈 ID 未偏移**。`MasterSlaveConfig`（CAN `0x470`）把臂设为示教
输入臂后，其反馈 ID 会整体偏移 `0x10` 或 `0x20`（`0x2Ax` → `0x2Bx`/`0x2Cx`），
同时该臂会主动发送 `0x151`、`0x155`–`0x157`、`0x159` 等关节模式控制指令。
实测四条总线上既没有偏移 ID，master 侧也没有出现上述控制指令 ID，因此**两台
master 臂目前仍是常规（运动输出臂）状态，尚未被配置为示教输入臂**。未收到
`0x470` 的臂默认就是常规状态。

偏移量无法由单个 ID 判定：高速反馈 `0x251` 加 `0x20` 与低速反馈 `0x261` 加
`0x10` 都落在 `0x271`，只有整组 ID 才能区分，`piper_bus_probe` 因此按整组判定。

## 拖动法确认接口与实体的对应

`piper_bus_probe` 能证明每条接口上都有一台健康的 Piper 臂，但**证明不了哪台接口
对应你手上哪一台机械臂**。区分这一点最直接的办法是拖一下：只拖动一台臂，看哪条
接口的读数在变。

```bash
ros2 run piper piper_joint_watch
```

程序实时显示四条接口各自的六个关节角度（度），并给出两样东西帮你判读：

- **当前角度**：四台臂的绝对角度，静止时应各自稳定不动。
- **相对基准的最大位移**：自按下 `r` 固定基准以来，每个关节偏离基准的最大幅度。
  记录的是**峰值**而不是当前值，所以即使你把臂拖走又拖回原位，位移记录仍然保留，
  不会出现「拖了却看不到」的情况。

底部一行会直接写出**哪条接口正在移动**。按 `r` 重置基准，按 `q` 退出；退出时打印
一张总结表，列出每条接口上被拖动过的关节及其位移幅度。

判读方式：

| 观察到的现象 | 结论 |
| --- | --- |
| 拖动某台臂时，**只有一条**接口的位移变化，且接口名与预期一致 | 映射正确 |
| 拖动某台臂时，两条接口同时变化 | **映射有误**，或有臂被机械耦合，须停机排查 |
| 拖动某台臂时，没有任何接口变化 | 该臂未接入被监视的总线，或接线/接口名有误 |
| 某条接口始终显示「无角度帧」 | 该接口上没有可通信的 Piper 臂 |

用 `--arm` 可以只看其中几条接口，例如只确认两条 master：

```bash
ros2 run piper piper_joint_watch --arm can_fl --arm can_mr
```

### 运行前要知道的事

- **本工具绝对只读。** 它只调用 `recv()`，代码中没有任何发送路径，不会使能、
  失能或运动任何关节。实测四条接口的 `tx_packets` 在运行前后完全一致
  （`can_fl`、`can_mr` 保持 0）。所以人手接触机械臂时运行是安全的。
- **四台臂当前都处于失能状态**，因此可以自由拖动。失能状态下拖动需要克服重力，
  承载重力的关节会下坠，这是正常的，不是故障。
- 请**逐台**拖动，每次只碰一台臂，判读才会清晰。
- 运行本工具期间不要同时运行控制节点或任何会发送 CAN 帧的程序，以免拖动时的
  读数被别的指令干扰。
- 角度原始分辨率为 `0.001` 度，显示到小数点后三位。判定「被移动过」的默认阈值
  是 `5.0` 度，可用 `--moved-threshold` 调整。


