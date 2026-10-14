# Pi05 两 CAN 接口与主从臂映射

机器映射以 `config/pi05_can_map.json` 为准，每侧一个 USB-CAN 适配器：

| 配置键 | 默认接口 | 连接机械臂 |
| --- | --- | --- |
| `left` | `can_left` | `master_left`、`follower_left` |
| `right` | `can_right` | `master_right`、`follower_right` |

USB 序列号及端口用于识别适配器，不能区分同一总线上的两台机械臂。

```bash
ros2 run piper piper_pi05_can show
ros2 run piper piper_pi05_can verify all --require-up
```

适配器映射变化后先校验配置；激活会改变接口状态，不要在运动期间执行：

```bash
sudo python3 src/piper/piper/pi05_can.py activate --apply
```

## 地址配置

当前方案使用官方角色：主臂 `0xFA`、从臂 `0xFC`，三个地址偏移均为 `0`。
每次只能隔离一台臂并广播一次 `0x470` 配置帧；左右两侧分别配置。
完整命令、上电顺序和只读核验见[官方原生摇操操作说明](OFFICIAL_NATIVE_TELEOP.md)。
旧 `0xFC + 0x20` 的测量记录保存在[历史方案](TWO_CAN_HOST_TELEOP_IMPLEMENTATION.md)。
