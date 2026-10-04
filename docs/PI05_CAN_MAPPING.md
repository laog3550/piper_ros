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

当前方案使用主臂 `0xFC + feedback_offset=0x20 + ctrl_offset=0x20`，从臂保持
`0xFC + 0x00`。没有 `0xFA` 固件随动入口。

仅在从臂已断电或断开 CAN、总线上只剩主臂时配置：

```bash
ros2 run piper piper_master_slave --port can_left
ros2 run piper piper_master_slave --port can_left --isolated-master --send
```

工具只发送 `0x470 FC 20 20 00 00 00 00 00`，不清零偏移、不使能、不发送运动指令。
发送后给主臂断电重启，先单臂确认 `0x2C1`～`0x2C8` 约 `200 Hz`、默认窗口消失；
再连接从臂，用以下命令确认两套地址：

```bash
ros2 run piper piper_two_can_identity --side left
ros2 run piper piper_joint_watch --arm can_left:主@0x20 --arm can_left:从
```

`0x251`～`0x266` 在当前固件上不偏移，不能用它们识别实体主臂或从臂。
`0x471` 是整侧使能广播；`0x470` 是配置广播，绝不能用它作为正常退出手段。

右侧需独立完成同样的离线配置。遥操作按 [操作说明](PIPER_TELEOP.md) 执行；
现场测量记录见 [实现方案](TWO_CAN_HOST_TELEOP_IMPLEMENTATION.md)。
