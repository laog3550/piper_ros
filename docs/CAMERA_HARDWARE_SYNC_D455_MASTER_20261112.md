# 三路相机硬件同步：D455 主、Orbbec 从（2026-11-12）

## 目的

当前三路相机是自由运行：D455 的 `depth_module.inter_cam_sync_mode=0`，两路 Orbbec 的 `sync_mode` 为 standalone，三路曝光相位不受控。episode_000007 的顶部—左腕相位差中位约 29.2 ms、p95 约 33.0 ms，超过当时的 30 ms SFT 对齐门禁；2026-11-12 当前采集/SFT 阈值已放宽到 40 ms，30 ms 保留为严格模式。

`enable_sync`（D455）和 `enable_frame_sync`（Orbbec）只同步单相机内部的 depth/color；`global_time_enabled` 和 `enable_sync_host_time` 只把时间戳对到主机时钟。它们都不控制三路相机的曝光相位，因此不能替代跨相机硬件触发。

本方案让 D455 作为主相机输出触发，两路 Orbbec 作为从相机接收触发。

## 接线

1. D455 的 sync out 接到分线器，再分别接到左腕、右腕 Orbbec 的 sync in。
2. 三路共地；使用厂商 sync 线或确认针脚、极性、脉冲宽度的转接线。
3. D455 的 sync GPIO 为 3.3V；Orbbec 的 sync IO 电平按相机手册确认。必要时通过 `orbbec_sync_io_voltage_level` 指定驱动期望的整数值；默认 `-1` 表示使用驱动默认。
4. 断电接线，不要带电插拔 sync 线。
5. 只能有一个触发源；从相机必须设置 `trigger_out_enabled=false`，否则多个输出可能争用同一条线。

## 启动

相机 launch 新增 `camera_sync` 参数，默认仍是 `standalone`，避免未接线时误启动：

```bash
cd /home/mips/piper_ros
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch piper start_cameras.launch.py \
  camera_fps:=15 camera_sync:=d455_master
```

可调参数：

- `camera_sync_delay_sec`：D455 主相机与从相机的启动间隔，默认 5 秒；D455 启动慢时可加到 8 秒。
- `orbbec_sync_io_voltage_level`：Orbbec sync IO 电平，默认 -1；按相机手册设置。若驱动要求枚举值，不要直接填 3.3，先查手册/驱动说明。
- `camera` 必须保持 `all`；方案 A 需要三路同时启动。
- `camera_fps` 用 15 或 30；D455 没有 640×480@10 档位。

`d455_master` 模式下的参数：

| 相机 | 参数 |
| --- | --- |
| D455 | `depth_module.inter_cam_sync_mode=1`（master）、`depth_module.output_trigger_enabled=true`、`depth_module.global_time_enabled=true`、`rgb_camera.global_time_enabled=true` |
| 左/右腕 Orbbec | `sync_mode=secondary_synced`、`trigger_out_enabled=false`、`frames_per_trigger=1`、`sync_io_voltage_level` 按需 |

启动顺序为 D455 → 左腕 → 右腕，默认间隔 5 秒。`d455_master` 模式会在 launch 日志里提示必须接好 sync 线；未接线时 Orbbec 可能不出流。

## 验证

1. 参数确认：

```bash
ros2 param get /camera_top/D455 depth_module.inter_cam_sync_mode   # 1
ros2 param get /camera_top/D455 depth_module.output_trigger_enabled # true
ros2 param get /camera_wrist_left/camera_wrist_left sync_mode       # secondary_synced
ros2 param get /camera_wrist_left/camera_wrist_left trigger_out_enabled # false
```

2. 三路 `ros2 topic hz` 都应稳定在配置帧率，且没有掉流。

3. 录 10 秒三路彩色图像，按 Header 计算顶部—左腕、顶部—右腕的最近帧差。目标：

- 中位 < 5 ms；
- p95 < 10 ms；
- 最大 < 30 ms。

4. 再录一条短 Episode，跑：

```bash
cd /home/mips/piper_ros/tools/episode_recorder
/tmp/piper-atomic-venv/bin/python audit_real_data.py \
  /home/mips/pi05_datas/banana_task_episodes \
  --episode-root /home/mips/pi05_datas/banana_task_episodes/episode_00000N \
  --gripper-calibration /home/mips/piper_ros/config/gripper_collection_calibration.20261112.json \
  --action-source follower_next_state \
  --max-camera-skew-ms 40 --max-source-skew-ms 40 --extract-valid-segments
```

确认完成帧在连续段内、16 步窗口成立。已有 episode_000007 不会因接好线而追溯变为合格；需要重新采集。

## 无线/外部触发（避免腕部到顶部的长线）

如果不想在顶部和腕部之间走 sync 线，用 `camera_sync:=external_trigger`：

```bash
ros2 launch piper start_cameras.launch.py \
  camera_fps:=15 camera_sync:=external_trigger
```

- D455：`depth_module.inter_cam_sync_mode=2`（slave）、`depth_module.output_trigger_enabled=false`。
- 左/右腕 Orbbec：`sync_mode=secondary_synced`、`trigger_out_enabled=false`、`frames_per_trigger=1`。
- 外部触发源必须先运行；三路相机都等待同一个触发。
- 外部触发可以是无线接收模块：顶部一个、左右腕各一个。接收模块只接相机 sync 口和短电源线，腕部模块固定在相机附近，不需要跨臂长线。
- 无线链路建议 15 Hz 广播、相对抖动 <5 ms。当前 40 ms 门禁很宽松（严格模式 30 ms），商用无线触发或 DIY ESP-NOW/nRF24L01 + MCU 输出 3.3V 脉冲均可；接收端脉冲宽度/极性按相机手册。
- 腕部接收模块会增加重量和电池/供电；固定牢靠，先低速测试，确认不干涉、不缠绕。
- 不同相机的触发到曝光延迟不同，可用 `trigger2image_delay_us` 等参数微调；目标顶部—左腕中位 <5 ms。
- 验证仍用同一套 10 秒 Header 相位差检查。

## 无额外硬件的 60 fps 方案

如果暂时不加触发硬件，可以把相机帧率提到 60：

```bash
ros2 launch piper start_cameras.launch.py camera_fps:=60
```

- 15 fps 帧周期 66.7 ms；60 fps 为 16.7 ms，最近帧/因果帧的相位窗口减到约 1/4。
- 当前 40 ms 门禁在 60 fps 下更可能通过，但这不是固定相位，仍需实测 skew。
- 代价：USB、CPU、磁盘和 CAN 录制负载约 4 倍；`config/episode_recorder.pi05.json` 的 `expected_fps` 要同步改成 60；SFT 转换仍按 10 Hz。先做短时负载测试，确认 raw CAN 没有丢帧。
- 30 fps 是折中，但 33.3 ms 周期仍可能接近 30 ms 严格门禁，必须实测。

## 2026-11-12 30 fps 实测与 15 fps 结论

30 fps 深度+彩色会使 Orbbec 发布帧率在 1–13 fps 间塌陷；关闭深度后 30 fps 纯彩色也只能到约 20 fps，控制台预览和 DDS 资源不稳定。最终未采用 30 fps，改为 15 fps 纯彩色固定曝光；三路稳定 15.0–15.2 fps。15 fps 顶部—左腕 p95 约 33 ms，当前 40 ms 阈值可用，30 ms 保留严格模式。相机证据 v7/验收 v7 对应当前 15 fps 固定曝光/白平衡批次（顶部 80 µs/gain 32）；30 fps v2、15 fps 自动曝光 v3、固定曝光但自动白平衡 v4、顶部 4600K v5、顶部 166 µs v6 作为历史证据。

现场把三路相机切到 30 fps（standalone，无 sync 线），10 秒在线采样：

| 相机对 | 中位 | p95 | 最大 | >30 ms |
| --- | ---: | ---: | ---: | ---: |
| 顶部—左腕 | 13.33 ms | 24.33 ms | 74.29 ms | 7 / 248 |
| 顶部—右腕 | 4.68 ms | 40.27 ms | 140.75 ms | 14 / 248 |

相比 15 fps 时顶部—左腕中位 29.24 ms、p95 32.97 ms，30 fps 明显改善；但仍有少量超过 30 ms 的帧，30 fps 不是固定相位。左臂 SFT 只使用顶部 + 左腕，右腕的较大偏差不进入左臂数据集；如果做右臂任务，需要单独看顶部—右腕。

`config/episode_recorder.pi05.json` 的 `expected_fps` 已改为 15，相机 launch 默认 `camera_fps=15`。30 fps 下图像写入和 USB/CPU 负载约为 15 fps 的两倍，第一条新 Episode 仍应检查 raw CAN 是否丢帧；如果完成帧仍被 40 ms 门禁拒绝，再考虑 60 fps 或无线触发，不要继续放宽。

## 顶部固定 15 fps 锁定（2026-11-12）

现场发现：D455 在 `640x480@15`、固定曝光、standalone 且关闭深度时，驱动打开的是 15 fps profile，实际却稳定输出 **16.59 fps**。这会让 SFT 的 `expected_fps=15`、相机缺口统计和顶部—腕部相位检查与实际不符。

对比实验（固定曝光 80 µs）：

| 顶部配置 | 实测 |
| --- | ---: |
| depth 关、standalone | 16.59 fps |
| depth 开、standalone | 16.59 fps |
| depth 关、主触发模式 | 16.59 fps |
| **depth 开、主触发模式** | **14.99 fps** |
| depth 开、standalone、自动曝光 | 14.99 fps |

结论：必须让 D455 以深度帧时钟作为触发主时钟，同时保留深度流；仅打开深度或仅关闭自动曝光都不够。

launch 新增 `top_fps_lock`（默认 `true`）：

- `camera_fps:=15` 时，顶部强制 `enable_depth=true`、`align_depth.enable=false`，并设置 `depth_module.inter_cam_sync_mode=1`、`depth_module.output_trigger_enabled=true`、`depth_module.global_time_enabled=true`、`rgb_camera.global_time_enabled=true`。
- 顶部深度只用于帧时钟，SFT 录制仍只录彩色；腕部参数不受影响。
- `enable_depth:=false` 仍可让腕部无深度，但顶部会单独启用深度；完全关闭顶部深度需显式 `top_fps_lock:=false`，此时顶部回到 16.59 fps。
- `camera_sync:=external_trigger` 的外部触发参数会覆盖顶部主模式。

2026-11-12 14:39 最终配置实测：顶部 14.988 fps，左腕/右腕 15.008 fps；顶部稳态无超过 100 ms 的缺口。15 秒重叠区间相位差：顶部—左腕中位 28.32 ms、p95 32.80 ms、最大 33.29 ms（0/210 >40 ms）；顶部—右腕中位 8.41 ms、p95 17.37 ms。该锁定只固定顶部单相机帧率，不替代三路硬件触发。

该启动批次的顶部 K 从 v1 的 `fx=388.6639, fy=388.0994` 变为 `fx=388.4529, fy=387.8886`（约 0.211 px），左/右腕不变。已新建：

- 内参快照：`config/collection_camera_info_snapshot.20261112_v2.json`
- 契约：`config/camera_training_contract.v4.20261112.json`
- 证据包：`/home/mips/pi05_datas/validation/camera_evidence_20261112_v8/`
- 安装复核：`/home/mips/pi05_datas/validation/camera_mount_review_20261112_v8.json`
- 验收：`/home/mips/pi05_datas/validation/camera_training_acceptance_20261112_v8.json`（`training_camera_evidence_ready=true`）
- 状态：`config/camera_calibration_status.20261112_v8.json`

控制台默认 `calibration_version` 已切到 `camera_info_snapshot_20261112_v2`。

## 失败时的回退

- 如果 D455 主 + Orbbec 从无法稳定锁定，先试 Orbbec 主 + D455 从（方案 B），或外部 15 Hz 触发源（方案 C）。
- 40 ms 是当前折中，不是硬件同步的替代；如果 40 ms 仍拒绝完成帧，再考虑 60 fps 或无线触发，不要继续放宽。不同 Episode 的相位仍可能不一致，模型会看到不一致的多模态时序。
- 软件上可以把 `camera_fps` 提到 30 来减小最近帧相位差，但需要重新验证 CAN 录制负载；它不是硬件同步的替代。

## 当前状态

launch 代码已支持 `camera_sync:=d455_master`，默认 `standalone`；`top_fps_lock` 默认 `true`，在 standalone 下单独把顶部 D455 锁到 15.0 fps，但不建立三路跨相机硬件触发。现场接线并重启相机后，用上面的参数和验证步骤确认；在验证通过前不要恢复训练采集。
