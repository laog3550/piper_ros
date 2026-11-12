# episode_000008 检查（2026-11-12）

## 结论

**“成功帧不匹配”是桌面控制台的误报，已修复。** Episode 8 的完成帧数据本身有效：完成帧与自动候选一致，完成帧门禁通过，40 ms 审计得到 153 个对齐帧、1 个连续段、完成前 93 帧/78 个 16 步窗口。

但 episode 8 **还不能作为已验收 SFT 来源**，原因不是完成帧，而是顶部 D455 的 CameraInfo 与 Episode 声明的旧快照不一致；另外本轮末尾 stale 自动裁切没有触发。

## 1. 控制台误报

`desktop_console.skill_completion_issue()` 只接受 `causal_rgb_bundle_v1`，而 episode 8 使用 `causal_follower_next_state_v1`，因此错误提示“完成帧对齐策略不匹配”。

已改为接受两种策略：

- `causal_rgb_bundle_v1`
- `causal_follower_next_state_v1`

控制台已重启。该检查只影响 UI 提示，不影响录制器已保存的完成帧证明。

## 2. 完成帧与数据门禁

- 自动候选和 `skill_complete` 标注都指向顶部图像 Header `1794460143930125488`，偏移 9.384 秒。
- `completion_validation.frame_gate_passed=true`，连续片段 93 帧、78 个 16 步窗口。
- 40 ms 审计：153 个对齐帧、1 个连续段、完成前选中 93 帧/78 窗口；拒绝原因只剩相机来源验收。

也就是说，完成帧匹配、连续性、16 步窗口都没有问题。

## 3. 真实阻塞：顶部 CameraInfo 版本不一致

episode 8 的顶部 D455 CameraInfo：

```text
K = [388.6639404296875, ..., 388.099365234375, ...]
```

Episode 声明的 `camera_info_snapshot_20261110_v1`（以及 11-12 相机证据包）中的顶部 K：

```text
K = [388.1363525390625, ..., 387.57257080078125, ...]
```

- `fx/fy` 差约 **0.5276 px**；`D` 一致；左/右腕一致。
- 这是 2026-11-12 12:54 重启 30 fps 相机后的新 D455 启动批次；11-12 的相机验收包是在 15 fps 启动批次采集的。
- 因此 episode 8 的 `calibration_version=camera_info_snapshot_20261110_v1` 与实际顶部内参不匹配，不能用旧相机验收给它授予来源资格。
- 本次生成的 episode 8 源导出验收文件已判定无效并删除；相机绑定文件保留为诊断证据：`/home/mips/pi05_datas/validation/episode_59847de9_camera_binding_20261112.json`（顶部 `profile_matches_accepted_capture=false`）。

当前 30 fps 启动批次的新相机验收已经建立：

- 快照：`config/collection_camera_info_snapshot.20261112.json`（`camera_info_snapshot_20261112_v1`）
- 契约：`config/camera_training_contract.v3.20261112.json`
- 证据：`/home/mips/pi05_datas/validation/camera_evidence_20261112_v2/`
- 验收：`/home/mips/pi05_datas/validation/camera_training_acceptance_20261112_v2.json`（`training_camera_evidence_ready=true`）
- 状态：`config/camera_calibration_status.20261112_v2.json`

控制台默认 `calibration_version` 已切换到 `camera_info_snapshot_20261112_v1`。但 episode 8 是在新版本建立前录制，元数据仍声明旧版本，顶部内参与旧快照不符；它需要在新版本下重采，不能通过修改原始 Episode 追溯。

## 4. 末尾自动裁切未触发

episode 8 的 `stop_reason=duration_limit`，主目标在约 9.396 秒后进入持续 stale，但录制一直持续到 15.113 秒，留下约 **5.72 秒**零动作尾部。

自动候选仍按末尾 stale 前图像生成，完成帧没有受影响；但自动裁切没有按预期停止。最可能原因是实时 stale 订阅使用 best-effort、depth 5，在 30 fps 录制负载下丢消息，导致 stale 计数/时间条件不满足。

已改为可靠 QoS、depth 100 的 `/piper_vector/{arm}/master_target` 订阅，并重启录制器。下一條 Episode 需要确认 `stop_reason=stale_tail`、`trimmed_tail_ns` 和逻辑终点是否正常；本次没有追溯裁切 episode 8。

## 5. 当前状态与下一步

- 相机：30 fps，PID 677644。
- 录制器：PID 679839，40 ms 配置，状态 `idle`。
- 控制台：PID 679640。
- 相关测试：134 passed。

下一步：

1. 当前采集配置已改为 15 fps 纯彩色固定曝光/白平衡，顶部曝光 80 µs/gain 32；相机证据 v7/验收 v7 对应当前批次。用 `camera_info_snapshot_20261112_v1` 重采一条数据，确认自动裁切触发、完成帧在段内、40 ms 审计 `eligible=true`。
2. 在重采通过前，不把 episode 8 计入 SFT 数据；它的完成帧和动作连续性不是问题，问题是相机来源版本。
