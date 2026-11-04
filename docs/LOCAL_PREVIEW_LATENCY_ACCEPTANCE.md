# 本地采集控制台预览延迟验收

基线：`humble` 的 `12ebf63`；检查日期：2026-11-04（Asia/Shanghai）。本页为只读性能基线和后续 PyQt5 控制台验收口径，不包含运动、CAN 或相机参数操作。

## 当前可测状态与网页结构基线

2026-11-04 20:06 CST，本机没有三路相机节点、录制服务或网页服务；`ros2 node list` 和相关 `ros2 topic list -t` 为空，常用网页端口无监听。因此**本次没有在线 ROS 帧率、Header 年龄或 HTTP 更新率实测值**。用下文探针试跑 `2 s` 得到三路 RGB 各 `0` 帧、网页 JPEG 请求均连接失败；这只验证离线报告行为，不能充当延迟基线。

当前代码给出的节拍上限是：

| 路径 | 代码行为 | 能推出什么 |
| --- | --- | --- |
| 网页服务端 | [`skill_web_console.py`](../tools/episode_recorder/skill_web_console.py) 对每路 RGB 的 JPEG 编码至少间隔 `0.4 s` | 理想情况下每路缓存最多约 `2.5` 次新图像/秒；编码慢时更少。 |
| 网页客户端 | [`skill_web_console.html`](../tools/episode_recorder/skill_web_console.html) 每 `1500 ms` 调一次 `refresh()`，等 `/api/status` 成功后才重设三张图的 URL | 理想情况下每路浏览器至多约 `0.67` 次图像请求/秒；状态接口慢或失败会再降低频率。这是代码上限，**不是现场绘制帧率实测**。 |
| 现有本地 PyQt5 | [`desktop_console.py`](../tools/episode_recorder/desktop_console.py) 的图像定时器为 `33 ms`，深度提交另有 `0.2 s` 节流 | 设计上 RGB 可接近 `30 fps`、Depth 可接近 `5 fps`；实际刷新和绘制延迟仍须实测。现有 `display_fps` 计数在 `setPixmap` 后增加，不等于屏幕实际绘制率。 |

网页的 `/camera/{i}.jpg` 只返回最新 JPEG 字节，没有原图 Header 时间或序号。仅测 HTTP RTT 或 JPEG 哈希变化，无法推算图像从曝光到浏览器显示的时延。

## 只读探针与测量口径

工具：[`preview_latency_probe.py`](../tools/episode_recorder/preview_latency_probe.py)。它按录制配置订阅三路 RGB，使用与网页相同的 `qos_profile_sensor_data`；可选地只对本机网页 JPEG 地址执行 GET，不调用录制控制接口、不写相机/CAN 参数。推荐在三机和网页服务均已由现场操作者正常启动、画面稳定后运行 `60 s`：

```bash
cd /home/mips/piper_ros
source /opt/ros/humble/setup.bash
source /home/mips/piper_ros/install/setup.bash
python3 tools/episode_recorder/preview_latency_probe.py \
  --config /tmp/piper_episode_config.json --seconds 60 \
  --web-url http://127.0.0.1:8765 \
  --output /tmp/piper-preview-baseline.json
```

若网页未运行，省略 `--web-url`；若使用旧版深度预览网页，再加 `--web-console legacy`。实际端口或配置文件不同则使用运行中的值。探针不会自行启动相机、网页或录制器。输出中的每路 `header_fps` 是首末 Header 跨度上的帧率，`receive_fps` 是本探针回调到达率；`header_gap_gt_50ms`、`header_gap_gt_100ms`、`header_nonincreasing` 是源 Header 异常计数。`header_to_callback_age_ms = 本机回调入口 time.time_ns − Image.header.stamp`，按最近秩报告 `p50/p95/p99/max`；其中包含相机采集、USB、驱动、DDS、进程调度和本探针订阅开销。出现零 Header、负年龄或远超预期的年龄时先查时钟域，不能截断为零后计算。**该值不是 GUI 延迟。**

可选网页字段 `jpeg_hash_changes` 与 `observed_change_rate_hz_lower_bound` 表示高于浏览器频率的 GET 抽样中实际看到不同 JPEG 字节的次数及其速率；静止画面可能编码出相同字节，所以只是服务端缓存变化的**可观测下界**。`http_rtt_ms` 只表示本机 GET 往返，不是浏览器绘制耗时。网页实际 `0.67 Hz` 上限需按代码核对，若要实测浏览器绘制还须在浏览器侧记录每张图的 `load`/绘制时间与原帧标识。

## 本地控制台的建议验收目标

在三路 RGB-D 同时运行、录制器保持正常连接、连续 `60 s` 的条件下，分别报告如下指标。目标是本地控制台的性能门槛；本次离线检查没有判定通过。

| 指标 | 建议目标 | 统计位置与判定方式 |
| --- | --- | --- |
| ROS 输入健康 | 每路 RGB Header 约 `30 fps`；零/负 Header 与帧间隙逐一列出 | 使用只读探针区分源流问题和 UI 问题。源流不稳定时不把 UI 卡顿归因于本地控制台。历史 3 小时相机测试曾约 `30 fps`，顶部有孤立异常。 |
| RGB 可见预览 | 三路每路至少 `15` 个**新源帧**/秒，连续 `60 s`；不得出现超过 `500 ms` 的无新画面停顿 | 按源 Header 或帧序号去重，在实际 `paintEvent` 或等效可见绘制点计数；`setPixmap` 次数和 QTimer 次数不能替代。 |
| Depth 可见预览 | 三路每路至少 `4` 个新源帧/秒 | 现有深度处理按 `0.2 s` 节流；同样按实际绘制且去重计数。 |
| GUI 自身延迟 | 回调入口到实际绘制的 `p95 ≤ 100 ms`、`p99 ≤ 200 ms` | 两点均使用同一进程的 `time.monotonic_ns`。包含队列、转换、缩放和 Qt 绘制调度；不包含相机/ROS 源年龄。 |
| 整体 Header 到绘制 | `p95 ≤ 200 ms`，并报告 `p99/max` | 对同一帧用 `Header→回调年龄 + 回调→绘制单调时钟差`；需先确认 Header 与本机 Unix 时钟同域。分别保留两段分布，不能把源年龄当成 GUI 延迟。 |

要真实验收后两项，PyQt 端需为每次显示的源帧关联 `{role, header_ns, callback_wall_ns, callback_mono_ns, paint_mono_ns}`；在 `paintEvent` 或更接近实际绘制的点写只读日志。若只记录 `setPixmap`，结果应命名为“提交到控件延迟”。操作系统合成显示与显示器光子时延仍不在软件测量范围内。比较网页和 PyQt 时保持同一相机配置、图像尺寸、现场光照、并发录制负载和测量时长。

## 阻断与结论

当前阻断为所有相机/网页服务离线；没有在线延迟数值，不能宣称本地控制台已达到上表目标。网页的 `1500 ms` 请求周期足以解释低刷新率的结构性上限，但现场总延迟还取决于相机源年龄、服务端状态请求、网络/浏览器和显示，不可由该常数直接得出。下一步在服务恢复后先运行只读探针，保存源流及 JPEG 缓存基线；再对 PyQt 绘制点采样，逐段比较并验收。
