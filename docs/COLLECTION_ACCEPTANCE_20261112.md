# 数据采集修改验收（2026-11-12）

## 结论与范围

**部分通过，整体尚不合格。** 原始保存与 SFT 校验已基本分离，去重修改在真实数据中恢复了有效采样点，自动完成已改为候选。但存在三个应优先修复的问题：启动脚本无法在普通终端环境中运行、录制中的人工完成标记仍被来源校验撤销、帧门禁结果被写成训练资格通过。另有候选失败详情缺失和诊断窗口计数竞态。

本次验收基于 `humble` 分支的未提交工作区，基础提交为 `b6f752b1d089fe32abc1dd83f420d51ecf15d80d`。逐文件 SHA-256、运行进程和复现脚本保存在 `/tmp/piper_collection_acceptance_20261112/verification.json` 及同目录证据中。没有修改生产代码、历史或暂存 Episode，没有发送 CAN 控制指令，也没有重启现场服务。

## 应优先修复的问题

### P1：启动脚本在加载 ROS 环境前启用 nounset

位置：`tools/episode_recorder/start_collection.sh:4` 至 `:8`。

脚本的 `set -euo pipefail` 会令 ROS Humble 的环境脚本在读取未设置变量时退出。以不存在的模式调用脚本，预期应经过环境加载后输出用法并返回 2，实际在进入模式判断和自检之前返回 1：

```text
/opt/ros/humble/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable
```

复现命令不会启动任何服务：

```bash
env -u AMENT_TRACE_SETUP_FILES -u COLCON_TRACE \
    -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH \
    bash tools/episode_recorder/start_collection.sh \
    __acceptance_invalid_mode__ /tmp/piper_episode_config.json
```

修改方式：加载 ROS/工作区环境时暂时关闭 `nounset`，完成后再启用；控制台解释器应明确指向已验收的系统解释器。增加干净环境下的启动入口测试。单独运行两个 `preflight.py` 模式均通过，不代表完整启动脚本可用。

### P1：人工确认的完成时刻仍被 SFT 校验撤销

位置：`tools/episode_recorder/episode_recorder.py:871` 至 `:893`。

停止流程将“图像不在 bag 中”和“图像存在但连续示教不足”放在同一个异常分支。后一种情况也会把人工 `skill_complete` 从 `annotations` 移入 `unverified_completion_attempts`；随后自动流程因没有完成标注而生成尾部候选。

在真实暂存 bag 的隔离元数据副本上复现：录制中标记了 Header 位于开始后 **11.614160938 秒**的真实顶部图像；停止后主要标注列表为空，产生约 **12.615010060 秒 Header**的新候选（界面按接收时刻显示 **12.639 秒**）。原人工记录仍在失败尝试中，但已不再作为确定的任务完成事件使用。

修改方式：图像存在性和训练来源有效性分别记录。真实存在的人工完成帧保留其 Header 与事件语义，SFT 不足写入 `source_validation_error`；只有来源引用本身无效才要求重新确定图像。存在人工完成事件时，不用更晚的自动候选替代它。补充“录制中人工标记、停止时连续帧不足”的回归；现有名称含 manual 的时间对齐测试主要覆盖自动候选后的确认路径。

### P1：训练资格字段超出了实际检查范围

位置：`tools/episode_recorder/episode_recorder.py:100` 至 `:128`、`:1010`。

`assess_training_eligibility()` 仅检查完成帧的部分证明字段，通过便返回 `eligible=true`，保存到顶层 `training_eligibility`。它没有完成相机验收或独立来源验收。

真实数据复现中，一个确有 **22 帧**连续有效示教的完成点通过了帧门禁；同时转换器仍报告 `calibration 尚未验收`。新函数返回顶层 `eligible=true`，而该 Episode 的 `task_metadata.training_eligibility.eligible` 仍为 `false`，两者冲突。

这不会直接绕过现有离线转换器的独立检查，但下游按 `eligible` 筛选时会误解资格。建议单独保存 `completion_validation.frame_gate_passed`，完整训练资格在独立验收前保持 `eligible=false`。若保留局部资格字段，必须明确其作用范围并消除两个同名资格对象的冲突。

## 仍需完善的问题

### P2：完成候选失败时缺少实际连续帧数

位置：`tools/episode_recorder/episode_recorder.py:943` 至 `:945`。

`validate_completion()` 失败时只返回异常文本，`source_validation` 为 `None`，所以候选的 `contiguous_frames`、`window_count`、`segment_index` 都变成 `null`。本次真实候选所属片段有 1 个有效采样点，界面无法按设计显示实际帧数与所需 16 帧的差距。

建议校验器返回结构化结果，失败也保留候选来源、片段长度、完成前帧数、窗口数和原因；其中“整个片段长度”和“完成前帧数”应分别命名，不能混用。

### P2：诊断窗口快照与清零不是原子操作

位置：`src/piper/piper/piper_direct_can_node.py:194` 至 `:211`。

当前先持锁复制统计，释放锁读取队列状态，再重新持锁清零窗口峰值。发布线程在复制后、清零前更新的峰值既不在当前快照中，也被下一步清零，下一周期无法报告。

确定性调度复现：在该间隙加入一次 **25 ms** 发布耗时，当前和下一窗口均报告 **0 ms**，累计最大值仍为 **25 ms**。这影响“按录制窗口统计”的可信度。

建议在同一 `_stats_lock` 临界区完成统计复制与窗口清零，再读取队列状态。队列状态可为稍后采样，但不能丢失时间窗口内的耗时统计。

## 已通过的检查

| 检查 | 结果 | 边界 |
| --- | --- | --- |
| 转换、原子技能、时间对齐、标注和训练门禁 | 106 项通过 | 使用实际 LeRobot/rosbags 环境，未代表全部仓库测试 |
| CAN 管线、传输和实例锁所选测试 | 11 项通过 | 假 CAN 输入及隔离 ROS 域，不发送硬件指令 |
| 桌面集成脚本 | 通过 | 合成 ROS 图像、原始保存、标注、跨目录保存与重启列表 |
| 技能 Web 集成脚本 | 通过 | 合成 ROS 流和 HTTP/录制闭环 |
| 控制台依赖自检 | 通过 | NumPy 1.21.5、OpenCV 4.5.4；完整启动入口仍失败 |
| 录制器依赖自检 | 通过 | 虚拟环境 NumPy 2.2.6、rosbags 可导入 |
| 差异空白检查 | 通过 | `git diff --check` |

### 真实 Episode 只读复验

来源：`/home/mips/pi05_datas/banana_task_episodes/.staging/f51d802581234807bfc037dd849bfd8f`。使用原始 bag，只在隔离目录中模拟停止/标注流程；没有把暂存状态或标注回写到原目录。

| 项目 | 修改前 | 当前代码 |
| --- | ---: | ---: |
| 10 Hz 采样网格点 | 142 | 142 |
| 有效采样点 | 87 | 92 |
| 连续片段数 | 6 | 7 |
| 完成候选所在末段有效采样点 | 1 | 1 |
| 该完成候选通过连续 16 步门禁 | 否 | 否 |

多出的片段是恢复了此前被后续 stale 副本否定的来源后出现的短片段，并不表示新增真实断流。有效片段长度为 `22、1、19、17、25、7、1`。本条数据仍不满足成功 SFT 的完成段要求；候选确认和原始成功保存不能改变这一事实。

源组合 SHA-256 在复验前后保持：

```text
843458920e445d573a09726cae290ace12a900345ac3c2a24bff767b3be5ef82
```

## 七项修改的验收状态

| 原问题 | 当前判断 |
| --- | --- |
| 主目标持续输入 | 主端部分未完成；采集端新增诊断不能替代主端序号/持续上报 |
| stale 副本追溯否定有效帧 | 本次真实案例通过，来源时间和真实缺口保持 |
| 完成候选与人工确认 | 自动候选路径通过；录制中的人工标记保留未通过 |
| 原始保存与 SFT 资格分离 | 保存路径通过；资格字段语义未通过 |
| 界面失败解释 | 部分通过；失败候选的实际帧数仍缺失 |
| 按窗口延迟诊断 | 部分通过；窗口峰值可能因竞态丢失 |
| 固定启动环境 | 单独依赖自检通过；完整启动入口未通过 |

## 现场运行状态与后续验收

录制服务 PID `634641` 和桌面界面 PID `634899` 的启动时间均早于本次文件修改，仍为旧进程。只读 CAN 节点 PID `641593`/`641597` 的启动时间晚于驱动修改。现场录制服务处于 `stopped`，上述暂存 Episode 仍未处理；本次没有为验收而重启或丢弃它。

修复上述问题后，需要先妥善保存或处理暂存数据，再启动新版服务和界面，最后采集一条新的真实 Episode。新的主动示教与全负载测试尚未进行；合成集成结果不能作为该项通过证据。验收应分别统计有持续真实输入的区间和停发区间，不能要求静止停发时的所有主目标 stale 都消失。

临时证据目录：`/tmp/piper_collection_acceptance_20261112/`。主要文件为 `verification.json`、`acceptance_gaps.json`、`diagnostic_window_race.json`、`real_episode/audit.json`、对应复现脚本及测试日志；临时目录不作为长期数据归档。
