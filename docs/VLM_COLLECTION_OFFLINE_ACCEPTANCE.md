# VLM 采集导出到上层流程的离线验收（2026-10-20）

## 范围与结论

以本地 `humble` 的 `0f40125` 为起点，只运行合成 ROS 流、已有离线数据和本地公开模型；没有连接机械臂、CAN 或动作话题。新采集接口可保存人工规划样本及单技能结果，合成 Web 测试导出了 **1 份规划、3 份转移**，分别覆盖 `success`、`failure`、`unknown`。这些 JSON 是带 rosbag 路径的索引与人工标注，**不能直接作为**现有 `LongSequencePlanner`、VLM critic 或 `TaskOrchestrator` 的输入，也不是训练就绪样本。真实数据目录中未发现这种新格式的规划或转移导出。

本轮发现并修复 `plan_id` 关联缺陷：录制前及导出时均核对任务描述、技能名、目标 ID 和完成判据是否对应同一规划步骤；历史 Episode 若关联错配，导出会拒绝。规划样本给出场景 ID 目录时，步骤目标也必须存在于对应目录。Web 的 `/api/vlm/export` 与 `/api/plans` 现在把录制服务的拒绝映射为 HTTP `409`，避免以 HTTP `200` 包装失败。

## 新样本与运行契约对照

| 入口 | 导出已经提供 | 仍缺少或不兼容 |
| --- | --- | --- |
| 长序列规划 | 长任务文本、场景 ID 列表、人工步骤顺序、技能与完成判据文本 | v1 `observation`、`request_id`、执行臂、`object_state`、经核实的 `goal_state`、`scene_version`、已完成步骤；步骤须限于 `pick/place` 且最多 5 步，具备 `step_id`、依赖和结构化完成条件。合成样本使用 `grasp`，不能无依据地改写为 `pick`。 |
| VLM critic | Episode ID、单技能文字、人工结果与下一决策、bag 路径及话题 | 与同次技能绑定的本地不可变执行前/后 RGB、帧 Header 时间、`arm_mode`、`calibration_id`、目标 ID、四项有来源的几何摘要。人工 `failure_reason` 与 critic 枚举不同，不能直接映射；人工 `next_decision` 也不能直接当作自动状态转移。 |
| `TaskOrchestrator` / `ActionSupervisor` | 技能片段关联与人工结果 | 当前 v1 观测、实测反馈、策略输出的 Piper 7 维绝对目标、安全参数、动作时间戳及独立 critic。导出仅给 rosbag 引用，不能触发执行。 |
| Piper 训练/离线推理 | 完整 Episode 的路径和人工结果标签 | `to_lerobot.py` 所需的执行臂、Unix 时间域、动作空间及来源、标定/归一化版本、实测状态和示教动作话题；逐帧 `pick/place` 标签、来源独立的训练/验证划分、Piper checkpoint 与绑定归一化文件。 |

合成导出顶层为 `schema_version`、`planning_samples`、`transition_samples`。规划样本使用 `scene.object_ids`、`scene.location_ids` 与 `{order, skill, target_object_id, target_location_id, completion_criteria}`；转移样本包含 `context`、`human_annotation`、`rosbag_path`、`topics`、时间与完整性摘要。直接调用现有 `parse_plan` 得到“缺少可核对的任务目标终态”；直接调用 critic 的 `validate_input` 得到“输入字段不符合契约”。人工计划没有稳定 `step_id`，重复技能步骤可能无法与单个 Episode 唯一关联；本轮修复只能保证至少匹配一个步骤，不能声称已解决这个歧义。

要形成模型样本，需从 bag 按原始 Header 时间与技能边界选择帧、导出带摘要的本地 RGB 文件并核对观测与标定版本；VLM critic 还需同一目标的前后帧配对及经核实的几何/夹持状态。现有 `snapshot.py` 可生成观测结构、`to_lerobot.py` 可处理满足动作契约的 Episode，但都没有把新 `transition_samples` 自动转成上述前后帧 critic 样本；导出中的 bag 路径不能直接当作图像文件。身份、场景版本、目标区域和世界状态须由独立来源核对，不能从人工文字或文件名补造。训练还需核对动作/反馈实体归属、夹爪映射及逐帧技能标签。

## 离线运行证据

| 路径 | 输入与结果 | 性能与限制 |
| --- | --- | --- |
| 合成采集与导出 | `test_web_console_integration.py` 通过：1 份规划、3 份结果标注和转移；错配 `plan_id` 在录制与导出时均被拒绝。`test_desktop_console_integration.py` 与 `test_preview_depth.py` 通过。 | 合成 ROS 流使用独立测试域；bag 的反馈是合成字符串，不是 Piper 实测状态或示教动作。 |
| 真实 Qwen3-VL-2B-Instruct 规划 | 固定 revision `89644892e4d85e24eaac8bacfd4f463576704203`。此前三个合成场景中严格计划与预期顺序均为 `3/3`；本轮在当前 HEAD 重新运行四步合成世界，计划顺序正确，刷新 5 次后 `DONE`，合成监督器放行 8 个目标。 | 本轮模型调用 `26.12 s`；此前同模型三例整卡显存峰值 `5974–6045 MiB`，本轮单次运行未重新采样显存。`DONE` 使用合成 critic `SUCCESS` 与合成世界，不代表真实技能成功。[本轮原始计划与结果](../tools/vla_contract/evidence/qwen3vl_collection_audit_runner_20261020.json)。 |
| 真实 Qwen3-VL-2B-Instruct critic | 两组合成 `pick` 前后帧，预期分别为 `SUCCESS` 与 `FAILURE`。首例输出带 Markdown 围栏，严格 JSON 解析拒绝并降级 `UNKNOWN`；次例错误地输出 `SUCCESS`。决策正确 `0/2`，两次建议均为 `HUMAN_REVIEW`。 | 模型调用 `14.23 s`、`11.81 s`；整卡显存基线约 `1311 MiB`，峰值 `5967 MiB`。这是两个简化图形，不是独立真实测试集；不能推断现场准确率。[逐例原文与指标](../tools/vla_contract/evidence/qwen3vl_critic_synthetic_20261020.json)。 |
| 真实采集数据只读审计 | `/home/mips/piper_tjp/data` 有 6 个原始 bag 来源，但符合训练契约的 Episode、帧、训练集和测试集均为 `0`。`real_skill_acceptance.py` 按预期返回 `FAIL`（退出码 `2`），缺 Piper checkpoint、独立验证 Episode 和评估清单。 | 两份候选对齐包各有 139 条，均为 `training_ready=false`；不能计为通过的训练或验证数据。 |

测试命令：

```bash
python3 -m unittest discover -s tools/episode_recorder -p 'test_vlm_annotations.py'
python3 -m unittest discover -s tools/vla_contract -p 'test_*.py'
python3 -m unittest discover -s tools/vlm_critic -p 'test_*.py'
source /opt/ros/humble/setup.bash
source /home/mips/piper_ros/install/setup.bash
python3 tools/episode_recorder/test_web_console_integration.py
python3 tools/episode_recorder/test_preview_depth.py
python3 tools/episode_recorder/test_desktop_console_integration.py
python3 tools/episode_recorder/audit_real_data.py /home/mips/piper_tjp/data --output /tmp/piper_vlm_audit_real_data.json
python3 tools/vla_contract/real_skill_acceptance.py /home/mips/piper_tjp/data
```

`real_skill_acceptance.py` 的退出码 `2` 是证据不足的预期拒绝。真实 VLM 规划证据的复跑命令见 `tools/vla_contract/verify_refreshed_vlm_runner.py`；critic 两例由本地隔离 worker 生成，原始模型文本已保存。整卡显存数字来自 `nvidia-smi` 轮询，不能视为单进程精确占用。
