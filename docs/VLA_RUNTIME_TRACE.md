# VLA 运行轨迹导出契约（v1）

`tools/vla_contract/runtime_trace.py` 提供可选 `TraceSession`。默认不创建轨迹；只有调用方显式传入 `trace_hook` 时，`SequenceRunner` 才会把同一钩子交给每步 `TaskOrchestrator`。轨迹记录已经发生的输入、判断和结果，既不替代 `ActionSupervisor`，也不增加动作出口。导出异常时当前任务停止或转 `HUMAN_REVIEW`，不会继续下发后续目标。

调用方为每次运行创建新的目录：

```python
from runtime_trace import TraceSession, read_trace
from long_sequence_planner import SequenceRunner

trace = TraceSession(
    "/tmp/piper-runtime-traces",
    source_kind="synthetic",
    models={"planner": "Qwen3-VL-2B-Instruct@固定revision", "policy": "synthetic-policy"},
    links={"episode_id": "episode-example", "task_group_id": "group-example",
           "plan_id": "plan-example", "recovery_id": None},
)
runner = SequenceRunner(planner, task_factory, world_reader, trace_hook=trace)
result = runner.run(context, clock)
manifest, events = read_trace(trace.path)
```

`source_kind` 必须明确为 `synthetic` 或 `real`。真实来源还须给出 `episode_id` 和 `episode_manifest_sha256`，但这些引用仅提供可追溯性，**不能单独证明真实执行成功**。`models.planner`、`models.policy` 应绑定 Qwen 与 π0.5 的实际 checkpoint/revision；合成策略应如实写成合成模型。录制器产生的 `recovery_id` 可放在 `links`，此时所有事件保留该 ID；没有外部 ID 的内部重试或重新规划使用稳定派生 ID。`episode_id`、`task_group_id` 和 `plan_id` 存在 manifest 中，事件另带 `request_id`、`step_id`、`skill_id`、`recovery_id`、`observation_id`。

每个运行目录包含 `manifest.json` 和逐行 JSON 的 `events.jsonl`。manifest 固定 `schema_version=piper_runtime_trace/1.0`、运行 ID、来源、模型、关联 ID、事件数、事件文件 SHA-256、终态；`training_eligible=false` 与 `real_success_verified=false` 始终保持。JSONL 每行有递增 `seq`、事件类型、运行与来源引用、写入 Unix 时间、运行时钟、关联 ID 和数据。图像只记录不可变 URI、帧 ID 与 Header 时间，不复制像素；状态摘要记录关节/夹爪位置。

| 事件 | 内容 |
| --- | --- |
| `plan_request`、`plan_result` | Qwen 输入摘要、严格计划及可用的原始模型文本；格式错误的原文随拒绝事件保存。 |
| `plan_preflight`、`step_preflight` | 规划后及每步前的刷新观测、前置状态、接受/拒绝与原因；计划引用只在现有校验通过后更新。 |
| `policy_request`、`policy_result` | 策略输入观测/技能目标与输出 `action_chunk`；π0.5 的身份由 manifest 绑定。原始策略结果不等于获准动作。 |
| `preflight_accepted`、`preflight_rejected` | `ActionSupervisor.submit()` 的准入结果。 |
| `execution_request`、`execution_result` | 仅记录监督器 `tick()` 已批准的单个目标及执行器反馈；不读取或发送 CAN。 |
| `verify_result`、`world_verify` | critic 结果、可用的原始 critic 输出、独立世界状态和完成条件核对。`HUMAN_REVIEW` 不会转成自动成功。 |
| `recovery_requested`、`recovery_started`、`skill_failure`、`run_failure`、`run_terminal` | 重试与失败类别、原因、最终状态。 |

事件同步写入并 `fsync`，manifest 以临时文件替换并同步目录；目录权限为 `0700`，文件为 `0600`。新会话拒绝复用已有运行 ID。崩溃后留下的 manifest 为 `incomplete`，默认 `read_trace()` 拒绝；它也核对 JSONL 的行数、序号和 SHA-256。该摘要只检测意外损坏，不是签名或人工标注认证。即使合成运行返回 `DONE`，manifest 仍禁止把它作为真实成功或训练就绪样本。真实轨迹用于 Qwen LoRA 前仍需核对 Episode 原始证据、图像/状态身份、人工复核与标签清洗。

可审阅的[合成 manifest](../tools/vla_contract/evidence/runtime_trace_synthetic_example/manifest.json)与[对应 JSONL](../tools/vla_contract/evidence/runtime_trace_synthetic_example/events.jsonl)来自四步合成任务，共 53 个事件；其中 `DONE`、`fixture-planner`、`synthetic-policy` 和 `source_kind=synthetic` 均按原样标注，不代表公开模型或真实 π0.5 的推理结果。

运行测试：

```bash
python3 -m unittest discover -s tools/vla_contract -p 'test_runtime_trace.py' -v
python3 -m unittest discover -s tools/vla_contract -p 'test_*.py'
```

测试只用合成观测、策略、执行器和 critic；覆盖成功、人工复核、非法模型文本、预检拒绝、重试 ID、写入故障、重启与截断文件。尚未加载真实 π0.5 Piper checkpoint，也未连接机器人。
