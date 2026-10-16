# VLM critic 离线基线

此工具只在技能结束后评估单臂 `pick` / `place`，不连接 ROS、CAN 或机械臂驱动。它不生成动作。每次输出均要求 `HUMAN_REVIEW`；上层不得把结果直接转换为重试或运动命令。

## 输入与运行

运行 `python3 tools/vlm_critic/critic.py sample.json`。`sample.json` 必须包含 `sample_id`、`skill_id`（`pick` 或 `place`）、`arm_mode`（`LEFT` 或 `RIGHT`）、`target_id`、`calibration_id`、`pre_rgb`、`post_rgb` 和 `geometry`。两张图像须为本地 RGB 文件；相对路径从 JSON 所在目录解析。`geometry` 恰有 `pre_in_gripper`、`post_in_gripper`、`post_in_region`、`post_target_visible` 四项，每项为 `true`、`false` 或 `null`。几何值应由有标定和目标关联的上游离线评估产生；本工具不从图像自动计算几何值，也不验证该摘要的真实性。未知值填写 `null`。

示例（图像路径须替换为实际文件）：

```json
{
  "sample_id": "episode-001-pick",
  "skill_id": "pick",
  "arm_mode": "LEFT",
  "target_id": "object-1",
  "calibration_id": "calib-001",
  "pre_rgb": "pre.png",
  "post_rgb": "post.png",
  "geometry": {
    "pre_in_gripper": null,
    "post_in_gripper": null,
    "post_in_region": null,
    "post_target_visible": null
  }
}
```

有完整几何证据时先按规则判定：`pick` 比较夹持状态；`place` 检查目标进入指定区域且离开夹爪。否则输出 `UNKNOWN`。规则输出的 `confidence=1` 仅表示给定布尔摘要对规则是确定的，**不是**真实视觉识别准确率，也不表示上游几何可靠。

输出字段固定为 `schema_version=piper_vlm_critic/1.0`、`sample_id`、`skill`（如 `left_pick`）、`target_id`、`result`、`failure_type`、`confidence`、`evidence`、`backend`、`recommended_transition=HUMAN_REVIEW`、`frame_refs`、`calibration_id`。`result` 为 `SUCCESS` / `FAILURE` / `UNKNOWN`；失败类型采用部署指南的 `EMPTY_GRASP`、`SLIP`、`WRONG_OBJECT`、`COLLISION_RISK`、`OCCLUDED`、`DEPTH_INVALID`、`OUT_OF_WORKSPACE`、`TIMEOUT`、`UNKNOWN`，并为 `place` 增加 `PLACE_MISS`；成功时为 `NONE`。证据每项包含 `source` 与非空 `observation`。未知结果的置信度固定为 0。模型输出被严格检查；无效 JSON、非法枚举、缺证据、超时或模型不可用均回退为 `UNKNOWN`。

机器可读的输出约束见 `tools/vlm_critic/output.schema.json`；运行时还会核对样本引用、技能与失败类型的对应关系。

## 可选本地零样本模型

后端使用 [Transformers 多模态推理接口](https://huggingface.co/docs/transformers/chat_templating_multimodal)。可选公开权重为 [SmolVLM-256M-Instruct](https://huggingface.co/HuggingFaceTB/SmolVLM-256M-Instruct) revision `7e3e67edbbed1bf9888184d9df282b700a323964`（约 513 MB 权重）或 [Qwen2.5-VL-3B-Instruct](https://huggingface.co/Qwen/Qwen2.5-VL-3B-Instruct) revision `66285546d2b821cf421d4f5eb2576359d3770cd3`（数 GB 权重）。建议在独立 Python 环境安装适合本机 CUDA 的 `torch`，再安装 `tools/vlm_critic/requirements-vlm.txt`。本机验证使用 `torch==2.14.1+cu126`，GPU 为 NVIDIA RTX 5880 Ada Generation。

SmolVLM 下载示例：

```bash
python3 -c "from huggingface_hub import snapshot_download; print(snapshot_download('HuggingFaceTB/SmolVLM-256M-Instruct', revision='7e3e67edbbed1bf9888184d9df282b700a323964', allow_patterns=['*.json', '*.txt', '*.safetensors'], ignore_patterns=['onnx/*']))"
```

将打印出的完整本地目录传给 `python3 tools/vlm_critic/critic.py sample.json --model /本地模型目录 --timeout 120 --min-confidence 0.7`。脚本只加载本地权重，不自动下载。每次调用是隔离子进程，超时后可终止；首次载入模型也计入超时。低于 `--min-confidence` 的模型结论回退为 `UNKNOWN`。`confidence` 是模型自报分数，未经校准。

验证命令：`python3 -m unittest tools.vlm_critic.test_critic -v`。当前测试只使用合成图像与人工设置的几何摘要，用于检查输入、规则、输出契约和回退路径。真实识别率、失败类型准确率和阈值须等真实标注数据与独立测试集具备后再评估。

2026-10-16 的本机烟雾测试使用上述 SmolVLM 固定 revision 和合成红球前后图。权重 SHA256 已校验为 `74dea5904032e5ae99a2e0eef5179e6ac0f1dedc3ab0c7c2a5d4d387c843203e`。一次完整模型调用约 11–13 秒；轮询 `nvidia-smi` 看到整卡占用由约 1178 MiB 升至约 3370 MiB（增量约 2192 MiB，非专属进程精确测量）。模型生成了非 JSON 文本，正式 CLI 将该结果判为 `UNKNOWN` 并要求人工复核。此样例只证明图像推理链路可运行及降级生效，不证明技能识别能力。
