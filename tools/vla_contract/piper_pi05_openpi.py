#!/usr/bin/env python3
"""注册 Piper π0.5 本地训练/推理配置并核验真实数据；运行：python3 tools/vla_contract/piper_pi05_openpi.py --help。"""

import argparse
import dataclasses
import hashlib
from importlib.metadata import PackageNotFoundError, version
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import types
from contextlib import redirect_stderr, redirect_stdout

import numpy as np

from piper_pi05_adapter import ACTION_HORIZON, PiperInputs, PiperOutputs


OPENPI_COMMIT = "215abfb217dbac7d5f1273282331b9b1866c0479"
CONFIG_NAME = "pi05_piper_absolute_7"
ASSET_ID = "local/piper_train"
NORMALIZATION_SCHEMA = "piper_train_normalization/1"
CHECKPOINT_NORMALIZATION = f"assets/{ASSET_ID}/norm_stats.json"
SINGLE_ARM_SKILLS = {"pick_and_place_banana_in_box": "banana_in_box"}


def digest(path):
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def checkpoint_hashes(directory):
    files = sorted(path for path in directory.rglob("*") if path.is_file())
    if not files:
        raise ValueError("训练结束后没有 checkpoint 文件")
    return {str(path.relative_to(directory)): digest(path) for path in files}


def episode_source_digest(directory):
    files = [directory / "episode.json", directory / "bag" / "metadata.yaml",
             *sorted((directory / "bag").glob("*.db3"))]
    if len(files) < 3 or any(not path.is_file() for path in files):
        raise ValueError(f"{directory}: Episode 来源文件不完整")
    result = hashlib.sha256()
    for path in files:
        result.update(path.name.encode())
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                result.update(block)
    return result.hexdigest()


def dataset_files_digest(directory):
    """Fingerprint the exact local split read by OpenPI, including data and metadata."""
    files = sorted(path for path in directory.rglob("*") if path.is_file())
    if not directory.is_dir() or not files:
        raise ValueError("训练划分目录为空")
    rows = [(str(path.relative_to(directory)), digest(path)) for path in files]
    return hashlib.sha256(json.dumps(rows, ensure_ascii=False).encode()).hexdigest()


def check_work_dir(work_dir, dataset):
    manifest = json.loads((dataset / "conversion_manifest.json").read_text(encoding="utf-8"))
    sources = [dataset.resolve(), *(Path(row["source"]).resolve() for row in manifest["episodes"])]
    if any(work_dir.resolve().is_relative_to(source) for source in sources):
        raise ValueError("工作目录须位于转换目录和原始 Episode 之外，保持来源只读")
    if work_dir.exists() and any(work_dir.iterdir()):
        raise ValueError("工作目录必须为空，防止混入旧统计或 checkpoint")


def evidence_file(reference, label):
    if not isinstance(reference, dict) or not isinstance(reference.get("path"), str):
        raise ValueError(f"缺少 {label} 的文件与 SHA-256 证据")
    path = Path(reference["path"])
    if not path.is_absolute() or not path.is_file() or reference.get("sha256") != digest(path):
        raise ValueError(f"{label} 文件不存在或 SHA-256 不匹配")
    return path


def normalization_stats(path):
    """Require actual finite seven-dimensional OpenPI statistics, not a raw schema file."""
    document = json.loads(path.read_text(encoding="utf-8"))
    stats = document.get("norm_stats") if isinstance(document, dict) else None
    if not isinstance(stats, dict) or set(stats) != {"state", "actions"}:
        raise ValueError("归一化资产须包含 state/actions 统计，不能用原始数据约定替代")
    for key in ("state", "actions"):
        row = stats[key]
        if not isinstance(row, dict) or set(row) != {"mean", "std", "q01", "q99"}:
            raise ValueError(f"{key} 归一化统计缺少 mean/std/q01/q99")
        for field, values in row.items():
            array = np.asarray(values)
            if array.shape != (7,) or array.dtype.kind not in "fi" or not np.isfinite(array).all():
                raise ValueError(f"{key}.{field} 必须为 7 维有限数统计")
        if np.any(np.asarray(row["std"]) < 0) or np.any(np.asarray(row["q01"]) > row["q99"]):
            raise ValueError(f"{key} 归一化统计范围无效")
    return stats


def normalization_version_valid(value, data_convention_version):
    return (isinstance(value, str) and bool(value.strip()) and value == value.strip()
            and not value.upper().startswith("UNVERIFIED") and value != data_convention_version)


def check_normalization(proof, source_evidence):
    version = proof.get("normalization_version")
    if not normalization_version_valid(version, source_evidence["data_convention_version"]):
        raise ValueError("训练 normalization_version 须独立于原始 data_convention_version")
    if proof.get("normalization_verified") is not True:
        raise ValueError("训练归一化统计尚未独立核验")
    asset = evidence_file(proof.get("normalization"), "训练归一化资产")
    normalization_stats(asset)
    record_path = evidence_file(proof.get("normalization_provenance"), "归一化来源记录")
    record = json.loads(record_path.read_text(encoding="utf-8"))
    expected = {
        "schema_version": NORMALIZATION_SCHEMA, "status": "computed_from_audited_train_split",
        "normalization_version": version, "normalization_sha256": digest(asset),
        "source_split": "train", "openpi_commit": OPENPI_COMMIT,
        **{key: source_evidence[key] for key in (
            "data_convention_version", "conversion_manifest_sha256", "source_audit_sha256",
            "train_dataset_sha256", "train_episodes")},
    }
    if not isinstance(record, dict) or any(record.get(key) != value for key, value in expected.items()):
        raise ValueError("归一化统计未绑定当前已审计的 train 划分、来源或资产 SHA-256")
    return {"normalization_version": version,
            "normalization": {"path": str(asset), "sha256": digest(asset)},
            "normalization_provenance": {"path": str(record_path), "sha256": digest(record_path)}}


def write_normalization_provenance(asset, evidence, version, output):
    if not normalization_version_valid(version, evidence["data_convention_version"]):
        raise ValueError("统计版本不能为空、UNVERIFIED 或原始数据约定版本")
    normalization_stats(asset)
    record = {"schema_version": NORMALIZATION_SCHEMA, "status": "computed_from_audited_train_split",
              "normalization_version": version, "normalization_sha256": digest(asset),
              "source_split": "train", "openpi_commit": OPENPI_COMMIT,
              **{key: evidence[key] for key in (
                  "data_convention_version", "conversion_manifest_sha256", "source_audit_sha256",
                  "train_dataset_sha256", "train_episodes")}}
    output.write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8")
    return {"status": "normalization_computed_pending_independent_review",
            "normalization_version": version,
            "normalization": {"path": str(asset.resolve()), "sha256": digest(asset)},
            "normalization_provenance": {"path": str(output.resolve()), "sha256": digest(output)}}


def run_openpi_script(root, script, config):
    from importlib.util import module_from_spec, spec_from_file_location
    spec = spec_from_file_location(f"piper_{Path(script).stem}", root / "scripts" / script)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    module.main(CONFIG_NAME if script == "compute_norm_stats.py" else config)


def bind_checkpoint_normalization(checkpoint, evidence):
    asset = checkpoint / CHECKPOINT_NORMALIZATION
    approved = evidence_file(evidence.get("normalization"), "已核准归一化资产")
    if normalization_stats(asset) != normalization_stats(approved):
        raise ValueError("checkpoint 内的归一化统计与独立核准资产不一致")
    return {"checkpoint_normalization_path": CHECKPOINT_NORMALIZATION,
            "checkpoint_normalization_sha256": digest(asset)}


def verify_checkpoint_normalization(checkpoint, record):
    if (not normalization_version_valid(record.get("normalization_version"), record.get("data_convention_version"))
            or record.get("checkpoint_normalization_path") != CHECKPOINT_NORMALIZATION
            or not isinstance(record.get("normalization"), dict)
            or not record["normalization"].get("sha256")
            or not isinstance(record.get("normalization_provenance"), dict)
            or not record["normalization_provenance"].get("sha256")
            or not record.get("train_dataset_sha256") or not record.get("train_episodes")):
        raise ValueError("训练记录缺少独立统计版本、train 来源或 checkpoint 绑定")
    asset = checkpoint / CHECKPOINT_NORMALIZATION
    if not asset.is_file() or record.get("checkpoint_normalization_sha256") != digest(asset):
        raise ValueError("checkpoint 归一化资产 SHA-256 不匹配")
    normalization_stats(asset)


def check_openpi(root, base_checkpoint):
    revision = subprocess.check_output(["git", "-C", str(root), "rev-parse", "HEAD"], text=True).strip()
    if revision != OPENPI_COMMIT:
        raise ValueError(f"openpi commit 不符：{revision}")
    if not (base_checkpoint / "params").is_dir() or not (base_checkpoint / "assets").is_dir():
        raise ValueError("缺少完整的官方 pi05_droid checkpoint")
    sys.path.insert(0, str(root / "src"))
    return revision


def install_lerobot_v3_compat(site_packages):
    """Bind the fixed OpenPI reader's old import path to an isolated LeRobot v3 install."""
    site_packages = site_packages.resolve()
    if not site_packages.is_dir():
        raise ValueError("LeRobot v3 site-packages 目录不存在")
    if str(site_packages) not in sys.path:
        raise ValueError("须在启动 Python 前把 LeRobot v3 site-packages 加入 PYTHONPATH；运行中切换会混用依赖")
    try:
        installed_version = version("lerobot")
    except PackageNotFoundError as error:
        raise ValueError("训练读取环境未安装 lerobot==0.4.4") from error
    if installed_version != "0.4.4":
        raise ValueError("训练读取环境必须提供 lerobot==0.4.4")
    import lerobot
    import lerobot.datasets.lerobot_dataset as dataset_v3
    if not Path(dataset_v3.__file__).resolve().is_relative_to(site_packages):
        raise ValueError("LeRobot v3 模块未从指定环境加载")
    common = types.ModuleType("lerobot.common")
    common.__path__ = []
    datasets = types.ModuleType("lerobot.common.datasets")
    datasets.__path__ = []
    common.datasets = datasets
    datasets.lerobot_dataset = dataset_v3
    lerobot.common = common
    sys.modules["lerobot.common"] = common
    sys.modules["lerobot.common.datasets"] = datasets
    sys.modules["lerobot.common.datasets.lerobot_dataset"] = dataset_v3


def verify_openpi_v3_reader(dataset, base_checkpoint, arm_mode):
    """Read one real v3 batch through the fixed OpenPI training transforms."""
    info_path = dataset / "train" / "meta" / "info.json"
    if json.loads(info_path.read_text(encoding="utf-8")).get("codebase_version") != "v3.0":
        raise ValueError("训练集不是 LeRobot v3.0")
    from openpi.training import data_loader
    import lerobot.datasets.lerobot_dataset as dataset_v3
    with tempfile.TemporaryDirectory(prefix="piper-openpi-v3-reader-") as temporary:
        home = Path(temporary) / "lerobot"
        link = home / "local" / "piper_train"
        link.parent.mkdir(parents=True)
        link.symlink_to((dataset / "train").resolve(), target_is_directory=True)
        previous = os.environ.get("HF_LEROBOT_HOME")
        previous_home = dataset_v3.HF_LEROBOT_HOME
        os.environ["HF_LEROBOT_HOME"] = str(home)
        dataset_v3.HF_LEROBOT_HOME = home
        try:
            config = build_config("local/piper_train", base_checkpoint, Path(temporary), 1, 1, arm_mode)
            config = dataclasses.replace(config, num_workers=0)
            loader = data_loader.create_data_loader(
                config, num_batches=1, skip_norm_stats=True, framework="pytorch")
            observation, actions = next(iter(loader))
        finally:
            if previous is None:
                os.environ.pop("HF_LEROBOT_HOME", None)
            else:
                os.environ["HF_LEROBOT_HOME"] = previous
            dataset_v3.HF_LEROBOT_HOME = previous_home
    if tuple(observation.state.shape) != (1, 32) or tuple(actions.shape) != (1, ACTION_HORIZON, 32):
        raise ValueError("固定 OpenPI 训练读取器输出的 Piper 状态或动作形状错误")
    return {"state_shape": list(observation.state.shape), "action_shape": list(actions.shape)}


def verify_base_checkpoint(base_checkpoint):
    manifest = Path(__file__).with_name("pi05_droid.sha256")
    rows = manifest.read_text(encoding="utf-8").splitlines()
    if len(rows) != 20:
        raise ValueError("官方 checkpoint SHA-256 清单不完整")
    for row in rows:
        expected, relative = row.split("  ", 1)
        file = (base_checkpoint / relative).resolve()
        if not file.is_relative_to(base_checkpoint.resolve()) or digest(file) != expected:
            raise ValueError(f"官方 checkpoint SHA-256 不匹配：{relative}")


def check_dataset(dataset, evidence, audit_report, *, require_normalization=True):
    """只接受由 Episode 转换器产出的本地 LeRobot v3 与独立核验凭据。"""
    manifest_path = dataset / "conversion_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    proof = json.loads(evidence.read_text(encoding="utf-8"))
    audit = json.loads(audit_report.read_text(encoding="utf-8"))
    if (manifest.get("format") != "LeRobotDataset v3.0"
            or manifest.get("purpose") != "success_sft"
            or manifest.get("arm_mode") not in ("LEFT", "RIGHT")
            or manifest.get("action_space") != "absolute_joint_position_7"
            or manifest.get("joint_names") != [f"{manifest.get('arm_mode', '').lower()}.joint{i}" for i in range(1, 7)]
            + [f"{manifest.get('arm_mode', '').lower()}.gripper"]):
        raise ValueError("LeRobot 转换清单不是单臂 Piper 绝对 7 维数据")
    episodes = manifest.get("episodes")
    if not isinstance(episodes, list) or not episodes or not all(isinstance(row, dict) for row in episodes):
        raise ValueError("转换清单没有 Episode")
    allowed_status = ({"verified_for_piper_pi05_training"} if require_normalization else
                      {"verified_for_piper_pi05_source_data", "verified_for_piper_pi05_training"})
    if (proof.get("status") not in allowed_status
            or proof.get("conversion_manifest_sha256") != digest(manifest_path)
            or proof.get("source_audit_sha256") != digest(audit_report)
            or proof.get("action_space") != "absolute_joint_position_7"
            or proof.get("arm_mode") != manifest["arm_mode"]
            or proof.get("action_provenance") != "human_demonstration"
            or proof.get("gripper_mapping_verified") is not True
            or proof.get("calibration_verified") is not True):
        raise ValueError("缺少与数据清单匹配的独立 Piper 训练核验证据")
    ids = {row.get("episode_id") for row in episodes}
    if any(not isinstance(value, str) or not value for value in ids) or len(ids) != len(episodes):
        raise ValueError("Episode ID 缺失或重复")
    source_ids = {row.get("source_episode_id", row["episode_id"]) for row in episodes}
    if any(not isinstance(value, str) or not value for value in source_ids):
        raise ValueError("源 Episode ID 缺失")
    audited = {row.get("episode_id"): row for row in audit.get("sources", [])
               if row.get("eligible") is True}
    if (audit.get("schema_version") != "piper_real_data_audit_v2"
            or audit.get("split_ready") is not True
            or audit.get("eligible_episode_count", 0) < len(source_ids)
            or source_ids - set(audited)):
        raise ValueError("独立来源审计未核准所有训练 Episode")
    labels = proof.get("episode_skills")
    if not isinstance(labels, dict) or set(labels) != ids or set(labels.values()) - set(SINGLE_ARM_SKILLS.values()):
        raise ValueError("核验证据必须逐 Episode 标注 banana_in_box")
    counts = {(split, skill): 0 for split in ("train", "test") for skill in SINGLE_ARM_SKILLS.values()}
    groups = {"train": set(), "test": set()}
    counted_sources = set()
    for row in episodes:
        split = row.get("split")
        if split not in ("train", "test") or not isinstance(row.get("frames"), int) or row["frames"] < ACTION_HORIZON:
            raise ValueError("Episode 划分或连续帧数不满足 16 步训练")
        skill, group_id, task_text = row.get("skill_id"), row.get("group_id"), row.get("task_text")
        source_id = row.get("source_episode_id", row["episode_id"])
        audited_row = audited[source_id]
        if (skill not in SINGLE_ARM_SKILLS or labels[row["episode_id"]] != SINGLE_ARM_SKILLS[skill]
                or not isinstance(group_id, str) or not group_id.strip()
                or not isinstance(task_text, str) or not task_text.strip()
                or audited_row.get("kind") != "native_episode"
                or audited_row.get("source") != row.get("source")
                or audited_row.get("skill_id") != skill
                or audited_row.get("group_id") != group_id
                or audited_row.get("task_text") != task_text
                or audited_row.get("suggested_split") != split):
            raise ValueError("Episode 技能、任务文本、场景组或审计切分不一致")
        source = Path(row.get("source", ""))
        if not source.is_absolute() or row.get("source_sha256") != episode_source_digest(source):
            raise ValueError("Episode 来源文件摘要不匹配")
        meta = json.loads((source / "episode.json").read_text(encoding="utf-8"))
        fields = meta.get("task_metadata")
        initial = fields.get("initial_conditions") if isinstance(fields, dict) else None
        if (meta.get("schema_version") != 2 or meta.get("mode") != "skill"
                or meta.get("state") != "saved" or meta.get("episode_id") != source_id
                or meta.get("outcome") != "success" or meta.get("skill") != skill
                or not isinstance(meta.get("task"), str)
                or meta["task"].strip() != task_text
                or not isinstance(fields, dict)
                or fields.get("arm_mode") != manifest["arm_mode"]
                or fields.get("time_domain") != "unix_ns"
                or fields.get("action_space") != "absolute_joint_position_7"
                or fields.get("action_provenance") != "human_demonstration"
                or fields.get("calibration_version") != row.get("calibration_version")
                or fields.get("data_convention_version") != row.get("data_convention_version")
                or not isinstance(initial, dict)
                or not isinstance(initial.get("perturbation_id"), str)
                or initial["perturbation_id"].strip() != group_id):
            raise ValueError("Episode v2 原子技能来源与转换清单不一致")
        if any(not isinstance(row.get(field), str) or not row[field].strip()
               for field in ("calibration_version", "data_convention_version")):
            raise ValueError("Episode 缺标定或原始数据约定版本")
        groups[split].add(group_id)
        # Multiple exported segments from one demonstration do not inflate the minimum demo count.
        if source_id not in counted_sources:
            counts[split, labels[row["episode_id"]]] += 1
            counted_sources.add(source_id)
    if groups["train"] & groups["test"]:
        raise ValueError("同一 perturbation_id 场景组跨越训练/测试划分")
    for field in ("calibration_version", "data_convention_version"):
        versions = {row[field] for row in episodes}
        if len(versions) != 1 or proof.get(field) != next(iter(versions)):
            raise ValueError(f"{field} 在 Episode 和核验证据中不一致")
    if any(counts["train", skill] < 10 or counts["test", skill] < 3 for skill in SINGLE_ARM_SKILLS.values()):
        raise ValueError(f"单臂香蕉入盒至少需要 10 个训练 Episode 和 3 个测试 Episode：{counts}")
    for split in ("train", "test"):
        if not (dataset / split / "meta" / "info.json").is_file():
            raise ValueError(f"{split} LeRobot v3 数据集不存在")
    source_evidence = {"arm_mode": manifest["arm_mode"],
            "calibration_version": proof["calibration_version"],
            "data_convention_version": proof["data_convention_version"],
            "counts": {f"{a}_{b}": v for (a, b), v in counts.items()},
            "conversion_manifest_sha256": digest(manifest_path), "evidence_sha256": digest(evidence),
            "source_audit_sha256": digest(audit_report),
            "train_dataset_sha256": dataset_files_digest(dataset / "train"),
            "train_episodes": sorted([
                {"episode_id": row["episode_id"],
                 "source_episode_id": row.get("source_episode_id", row["episode_id"]),
                 "source_sha256": row["source_sha256"], "frames": row["frames"], "group_id": row["group_id"]}
                for row in episodes if row["split"] == "train"], key=lambda row: row["episode_id"])}
    if require_normalization:
        source_evidence.update(check_normalization(proof, source_evidence))
    return source_evidence


def build_config(repo_id, base_checkpoint, work_dir, steps, batch_size, arm_mode):
    """openpi 官方 API：独立图像槽、7 维绝对位置、Piper 专属归一化。"""
    from openpi import transforms
    from openpi.models.pi0_config import Pi0Config
    from openpi.training import config as cfg
    from openpi.training import weight_loaders

    if arm_mode not in ("LEFT", "RIGHT"):
        raise ValueError("仅支持单臂 Piper")
    repack = transforms.Group(inputs=[transforms.RepackTransform({
        "images": {"top_rgb": "observation.images.top_rgb",
                   "wrist_rgb": f"observation.images.{arm_mode.lower()}_wrist_rgb"},
        "state": "observation.state", "actions": "action", "prompt": "task",
    })])

    @dataclasses.dataclass(frozen=True)
    class PiperTransformFactory:
        def __call__(self, model_config):
            if not getattr(model_config, "pi05", False) or model_config.action_dim != 32:
                raise ValueError("Piper 适配仅支持 π0.5 的 32 维内部动作")
            return transforms.Group(inputs=[PiperInputs(arm_mode.lower())], outputs=[PiperOutputs()])

    data = cfg.SimpleDataConfig(
        repo_id=repo_id,
        assets=cfg.AssetsConfig(asset_id=ASSET_ID),
        base_config=cfg.DataConfig(action_sequence_keys=("action",), prompt_from_task=False,
                                   repack_transforms=repack),
        data_transforms=PiperTransformFactory(),
    )
    result = cfg.TrainConfig(
        name=CONFIG_NAME, exp_name="verified_piper", model=Pi0Config(pi05=True, action_dim=32,
                                                                       action_horizon=ACTION_HORIZON),
        data=data, weight_loader=weight_loaders.CheckpointWeightLoader(str(base_checkpoint / "params")),
        assets_base_dir=str(work_dir / "assets"), checkpoint_base_dir=str(work_dir / "checkpoints"),
        num_train_steps=steps, batch_size=batch_size, save_interval=min(1000, steps),
        log_interval=min(50, steps), wandb_enabled=False, num_workers=0,
        policy_metadata={"robot": "Piper", "action_space": "absolute_joint_position_7",
                         "state": "joint1..joint6 rad; gripper m", "action": "absolute joint1..joint6 rad; gripper m"},
    )
    cfg._CONFIGS_DICT[CONFIG_NAME] = result
    return result


def load_validated_policy(openpi_root, checkpoint, record_path, evaluation_path):
    """推理入口：只加载有训练来源、逐文件摘要及独立留出集评估的 Piper 权重。"""
    record = json.loads(record_path.read_text(encoding="utf-8"))
    evaluation = json.loads(evaluation_path.read_text(encoding="utf-8"))
    if record.get("status") != "trained_unvalidated" or evaluation.get("status") != "piper_offline_validated":
        raise ValueError("Piper 权重尚未完成独立离线评估")
    if record.get("checkpoint_sha256") != checkpoint_hashes(checkpoint):
        raise ValueError("Piper checkpoint 文件摘要不匹配")
    verify_checkpoint_normalization(checkpoint, record)
    if evaluation.get("training_record_sha256") != digest(record_path):
        raise ValueError("离线评估未绑定此训练记录")
    check_openpi(openpi_root, Path(record["base_checkpoint"]))
    verify_base_checkpoint(Path(record["base_checkpoint"]))
    from openpi.policies import policy_config
    config = build_config("local/piper_train", Path(record["base_checkpoint"]),
                          record_path.parent, record["steps"], record["batch_size"], record["arm_mode"])
    return policy_config.create_trained_policy(config, checkpoint)


def main():
    parser = argparse.ArgumentParser(description="Piper π0.5 离线训练门禁与配置；不连接 ROS/CAN")
    parser.add_argument("--openpi-root", type=Path, required=True)
    parser.add_argument("--base-checkpoint", type=Path, required=True)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--evidence", type=Path, required=True)
    parser.add_argument("--audit-report", type=Path, required=True)
    parser.add_argument("--work-dir", type=Path, required=True)
    parser.add_argument("--lerobot-v3-site-packages", type=Path,
                        help="隔离安装 lerobot==0.4.4 的 site-packages；固定 OpenPI 源码保持不变")
    parser.add_argument("--steps", type=int, default=20000)
    parser.add_argument("--batch-size", type=int, default=8)
    stage = parser.add_mutually_exclusive_group()
    stage.add_argument("--prepare-normalization", action="store_true",
                       help="来源门禁通过后仅从 train 计算统计，输出待独立核验资产，不训练")
    stage.add_argument("--train", action="store_true", help="门禁通过后使用已独立核准的统计资产微调，不重新计算")
    parser.add_argument("--normalization-version", help="--prepare-normalization 生成的独立训练统计版本")
    args = parser.parse_args()
    if args.steps <= 0 or args.batch_size <= 0:
        parser.error("steps 和 batch-size 必须为正整数")
    if args.prepare_normalization and not args.normalization_version:
        parser.error("--prepare-normalization 需要 --normalization-version")
    if args.normalization_version and not args.prepare_normalization:
        parser.error("--normalization-version 只用于 --prepare-normalization")
    try:
        evidence = check_dataset(args.dataset, args.evidence, args.audit_report,
                                 require_normalization=not args.prepare_normalization)
        if args.prepare_normalization and not normalization_version_valid(
                args.normalization_version, evidence["data_convention_version"]):
            raise ValueError("统计版本不能为空、UNVERIFIED 或原始数据约定版本")
        revision = check_openpi(args.openpi_root, args.base_checkpoint)
        if args.lerobot_v3_site_packages is None:
            raise ValueError("训练读取预检缺少 --lerobot-v3-site-packages")
        install_lerobot_v3_compat(args.lerobot_v3_site_packages)
        reader = verify_openpi_v3_reader(args.dataset, args.base_checkpoint, evidence["arm_mode"])
        if not args.train and not args.prepare_normalization:
            print(json.dumps({"status": "data_gate_and_v3_reader_passed", "openpi_commit": revision,
                              "v3_reader": reader,
                              **evidence}, ensure_ascii=False, indent=2))
            return 0
        verify_base_checkpoint(args.base_checkpoint)
        check_work_dir(args.work_dir, args.dataset)
        # LeRobot 的本地 repo_id 解析依赖 HF_LEROBOT_HOME；只建立到经核验数据的只读引用。
        home = args.work_dir / "lerobot"
        repo = home / "local" / "piper_train"
        repo.parent.mkdir(parents=True)
        repo.symlink_to((args.dataset / "train").resolve(), target_is_directory=True)
        os.environ["HF_LEROBOT_HOME"] = str(home)
        import lerobot.datasets.lerobot_dataset as dataset_v3
        dataset_v3.HF_LEROBOT_HOME = home
        cfg = build_config("local/piper_train", args.base_checkpoint, args.work_dir,
                           args.steps, 1 if args.prepare_normalization else args.batch_size, evidence["arm_mode"])
        asset = cfg.assets_dirs / ASSET_ID / "norm_stats.json"
        if not args.prepare_normalization:
            asset.parent.mkdir(parents=True, exist_ok=True)
            approved = evidence_file(evidence["normalization"], "已核准归一化资产")
            shutil.copyfile(approved, asset)
            if digest(asset) != evidence["normalization"]["sha256"]:
                raise ValueError("复制后的训练归一化资产 SHA-256 不匹配")
        with (args.work_dir / "training.log").open("w", encoding="utf-8") as log:
            with redirect_stdout(log), redirect_stderr(log):
                run_openpi_script(args.openpi_root,
                                  "compute_norm_stats.py" if args.prepare_normalization else "train.py", cfg)
                log.flush()
        current_evidence = check_dataset(args.dataset, args.evidence, args.audit_report,
                                         require_normalization=not args.prepare_normalization)
        if current_evidence != evidence:
            raise ValueError("统计计算或训练期间数据、独立证据或来源审计发生变化")
        if args.prepare_normalization:
            result = write_normalization_provenance(
                asset, evidence, args.normalization_version, args.work_dir / "normalization_provenance.json")
            print(json.dumps(result, ensure_ascii=False, indent=2))
            return 0
        steps = sorted((path for path in cfg.checkpoint_dir.iterdir() if path.is_dir() and path.name.isdigit()),
                       key=lambda path: int(path.name))
        if not steps:
            raise ValueError("openpi 训练未生成 checkpoint")
        checkpoint = steps[-1]
        normalization_binding = bind_checkpoint_normalization(checkpoint, evidence)
        hashes = checkpoint_hashes(checkpoint)
        record = {"status": "trained_unvalidated", "openpi_commit": revision,
                  "base_checkpoint": str(args.base_checkpoint.resolve()), **evidence,
                  "config": CONFIG_NAME, "steps": args.steps, "batch_size": args.batch_size,
                  "checkpoint_dir": str(checkpoint), "checkpoint_sha256": hashes,
                  **normalization_binding,
                  "training_log_sha256": digest(args.work_dir / "training.log")}
        (args.work_dir / "training_record.json").write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps(record, ensure_ascii=False, indent=2))
        return 0
    except (ValueError, FileNotFoundError, KeyError, OSError) as error:
        print(f"拒绝：{error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
