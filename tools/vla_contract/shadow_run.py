#!/usr/bin/env python3
"""ROS 2 只读影子诊断与合成整链验收。运行：python3 tools/vla_contract/shadow_run.py --help。绝不连接驱动或 CAN。"""

import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import sys
import tempfile
import time

from PIL import Image as PilImage

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from action_supervisor import ActionSupervisor
from offline_simulator import synthetic_profile
from openpi05_offline import RAW_ACTION_SPACE, preflight
from task_orchestrator import Skill, TaskOrchestrator
from validate_contract import synthetic, validate_observation
from vlm_critic_adapter import OfflineVlmCriticAdapter
from tools.vlm_critic.critic import verdict

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "vla_observation"))
from snapshot import CAMERAS, TOPICS, SnapshotBuilder, stamp_ns  # noqa: E402


DIAGNOSTIC_TOPIC = "/vla_shadow/diagnostics"
PROPOSAL_TOPIC = "/vla_shadow/proposed_action"


def event(stage, status, reason, **details):
    return {"mode": "shadow", "stage": stage, "status": status, "reason": reason, **details}


def inspect_snapshot(record, *, image_dir=None, now_ns=None, max_age_ns=150_000_000,
                     max_skew_ns=50_000_000):
    """只读快照逐级检查；真实路径没有动作映射，也不生成目标。"""
    output = []
    obs = record.get("observation")
    if not record.get("contract_observation_valid") or obs is None:
        output.append(event("OBSERVE", "REJECTED", "invalid_snapshot",
                            rejection_reasons=record.get("rejection_reasons", [])))
        return output
    try:
        validate_observation(obs)
        now = time.time_ns() if now_ns is None else now_ns
        stamps = [source["header_timestamp_ns"] for source in (*obs["images"].values(), *obs["state"].values())]
        if not 0 <= now - obs["observation_timestamp_ns"] <= max_age_ns:
            raise ValueError("观测过期或来自未来")
        if max(stamps) - min(stamps) > max_skew_ns:
            raise ValueError("RGB 与状态时间偏差超限")
    except (ValueError, KeyError, TypeError) as exc:
        output.append(event("OBSERVE", "REJECTED", str(exc)))
        return output
    output.append(event("OBSERVE", "VALID", "timestamp_consistent", observation_id=obs["observation_id"]))
    sources = record.get("sources", {})
    missing = []
    for role in ("top", f"{obs['arm_mode'].lower()}_wrist"):
        camera = sources.get(role, {})
        for field in ("rgb", "depth", "camera_info", "roi_depth", "world_geometry"):
            if camera.get(field, {}).get("valid") is not True:
                missing.append(f"{role}.{field}:{camera.get(field, {}).get('reason', 'missing')}")
    if not record.get("gripper_mapping_verified", False):
        missing.append("gripper_mapping_unverified")
    if not record.get("calibration_verified", False):
        missing.append("calibration_unverified")
    if not record.get("tf_verified", False) or not record.get("world_geometry_valid", False):
        missing.append("tf_or_world_geometry_unverified")
    if missing:
        output.append(event("CALIBRATION", "REJECTED", "site_gates_unverified", rejection_reasons=missing))
        return output
    # 即便外部补齐现场声明，仍要验证离线图像、时间和契约。
    skill = record.get("skill_goal")
    if skill is None or image_dir is None:
        output.append(event("POLICY_INPUT", "REJECTED", "skill_goal_or_image_dir_missing"))
        return output
    try:
        result = preflight({"observation": obs, "skill_goal": skill}, now_ns=now,
                           max_age_ns=max_age_ns, max_skew_ns=max_skew_ns,
                           image_dir=Path(image_dir))
    except (ValueError, OSError, KeyError, ImportError) as exc:
        output.append(event("POLICY_INPUT", "REJECTED", str(exc)))
        return output
    output.append(event("POLICY_INPUT", "VALID", "offline_preflight", preflight=result))
    output.append(event("PLAN", "REJECTED", "model_action_space_mismatch",
                        model_action_space=RAW_ACTION_SPACE,
                        required_action_space="absolute_joint_position_7",
                        missing_bridge=result["missing_bridge"]))
    return output


class FixtureClock:
    def __init__(self, now_ns):
        self.value = now_ns

    def now_ns(self):
        return self.value

    def sleep_until(self, due):
        self.value = max(self.value, due)


class FixtureObserver:
    def __init__(self, observation, clock):
        self.observation, self.clock = observation, clock
        self.position = list(observation["state"][observation["arm_mode"].lower()]["position"])
        self.count = 0

    def observe(self, skill, mode, now_ns):
        self.count += 1
        self.clock.value += 1_000_000
        obs = deepcopy(self.observation)
        obs["observation_id"] = f"fixture-{self.count}"
        obs["observation_timestamp_ns"] = self.clock.now_ns()
        for frame in obs["images"].values():
            frame["header_timestamp_ns"] = self.clock.now_ns() - 2_000_000
        for state in obs["state"].values():
            state["header_timestamp_ns"] = self.clock.now_ns() - 1_000_000
            state["position"] = self.position.copy()
        return obs, {mode.lower(): (self.position.copy(), [0.0] * 7, self.clock.now_ns(), True)}


class FixturePolicy:
    """显式合成动作；先经过官方 DROID 输入预检，但绝不使用其速度输出。"""

    def __init__(self, image_dir, clock):
        self.image_dir, self.clock = image_dir, clock
        self.preflights = []

    def plan(self, obs, goal):
        result = preflight({"observation": obs, "skill_goal": goal}, now_ns=self.clock.now_ns(),
                           max_age_ns=50_000_000, max_skew_ns=10_000_000,
                           image_dir=self.image_dir)
        self.preflights.append(result)
        action = synthetic(obs["arm_mode"], goal["skill_id"])["action_chunk"]
        action["model_version"] = "synthetic-fixture-only"
        action["observation_id"] = obs["observation_id"]
        action["observation_timestamp_ns"] = obs["observation_timestamp_ns"]
        action["inference_timestamp_ns"] = self.clock.now_ns()
        arm = obs["arm_mode"].lower()
        first = list(obs["state"][arm]["position"])
        first[0] += 0.01
        action["targets"][arm] = [first.copy(), first.copy()]
        return action


class ProposalOnlySink:
    """仅存拟议目标，并注入合成反馈供状态机验收；没有 ROS publisher 或驱动句柄。"""

    def __init__(self, observer):
        self.observer = observer
        self.proposals = []
        self.cancelled = False

    def dispatch(self, approved, issue_ns, deadline_ns):
        self.proposals.append(event("EXECUTE", "PROPOSED", "supervisor_approved_fixture_target",
                                    issue_ns=issue_ns, target=approved))
        self.observer.clock.value += 1_000_000
        self.observer.position = list(approved[self.observer.observation["arm_mode"].lower()])
        return self.observer.position.copy(), [0.0] * 7, self.observer.clock.now_ns(), True

    def cancel(self):
        self.cancelled = True


def run_fixture():
    """合成状态流以真实契约、预检、监督器、编排器顺序验收。"""
    with tempfile.TemporaryDirectory(prefix="piper-shadow-fixture-") as temporary:
        image_dir = Path(temporary)
        data = synthetic()
        obs = data["observation"]
        for name, frame in obs["images"].items():
            filename = f"{name}.png"
            PilImage.new("RGB", (640, 480), (20, 30, 40)).save(image_dir / filename)
            frame["uri"] = filename
        PilImage.new("RGB", (640, 480), (10, 20, 30)).save(image_dir / "before.png")
        clock = FixtureClock(obs["observation_timestamp_ns"])
        observer = FixtureObserver(obs, clock)
        policy = FixturePolicy(image_dir, clock)
        sink = ProposalOnlySink(observer)
        supervisor = ActionSupervisor(synthetic_profile())
        def sample_builder(skill, post):
            return {"sample_id": post["observation_id"], "skill_id": skill.name,
                    "arm_mode": post["arm_mode"], "target_id": skill.target,
                    "calibration_id": post["calibration_version"],
                    "pre_rgb": "before.png", "post_rgb": post["images"]["top_rgb"]["uri"],
                    "geometry": {"pre_in_gripper": None, "post_in_gripper": None,
                                 "post_in_region": None, "post_target_visible": None}}
        critic = OfflineVlmCriticAdapter(sample_builder, lambda sample: verdict(sample), image_dir)
        task = TaskOrchestrator((Skill("pick", "object-1"),), "LEFT", observer, policy,
                                critic, sink, clock, supervisor)
        board = task.run()
        events = [event("OBSERVE", "VALID", "synthetic_snapshot", observation_id=obs["observation_id"])]
        if policy.preflights:
            events.append(event("POLICY_INPUT", "VALID", "official_droid_input_preflight",
                                preflight=policy.preflights[0]))
        events += [event(phase, "ENTERED", reason, skill_index=index, attempt=attempt)
                  for index, attempt, phase, reason in board.history]
        return {"fixture_only": True, "phase": board.phase, "reason": board.reason,
                "supervisor_state": supervisor.state, "preflight_count": len(policy.preflights),
                "critic_transition": critic.last_output["recommended_transition"] if critic.last_output else None,
                "proposals": sink.proposals, "events": events, "sink_cancelled": sink.cancelled}


def run_replay(args):
    with Path(args.snapshots).open(encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                record = json.loads(line)
                for result in inspect_snapshot(record, image_dir=args.image_dir,
                                               now_ns=args.now_ns, max_age_ns=args.max_age_ns,
                                               max_skew_ns=args.max_skew_ns):
                    print(json.dumps(result, ensure_ascii=False))


def publish_fixture_ros(result, hold_sec):
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    if not math.isfinite(hold_sec) or hold_sec < 0:
        raise ValueError("--hold-sec 需要有限非负数")
    rclpy.init()
    node = rclpy.create_node("piper_vla_shadow_fixture")
    qos = QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    diagnostics = node.create_publisher(String, DIAGNOSTIC_TOPIC, qos)
    proposals = node.create_publisher(String, PROPOSAL_TOPIC, qos)
    try:
        for item in result["events"]:
            diagnostics.publish(String(data=json.dumps(item, ensure_ascii=False)))
        for item in result["proposals"]:
            proposals.publish(String(data=json.dumps(item, ensure_ascii=False)))
        deadline = time.monotonic() + hold_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=max(0.0, min(0.1, deadline - time.monotonic())))
    finally:
        node.destroy_node()
        rclpy.shutdown()


def run_live(args):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image, JointState
    from std_msgs.msg import String

    config = json.loads(Path(args.config).read_text(encoding="utf-8"))
    builder = SnapshotBuilder(config)
    rclpy.init()
    node = rclpy.create_node("piper_vla_shadow_read_only")
    diagnostics = node.create_publisher(String, DIAGNOSTIC_TOPIC, 10)
    proposals = node.create_publisher(String, PROPOSAL_TOPIC, 10)
    anchors = []
    subscriptions = []

    def receive(topic):
        def callback(message):
            try:
                stamp = stamp_ns(message)
            except ValueError:
                return
            builder.add(topic, message, f"ros://{topic}?header_timestamp_ns={stamp}")
            if topic == CAMERAS["top"][0]:
                anchors.append(stamp)
        return callback

    for topic, (_, kind) in TOPICS.items():
        message_type = {"rgb": Image, "depth": Image, "info": CameraInfo, "state": JointState}[kind]
        subscriptions.append(node.create_subscription(message_type, topic, receive(topic), qos_profile_sensor_data))
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.time_ns()
            while anchors and now >= anchors[0] + config["max_cross_camera_skew_ns"]:
                record = builder.build(args.arm_mode, anchors.pop(0), now)
                for result in inspect_snapshot(record, now_ns=now, max_age_ns=args.max_age_ns,
                                               max_skew_ns=args.max_skew_ns):
                    message = String(data=json.dumps(result, ensure_ascii=False))
                    diagnostics.publish(message)
                    if result["status"] == "PROPOSED":
                        proposals.publish(message)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description="只读影子诊断；目标只在显式合成夹具中拟议")
    sub = parser.add_subparsers(dest="command", required=True)
    fixture = sub.add_parser("fixture", help="合成整链验收，不使用相机或真实模型")
    fixture.add_argument("--publish-ros", action="store_true", help="将合成诊断和拟议动作发到影子专用话题")
    fixture.add_argument("--hold-sec", type=float, default=2.0, help="ROS 发布后保留节点的秒数")
    replay = sub.add_parser("replay", help="检查已保存的快照 JSONL")
    replay.add_argument("--snapshots", required=True)
    replay.add_argument("--image-dir")
    replay.add_argument("--now-ns", type=int)
    live = sub.add_parser("live", help="只读订阅并发布独立诊断话题")
    live.add_argument("--config", required=True)
    live.add_argument("--arm-mode", choices=("LEFT", "RIGHT"), default="LEFT")
    for command in (replay, live):
        command.add_argument("--max-age-ns", type=int, default=150_000_000)
        command.add_argument("--max-skew-ns", type=int, default=50_000_000)
    args = parser.parse_args()
    if args.command == "fixture":
        result = run_fixture()
        print(json.dumps(result, ensure_ascii=False, indent=2))
        if args.publish_ros:
            publish_fixture_ros(result, args.hold_sec)
    elif args.command == "replay":
        run_replay(args)
    else:
        run_live(args)


if __name__ == "__main__":
    main()
