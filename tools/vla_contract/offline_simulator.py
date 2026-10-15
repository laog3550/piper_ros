#!/usr/bin/env python3
"""仅用合成参数演练监督状态机。运行：python3 tools/vla_contract/offline_simulator.py。绝不发送 ROS/CAN 命令。"""

from dataclasses import replace

from action_supervisor import ActionSupervisor, ArmLimits, SafetyError, SafetyProfile
from validate_contract import synthetic


DT = 100_000_000


def synthetic_profile(mode="LEFT", collision=None, zone=None):
    """测试专用虚构限位，不能用于真实机械臂。"""
    arm_limits = ArmLimits((-1.0,) * 6 + (0.0,), (1.0,) * 6 + (0.1,),
                           (1.0,) * 7, (10.0,) * 7)
    arms = ("left", "right") if mode == "BIMANUAL" else (mode.lower(),)
    return SafetyProfile("offline-only", "synthetic-calibration", {arm: arm_limits for arm in arms},
                         50_000_000, 50_000_000, 50_000_000, 50_000_000, 150_000_000,
                         1_000_000_000, 10_000_000, (0.001,) * 7,
                         collision or (lambda paths: True),
                         zone or (lambda arm, before, after: False))


def make_case(mode="LEFT"):
    data = synthetic(mode)
    for arm in data["action_chunk"]["targets"]:
        first = [0.01] + [0.0] * 5 + [0.04]
        data["action_chunk"]["targets"][arm] = [first.copy(), first.copy()]
    profile = synthetic_profile(mode)
    supervisor = ActionSupervisor(profile)
    now = data["action_chunk"]["inference_timestamp_ns"] + 1_000_000
    for arm, state in data["observation"]["state"].items():
        supervisor.update_feedback(arm, state["position"], [0.0] * 7, now)
    return supervisor, data, now


def run_scenarios():
    results = {}
    supervisor, data, now = make_case()
    supervisor.submit(data, now)
    assert supervisor.tick(now + DT) is not None
    supervisor.update_feedback("left", data["action_chunk"]["targets"]["left"][0],
                               [0.1] + [0.0] * 6, now + DT + 1)
    assert supervisor.tick(now + 2 * DT) is not None
    supervisor.update_feedback("left", data["action_chunk"]["targets"]["left"][1],
                               [0.0] * 7, now + 2 * DT + 1)
    assert supervisor.tick(now + 2 * DT + 1) is None
    assert supervisor.state == "HOLD"
    results["正常动作"] = supervisor.state

    supervisor, data, now = make_case()
    try:
        supervisor.submit(data, now + 100_000_000)
    except SafetyError:
        pass
    assert supervisor.state == "HOLD"
    results["陈旧动作"] = supervisor.state

    supervisor, data, now = make_case()
    supervisor.submit(data, now)
    assert supervisor.tick(now + 160_000_000) is None
    assert supervisor.state == "STOP"
    results["反馈掉线"] = supervisor.state

    supervisor, data, now = make_case("BIMANUAL")
    supervisor.profile = replace(supervisor.profile, segment_uses_shared_zone=lambda arm, before, after: True)
    try:
        supervisor.submit(data, now)
    except SafetyError:
        pass
    assert supervisor.state == "HOLD"
    results["双臂共享区冲突"] = supervisor.state

    supervisor, data, now = make_case("BIMANUAL")
    supervisor.submit(data, now)
    supervisor.update_feedback("left", [0.0] * 6 + [0.04], [0.0] * 7, now + 1, healthy=False)
    assert supervisor.state == "STOP" and supervisor.tick(now + DT) is None
    try:
        supervisor.submit(data, now)
    except SafetyError:
        pass
    for arm in ("left", "right"):
        supervisor.update_feedback(arm, [0.0] * 6 + [0.04], [0.0] * 7, now + 2)
    supervisor.reset(now + 2)
    assert supervisor.state == "HOLD"
    results["单侧故障联动与复位"] = supervisor.state
    return results


if __name__ == "__main__":
    for scenario, state in run_scenarios().items():
        print(f"{scenario}: {state}")
