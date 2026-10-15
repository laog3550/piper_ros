#!/usr/bin/env python3
"""VLA 动作的纯离线安全监督。运行：python3 -m unittest discover -s tools/vla_contract -p 'test_*.py'。不连接 ROS/CAN。"""

from dataclasses import dataclass
import math

from validate_contract import ARMS, validate


class SafetyError(ValueError):
    pass


def _finite(value, positive=False):
    return type(value) in (int, float) and math.isfinite(value) and (not positive or value > 0)


def _vector(value, name):
    if not isinstance(value, (tuple, list)) or len(value) != 7 or not all(_finite(x) for x in value):
        raise SafetyError(f"{name}: 需要七个有限数")
    return tuple(value)


@dataclass(frozen=True)
class ArmLimits:
    lower: tuple
    upper: tuple
    max_velocity: tuple
    max_acceleration: tuple

    def check(self):
        for name in ("lower", "upper", "max_velocity", "max_acceleration"):
            _vector(getattr(self, name), name)
        if any(lo >= hi for lo, hi in zip(self.lower, self.upper)):
            raise SafetyError("关节限位无效")
        if any(x <= 0 for x in (*self.max_velocity, *self.max_acceleration)):
            raise SafetyError("速度/加速度限位无效")


@dataclass(frozen=True)
class SafetyProfile:
    """全部参数必须来自已验证配置；回调必须检查整段扫掠轨迹。"""

    profile_id: str
    calibration_version: str
    limits: dict
    max_action_age_ns: int
    max_observation_age_ns: int
    max_source_age_ns: int
    max_feedback_age_ns: int
    watchdog_ns: int
    max_action_duration_ns: int
    max_schedule_lag_ns: int
    feedback_position_tolerance: tuple
    swept_collision_free: object
    segment_uses_shared_zone: object

    def check(self, arms):
        if not self.profile_id or not self.calibration_version:
            raise SafetyError("安全配置/标定版本缺失")
        if set(self.limits) != set(arms):
            raise SafetyError("安全配置缺少执行臂限位")
        for arm in arms:
            self.limits[arm].check()
        for name in ("max_action_age_ns", "max_observation_age_ns", "max_source_age_ns", "max_feedback_age_ns",
                     "watchdog_ns", "max_action_duration_ns", "max_schedule_lag_ns"):
            value = getattr(self, name)
            if type(value) is not int or value <= 0:
                raise SafetyError(f"{name}: 未配置有效阈值")
        tolerance = _vector(self.feedback_position_tolerance, "feedback_position_tolerance")
        if any(x < 0 for x in tolerance):
            raise SafetyError("反馈位置容差无效")
        if not callable(self.swept_collision_free) or not callable(self.segment_uses_shared_zone):
            raise SafetyError("缺少已标定的碰撞/共享区检查器")


class ActionSupervisor:
    """只返回获准的离线目标；STOP 锁存后必须显式 reset。"""

    def __init__(self, profile):
        self.profile = profile
        self.state = "HOLD"
        self.reason = "尚无动作"
        self.feedback = {}
        self.plan = None
        self.index = 0
        self.last_target = None
        self.last_issue_ns = None

    def update_feedback(self, arm, position, velocity, timestamp_ns, healthy=True):
        if arm not in ("left", "right") or type(timestamp_ns) is not int or timestamp_ns <= 0:
            if self.state == "RUNNING":
                self.stop("反馈格式无效")
            raise SafetyError("反馈来源或时间戳无效")
        if not healthy:
            self.stop(f"{arm}: 反馈故障")
            return
        try:
            pos, vel = _vector(position, "position"), _vector(velocity, "velocity")
        except SafetyError:
            if self.state == "RUNNING":
                self.stop("反馈数值无效")
            raise
        old = self.feedback.get(arm)
        if old and timestamp_ns <= old[2]:
            self.stop(f"{arm}: 反馈时间未前进")
            return
        self.feedback[arm] = (pos, vel, timestamp_ns)
        if self.state == "RUNNING" and arm in self.plan[1]:
            limit = self.profile.limits[arm]
            if any(not lo <= x <= hi for x, lo, hi in zip(pos, limit.lower, limit.upper)) or any(
                    abs(v) > cap for v, cap in zip(vel, limit.max_velocity)):
                self.stop(f"{arm}: 实时反馈越界")
                return
            arms = self.plan[1]
            paths = {name: [old[0], pos] if name == arm else [self.feedback[name][0]] * 2 for name in arms}
            try:
                if self.profile.swept_collision_free(paths) is not True:
                    raise SafetyError("实时扫掠碰撞检查未通过")
                if len(arms) == 2:
                    occupancy = [self.profile.segment_uses_shared_zone(name, *paths[name]) for name in arms]
                    if any(type(x) is not bool for x in occupancy) or all(occupancy):
                        raise SafetyError("实时共享区互斥检查未通过")
            except Exception as exc:
                self.stop(str(exc))

    def stop(self, reason):
        self.state, self.reason, self.plan, self.last_target = "STOP", reason, None, None

    def hold(self, reason="操作员 HOLD"):
        if self.state != "STOP":
            self.state, self.reason, self.plan, self.last_target = "HOLD", reason, None, None

    def reset(self, now_ns):
        if self.state != "STOP":
            raise SafetyError("仅 STOP 可复位")
        if type(now_ns) is not int or now_ns <= 0:
            raise SafetyError("复位时间无效")
        arms = tuple(self.feedback)
        if not arms:
            raise SafetyError("复位缺少反馈")
        self.profile.check(arms)
        self._check_feedback(arms, now_ns)
        paths = {arm: [self.feedback[arm][0]] for arm in arms}
        if self.profile.swept_collision_free(paths) is not True:
            raise SafetyError("复位姿态碰撞检查未通过")
        if len(arms) == 2:
            occupancy = [self.profile.segment_uses_shared_zone(arm, path[0], path[0])
                         for arm, path in paths.items()]
            if any(type(x) is not bool for x in occupancy) or any(occupancy):
                raise SafetyError("复位时共享区检查未通过")
        self.state, self.reason = "HOLD", "故障已人工复位"

    def _check_feedback(self, arms, now_ns):
        for arm in arms:
            if arm not in self.feedback:
                raise SafetyError(f"{arm}: 无反馈")
            age = now_ns - self.feedback[arm][2]
            if age < 0 or age > self.profile.max_feedback_age_ns:
                raise SafetyError(f"{arm}: 反馈过期或来自未来")
            limit = self.profile.limits[arm]
            pos, vel, _ = self.feedback[arm]
            if any(not lo <= x <= hi for x, lo, hi in zip(pos, limit.lower, limit.upper)):
                raise SafetyError(f"{arm}: 当前姿态越界")
            if any(abs(v) > cap for v, cap in zip(vel, limit.max_velocity)):
                raise SafetyError(f"{arm}: 当前速度越界")

    def submit(self, data, now_ns):
        if self.state == "STOP":
            raise SafetyError("STOP 已锁存")
        if self.state == "RUNNING":
            self.stop("执行期间收到新动作")
            raise SafetyError(self.reason)
        try:
            validate(data)
            obs, goal, action = (data[key] for key in ("observation", "skill_goal", "action_chunk"))
            arms = ARMS[action["arm_mode"]]
            self.profile.check(arms)
            if goal["safety_profile"] != self.profile.profile_id or obs["calibration_version"] != self.profile.calibration_version:
                raise SafetyError("安全配置或标定版本不匹配")
            if type(now_ns) is not int or now_ns <= 0:
                raise SafetyError("当前时间无效")
            for stamp, max_age in ((obs["observation_timestamp_ns"], self.profile.max_observation_age_ns),
                                   (action["inference_timestamp_ns"], self.profile.max_action_age_ns)):
                if not 0 <= now_ns - stamp <= max_age:
                    raise SafetyError("观测/动作过期或来自未来")
            for source in (*obs["images"].values(), *obs["state"].values()):
                if obs["observation_timestamp_ns"] - source["header_timestamp_ns"] > self.profile.max_source_age_ns:
                    raise SafetyError("快照源数据过期")
            self._check_feedback(arms, now_ns)
            duration = action["dt_ns"] * action["horizon"]
            if duration > self.profile.max_action_duration_ns or duration > goal["timeout_sec"] * 1e9:
                raise SafetyError("动作超过时限")
            paths = {}
            for arm in arms:
                start, initial_velocity, _ = self.feedback[arm]
                if any(abs(a - b) > tol for a, b, tol in zip(start, obs["state"][arm]["position"],
                                                              self.profile.feedback_position_tolerance)):
                    raise SafetyError(f"{arm}: 观测与当前反馈不一致")
                path = [start] + [tuple(row) for row in action["targets"][arm]]
                limit = self.profile.limits[arm]
                prev_velocity = initial_velocity
                for before, after in zip(path, path[1:]):
                    if any(not lo <= x <= hi for x, lo, hi in zip(after, limit.lower, limit.upper)):
                        raise SafetyError(f"{arm}: 位置越界")
                    velocity = tuple((b - a) * 1e9 / action["dt_ns"] for a, b in zip(before, after))
                    acceleration = tuple((v - p) * 1e9 / action["dt_ns"] for v, p in zip(velocity, prev_velocity))
                    if any(abs(v) > cap for v, cap in zip(velocity, limit.max_velocity)):
                        raise SafetyError(f"{arm}: 速度越界")
                    if any(abs(a) > cap for a, cap in zip(acceleration, limit.max_acceleration)):
                        raise SafetyError(f"{arm}: 加速度越界")
                    prev_velocity = velocity
                if any(v != 0 for v in prev_velocity):
                    raise SafetyError(f"{arm}: 末端非静止")
                paths[arm] = path
            if self.profile.swept_collision_free(paths) is not True:
                raise SafetyError("扫掠碰撞检查未通过")
            if len(arms) == 2:
                for step in range(action["horizon"]):
                    occupancy = [self.profile.segment_uses_shared_zone(arm, paths[arm][step], paths[arm][step + 1])
                                 for arm in arms]
                    if any(type(x) is not bool for x in occupancy) or all(occupancy):
                        raise SafetyError("共享区互斥检查未通过")
        except Exception as exc:
            self.hold(f"动作拒绝：{exc}")
            raise SafetyError(self.reason) from exc
        self.plan = (action, tuple(arms), now_ns, min(self.profile.max_action_duration_ns,
                                                    int(goal["timeout_sec"] * 1e9)))
        self.index, self.state, self.reason, self.last_target = 0, "RUNNING", "动作已通过预检", None

    def tick(self, now_ns):
        if self.state != "RUNNING":
            return None
        action, arms, start, timeout = self.plan
        try:
            if type(now_ns) is not int or now_ns < start:
                raise SafetyError("执行时钟异常")
            if now_ns - start > timeout:
                raise SafetyError("动作超时")
            for arm in arms:
                age = now_ns - self.feedback[arm][2]
                if age < 0 or age > self.profile.watchdog_ns:
                    raise SafetyError(f"{arm}: watchdog 超时")
            if self.index == action["horizon"]:
                if all(self.feedback[arm][2] > self.last_issue_ns for arm in arms):
                    if not self._targets_reached(arms):
                        raise SafetyError("最终目标反馈不一致")
                    self.hold("动作完成")
                return None
            due = start + (self.index + 1) * action["dt_ns"]
            if now_ns < due:
                return None
            if now_ns - due > self.profile.max_schedule_lag_ns:
                raise SafetyError("调度超时")
            if self.last_target is not None:
                if any(self.feedback[arm][2] <= self.last_issue_ns for arm in arms) or not self._targets_reached(arms):
                    raise SafetyError("上一目标反馈不一致")
            target = {arm: tuple(action["targets"][arm][self.index]) for arm in arms}
            self.index += 1
            self.last_target, self.last_issue_ns = target, now_ns
            return target
        except SafetyError as exc:
            self.stop(str(exc))
            return None

    def _targets_reached(self, arms):
        return all(all(abs(actual - expected) <= tolerance for actual, expected, tolerance in zip(
            self.feedback[arm][0], self.last_target[arm], self.profile.feedback_position_tolerance))
            for arm in arms)
