#!/usr/bin/env python3
"""Reusable teleoperation filters and jerk-limited smoothing."""

import math
from typing import Dict, Tuple

DEFAULT_FILTER_TAU_S = 0.02
# Deadband, in degrees, applied to the follow-phase master reading before it
# reaches the filter.  Any move smaller than this is not passed on at all, so
# the follower holds perfectly still instead of echoing a hand that is
# nominally resting: One Euro already flattens the 5-15 Hz band, but a slow
# wobble below its 1 Hz cutoff still gets through.  The cost is that the
# target may sit up to this far from the master's true angle after a move.
DEFAULT_DEADBAND_DEG = 0.2
# 速度门限：输入速度低于它才认为"手停着"。实测手在静止时的慢晃（0.5 Hz、±0.15 度）
# 速度约 0.47 deg/s，而人刻意慢拖至少有 1 deg/s 量级，所以 0.8 能把两者分开。
DEFAULT_DEADBAND_SPEED_DEG_S = 0.8
# One Euro filter defaults (Casiez et al., CHI 2012).  The cutoff is
# ``min_cutoff + beta * speed`` in Hz and deg/s, so beta is the adaptive part:
# 0.3 Hz per deg/s.  Chosen from a measured comparison at 50 Hz against a
# 10 Hz +-0.3 degree hand tremor: at rest this leaves 0.036 degrees of wobble
# where the fixed low-pass (tau=0.02s) leaves 0.146, and while dragging at
# 100 deg/s it lags 0.43 degrees where the low-pass lags 2.00.
DEFAULT_ONE_EURO_MIN_CUTOFF_HZ = 1.0
DEFAULT_ONE_EURO_BETA = 0.3
DEFAULT_ONE_EURO_D_CUTOFF_HZ = 1.0
# α-β filter defaults.  They are pole-placed rather than hand-tuned.  For the
# constant-velocity observer, a repeated discrete error pole λ gives
# α=1-λ² and β=(1-λ)².  λ=0.65 at the 50 Hz teleop rate is a 46.4 ms
# continuous-time constant.
#
# 2026-09-24 第二轮真机反馈指出运动中仍有明显抖动，因此从追求暂态的 0.40
# 调回稳定优先的 0.65。与下面 12 rad/s 的平滑级组合后，在同一份 90 秒左臂
# master 记录上，5~15 Hz 指令 RMS 从 0.0192° 降到 0.0063°；20~80 deg/s
# 档的指令侧中位偏差从 0.715° 增到 1.228°，仍远小于 follower 约 9.5° 的
# 实测机械滞后。
# **匀速段的滞后与 α、β 无关**：速度估计收敛后前馈把稳态滞后补成 0，改这两个
# 只影响起步与变速的暂态。要再压暂态就得同时开大平滑级带宽，代价更大
# （15→20 rad/s 抖动 0.052°→0.115°，而暂态只再降约两成）。
DEFAULT_ALPHA_BETA_POLE = 0.65
DEFAULT_ALPHA_BETA_ALPHA = 1.0 - DEFAULT_ALPHA_BETA_POLE ** 2  # 0.5775
DEFAULT_ALPHA_BETA_BETA = (1.0 - DEFAULT_ALPHA_BETA_POLE) ** 2  # 0.1225
# A sample gap this large means the constant-velocity model is no longer a
# trustworthy description of what happened between observations.  Re-anchor
# on the measurement instead of extrapolating stale velocity through the gap.
DEFAULT_ALPHA_BETA_MAX_DT_S = 0.10


def _smoothing_factor(dt: float, cutoff_hz: float) -> float:
    """First-order low-pass coefficient for one interval and cutoff."""
    rate = 2.0 * math.pi * cutoff_hz * dt
    return rate / (rate + 1.0)


class AlphaBetaFilter:
    """Constant-velocity position and velocity estimator, one per joint.

    Each update first predicts position from the previous velocity and then
    applies a fixed-gain correction from the measured position residual.  The
    result is a smoothed position plus a velocity estimate suitable for the
    follow-phase feed-forward path.  Unlike differentiating a low-pass output,
    the residual explicitly accounts for the motion predicted during ``dt``.
    """

    def __init__(self, alpha: float = DEFAULT_ALPHA_BETA_ALPHA,
                 beta: float = DEFAULT_ALPHA_BETA_BETA,
                 max_dt: float = DEFAULT_ALPHA_BETA_MAX_DT_S):
        if not 0.0 < alpha <= 1.0:
            raise ValueError('alpha must be in (0, 1]')
        if not 0.0 <= beta <= 1.0:
            raise ValueError('beta must be in [0, 1]')
        if max_dt <= 0.0:
            raise ValueError('max_dt must be positive')
        self.alpha = float(alpha)
        self.beta = float(beta)
        self.max_dt = float(max_dt)
        self._state: Dict[int, Tuple[float, float]] = {}

    def reset(self, values):
        """Seed every joint at rest on the last published target."""
        self._state = {
            joint: (float(value), 0.0) for joint, value in values.items()
        }
        return dict(values)

    def velocities(self):
        """Return the newest estimated velocity in degrees per second."""
        return {joint: state[1] for joint, state in self._state.items()}

    def update(self, values, dt: float):
        """Predict and correct every observed joint for one sample interval."""
        estimated = {}
        for joint, measurement in values.items():
            state = self._state.get(joint)
            if state is None:
                self._state[joint] = (float(measurement), 0.0)
                estimated[joint] = float(measurement)
                continue
            position, velocity = state
            if dt <= 0.0:
                estimated[joint] = position
                continue
            if dt > self.max_dt:
                # A scheduler pause must not project the old velocity through
                # the whole gap.  Start a new estimate at the fresh sample.
                self._state[joint] = (float(measurement), 0.0)
                estimated[joint] = float(measurement)
                continue
            predicted = position + velocity * dt
            residual = float(measurement) - predicted
            position = predicted + self.alpha * residual
            velocity = velocity + self.beta * residual / dt
            self._state[joint] = (position, velocity)
            estimated[joint] = position
        return estimated


class OneEuroFilter:
    """
    Velocity-adaptive low-pass, one state per joint.

    A fixed cutoff cannot both kill hand tremor and avoid lag: suppressing the
    8-12 Hz band means a cutoff near 1 Hz, which lags by ``speed / (2*pi*fc)``
    -- over 1.5 degrees while dragging at 30 deg/s.  This filter moves its own
    cutoff with the signal instead: ``fc = min_cutoff + beta * |dx|``, where
    ``dx`` is the derivative smoothed by its own low-pass (``d_cutoff``).  The
    derivative smoother is what keeps tremor from raising the cutoff that is
    supposed to suppress it, so keep ``d_cutoff`` low.

    Parameters are per-joint and seeded by :meth:`reset`, and ``dt`` is the
    measured loop interval, exactly as in :class:`LowPassFilter`.

    Like any smoothing filter this shapes the signal without bounding it: a
    step still leaves the output short by roughly ``dx * dt / tau_effective``,
    so a speed ceiling is still the driver's business, not the filter's.
    """

    def __init__(self, min_cutoff: float = DEFAULT_ONE_EURO_MIN_CUTOFF_HZ,
                 beta: float = DEFAULT_ONE_EURO_BETA,
                 d_cutoff: float = DEFAULT_ONE_EURO_D_CUTOFF_HZ):
        if min_cutoff <= 0.0:
            raise ValueError('min_cutoff must be positive')
        if d_cutoff <= 0.0:
            raise ValueError('d_cutoff must be positive')
        if beta < 0.0:
            raise ValueError('beta must not be negative')
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        self._state: Dict[int, Tuple[float, float]] = {}

    def reset(self, values):
        """Seed the filter with a reading so the first output has no jump."""
        self._state = {
            joint: (float(value), 0.0) for joint, value in values.items()
        }
        return dict(values)

    def velocities(self):
        """
        Return the filtered derivative per joint, in degrees per second.

        This is the smoothed speed estimate the filter already maintains; the
        follow-phase smoother uses it as velocity feed-forward so that a
        steady drag is tracked without lag.
        """
        return {joint: state[1] for joint, state in self._state.items()}

    def update(self, values, dt: float):
        """Advance the filter by ``dt`` seconds and return the smoothed values."""
        smoothed = {}
        for joint, sample in values.items():
            state = self._state.get(joint)
            if state is None:
                # A joint that appears for the first time is taken as-is:
                # there is no history to smooth against.
                self._state[joint] = (sample, 0.0)
                smoothed[joint] = sample
                continue
            x_prev, dx_prev = state
            if dt <= 0.0:
                # No time passed, so nothing is smoothed towards yet.
                smoothed[joint] = x_prev
                continue
            a_d = _smoothing_factor(dt, self.d_cutoff)
            dx_hat = a_d * (sample - x_prev) / dt + (1.0 - a_d) * dx_prev
            cutoff = self.min_cutoff + self.beta * abs(dx_hat)
            a = _smoothing_factor(dt, cutoff)
            x_hat = a * sample + (1.0 - a) * x_prev
            self._state[joint] = (x_hat, dx_hat)
            smoothed[joint] = x_hat
        return smoothed


class DeadbandGate:
    """
    Hold the reading still while the hand is at rest; follow it otherwise.

    A joint's output is held only when **both** conditions hold: the input has
    not moved further than ``threshold`` from its anchor, and the input's
    smoothed speed is below ``speed_threshold``.  The speed test is what keeps
    a *slow but deliberate* drag continuous: gating on amplitude alone chops
    such a drag into steps of one threshold, and between the steps the
    commanded velocity is zero -- the arm then stops and re-accelerates over
    and over (2026-09-23 实测：慢拖时指令速度反复归零，操作者描述为"每一小段
    速度都会归为零然后重新加速"）。As soon as the input is moving, the anchor
    follows it and nothing is held.

    The speed estimate is this gate's own: a first-order low-pass (1 Hz) of
    the per-cycle difference, so it does not depend on anything upstream.

    ``threshold_deg = 0`` passes everything through unchanged, and
    ``speed_threshold = 0`` reduces the gate to the amplitude-only behaviour.
    """

    # 速度估计的低通截止频率。3 Hz 让它在约 0.05 秒（3 个周期）内建立起来，
    # 于是"慢拖被切成台阶"只剩开头这几十毫秒、幅度远小于阈值；再慢就来不及
    # 区分"手停着晃"与"手在动"。
    SPEED_CUTOFF_HZ = 3.0

    def __init__(self, threshold_deg: float = DEFAULT_DEADBAND_DEG,
                 speed_threshold: float = DEFAULT_DEADBAND_SPEED_DEG_S):
        if threshold_deg < 0.0:
            raise ValueError('threshold must not be negative')
        if speed_threshold < 0.0:
            raise ValueError('speed threshold must not be negative')
        self.threshold = float(threshold_deg)
        self.speed_threshold = float(speed_threshold)
        self._anchor: Dict[int, float] = {}
        self._speed: Dict[int, float] = {}
        self._last: Dict[int, float] = {}

    def reset(self, values):
        """Anchor every joint to the given reading and forget its speed."""
        self._anchor = {joint: float(value) for joint, value in values.items()}
        self._speed = {joint: 0.0 for joint in values}
        self._last = dict(self._anchor)
        return dict(values)

    def update(self, values, dt: float):
        """Return the reading to use, holding joints that are simply resting."""
        alpha = _smoothing_factor(max(dt, 1e-9), self.SPEED_CUTOFF_HZ)
        gated = {}
        for joint, sample in values.items():
            anchor = self._anchor.get(joint, sample)
            # 速度取**相邻两帧输入**之差：若拿"与锚点之差"来算，锚点被保持期间
            # 这个差会一路虚高，反而把静止判成运动。
            previous = self._last.get(joint, sample)
            speed = self._speed.get(joint, 0.0)
            if dt > 0.0:
                speed = (alpha * abs(sample - previous) / dt
                         + (1.0 - alpha) * speed)
            # 速度门限为 0 表示不做速度判断，退化成只按幅度保持。
            moving = (self.speed_threshold > 0.0
                      and speed >= self.speed_threshold)
            if moving or abs(sample - anchor) >= self.threshold:
                anchor = sample
                gated[joint] = sample
            else:
                gated[joint] = anchor
            self._anchor[joint] = anchor
            self._speed[joint] = speed
            self._last[joint] = sample
        return gated


# 五次插值（jerk 受限平滑）的默认参数。
#
# 这一级要解决的是"大幅度移动时抖动严重"：One Euro 的截止频率随速度线性打开
# （100 deg/s 时约 31 Hz），手抖和结构振动都会跟过去。用一条**带宽固定**的
# 三阶级联环（位置→速度→加速度→jerk）把 5~15 Hz 整段压掉，同时靠**速度
# 前馈**保住跟手性：2026-09-23 在真实拖动数据上实测，5~15 Hz 衰减 95~98%，
# 而滞后中位数只有 0.0~0.24 度（比不加这一级时的 1.5 度更紧）。
#
# 级联环按三阶 Butterworth 极点配置：特征多项式 s³ + 2ωs² + 2ω²s + ω³，
# 于是 k_p = ω/2、k_v = ω、k_a = 2ω，只有一个可调参数 ω（带宽）。ω 越大越
# 跟手、抖动抑制越弱；当前默认 12 rad/s 是左臂真实轨迹离线重放后的折中值，
# 150 deg/s 急停的指令侧过冲约 8 度。
DEFAULT_SMOOTH_BANDWIDTH_RAD_S = 12.0
# 速度、加速度、jerk 的硬上限：由实测加速度分布定的安全网（90 分位约 4700、
# 峰值约 8500 deg/s²），正常情况下由 ω 决定形状，这几个上限只在猛拉时兜底。
DEFAULT_SMOOTH_MAX_VELOCITY_DEG_S = 172.0
DEFAULT_SMOOTH_MAX_ACCELERATION_DEG_S2 = 6000.0
DEFAULT_SMOOTH_MAX_JERK_DEG_S3 = 60000.0


def _clamp(value, low, high):
    """Clamp ``value`` into ``[low, high]``."""
    return low if value < low else (high if value > high else value)


def smoother_gains(bandwidth_rad_s: float):
    """Return the (k_p, k_v, k_a) gains placed on Butterworth poles."""
    return (bandwidth_rad_s / 2.0, bandwidth_rad_s, 2.0 * bandwidth_rad_s)


class MotionSmoother:
    """
    Jerk-limited smoothing stage: the stable form of a quintic interpolation.

    Every joint keeps (position, velocity, acceleration) and each cycle walks
    one step of a cascaded loop whose poles are placed at ``-bandwidth``:
    the position error becomes a desired velocity, that becomes a desired
    acceleration, that becomes a jerk.  Because jerk is the inner-most
    command, the **acceleration is continuous** -- there are no acceleration
    jumps, which is what a quintic segment buys and what this stage is for.

    Velocity feed-forward (``velocities``, normally the One Euro derivative
    estimate) keeps a steady drag tracked without lag, so smoothing does not
    cost tracking.  What it does cost is the braking distance: when the
    reference stops abruptly the tracker overshoots by about ``v / bandwidth``
    (about 8 degrees for a 150 deg/s stop at the default 12 rad/s).

    Limits are hard: acceleration and jerk never exceed the values passed in.
    The velocity ceiling can be exceeded by a fraction of a percent (0.35% was
    measured) because the jerk limit cannot reduce an established acceleration
    within a single cycle -- a safety-net-level deviation, not a runaway.
    """

    def __init__(self,
                 bandwidth: float = DEFAULT_SMOOTH_BANDWIDTH_RAD_S,
                 max_velocity: float = DEFAULT_SMOOTH_MAX_VELOCITY_DEG_S,
                 max_acceleration: float =
                 DEFAULT_SMOOTH_MAX_ACCELERATION_DEG_S2,
                 max_jerk: float = DEFAULT_SMOOTH_MAX_JERK_DEG_S3):
        for name, value in (('bandwidth', bandwidth),
                            ('max_velocity', max_velocity),
                            ('max_acceleration', max_acceleration),
                            ('max_jerk', max_jerk)):
            if value <= 0.0:
                raise ValueError(f'{name} must be positive')
        self.bandwidth = float(bandwidth)
        self.max_velocity = float(max_velocity)
        self.max_acceleration = float(max_acceleration)
        self.max_jerk = float(max_jerk)
        self._state: Dict[int, Tuple[float, float, float]] = {}

    def reset(self, values):
        """Start every joint at rest on the given pose."""
        self._state = {
            joint: (float(value), 0.0, 0.0) for joint, value in values.items()
        }
        return dict(values)

    def update(self, positions, velocities, dt: float):
        """Advance every joint by ``dt`` seconds toward ``positions``."""
        k_p, k_v, k_a = smoother_gains(self.bandwidth)
        smoothed = {}
        for joint, goal in positions.items():
            state = self._state.get(joint)
            if state is None:
                self._state[joint] = (goal, 0.0, 0.0)
                smoothed[joint] = goal
                continue
            position, velocity, acceleration = state
            if dt <= 0.0:
                smoothed[joint] = position
                continue
            feed_forward = velocities.get(joint, 0.0)
            v_des = _clamp(k_p * (goal - position) + feed_forward,
                           -self.max_velocity, self.max_velocity)
            a_des = _clamp(k_v * (v_des - velocity),
                           -self.max_acceleration, self.max_acceleration)
            # 收紧（不是赋值）：这一步的加速不许把速度顶出上限
            a_des = min(a_des, (self.max_velocity - velocity) / dt)
            a_des = max(a_des, (-self.max_velocity - velocity) / dt)
            jerk = _clamp(k_a * (a_des - acceleration),
                          -self.max_jerk, self.max_jerk)
            acceleration += jerk * dt
            velocity += acceleration * dt
            position += velocity * dt
            self._state[joint] = (position, velocity, acceleration)
            smoothed[joint] = position
        return smoothed


class LowPassFilter:
    """
    First-order IIR smoothing of a per-joint reading: y = a*x + (1-a)*y.

    Kept as the non-default comparison path (``--filter lowpass``): its cutoff
    is fixed, so the follow phase either lags on fast drags or lets hand tremor
    through -- see :class:`OneEuroFilter`, which is the default for exactly
    that reason.

    ``a`` is derived from the measured interval rather than the configured
    period, because the control loop's real interval fluctuates and a fixed
    ``a`` would change the filter's cutoff with the loop's load:
    ``a = dt / (tau + dt)``.
    """

    def __init__(self, tau: float = DEFAULT_FILTER_TAU_S):
        if tau < 0.0:
            raise ValueError('tau must not be negative')
        self.tau = float(tau)
        self._state: Dict[int, float] = {}

    def reset(self, values):
        """Seed the filter with a reading so the first output has no jump."""
        self._state = dict(values)
        return dict(self._state)

    def update(self, values, dt: float):
        """Advance the filter by ``dt`` seconds and return the smoothed values."""
        if self.tau <= 0.0:
            alpha = 1.0
        elif dt > 0.0:
            alpha = dt / (self.tau + dt)
        else:
            # No time passed, so nothing is smoothed towards yet: hold the
            # last output and seed joints that appear for the first time.
            alpha = 0.0
        smoothed = {}
        for joint, sample in values.items():
            previous = self._state.get(joint)
            value = (sample if previous is None
                     else alpha * sample + (1.0 - alpha) * previous)
            self._state[joint] = value
            smoothed[joint] = value
        return smoothed


__all__ = [
    'DEFAULT_FILTER_TAU_S',
    'DEFAULT_DEADBAND_DEG',
    'DEFAULT_DEADBAND_SPEED_DEG_S',
    'DEFAULT_ONE_EURO_MIN_CUTOFF_HZ',
    'DEFAULT_ONE_EURO_BETA',
    'DEFAULT_ONE_EURO_D_CUTOFF_HZ',
    'DEFAULT_ALPHA_BETA_POLE',
    'DEFAULT_ALPHA_BETA_ALPHA',
    'DEFAULT_ALPHA_BETA_BETA',
    'DEFAULT_ALPHA_BETA_MAX_DT_S',
    'AlphaBetaFilter',
    'OneEuroFilter',
    'DeadbandGate',
    'DEFAULT_SMOOTH_BANDWIDTH_RAD_S',
    'DEFAULT_SMOOTH_MAX_VELOCITY_DEG_S',
    'DEFAULT_SMOOTH_MAX_ACCELERATION_DEG_S2',
    'DEFAULT_SMOOTH_MAX_JERK_DEG_S3',
    'smoother_gains',
    'MotionSmoother',
    'LowPassFilter',
]
