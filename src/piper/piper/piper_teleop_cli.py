"""Command-line contract and validation for Piper teleoperation."""

from argparse import ArgumentParser

from piper.piper_feedback import (
    DEFAULT_ALPHA_BETA_ALPHA,
    DEFAULT_ALPHA_BETA_BETA,
    DEFAULT_ALPHA_BETA_MAX_DT_S,
    DEFAULT_DEADBAND_DEG,
    DEFAULT_DEADBAND_SPEED_DEG_S,
    DEFAULT_FILTER_TAU_S,
    DEFAULT_GRIPPER_DEADBAND_M,
    DEFAULT_GRIPPER_EFFORT_NM,
    DEFAULT_GRIPPER_SCALE,
    DEFAULT_ONE_EURO_BETA,
    DEFAULT_ONE_EURO_D_CUTOFF_HZ,
    DEFAULT_ONE_EURO_MIN_CUTOFF_HZ,
    DEFAULT_QUICK_RESET_MIN_TRAVEL_DEG,
    DEFAULT_QUICK_RESET_RELEASE_SPEED_DEG_S,
    DEFAULT_QUICK_RESET_TRIGGER_SPEED_DEG_S,
    DEFAULT_QUICK_RESET_WINDOW_S,
    DEFAULT_RETURN_MAX_PEAK_DEG_S,
    DEFAULT_RETURN_SPEED_DEG_S,
    DEFAULT_SMOOTH_BANDWIDTH_RAD_S,
    DEFAULT_SMOOTH_MAX_ACCELERATION_DEG_S2,
    DEFAULT_SMOOTH_MAX_JERK_DEG_S3,
    DEFAULT_SMOOTH_MAX_VELOCITY_DEG_S,
)
from piper.piper_interfaces import SIDES


DEFAULT_MASTER_TOPIC = '/joint_states_single'
DEFAULT_SIDE = 'left'
DEFAULT_ALIGN_SECONDS = 2.0
DEFAULT_MAX_STEP_DEG = 0.0
DEFAULT_SPEED = 100
DEFAULT_RATE_HZ = 50.0
FILTERS = ('alpha-beta', 'one-euro', 'lowpass', 'none')
DEFAULT_FILTER = 'alpha-beta'
MIN_ALIGN_SECONDS = 1.0


def build_parser(description=None):
    """Build the public teleoperation CLI in one extension-friendly place."""
    parser = ArgumentParser(description=description)
    parser.add_argument('--side', choices=SIDES, default=DEFAULT_SIDE,
                        help='要驱动的 follower（默认 %(default)s）')
    parser.add_argument('--master-topic', default=DEFAULT_MASTER_TOPIC,
                        help='master 臂关节角度话题（默认 %(default)s）')
    parser.add_argument('--align-seconds', type=float,
                        default=DEFAULT_ALIGN_SECONDS,
                        help='对齐阶段时长，秒（默认 %(default)s）')
    parser.add_argument('--max-step-deg', type=float,
                        default=DEFAULT_MAX_STEP_DEG,
                        help='每周期目标最大变化；0 表示不限制（默认 %(default)s）')
    parser.add_argument('--speed', type=int, default=DEFAULT_SPEED,
                        help='follower 速度百分比 1-100（默认 %(default)s）')
    parser.add_argument('--rate', type=float, default=DEFAULT_RATE_HZ,
                        help='发布频率 Hz（默认 %(default)s）')
    parser.add_argument('--duration', type=float, default=None,
                        help='运行指定秒数后结束；默认不限时')
    parser.add_argument('--quick-reset', action='store_true',
                        help='启用 master 双击手势快速复位')
    parser.add_argument('--quick-reset-window', type=float,
                        default=DEFAULT_QUICK_RESET_WINDOW_S)
    parser.add_argument('--quick-reset-speed', type=float,
                        default=DEFAULT_QUICK_RESET_TRIGGER_SPEED_DEG_S)
    parser.add_argument('--quick-reset-release-speed', type=float,
                        default=DEFAULT_QUICK_RESET_RELEASE_SPEED_DEG_S)
    parser.add_argument('--quick-reset-min-travel', type=float,
                        default=DEFAULT_QUICK_RESET_MIN_TRAVEL_DEG)
    parser.add_argument('--quick-reset-duration', type=float, default=2.0)

    smoothing = parser.add_argument_group('跟随平滑')
    smoothing.add_argument('--smooth-bandwidth', type=float,
                           default=DEFAULT_SMOOTH_BANDWIDTH_RAD_S)
    smoothing.add_argument('--smooth-max-velocity', type=float,
                           default=DEFAULT_SMOOTH_MAX_VELOCITY_DEG_S)
    smoothing.add_argument('--smooth-max-acceleration', type=float,
                           default=DEFAULT_SMOOTH_MAX_ACCELERATION_DEG_S2)
    smoothing.add_argument('--smooth-max-jerk', type=float,
                           default=DEFAULT_SMOOTH_MAX_JERK_DEG_S3)
    smoothing.add_argument('--deadband-speed', type=float,
                           default=DEFAULT_DEADBAND_SPEED_DEG_S)
    smoothing.add_argument('--deadband-deg', type=float,
                           default=DEFAULT_DEADBAND_DEG)
    smoothing.add_argument('--filter', choices=FILTERS,
                           default=DEFAULT_FILTER)
    smoothing.add_argument('--alpha-beta-alpha', type=float,
                           default=DEFAULT_ALPHA_BETA_ALPHA)
    smoothing.add_argument('--alpha-beta-beta', type=float,
                           default=DEFAULT_ALPHA_BETA_BETA)
    smoothing.add_argument('--alpha-beta-max-dt', type=float,
                           default=DEFAULT_ALPHA_BETA_MAX_DT_S)
    smoothing.add_argument('--one-euro-min-cutoff', type=float,
                           default=DEFAULT_ONE_EURO_MIN_CUTOFF_HZ)
    smoothing.add_argument('--one-euro-beta', type=float,
                           default=DEFAULT_ONE_EURO_BETA)
    smoothing.add_argument('--one-euro-d-cutoff', type=float,
                           default=DEFAULT_ONE_EURO_D_CUTOFF_HZ)
    smoothing.add_argument('--filter-tau', type=float,
                           default=DEFAULT_FILTER_TAU_S)

    gripper = parser.add_argument_group('夹爪')
    gripper.add_argument('--no-gripper', dest='gripper', action='store_false')
    gripper.add_argument('--gripper-scale', type=float,
                         default=DEFAULT_GRIPPER_SCALE)
    gripper.add_argument('--gripper-effort', type=float,
                         default=DEFAULT_GRIPPER_EFFORT_NM)
    gripper.add_argument('--gripper-deadband', type=float,
                         default=DEFAULT_GRIPPER_DEADBAND_M)

    lifecycle = parser.add_argument_group('会话生命周期')
    lifecycle.add_argument('--return-speed', type=float,
                           default=DEFAULT_RETURN_SPEED_DEG_S)
    lifecycle.add_argument('--return-max-peak', type=float,
                           default=DEFAULT_RETURN_MAX_PEAK_DEG_S)
    lifecycle.add_argument('--no-return-home', action='store_true')
    lifecycle.add_argument('--enable', action='store_true',
                           help='真正发布运动指令；不加此参数只做干跑')
    lifecycle.add_argument(
        '--manage-enable', action='store_true',
        help='由遥操作会话调用本侧使能服务；只在 --enable 时生效')
    lifecycle.add_argument(
        '--disable-on-exit', action='store_true',
        help='会话结束并完成回位后失能；需同时使用 --manage-enable')
    return parser


def validate_options(options):
    """Return a Chinese refusal reason, or None for a valid configuration."""
    checks = (
        (options.align_seconds >= MIN_ALIGN_SECONDS,
         f'--align-seconds 不得小于 {MIN_ALIGN_SECONDS}'),
        (1 <= options.speed <= 100, '--speed 必须在 1..100'),
        (options.rate > 0.0, '--rate 必须为正'),
        (options.filter_tau >= 0.0, '--filter-tau 不得为负'),
        (0.0 < options.alpha_beta_alpha <= 1.0,
         '--alpha-beta-alpha 必须在 (0, 1]'),
        (0.0 <= options.alpha_beta_beta <= 1.0,
         '--alpha-beta-beta 必须在 [0, 1]'),
        (options.alpha_beta_max_dt > 0.0,
         '--alpha-beta-max-dt 必须为正'),
        (options.deadband_deg >= 0.0, '--deadband-deg 不得为负'),
        (options.deadband_speed >= 0.0, '--deadband-speed 不得为负'),
        (options.gripper_scale > 0.0, '--gripper-scale 必须为正'),
        (options.gripper_effort >= 0.0, '--gripper-effort 不得为负'),
        (options.gripper_deadband >= 0.0, '--gripper-deadband 不得为负'),
        (options.smooth_bandwidth >= 0.0, '--smooth-bandwidth 不得为负'),
        (options.smooth_max_velocity > 0.0
         and options.smooth_max_acceleration > 0.0
         and options.smooth_max_jerk > 0.0,
         '平滑级的三个上限都必须为正'),
        (options.return_speed > 0.0 and options.return_max_peak > 0.0,
         '--return-speed 与 --return-max-peak 必须为正'),
        (options.quick_reset_window > 0.0,
         '--quick-reset-window 必须为正'),
        (options.quick_reset_speed > 0.0,
         '--quick-reset-speed 必须为正'),
        (0.0 <= options.quick_reset_release_speed
         < options.quick_reset_speed,
         '--quick-reset-release-speed 必须小于 --quick-reset-speed'),
        (options.quick_reset_min_travel > 0.0,
         '--quick-reset-min-travel 必须为正'),
        (options.quick_reset_duration > 0.0,
         '--quick-reset-duration 必须为正'),
        (not options.disable_on_exit or options.manage_enable,
         '--disable-on-exit 需要同时使用 --manage-enable'),
    )
    return next((reason for valid, reason in checks if not valid), None)
