#!/usr/bin/env python3
"""Start unlimited teleoperation with master double-tap reset."""

import sys

from piper.piper_teleop import main as teleop_main


def _value_after(args, option, default):
    """Read a simple option value without changing the teleop parser."""
    for index, value in enumerate(args):
        if value == option and index + 1 < len(args):
            return args[index + 1]
        prefix = option + '='
        if value.startswith(prefix):
            return value[len(prefix):]
    return default


def main(args=None):
    """Run piper_teleop without a duration and with quick reset enabled."""
    argv = list(sys.argv[1:] if args is None else args)
    side = _value_after(argv, '--side', 'left')
    defaults = []
    if '--master-topic' not in argv and not any(
            value.startswith('--master-topic=') for value in argv):
        defaults.extend(['--master-topic', f'/joint_states_master_{side}'])
    if '--quick-reset' not in argv:
        defaults.append('--quick-reset')
    # launch_ros appends ``--ros-args`` even when no ROS-specific options are
    # present. Application defaults must stay before that marker or rclpy will
    # interpret them as ROS arguments and raise UnknownROSArgsError.
    insert_at = (argv.index('--ros-args')
                 if '--ros-args' in argv else len(argv))
    argv[insert_at:insert_at] = defaults
    return teleop_main(argv)


if __name__ == '__main__':
    raise SystemExit(main())
