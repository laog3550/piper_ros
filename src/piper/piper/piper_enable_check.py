#!/usr/bin/env python3
"""Independent read-only check of a Piper follower arm's enable state."""

# This tool is observation-only by construction: it opens a listen-only
# socket, filters for the drivers' feedback frames, and never has a code path
# that transmits.  It deliberately decodes the frames itself instead of using
# piper_sdk, so it can corroborate the running control node rather than
# repeat it.
#
# Unlike the arm_enable_status topic, which reports a joint as valid once its
# frame has been seen, this tool also bounds how old a frame may be, so a
# joint that stops reporting cannot keep contributing a stale enable bit.

import time
from argparse import ArgumentParser
from typing import Dict

import can

from piper.piper_enable_status import EnableState, aggregate, describe
from piper.piper_feedback import FEEDBACK_CAN_IDS, JOINT_COUNT, FeedbackTracker

DEFAULT_PORTS = ('can_fl', 'can_fr')
DEFAULT_DURATION = 3.0
DEFAULT_TIMEOUT = 0.5

EXIT_OK = 0
EXIT_PARTIAL = 1
EXIT_INCOMPLETE = 2
EXIT_FAILED = 3

EXPECTATIONS = {
    'any': (EnableState.ENABLED, EnableState.DISABLED),
    'enabled': (EnableState.ENABLED,),
    'disabled': (EnableState.DISABLED,),
}


def _listen(port: str) -> can.BusABC:
    """Open a receive-only view of one CAN interface."""
    bus = can.Bus(
        interface='socketcan',
        channel=port,
        receive_own_messages=False,
    )
    bus.set_filters([
        {'can_id': can_id, 'can_mask': 0x7FF, 'extended': False}
        for can_id in FEEDBACK_CAN_IDS
    ])
    return bus


def collect(port: str, duration: float, timeout: float) -> FeedbackTracker:
    """Watch one interface for a while and record the feedback it carries."""
    tracker = FeedbackTracker(timeout=timeout)
    bus = _listen(port)
    try:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            frame = bus.recv(timeout=min(0.2, max(0.0, deadline - time.monotonic())))
            if frame is None:
                continue
            tracker.update(frame.arbitration_id, frame.data)
    finally:
        bus.shutdown()
    return tracker


def report(port: str, tracker: FeedbackTracker) -> EnableState:
    """Print one arm's per-joint state and return its aggregate verdict."""
    now = time.monotonic()
    observations = tracker.observations(now)
    state = aggregate(observations)
    print(f'--- {port} ---')
    for joint in range(1, JOINT_COUNT + 1):
        feedback = tracker.last_known(joint)
        if feedback is None:
            print(f'  joint {joint}: no feedback frame received')
            continue
        note = 'fresh' if tracker.is_fresh(joint, now) else 'STALE'
        print(f'  joint {joint}: enabled={feedback.enabled} '
              f'age={tracker.age(joint, now):.3f}s {note} '
              f'voltage={feedback.voltage:.1f}V '
              f'foc_temp={feedback.foc_temperature}C')
    print(f'  per-joint   : {describe(observations)}')
    print(f'  verdict     : {EnableState(state).name}')
    if state is EnableState.PARTIAL:
        print('  WARNING: partial enable, do not treat this arm as enabled')
    return state


def _parser() -> ArgumentParser:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument(
        '--port', action='append', dest='ports', metavar='IFACE',
        help=f'CAN interface to watch (default: {" ".join(DEFAULT_PORTS)})',
    )
    parser.add_argument(
        '--duration', type=float, default=DEFAULT_DURATION,
        help='seconds to watch each interface (default: %(default)s)',
    )
    parser.add_argument(
        '--timeout', type=float, default=DEFAULT_TIMEOUT,
        help='maximum accepted feedback age in seconds (default: %(default)s)',
    )
    parser.add_argument(
        '--expect', choices=tuple(EXPECTATIONS), default='any',
        help='verdict required for a success exit code (default: %(default)s)',
    )
    return parser


def main(args=None) -> int:
    """Run the read-only enable check; transmits nothing on any interface."""
    options = _parser().parse_args(args)
    ports = options.ports or list(DEFAULT_PORTS)
    print('read-only check: no enable, disable or motion frame is sent')
    if options.duration <= 0:
        print('ERROR: --duration must be positive')
        return EXIT_FAILED
    if options.timeout <= 0:
        print('ERROR: --timeout must be positive')
        return EXIT_FAILED

    verdicts: Dict[str, EnableState] = {}
    for port in ports:
        try:
            tracker = collect(port, options.duration, options.timeout)
        except (OSError, can.CanError) as exc:
            print(f'ERROR: cannot watch {port}: {exc}')
            return EXIT_FAILED
        verdicts[port] = report(port, tracker)

    expected = EXPECTATIONS[options.expect]
    states = tuple(verdicts.values())
    print('=' * 60)
    print('summary: ' + ', '.join(
        f'{port}={EnableState(state).name}' for port, state in verdicts.items()))
    if any(state is EnableState.PARTIAL for state in states):
        print(f'RESULT: PARTIAL enable detected (expected {options.expect})')
        return EXIT_PARTIAL
    if any(state is EnableState.UNKNOWN for state in states):
        print(f'RESULT: enable state incomplete (expected {options.expect})')
        return EXIT_INCOMPLETE
    if all(state in expected for state in states):
        print(f'RESULT: all arms {options.expect}: OK')
        return EXIT_OK
    print(f'RESULT: state does not match expectation {options.expect}')
    return EXIT_PARTIAL


if __name__ == '__main__':
    raise SystemExit(main())
