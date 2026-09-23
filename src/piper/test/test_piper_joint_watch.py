"""Tests for the joint angle tracker and sampler of the watch tool."""

import time

import pytest

from piper.piper_feedback import ArmAngleTracker
from piper.piper_joint_watch import ArmSampler


def _frame(joint_a, joint_b, degrees):
    """Build a joint angle frame with both joints at the same angle."""
    raw = int(round(degrees * 1000))
    if raw < 0:
        raw += 1 << 32
    return (raw.to_bytes(4, 'big') + raw.to_bytes(4, 'big'))


def _update(tracker, can_id, degrees, now):
    """Feed both joints of one frame and return the joints reported moved."""
    return tracker.update(can_id, _frame(0, 0, degrees), now=now)


def test_baseline_is_the_first_reading_not_zero():
    tracker = ArmAngleTracker()
    _update(tracker, 0x2A5, 30.0, 0.0)
    # j1 and j2 start at 30 degrees; nothing has moved yet.
    assert tracker.angles()[1] == pytest.approx(30.0)
    assert tracker.offsets()[1] == pytest.approx(0.0)
    assert tracker.max_offsets()[1] == pytest.approx(0.0)


def test_moving_one_joint_pair_shows_up_as_displacement():
    tracker = ArmAngleTracker()
    _update(tracker, 0x2A5, 0.0, 0.0)
    _update(tracker, 0x2A5, 12.5, 0.1)
    assert tracker.offsets()[1] == pytest.approx(12.5)
    assert tracker.max_offsets()[1] == pytest.approx(12.5)
    assert tracker.moved_joints(5.0) == (1, 2)


def test_returning_to_the_start_keeps_the_recorded_peak():
    # A drag that comes back to where it began must still be visible, or the
    # operator could move an arm and see nothing.
    tracker = ArmAngleTracker()
    _update(tracker, 0x2A6, 0.0, 0.0)
    _update(tracker, 0x2A6, 20.0, 0.1)
    _update(tracker, 0x2A6, 0.0, 0.2)
    assert tracker.offsets()[3] == pytest.approx(0.0)
    assert tracker.max_offsets()[3] == pytest.approx(20.0)
    assert tracker.moved_joints(5.0) == (3, 4)


def test_untouched_arm_reports_no_displacement():
    tracker = ArmAngleTracker()
    for step in range(5):
        _update(tracker, 0x2A7, -7.0, step * 0.1)
    assert tracker.moved_joints(5.0) == ()
    assert all(peak == 0.0 for peak in tracker.max_offsets().values())


def test_negative_displacement_counts_too():
    tracker = ArmAngleTracker()
    _update(tracker, 0x2A7, 10.0, 0.0)
    _update(tracker, 0x2A7, -8.0, 0.1)
    assert tracker.offsets()[5] == pytest.approx(-18.0)
    assert tracker.max_offsets()[5] == pytest.approx(18.0)


def test_motion_detection_uses_a_time_window():
    tracker = ArmAngleTracker()
    _update(tracker, 0x2A5, 0.0, 0.0)
    _update(tracker, 0x2A5, 1.0, 1.0)
    assert tracker.is_moving(window=0.5, now=1.0) is True
    assert tracker.is_moving(window=0.5, now=1.4) is True
    assert tracker.is_moving(window=0.5, now=1.6) is False


def test_static_arm_is_not_reported_as_moving():
    tracker = ArmAngleTracker()
    for step in range(6):
        _update(tracker, 0x2A5, 4.0, step * 1.0)
    assert tracker.is_moving(window=0.5, now=5.0) is False


def test_no_frames_means_no_age_and_no_motion():
    tracker = ArmAngleTracker()
    assert tracker.age(now=1.0) is None
    assert tracker.is_moving(now=1.0) is False
    assert tracker.angles() == {}
    assert tracker.moved_joints(0.0) == ()


def test_set_baseline_clears_the_peaks():
    tracker = ArmAngleTracker()
    _update(tracker, 0x2A5, 0.0, 0.0)
    _update(tracker, 0x2A5, 40.0, 0.1)
    assert tracker.moved_joints(5.0) == (1, 2)
    tracker.set_baseline()
    assert tracker.moved_joints(5.0) == ()
    assert tracker.offsets()[1] == pytest.approx(0.0)


def test_angle_frames_of_other_ids_are_ignored():
    tracker = ArmAngleTracker()
    assert tracker.update(0x2A1, bytes(8), now=0.0) == ()
    assert tracker.angles() == {}


def test_two_trackers_do_not_share_state():
    # This is what makes per-interface mapping verifiable: moving one arm must
    # leave every other arm's tracker completely untouched.
    left = ArmAngleTracker()
    right = ArmAngleTracker()
    _update(left, 0x2A5, 0.0, 0.0)
    _update(right, 0x2A5, 0.0, 0.0)
    _update(left, 0x2A5, 25.0, 0.1)

    assert left.moved_joints(5.0) == (1, 2)
    assert right.moved_joints(5.0) == ()
    assert right.angles()[1] == pytest.approx(0.0)


class _FakeFrame:
    def __init__(self, arbitration_id, data):
        self.arbitration_id = arbitration_id
        self.data = data


class _FakeBus:
    """Replay a fixed list of frames, then keep returning nothing."""

    def __init__(self, frames):
        self.frames = list(frames)
        self.shutdown_called = False

    def recv(self, timeout=None):
        if self.frames:
            return self.frames.pop(0)
        return None

    def shutdown(self):
        self.shutdown_called = True


def _angle_frame(joint_a, joint_b, degrees):
    raw = int(round(degrees * 1000))
    if raw < 0:
        raw += 1 << 32
    return _FakeFrame(0x2A5, raw.to_bytes(4, 'big') + raw.to_bytes(4, 'big'))


def _run_sampler(frames, predicate, timeout=3.0):
    """Run one sampler over fake frames until predicate holds."""
    bus = _FakeBus(frames)
    sampler = ArmSampler('fake0', 'fake_role', bus_factory=lambda port: bus)
    sampler.start()
    deadline = time.monotonic() + timeout
    try:
        while time.monotonic() < deadline:
            if predicate(sampler):
                break
            time.sleep(0.01)
    finally:
        sampler.stop()
        sampler.join(timeout=1.0)
    return sampler, bus


def test_sampler_detects_a_drag_through_its_read_loop():
    # End to end over the sampler's own read loop: two frames, the second one
    # moved, must register as displacement on exactly that interface.
    frames = [_angle_frame(0, 0, 0.0), _angle_frame(0, 0, 15.0)]
    sampler, bus = _run_sampler(
        frames,
        lambda s: s.tracker.max_offsets().get(1, 0.0) >= 15.0,
    )
    assert sampler.error is None
    assert sampler.tracker.angles()[1] == pytest.approx(15.0)
    assert sampler.tracker.max_offsets()[1] == pytest.approx(15.0)
    assert sampler.tracker.moved_joints(5.0) == (1, 2)
    assert bus.shutdown_called is True


def test_sampler_ignores_frames_that_are_not_joint_angles():
    frames = [_FakeFrame(0x2A1, bytes(8)), _angle_frame(0, 0, 3.0)]
    sampler, _ = _run_sampler(
        frames, lambda s: 1 in s.tracker.angles())
    assert sampler.tracker.angles() == {1: pytest.approx(3.0), 2: pytest.approx(3.0)}
    assert sampler.tracker.moved_joints(5.0) == ()


def test_sampler_without_frames_reports_no_age():
    sampler, _ = _run_sampler([], lambda s: False, timeout=0.4)
    assert sampler.error is None
    assert sampler.snapshot(time.monotonic())[2] is None


def test_sampler_reports_a_bus_that_will_not_open():
    def boom(port):
        raise OSError('no such interface')

    sampler = ArmSampler('missing', 'role', bus_factory=boom)
    sampler.start()
    sampler.join(timeout=1.0)
    assert sampler.error == 'no such interface'
    assert sampler.snapshot(time.monotonic())[2] is None
