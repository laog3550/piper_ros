"""Tests for the official native FA/FC CAN role and observer paths."""

from collections import Counter

import pytest

from piper.piper_feedback import CORE_FEEDBACK_CAN_IDS
import piper.piper_two_can_teleop as teleop


def native_window(seconds=3.0):
    counts = Counter({can_id: int(200 * seconds)
                      for can_id in CORE_FEEDBACK_CAN_IDS})
    counts.update({can_id: int(50 * seconds)
                   for can_id in teleop.NATIVE_CONTROL_IDS})
    return teleop.Capture(seconds, counts)


def isolated_window(seconds=2.0):
    return teleop.Capture(
        seconds,
        Counter({can_id: int(200 * seconds)
                 for can_id in CORE_FEEDBACK_CAN_IDS}),
    )


@pytest.mark.parametrize('role,expected', [
    ('master', 0xFA), ('follower', 0xFC),
])
def test_role_configuration_sends_one_official_frame(monkeypatch, role, expected):
    sent = []

    class FakeBus:
        def send(self, frame):
            sent.append(frame)

        def shutdown(self):
            pass

    class FakeLease:
        def __init__(self, port):
            pass

        def acquire(self):
            pass

        def release(self):
            pass

    monkeypatch.setattr(teleop, 'CanWriterLease', FakeLease)
    monkeypatch.setattr(teleop.can, 'Bus', lambda **kwargs: FakeBus())
    monkeypatch.setattr(teleop, 'capture', lambda bus, seconds: isolated_window(seconds))
    monkeypatch.setattr('builtins.input', lambda prompt: f'CONFIGURE-{role.upper()}')
    assert teleop.configure_role('can_left', role, 2.0) == teleop.EXIT_OK
    assert len(sent) == 1
    assert sent[0].arbitration_id == 0x470
    assert bytes(sent[0].data) == bytes((expected, 0, 0, 0, 0, 0, 0, 0))


def test_role_configuration_rechecks_after_confirmation(monkeypatch):
    sent = []
    windows = [isolated_window(), isolated_window()]
    windows[1].counts[0x155] = 1

    class FakeBus:
        def send(self, frame):
            sent.append(frame)

        def shutdown(self):
            pass

    class FakeLease:
        def __init__(self, port):
            pass

        def acquire(self):
            pass

        def release(self):
            pass

    monkeypatch.setattr(teleop, 'CanWriterLease', FakeLease)
    monkeypatch.setattr(teleop.can, 'Bus', lambda **kwargs: FakeBus())
    monkeypatch.setattr(teleop, 'capture', lambda bus, seconds: windows.pop(0))
    monkeypatch.setattr('builtins.input', lambda prompt: 'CONFIGURE-FOLLOWER')
    assert teleop.configure_role('can_left', 'follower', 2.0) == teleop.EXIT_REFUSED
    assert sent == []


def test_recover_shifted_master_requires_boot_signature_and_fresh_isolation(monkeypatch):
    sent = []
    observed_seconds = []
    startup = teleop.Capture(30.0, Counter(teleop.SHIFTED_STARTUP_MIN_COUNTS))
    startup.errors = 1  # A single transient during power-on is diagnostic.
    windows = [startup, teleop.Capture(3.0)]

    class FakeBus:
        def send(self, frame):
            sent.append(frame)

        def shutdown(self):
            pass

    class FakeLease:
        def __init__(self, port):
            pass

        def acquire(self):
            pass

        def release(self):
            pass

    def take_window(bus, seconds, *ready_counts):
        observed_seconds.append(seconds)
        if ready_counts:
            assert ready_counts == (teleop.SHIFTED_STARTUP_MIN_COUNTS,)
        return windows.pop(0)

    monkeypatch.setattr(teleop, 'CanWriterLease', FakeLease)
    monkeypatch.setattr(teleop.can, 'Bus', lambda **kwargs: FakeBus())
    monkeypatch.setattr(teleop, 'capture', take_window)
    monkeypatch.setattr('builtins.input', lambda prompt: 'CONFIGURE-MASTER')
    assert teleop.configure_role('can_right', 'master', 30.0, True) == teleop.EXIT_OK
    assert observed_seconds == [30.0, 3.0]
    assert len(sent) == 1
    assert sent[0].arbitration_id == 0x470
    assert bytes(sent[0].data) == bytes((0xFA, 0, 0, 0, 0, 0, 0, 0))


def test_recover_shifted_master_rejects_peer_or_missing_startup_frames():
    startup = teleop.Capture(30.0, Counter(teleop.SHIFTED_STARTUP_MIN_COUNTS))
    assert teleop.verify_shifted_master_startup(startup) == []
    startup.counts[0x2A1] = 1
    assert '从臂仍在线' in '；'.join(teleop.verify_shifted_master_startup(startup))
    startup.counts[0x2A1] = 0
    startup.counts[0x179] = 9
    assert '0x179' in '；'.join(teleop.verify_shifted_master_startup(startup))

    fresh = teleop.Capture(3.0, Counter({0x2A1: 1}))
    assert '默认地址反馈' in '；'.join(teleop.verify_recovery_bus(fresh))
    expected = teleop.Capture(
        3.0, Counter({0x171: 3, 0x175: 3, 0x471: 2}),
        enable_payloads=Counter({bytes((7, 2, 0, 0, 0, 0, 0, 0)): 2}))
    assert teleop.verify_recovery_bus(expected) == []
    expected.enable_payloads[bytes((7, 1, 0, 0, 0, 0, 0, 0))] = 1
    assert '非预期 0x471' in '；'.join(teleop.verify_recovery_bus(expected))
    assert teleop.main(['--configure-role', 'master', '--send', '--isolated-arm',
                        '--recover-shifted-master']) == teleop.EXIT_REFUSED


def test_role_configuration_requires_explicit_isolation_and_send(monkeypatch):
    monkeypatch.setattr(teleop, 'configure_role', lambda *args: pytest.fail('sent'))
    assert teleop.main(['--configure-role', 'master']) == teleop.EXIT_REFUSED
    assert teleop.main(['--configure-role', 'master', '--send']) == teleop.EXIT_REFUSED
    assert teleop.main(['--send']) == teleop.EXIT_REFUSED


def test_native_signature_rejects_old_bridge_and_competing_control():
    window = native_window()
    assert teleop.verify_native(window) == []
    window.counts[0x2C1] = 100
    window.counts[0x171] = 100
    window.counts[0x150] = 1
    problems = '；'.join(teleop.verify_native(window))
    assert '偏移反馈' in problems
    assert '偏移控制帧' in problems
    assert '五类以外' in problems


def test_native_signature_rejects_missing_stream_and_bus_errors():
    window = native_window()
    window.counts[0x159] = 0
    window.errors = 1
    problems = '；'.join(teleop.verify_native(window))
    assert '0x159' in problems
    assert 'CAN 错误帧' in problems


def test_native_signature_accepts_one_frame_per_control_id():
    window = native_window()
    for can_id in teleop.NATIVE_CONTROL_IDS:
        window.counts[can_id] = 1
    assert teleop.verify_native(window) == []


def test_native_signature_observes_enable_frames_without_attributing_sender():
    window = native_window()
    window.counts[0x471] = 30
    assert teleop.verify_native(window) == []


def test_capture_records_enable_payload_for_read_only_diagnosis(monkeypatch):
    class Frame:
        arbitration_id = 0x471
        data = bytes((7, 2, 0, 0, 0, 0, 0, 0))
        is_error_frame = False
        is_extended_id = False
        is_remote_frame = False

    class FakeBus:
        def __init__(self):
            self.frames = [Frame()]

        def recv(self, timeout):
            return self.frames.pop(0) if self.frames else None

    times = iter((0.0, 0.0, 2.0, 2.0))
    monkeypatch.setattr(teleop.time, 'monotonic', lambda: next(times))
    window = teleop.capture(FakeBus(), 2.0)
    assert window.enable_payloads[Frame.data] == 1


def test_recovery_capture_stops_after_shifted_signature(monkeypatch):
    class Frame:
        def __init__(self, can_id):
            self.arbitration_id = can_id
            self.data = bytes(8)
            self.is_error_frame = False
            self.is_extended_id = False
            self.is_remote_frame = False

    class FakeBus:
        def __init__(self):
            self.frames = [Frame(can_id)
                           for can_id, minimum in teleop.SHIFTED_STARTUP_MIN_COUNTS.items()
                           for _ in range(minimum)]

        def recv(self, timeout):
            return self.frames.pop(0) if self.frames else None

    tick = 0.0

    def monotonic():
        nonlocal tick
        tick += 0.1
        return tick

    monkeypatch.setattr(teleop.time, 'monotonic', monotonic)
    window = teleop.capture(FakeBus(), 120.0, teleop.SHIFTED_STARTUP_MIN_COUNTS)
    assert window.seconds < 10.0
    assert teleop.verify_shifted_master_startup(window) == []


def test_isolated_arm_rejects_existing_controller_and_double_feedback():
    window = isolated_window()
    assert teleop.verify_isolated_arm(window, 'follower') == []
    window.counts[0x155] = 1
    window.counts[0x251] *= 2
    problems = '；'.join(teleop.verify_isolated_arm(window, 'follower'))
    assert '控制或配置帧' in problems
    assert '高速反馈帧率' not in problems  # Other five IDs stay single-arm.
    for can_id in range(0x251, 0x257):
        window.counts[can_id] *= 2
    assert '高速反馈帧率' in '；'.join(teleop.verify_isolated_arm(window, 'follower'))


def test_old_shifted_master_can_be_reconfigured_when_isolated():
    window = isolated_window()
    for can_id in range(0x2A1, 0x2A9):
        window.counts[can_id + 0x20] = window.counts.pop(can_id)
    assert teleop.verify_isolated_arm(window, 'master') == []
    assert '偏移反馈' in '；'.join(teleop.verify_isolated_arm(window, 'follower'))


def test_monitor_is_read_only_even_when_interrupted(monkeypatch):
    sent = []

    class FakeBus:
        def send(self, frame):
            sent.append(frame)

        def shutdown(self):
            pass

    class FakeLease:
        def __init__(self, port):
            raise AssertionError('只读监视不能占用 CAN 写入锁')

    monkeypatch.setattr(teleop, 'CanWriterLease', FakeLease)
    monkeypatch.setattr(teleop.can, 'Bus', lambda **kwargs: FakeBus())

    def interrupted(bus, seconds):
        raise KeyboardInterrupt

    monkeypatch.setattr(teleop, 'capture', interrupted)
    assert teleop.monitor('can_left', 3.0, None) == teleop.EXIT_OK
    assert sent == []


def test_monitor_waits_through_empty_startup_then_recognizes_native_frames(monkeypatch):
    class FakeBus:
        def shutdown(self):
            pass

    windows = [teleop.Capture(3.0), native_window()]
    monkeypatch.setattr(teleop.can, 'Bus', lambda **kwargs: FakeBus())
    monkeypatch.setattr(teleop, 'capture', lambda bus, seconds: windows.pop(0))
    times = iter((0.0, 0.0, 3.0, 6.0))
    monkeypatch.setattr(teleop.time, 'monotonic', lambda: next(times))
    assert teleop.monitor('can_left', 3.0, 6.0) == teleop.EXIT_OK
    assert windows == []


def test_monitor_describes_quiet_master_after_confirmed_link(monkeypatch, capsys):
    class FakeBus:
        def shutdown(self):
            pass

    quiet = isolated_window(3.0)
    windows = [native_window(), quiet]
    monkeypatch.setattr(teleop.can, 'Bus', lambda **kwargs: FakeBus())
    monkeypatch.setattr(teleop, 'capture', lambda bus, seconds: windows.pop(0))
    times = iter((0.0, 0.0, 3.0, 6.0))
    monkeypatch.setattr(teleop.time, 'monotonic', lambda: next(times))
    assert teleop.monitor('can_right', 3.0, 6.0) == teleop.EXIT_OK
    assert '主臂静止与输出中断无法仅靠这一窗口区分' in capsys.readouterr().out
