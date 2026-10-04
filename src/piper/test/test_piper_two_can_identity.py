"""Tests for the two-CAN master/follower identity safety gate."""

from collections import Counter

import pytest

from piper.piper_two_can_identity import (
    Capture,
    gated_error_count,
    peak_error_rate,
    CONFIG_CAN_ID,
    ENABLE_CAN_ID,
    FOLLOWER_FEEDBACK_IDS,
    LINKAGE_TEACHING_MODE,
    MASTER_FEEDBACK_IDS,
    IdentityStatus,
    evaluate_identity,
    print_result,
    socket_filters,
)


def _healthy_counts(duration=2.0, rate=200.0):
    count = int(duration * rate)
    return Counter({
        can_id: count
        for can_id in (*FOLLOWER_FEEDBACK_IDS, *MASTER_FEEDBACK_IDS)
    })


def test_complete_200_hz_windows_confirm_both_address_identities():
    result = evaluate_identity(
        _healthy_counts(), 2.0, side='left', port='can_left')

    assert result.confirmed
    assert result.status is IdentityStatus.CONFIRMED
    assert result.follower.complete
    assert result.master.complete
    assert result.follower.minimum_hz == pytest.approx(200.0)
    assert result.master.maximum_hz == pytest.approx(200.0)
    assert '0x2A1～0x2A8' in result.reasons[0]
    assert '0x2C1～0x2C8' in result.reasons[0]


@pytest.mark.parametrize('missing_id', [0x2A1, 0x2A8, 0x2C1, 0x2C8])
def test_any_missing_feedback_id_refuses_confirmation(missing_id):
    counts = _healthy_counts()
    del counts[missing_id]

    result = evaluate_identity(counts, 2.0)

    assert not result.confirmed
    assert result.status is IdentityStatus.FEEDBACK_INCOMPLETE
    window = (result.follower if missing_id in FOLLOWER_FEEDBACK_IDS
              else result.master)
    assert missing_id in window.missing_or_slow


def test_rate_below_minimum_refuses_confirmation():
    counts = _healthy_counts()
    counts[0x2C5] = 298  # 149 Hz over two seconds

    result = evaluate_identity(counts, 2.0)

    assert result.status is IdentityStatus.FEEDBACK_INCOMPLETE
    assert result.master.missing_or_slow == (0x2C5,)


@pytest.mark.parametrize('collision_id', [0x2A5, 0x2C5])
def test_approximately_400_hz_is_an_address_conflict(collision_id):
    counts = _healthy_counts()
    counts[collision_id] = 800

    result = evaluate_identity(counts, 2.0)

    assert result.status is IdentityStatus.ADDRESS_CONFLICT
    window = (result.follower if collision_id in FOLLOWER_FEEDBACK_IDS
              else result.master)
    assert window.conflicts == (collision_id,)


@pytest.mark.parametrize('command_id', [0x150, 0x155, 0x15F, 0x170, 0x177,
                                        0x17F])
def test_standard_or_offset_control_flow_refuses_confirmation(command_id):
    counts = _healthy_counts()
    counts[command_id] = 1

    result = evaluate_identity(counts, 2.0)

    assert result.status is IdentityStatus.CONTROL_PRESENT
    assert result.command_ids == (command_id,)


def test_runtime_configuration_frame_has_fault_precedence():
    counts = _healthy_counts()
    counts[CONFIG_CAN_ID] = 1
    counts[0x155] = 1

    result = evaluate_identity(counts, 2.0)

    assert result.status is IdentityStatus.CONFIG_CHANGED
    assert result.config_seen


def test_enable_broadcast_means_the_bus_was_not_quiet():
    counts = _healthy_counts()
    counts[ENABLE_CAN_ID] = 1

    result = evaluate_identity(counts, 2.0)

    assert result.status is IdentityStatus.CONTROL_PRESENT
    assert result.enable_seen


def test_firmware_linkage_master_role_is_rejected():
    result = evaluate_identity(
        _healthy_counts(), 2.0,
        master_modes={LINKAGE_TEACHING_MODE: 400},
    )

    assert result.status is IdentityStatus.UNSAFE_MASTER_ROLE
    assert result.master_modes == ((LINKAGE_TEACHING_MODE, 400),)
    assert '0xFA' in result.reasons[0]


def test_error_frame_refuses_confirmation():
    result = evaluate_identity(_healthy_counts(), 2.0, error_frames=1)

    assert result.status is IdentityStatus.CAN_ERROR


def test_low_speed_shared_feedback_cannot_fill_an_identity_window():
    counts = Counter({can_id: 400 for can_id in range(0x251, 0x267)})

    result = evaluate_identity(counts, 2.0)

    assert result.status is IdentityStatus.FEEDBACK_INCOMPLETE
    assert result.follower.missing_or_slow == FOLLOWER_FEEDBACK_IDS
    assert result.master.missing_or_slow == MASTER_FEEDBACK_IDS


def test_socket_filters_cover_feedback_and_safety_observations():
    filtered = {item['can_id'] for item in socket_filters()}

    assert set(FOLLOWER_FEEDBACK_IDS) <= filtered
    assert set(MASTER_FEEDBACK_IDS) <= filtered
    assert {0x150, 0x15F, 0x170, 0x17F, 0x470, 0x471} <= filtered
    assert all(item['can_mask'] == 0x7FF for item in socket_filters())
    assert all(item['extended'] is False for item in socket_filters())


@pytest.mark.parametrize(
    'kwargs',
    [
        {'duration_s': 0},
        {'duration_s': 1, 'min_hz': 0},
        {'duration_s': 1, 'min_hz': 150, 'conflict_hz': 150},
        {'duration_s': 1, 'error_frames': -1},
    ],
)
def test_invalid_thresholds_are_rejected(kwargs):
    with pytest.raises(ValueError):
        evaluate_identity({}, **kwargs)


def test_operator_report_is_chinese_and_states_the_gate(capsys):
    result = evaluate_identity(
        _healthy_counts(), 2.0, side='right', port='can_right')

    print_result(result)
    output = capsys.readouterr().out

    assert 'right / can_right' in output
    assert '身份确认：CONFIRMED' in output
    assert '主从臂地址身份确认通过' in output


def _capture(error_times, elapsed=2.0, degradation=0):
    return Capture(
        counts={}, elapsed=elapsed, master_modes={},
        error_times=tuple(error_times), degradation_frames=degradation)


def test_capture_reports_average_and_peak_error_rates():
    measured = _capture([0.1, 0.2, 0.3, 1.5], elapsed=2.0)

    assert measured.error_frames == 4
    assert measured.average_error_rate_hz == pytest.approx(2.0)
    assert measured.peak_error_rate_hz == pytest.approx(3.0)


def test_peak_error_rate_ignores_the_gaps_between_bursts():
    assert peak_error_rate([]) == 0.0
    assert peak_error_rate([0.0, 0.5, 0.9], 1.0) == pytest.approx(3.0)
    assert peak_error_rate([0.0, 5.0, 5.9], 1.0) == pytest.approx(2.0)


def test_gated_error_count_is_strict_when_asked():
    measured = _capture([0.1])

    assert gated_error_count(
        measured, error_policy='strict', max_error_rate_hz=20.0) == 1


def test_gated_error_count_tolerates_isolated_errors_under_the_cap():
    measured = _capture([0.1, 0.5, 1.2, 1.9], elapsed=2.0)   # 2 Hz 平均

    assert gated_error_count(
        measured, error_policy='recoverable', max_error_rate_hz=20.0) == 0


def test_gated_error_count_refuses_a_sustained_rate():
    measured = _capture([index * 0.01 for index in range(40)], elapsed=1.0)

    assert gated_error_count(
        measured, error_policy='recoverable', max_error_rate_hz=20.0) == 40


def test_gated_error_count_always_refuses_state_degradation():
    measured = _capture([0.1], degradation=1)

    assert gated_error_count(
        measured, error_policy='recoverable', max_error_rate_hz=1000.0) == 1


def test_default_policy_and_limits_are_the_field_decision():
    from piper.piper_two_can_identity import (
        DEFAULT_ERROR_POLICY, DEFAULT_MAX_ERROR_RATE_HZ, ERROR_RATE_WINDOW_S)

    assert DEFAULT_ERROR_POLICY == 'recoverable'
    assert DEFAULT_MAX_ERROR_RATE_HZ == 20.0
    assert ERROR_RATE_WINDOW_S == 5.0
