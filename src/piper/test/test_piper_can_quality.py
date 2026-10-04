"""Tests for the read-only CAN bus quality tool."""

import pytest

from piper.piper_can_quality import (
    SHARED_HIGH_SPEED_HZ,
    SHARED_LOW_SPEED_HZ,
    BusQuality,
    _enable_summary,
    _near_id_ranking,
    _shared_threshold,
    _verdict_rank,
    frame_bits,
    render,
    summary_line,
)


def _quality(**overrides) -> BusQuality:
    settings = dict(
        port='can_left',
        seconds=10.0,
        frames=61810,
        dlc_histogram={8: 61810},
        per_id_rate={},
        family_rates=(('测试族', 6181.0),),
        shared_ids=(),
        error_frames=0,
        error_classes=(),
        error_rate_hz=0.0,
        peak_error_rate_hz=0.0,
        degradation_frames=0,
        near_ids=(),
        nominal_load_kbit_s=330.0,
        worst_case_load_kbit_s=400.0,
        bitrate=1_000_000.0,
    )
    settings.update(overrides)
    return BusQuality(**settings)


def test_frame_bits_matches_the_classic_can_frame():
    # 8 字节标准帧：44 位固定开销 + 3 位帧间空间 + 64 位数据 = 111 位。
    assert frame_bits(8, worst_case_stuffing=False) == 111
    assert frame_bits(0, worst_case_stuffing=False) == 47
    # 最坏情况填充：每 4 位最多多 1 位，区域是 SOF..CRC（34 位）加数据。
    assert frame_bits(8) == 111 + (34 + 64 - 1) // 4
    assert frame_bits(8) > frame_bits(8, worst_case_stuffing=False)


def test_load_percent_is_the_worst_case_over_the_bitrate():
    quality = _quality(worst_case_load_kbit_s=834.0, bitrate=1_000_000.0)

    assert quality.load_percent == pytest.approx(83.4)


def test_verdict_follows_degradation_then_load_then_errors():
    assert _quality().verdict == 'CLEAN'
    assert _quality(error_frames=1).verdict == 'ERRORS'
    assert _quality(worst_case_load_kbit_s=900.0).verdict == 'OVERLOAD'
    assert _quality(error_frames=5, degradation_frames=1).verdict == 'DEGRADED'


def test_verdict_rank_orders_the_severities():
    assert _verdict_rank(_quality()) < _verdict_rank(
        _quality(error_frames=1))
    assert _verdict_rank(_quality(error_frames=1)) < _verdict_rank(
        _quality(worst_case_load_kbit_s=900.0))
    assert _verdict_rank(_quality(worst_case_load_kbit_s=900.0)) < (
        _verdict_rank(_quality(degradation_frames=1)))


def test_shared_id_thresholds_separate_the_two_families():
    assert _shared_threshold(0x251) == SHARED_HIGH_SPEED_HZ
    assert _shared_threshold(0x261) == SHARED_LOW_SPEED_HZ
    assert _shared_threshold(0x2A5) == SHARED_HIGH_SPEED_HZ
    assert SHARED_LOW_SPEED_HZ < SHARED_HIGH_SPEED_HZ


def test_near_id_ranking_flags_an_over_represented_id():
    from collections import Counter

    counts = Counter({0x251: 2400, 0x2A5: 200})
    near_before = Counter({0x251: 60})
    near_after = Counter({0x251: 40, 0x2A5: 1})

    ranked = _near_id_ranking(near_before, near_after, counts, 101, 2600)

    assert ranked
    assert ranked[0][0] == 0x251
    assert ranked[0][1] > 1.0
    assert ranked[0][2] == 100


def test_near_id_ranking_needs_enough_samples():
    from collections import Counter

    ranked = _near_id_ranking(Counter({0x251: 5}), Counter({0x251: 3}),
                              Counter({0x251: 2400}), 8, 2400)

    assert ranked == ()


def test_enable_summary_treats_a_mixed_joint_as_enabled():
    summary = _enable_summary({
        1: {False},
        2: {True},
        3: {True, False},
        4: set(),
    })

    assert summary == (False, True, True, False, False, False)


def test_render_and_summary_line_report_the_key_numbers():
    quality = _quality(
        worst_case_load_kbit_s=834.0,
        shared_ids=(0x251, 0x261),
        error_frames=586,
        error_rate_hz=9.77,
        peak_error_rate_hz=47.0,
        error_classes=(('控制器问题、协议违规', '00 00 04 00 00 00 00 01'),),
        near_ids=((0x251, 2.1, 120),),
        joint_enable_samples=(False,) * 6,
    )

    text = render(quality)
    line = summary_line(quality)

    assert '共享 ID' in text and '0x251' in text and '0x261' in text
    assert '错误帧近邻 ID' in text
    assert '控制器问题' in text
    assert '六关节使能（低速反馈）：全部 disabled' in text
    assert '负载 83%' in line
    assert '错误帧 586' in line
    assert '结论 OVERLOAD' in line
