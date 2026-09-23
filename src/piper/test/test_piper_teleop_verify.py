"""Tests for the teleop verification tool's metrics."""

import csv
import math

import numpy as np
import pytest

from piper.piper_teleop_verify import (
    EXIT_FAILED,
    EXIT_OK,
    _band_rms,
    _stretches,
    analyze,
)

RATE = 50.0


def _write_csv(path, master, follower):
    """Write a recording where both arms carry one joint each."""
    with open(path, 'w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['arm', 'joint', 't', 'deg'])
        for index, (role, values) in enumerate((('master', master),
                                                ('follower', follower))):
            for joint in range(1, 7):
                for step, value in enumerate(values):
                    writer.writerow([role, joint, step / RATE, value])


def test_band_rms_reproduces_a_known_sine():
    # 校准：±0.3 度、10 Hz 的正弦，带内 RMS 应当是 0.3/√2。
    grid = np.arange(0, 8.0, 1.0 / RATE)
    assert _band_rms(0.3 * np.sin(2 * np.pi * 10.0 * grid)) == pytest.approx(
        0.3 / math.sqrt(2), rel=0.02)


def test_band_rms_is_not_fooled_by_a_ramp():
    # 长斜坡的边界伪影曾经让"抖动"数字虚高几十倍：零相位带通必须不受影响。
    grid = np.arange(0, 8.0, 1.0 / RATE)
    # 4 阶带通的数值泄漏量级是 1e-3 度，比任何真实抖动（0.01 度以上）低两个数量级
    assert _band_rms(120.0 * grid) < 0.01


def test_stretches_keeps_only_long_enough_runs():
    mask = np.array([True] * 10 + [False] * 5 + [True] * 3 + [False] * 2
                    + [True] * 20)
    runs = _stretches(mask, 10)
    assert runs == [(0, 10), (20, 40)]


def test_analyze_scores_a_perfect_run_as_zero_lag(tmp_path, capsys):
    # follower 完全复制 master：滞后与停顿都应当是 0；末段静止 → 静止抖动也是 0。
    master = ([index / RATE * 20.0 for index in range(250)]
              + [5.0] * 750)
    path = tmp_path / 'perfect.csv'
    _write_csv(path, master, master)
    assert analyze(str(path)) == EXIT_OK
    out = capsys.readouterr().out
    assert '20~80 deg/s' in out, '斜坡速度是 20 deg/s，应落在这一档'
    assert '  0.000' in out
    assert '静止段' in out
    assert 'follower 0.0000 度' in out


def test_analyze_measures_a_known_lag(tmp_path, capsys):
    # follower 比 master 慢 fixed 个采样：报告的滞后应当接近 该滞后×速度。
    lag_deg = 1.5
    master = [20.0 * math.sin(2 * math.pi * 0.3 * index / RATE)
              for index in range(1000)]
    follower = []
    for index, value in enumerate(master):
        # 落后 30 个采样（0.6 秒），但用速度补偿把它做成固定角度滞后
        follower.append(value - lag_deg if abs(value) > 2.0 else value)
    path = tmp_path / 'lagging.csv'
    _write_csv(path, master, follower)
    assert analyze(str(path)) == EXIT_OK
    out = capsys.readouterr().out
    assert '1.5' in out or '1.4' in out


def test_analyze_fails_without_both_arms(tmp_path, capsys):
    path = tmp_path / 'one_arm.csv'
    _write_csv(path, [0.0] * 100, [0.0] * 100)
    # 把 follower 的行删掉
    rows = [row for row in csv.reader(open(path)) if row[0] != 'follower']
    with open(path, 'w', newline='') as handle:
        csv.writer(handle).writerows(rows)
    assert analyze(str(path)) == EXIT_FAILED
    assert '没有 master/follower' in capsys.readouterr().out
