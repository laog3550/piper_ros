"""Tests for the enable/disable tool: dry run must not touch any bus."""

import pytest

from piper import piper_arm_enable
from piper.piper_arm_enable import (
    DISABLE_FLAG,
    ENABLE_DISABLE_CAN_ID,
    ENABLE_FLAG,
    EXIT_FAILED,
    EXIT_OK,
    EXIT_UNCONFIRMED,
    MOTOR_NUM_ALL,
    build_payload,
)
from piper.piper_enable_status import EnableState


def test_payload_addresses_all_motors_and_carries_the_flag():
    assert build_payload(False) == bytes([MOTOR_NUM_ALL, DISABLE_FLAG] + [0] * 6)
    assert build_payload(True) == bytes([MOTOR_NUM_ALL, ENABLE_FLAG] + [0] * 6)
    assert MOTOR_NUM_ALL == 7  # 7 = every motor on the arm
    assert ENABLE_DISABLE_CAN_ID == 0x471


def test_dry_run_sends_nothing_at_all(monkeypatch, capsys):
    def explode(*args, **kwargs):
        raise AssertionError('干跑不允许打开任何套接字')

    monkeypatch.setattr(piper_arm_enable.can, 'Bus', explode)
    exit_code = piper_arm_enable.main([])
    out = capsys.readouterr().out
    assert exit_code == EXIT_OK
    assert '干跑' in out
    assert '07 01 00 00 00 00 00 00' in out
    assert '没有发送任何帧' in out
    # 广播帧的影响范围要说清楚
    assert '0x471 是广播，无臂地址' in out


def test_dry_run_defaults_to_disable_and_warns_about_sagging(capsys):
    piper_arm_enable.main([])
    out = capsys.readouterr().out
    assert '失能' in out
    assert '下坠' in out


def test_dry_run_can_plan_an_enable(capsys):
    piper_arm_enable.main(['--enable'])
    out = capsys.readouterr().out
    assert '07 02 00 00 00 00 00 00' in out
    assert '使能' in out


def test_dry_run_lists_the_requested_ports(capsys):
    piper_arm_enable.main(['--port', 'can_left'])
    out = capsys.readouterr().out
    assert 'can_left' in out
    assert 'can_right' not in out


def test_send_transmits_once_per_bus_and_verifies(monkeypatch, capsys):
    sent = []
    monkeypatch.setattr(piper_arm_enable, 'send_frame',
                        lambda port, payload: sent.append((port, payload)))
    monkeypatch.setattr(piper_arm_enable, '_verify',
                        lambda port, seconds, timeout: EnableState.DISABLED)
    exit_code = piper_arm_enable.main(['--send', '--port', 'can_left'])
    out = capsys.readouterr().out
    assert exit_code == EXIT_OK
    assert sent == [('can_left', build_payload(False))]
    assert '失能成功' in out


def test_unconfirmed_read_back_is_reported(monkeypatch, capsys):
    monkeypatch.setattr(piper_arm_enable, 'send_frame',
                        lambda port, payload: None)
    monkeypatch.setattr(piper_arm_enable, '_verify',
                        lambda port, seconds, timeout: EnableState.PARTIAL)
    exit_code = piper_arm_enable.main(['--send', '--port', 'can_left'])
    out = capsys.readouterr().out
    assert exit_code == EXIT_UNCONFIRMED
    assert '没有读到 DISABLED' in out
    assert '重跑一次本命令即可' in out


def test_a_failed_send_is_reported(monkeypatch, capsys):
    def boom(port, payload):
        raise OSError('Network is down')

    monkeypatch.setattr(piper_arm_enable, 'send_frame', boom)
    exit_code = piper_arm_enable.main(['--send', '--port', 'can_left'])
    assert exit_code == EXIT_FAILED
    assert '发送失败' in capsys.readouterr().out


def test_rejects_a_negative_verify_window():
    assert piper_arm_enable.main(['--verify-seconds', '-1']) == EXIT_FAILED


def test_enable_and_disable_are_mutually_exclusive():
    with pytest.raises(SystemExit):
        piper_arm_enable.main(['--enable', '--disable'])
