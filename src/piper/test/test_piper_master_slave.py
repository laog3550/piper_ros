"""Tests for the isolated FC/0x20 maintenance command."""

from collections import Counter

import pytest

from piper import piper_master_slave as config
from piper.piper_feedback import HIGH_SPEED_CAN_IDS


def counts(offsets=(0,), driver_arms=1):
    result = Counter({i: 400 * driver_arms for i in HIGH_SPEED_CAN_IDS})
    for offset in offsets:
        result.update({i + offset: 400 for i in range(0x2A1, 0x2A9)})
    return result


@pytest.mark.parametrize('offset', [0, 0x10, 0x20])
def test_isolated_master_configuration(monkeypatch, offset):
    sent = []
    monkeypatch.setattr(config, 'survey',
                        lambda *_: (counts((offset,)), {}, {}))
    monkeypatch.setattr(config, 'send_frame', lambda *args: sent.append(args))
    assert config.main(['--port', 'can_left']) == config.EXIT_OK
    assert not sent
    assert config.main(['--port', 'can_left', '--send']) == config.EXIT_REFUSED
    assert not sent
    args = ['--port', 'can_left', '--send', '--isolated-master']
    assert config.main(args) == 0
    assert sent == [('can_left', bytes.fromhex('FC 20 20 00 00 00 00 00'))]


@pytest.mark.parametrize('feedback', [
    counts((0, 0x20), 2), counts((0,), 2), Counter()])
def test_two_sources_or_no_source_refused(monkeypatch, feedback):
    monkeypatch.setattr(config, 'survey', lambda *_: (feedback, {}, {}))
    monkeypatch.setattr(config, 'send_frame',
                        lambda *_: pytest.fail('unexpected TX'))
    args = ['--port', 'can_left', '--send', '--isolated-master']
    assert config.main(args) == 1


@pytest.mark.parametrize('args', [
    ['--teach-input'], ['--ctrl-offset', '0x00'],
    ['--feedback-offset', '0x00']])
def test_old_firmware_or_offset_reset_paths_removed(args):
    with pytest.raises(SystemExit):
        config.main(['--port', 'can_left', *args])
