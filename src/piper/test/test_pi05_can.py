"""Tests for the Pi05 CAN mapping utility."""

import json
from pathlib import Path

import pytest

from piper.pi05_can import BUSES, PIPER_BITRATE, load_config


def _mapping():
    return {
        bus: {
            'interface': interface,
            'usb_port': usb_port,
            'serial': f'SERIAL_{bus.upper()}_ADAPTER',
            'bitrate': PIPER_BITRATE,
            'arms': [f'master_{bus}', f'follower_{bus}'],
        }
        for bus, interface, usb_port in zip(
            BUSES,
            ('can_left', 'can_right'),
            ('1-13:1.0', '1-4:1.0'),
        )
    }


def _write(tmp_path: Path, data) -> Path:
    path = tmp_path / 'mapping.json'
    path.write_text(json.dumps(data), encoding='utf-8')
    return path


def test_loads_complete_mapping(tmp_path):
    result = load_config(_write(tmp_path, _mapping()))
    assert tuple(result) == BUSES
    assert result['right'].interface == 'can_right'
    assert result['right'].usb_port == '1-4:1.0'
    assert result['left'].arms == ('master_left', 'follower_left')
    assert result['right'].arms == ('master_right', 'follower_right')


@pytest.mark.parametrize('missing', BUSES)
def test_rejects_missing_bus(tmp_path, missing):
    data = _mapping()
    del data[missing]
    with pytest.raises(ValueError, match='exactly the two'):
        load_config(_write(tmp_path, data))


def test_rejects_duplicate_interface_and_serial(tmp_path):
    data = _mapping()
    data['right']['interface'] = 'can_left'
    with pytest.raises(ValueError, match='duplicate interface'):
        load_config(_write(tmp_path, data))

    data = _mapping()
    data['right']['serial'] = data['left']['serial']
    with pytest.raises(ValueError, match='duplicate USB serial'):
        load_config(_write(tmp_path, data))


def test_rejects_non_piper_bitrate(tmp_path):
    data = _mapping()
    data['right']['bitrate'] = 500000
    with pytest.raises(ValueError, match='bitrate must be'):
        load_config(_write(tmp_path, data))


def test_rejects_arm_pair_that_does_not_match_the_bus(tmp_path):
    data = _mapping()
    data['left']['arms'] = ['master_right', 'follower_right']
    with pytest.raises(ValueError, match='must carry'):
        load_config(_write(tmp_path, data))

    data = _mapping()
    data['left']['arms'] = ['master_left']
    with pytest.raises(ValueError, match='must carry'):
        load_config(_write(tmp_path, data))


def test_rejects_missing_arms(tmp_path):
    data = _mapping()
    del data['left']['arms']
    with pytest.raises(ValueError, match='invalid arms list'):
        load_config(_write(tmp_path, data))
