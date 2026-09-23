"""Tests for the Pi05 CAN mapping utility."""

import json
from pathlib import Path

import pytest

from piper.pi05_can import PIPER_BITRATE, ROLES, load_config


def _mapping():
    return {
        role: {
            'interface': interface,
            'usb_port': usb_port,
            'serial': f'SERIAL_{role.upper()}',
            'bitrate': PIPER_BITRATE,
        }
        for role, interface, usb_port in zip(
            ROLES,
            ('can_ml', 'can_mr', 'can_fl', 'can_fr'),
            ('1-11:1.0', '1-4:1.0', '1-13:1.0', '1-2:1.0'),
        )
    }


def _write(tmp_path: Path, data) -> Path:
    path = tmp_path / 'mapping.json'
    path.write_text(json.dumps(data), encoding='utf-8')
    return path


def test_loads_complete_mapping(tmp_path):
    result = load_config(_write(tmp_path, _mapping()))
    assert tuple(result) == ROLES
    assert result['follower_right'].interface == 'can_fr'
    assert result['follower_right'].usb_port == '1-2:1.0'


@pytest.mark.parametrize('missing', ROLES)
def test_rejects_missing_role(tmp_path, missing):
    data = _mapping()
    del data[missing]
    with pytest.raises(ValueError, match='exactly the four'):
        load_config(_write(tmp_path, data))


def test_rejects_duplicate_interface_and_serial(tmp_path):
    data = _mapping()
    data['follower_right']['interface'] = 'can_fl'
    with pytest.raises(ValueError, match='duplicate interface'):
        load_config(_write(tmp_path, data))

    data = _mapping()
    data['follower_right']['serial'] = data['follower_left']['serial']
    with pytest.raises(ValueError, match='duplicate USB serial'):
        load_config(_write(tmp_path, data))


def test_rejects_non_piper_bitrate(tmp_path):
    data = _mapping()
    data['follower_right']['bitrate'] = 500000
    with pytest.raises(ValueError, match='bitrate must be'):
        load_config(_write(tmp_path, data))
