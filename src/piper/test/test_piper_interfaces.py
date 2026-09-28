"""接口注册表和遥操作入口的架构约束。"""

from pathlib import Path

import pytest

from piper.piper_interfaces import SIDES, arm_interface, interface_matrix
from piper.piper_teleop_cli import build_parser, validate_options
from piper import piper_feedback
from piper import piper_feedback_decode, piper_filters, piper_motion


def test_interface_registry_has_one_complete_entry_per_side():
    interfaces = interface_matrix()
    assert tuple(item.side for item in interfaces) == SIDES
    assert len({item.command_topic for item in interfaces}) == len(SIDES)
    assert len({item.enable_service for item in interfaces}) == len(SIDES)
    for item in interfaces:
        assert item.command_topic.startswith('/joint_ctrl_cmd_')
        assert item.feedback_topic.startswith('/joint_states_')
        assert item.default_master_topic.endswith(item.side)


def test_unknown_side_is_rejected():
    with pytest.raises(ValueError, match='unsupported side'):
        arm_interface('middle')


def test_cli_lifecycle_flags_require_one_session_owner():
    parser = build_parser()
    options = parser.parse_args(['--disable-on-exit'])
    assert '--manage-enable' in validate_options(options)

    options = parser.parse_args(['--manage-enable', '--disable-on-exit'])
    assert validate_options(options) is None


def test_fast_launch_does_not_spawn_detached_service_processes():
    launch_file = (Path(__file__).parents[1] / 'launch' /
                   'start_fast_two_teleop.launch.py')
    text = launch_file.read_text(encoding='utf-8')
    assert 'ExecuteProcess' not in text
    assert "'--manage-enable'" in text
    assert "'--disable-on-exit'" in text


def test_feedback_facade_preserves_the_split_public_api():
    split_exports = set(piper_feedback_decode.__all__)
    split_exports.update(piper_motion.__all__)
    split_exports.update(piper_filters.__all__)
    assert set(piper_feedback.__all__) == split_exports


def test_feedback_layers_have_disjoint_public_ownership():
    groups = [
        set(piper_feedback_decode.__all__),
        set(piper_motion.__all__),
        set(piper_filters.__all__),
    ]
    assert not (groups[0] & groups[1])
    assert not (groups[0] & groups[2])
    assert not (groups[1] & groups[2])
