"""验证实测闭合零点、录制版本绑定和导出防混用。"""
import copy
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from test_recorder_annotations import load_recorder_module
from test_temporal_alignment import phased_episode, read
import to_lerobot as converter

CALIBRATION = Path(__file__).resolve().parents[2] / 'config/gripper_collection_calibration.20261112.json'


@pytest.mark.parametrize('topic', [*converter.CAN_ACTIONS.values(), *converter.CAN_STATES.values()])
def test_measured_closed_reference_becomes_zero_only_in_conversion(topic):
    settings = converter.load_gripper_calibration(CALIBRATION)
    raw = settings['streams'][topic]['closed_reference']
    arm, kind = topic.split('/')[2:]
    stamp = SimpleNamespace(sec=1800000000, nanosec=0)
    values = [0.] * 6 + [raw]
    message = SimpleNamespace(joint_names=converter.JOINTS, position=values,
                              header=SimpleNamespace(stamp=stamp, frame_id='can_' + arm),
                              source_topic=f'/piper_direct/{arm}/{kind}',
                              source_can_ids=converter.CAN_IDS[kind], source_stamps=[stamp] * 4,
                              missing=[False] * 4, stale=[False] * 4, time_inconsistent=False,
                              valid=True, gripper_mapping_verified=True)
    corrected, _ = converter.can_vector(message, topic, 30_000_000, settings)
    assert corrected[6] == 0.
    assert message.position == [0.] * 6 + [raw]


def test_episode_rejects_other_zero_versions_and_requires_explicit_asset(tmp_path):
    module = load_recorder_module()
    evidence = module.gripper_calibration_evidence(CALIBRATION)
    path, meta = phased_episode(tmp_path)
    meta['task_metadata']['gripper_calibration_evidence'] = evidence
    (path / 'episode.json').write_text(json.dumps(meta))
    with pytest.raises(ValueError, match='已绑定夹爪零点'):
        read(path)
    settings = evidence['settings']
    kwargs = dict(input_format='can-vector', max_source_skew_ns=30_000_000,
                  extract_valid_segments=True)
    _, frames = converter.read_episode(path, ['left'], 10, 100_000_000, 2_000_000,
                                       gripper_calibration=settings, **kwargs)
    assert len(frames) >= 16
    wrong = copy.deepcopy(settings)
    wrong['streams'][converter.CAN_STATES['left']]['closed_reference'] += .0003
    with pytest.raises(ValueError, match='录制时绑定的零点不一致'):
        converter.read_episode(path, ['left'], 10, 100_000_000, 2_000_000,
                               gripper_calibration=wrong, **kwargs)
    assert meta['task_metadata']['gripper_calibration_evidence']['settings'] == settings
