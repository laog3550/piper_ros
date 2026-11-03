"""核对只读从臂状态导出包含夹爪，且写成可回读的 LeRobot v3。"""

from importlib.metadata import version

import pytest

from export_follower_state_lerobot import export
from test_to_lerobot import vector_fixture


def test_follower_state_roundtrip_includes_gripper(tmp_path):
    if version("lerobot") != "0.4.4":
        pytest.skip("需要 lerobot==0.4.4")
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    episode = vector_fixture(tmp_path, "episode_000001")
    output = tmp_path / "follower_state"
    manifest = export(episode, output, "local/piper_follower_state_test",
                      10, 100, 30, 50)
    dataset = LeRobotDataset("local/piper_follower_state_test", root=output)

    assert manifest["contains_gripper"] is True
    assert manifest["gripper_calibration_verified"] is False
    assert manifest["training_eligible"] is False
    assert manifest["frames"] == len(dataset) == 2
    assert manifest["joint_names"][6] == "left.gripper"
    assert manifest["joint_names"][13] == "right.gripper"
    state = dataset[0]["observation.state"]
    assert tuple(state.shape) == (14,)
    assert state[6].item() == pytest.approx(0.6)
    assert state[13].item() == pytest.approx(2.6)
