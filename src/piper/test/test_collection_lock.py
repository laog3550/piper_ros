"""验证采集实例在跨进程、部分占用及进程退出时的互斥行为。"""

import os
from pathlib import Path
import subprocess
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from piper.collection_lock import CollectionLocks  # noqa: E402


def test_overlapping_side_and_all_instances_release_partial_locks(tmp_path):
    right = CollectionLocks(['/piper_vector/right'], tmp_path)
    try:
        with pytest.raises(RuntimeError, match='拒绝重复发布'):
            CollectionLocks(['/piper_vector/left', '/piper_vector/right'], tmp_path)
        left = CollectionLocks(['/piper_vector/left'], tmp_path)
        left.close()
    finally:
        right.close()
    both = CollectionLocks(['/piper_vector/left', '/piper_vector/right'], tmp_path)
    both.close()


def test_process_exit_releases_lock_without_deleting_lock_file(tmp_path):
    code = """
import sys
from piper.collection_lock import CollectionLocks
lock = CollectionLocks(['/piper_direct/left'], sys.argv[1])
print('ready', flush=True)
sys.stdin.read()
"""
    env = dict(os.environ, PYTHONPATH=str(Path(__file__).resolve().parents[1]))
    child = subprocess.Popen([sys.executable, '-c', code, str(tmp_path)], env=env,
                             stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    try:
        assert child.stdout.readline().strip() == 'ready'
        with pytest.raises(RuntimeError, match='拒绝重复发布'):
            CollectionLocks(['/piper_direct/left'], tmp_path)
    finally:
        child.terminate()
        child.communicate(timeout=5)
    assert list(tmp_path.glob('*.lock'))
    replacement = CollectionLocks(['/piper_direct/left'], tmp_path)
    replacement.close()
