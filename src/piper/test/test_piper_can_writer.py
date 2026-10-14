"""Verify cooperative writer ownership without opening CAN interfaces."""

import subprocess
import sys

import pytest

from piper.piper_can_writer import CanWriterLease


def test_lease_blocks_other_process_and_releases(tmp_path, monkeypatch):
    monkeypatch.setattr('tempfile.gettempdir', lambda: str(tmp_path))
    lease = CanWriterLease('can_test')
    lease.acquire()
    script = '''
import fcntl
import sys
with open(sys.argv[1], 'r+') as stream:
    try:
        fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        sys.exit(9)
'''
    try:
        result = subprocess.run([sys.executable, '-c', script, lease.path],
                                timeout=5, check=False)
        assert result.returncode == 9
        with pytest.raises(OSError, match='占用'):
            CanWriterLease('can_test').acquire()
        other = CanWriterLease('can_other')
        other.acquire()
        other.release()
    finally:
        lease.release()
    result = subprocess.run([sys.executable, '-c', script, lease.path],
                            timeout=5, check=False)
    assert result.returncode == 0
