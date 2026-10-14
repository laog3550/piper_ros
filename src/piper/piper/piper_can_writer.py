"""Cooperative per-interface ownership for teleop and standalone alignment."""

import fcntl
import hashlib
import os
import tempfile


class CanWriterLease:
    """Keep the lock inode stable, including across CAN socket replacement."""

    def __init__(self, port):
        self.port = port
        digest = hashlib.sha256(port.encode()).hexdigest()
        self.path = os.path.join(
            tempfile.gettempdir(), f'piper-can-{digest}.lock')
        self._fd = None

    def acquire(self):
        """Refuse a second local writer before it can send anything."""
        fd = os.open(self.path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            os.close(fd)
            raise OSError(f'{self.port} 已被控制会话占用，禁止并行对齐/回位') from exc
        self._fd = fd

    def release(self):
        """Release ownership without unlinking the shared lock inode."""
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None
