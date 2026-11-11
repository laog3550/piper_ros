"""Process locks for read-only collection publishers on the same ROS domain."""

import fcntl
import hashlib
import os
from pathlib import Path
import stat
import tempfile


class CollectionLocks:
    def __init__(self, resources, directory=None):
        self._files = []
        if directory is None:
            domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
            directory = Path(tempfile.gettempdir()) / f'piper-readonly-{os.getuid()}' / str(domain)
        directory = Path(directory)
        directory.mkdir(mode=0o700, parents=True, exist_ok=True)
        info = directory.lstat()
        if not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
            raise RuntimeError(f'采集锁目录必须属于当前用户且权限为 0700：{directory}')
        try:
            for resource in sorted(set(resources)):
                name = hashlib.sha256(resource.encode()).hexdigest() + '.lock'
                fd = os.open(directory / name, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
                stream = os.fdopen(fd, 'r+')
                try:
                    fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    stream.close()
                    raise RuntimeError(f'采集实例已在运行，拒绝重复发布：{resource}') from None
                self._files.append(stream)
                stream.seek(0)
                stream.truncate()
                stream.write(f'pid={os.getpid()} resource={resource}\n')
                stream.flush()
        except BaseException:
            self.close()
            raise

    def close(self):
        for stream in self._files:
            stream.close()
        self._files.clear()

