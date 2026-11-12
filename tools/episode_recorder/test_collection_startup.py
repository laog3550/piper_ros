"""验证干净终端启动入口和隔离 ROS 域中的录制服务闭环。"""
import json
import os
from pathlib import Path
import socket
import subprocess
import time

import pytest

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'tools/episode_recorder/start_collection.sh'


def clean_environment():
    env = os.environ.copy()
    for key in ('AMENT_TRACE_SETUP_FILES', 'COLCON_TRACE', 'AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH'):
        env.pop(key, None)
    env['PYTHONDONTWRITEBYTECODE'] = '1'
    return env


def test_clean_terminal_reaches_argument_validation():
    result = subprocess.run(['bash', str(SCRIPT), 'invalid-mode'], env=clean_environment(),
                            capture_output=True, text=True, timeout=15)
    assert result.returncode == 2
    assert '用法' in result.stderr and 'unbound variable' not in result.stderr


def test_recorder_launcher_serves_status_and_stops_cleanly(tmp_path):
    if not Path('/opt/ros/humble/setup.bash').exists():
        pytest.skip('需要本机 ROS Humble')
    config = tmp_path / 'config.json'
    output = tmp_path / 'episodes'
    config.write_text(json.dumps({'output_dir': str(output),
                                 'topics': ['/acceptance/unused'], 'min_free_gb': 0}))
    env = clean_environment()
    env['ROS_DOMAIN_ID'] = '232'
    with (tmp_path / 'service.log').open('w+') as log:
        process = subprocess.Popen(['bash', str(SCRIPT), 'recorder', str(config)],
                                   env=env, stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    log.seek(0)
                    pytest.fail(log.read())
                try:
                    with socket.socket(socket.AF_UNIX) as client:
                        client.settimeout(2)
                        client.connect(str(output / 'recorder.sock'))
                        client.sendall(b'{"action":"status"}')
                        chunks = []
                        while chunk := client.recv(1000000):
                            chunks.append(chunk)
                        response = json.loads(b''.join(chunks))
                    assert response['ok'] and response['result']['state'] == 'idle'
                    break
                except (OSError, json.JSONDecodeError):
                    time.sleep(.1)
            else:
                pytest.fail('录制服务启动超时')
        finally:
            process.terminate()
            process.wait(timeout=10)
        assert process.returncode == 0
        assert not (output / 'recorder.sock').exists()
