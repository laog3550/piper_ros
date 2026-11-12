#!/usr/bin/env bash
# 固定采集启动方式：控制台使用系统 Python（NumPy 1.x + 系统 OpenCV），
# 录制器使用带 rosbags 的兼容虚拟环境；两者都忽略用户 site-packages。
set -eo pipefail
cd "$(dirname "$0")/../.."
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
MODE="${1:-recorder}"
CONFIG="${2:-/tmp/piper_episode_config.json}"
case "$MODE" in
  recorder)
    PYTHON="${PIPER_RECORDER_PYTHON:-/tmp/piper-atomic-venv/bin/python}"
    "$PYTHON" tools/episode_recorder/preflight.py --recorder
    exec "$PYTHON" tools/episode_recorder/episode_recorder.py serve --config "$CONFIG"
    ;;
  console)
    /usr/bin/python3 tools/episode_recorder/preflight.py --console
    exec /usr/bin/python3 tools/episode_recorder/desktop_console.py --config "$CONFIG"
    ;;
  *)
    echo "用法: $0 [recorder|console] [config]" >&2
    exit 2
    ;;
esac
