#!/usr/bin/env python3
"""启动自检：确认 NumPy、OpenCV、ROS 与离线读取器运行在同一兼容环境。"""

import sys


def main():
    mode = "recorder" if "--recorder" in sys.argv else "console" if "--console" in sys.argv else "all"
    modules = ["numpy", "rclpy"]
    if mode in ("recorder", "all"):
        modules.append("rosbags")
    if mode in ("console", "all"):
        modules.extend(("cv2", "PyQt5"))
    imported = {}
    for name in modules:
        try:
            imported[name] = __import__(name)
        except Exception as error:  # noqa: BLE001 - 启动自检需要报告原始导入错误
            print(f"启动自检失败（{mode}）：{name}: {error}", file=sys.stderr)
            print("请使用 tools/episode_recorder/start_collection.sh，或设置兼容的 PYTHONNOUSERSITE/PYTHONPATH", file=sys.stderr)
            return 1
    versions = [f"numpy={imported['numpy'].__version__}"]
    if "cv2" in imported:
        versions.append(f"opencv={imported['cv2'].__version__}")
    if "rosbags" in imported:
        versions.append("rosbags=ok")
    print(f"自检通过（{mode}）：" + " ".join(versions))
    return 0


if __name__ == "__main__":
    sys.exit(main())
