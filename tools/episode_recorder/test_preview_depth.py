#!/usr/bin/env python3
"""验证深度预览的散点抑制和范围显示。运行：python3 tools/episode_recorder/test_preview_depth.py。"""

import json
from pathlib import Path

import numpy as np
from sensor_msgs.msg import Image

from preview_depth import DEPTH_MAX_METERS, FAR_COLOR_BGR, camera_topics, depth_to_bgr, filter_depth, overlay_color_edges


def image(values):
    message = Image()
    message.height, message.width = values.shape
    message.step = message.width * 2
    message.encoding = "16UC1"
    message.data = values.astype("<u2").tobytes()
    return message


near = np.full((9, 9), 500, dtype=np.uint16)
near[4, 4] = 0
preview = depth_to_bgr(image(near))
assert np.array_equal(preview[4, 4], preview[4, 3])
assert np.array_equal(depth_to_bgr(image(np.full((9, 9), 1500, dtype=np.uint16)))[4, 4], FAR_COLOR_BGR)
assert not depth_to_bgr(image(np.zeros((9, 9), dtype=np.uint16)))[4, 4].any()
assert DEPTH_MAX_METERS == (0.5, 0.5, 1.0)
at_700_mm = image(np.full((9, 9), 700, dtype=np.uint16))
assert np.array_equal(depth_to_bgr(at_700_mm, max_meters=DEPTH_MAX_METERS[0])[4, 4], FAR_COLOR_BGR)
assert not np.array_equal(depth_to_bgr(at_700_mm, max_meters=DEPTH_MAX_METERS[2])[4, 4], FAR_COLOR_BGR)

surface = np.full((40, 40), 0.5, dtype=np.float32)
surface[20, 20] = 0.54
smoothed = filter_depth(surface)
assert 0.5 < smoothed[20, 20] < 0.54
surface[14:26, 14:26] = 0
assert filter_depth(surface)[20, 20] == 0
edge = np.full((40, 40), 0.3, dtype=np.float32)
edge[:, 20:] = 0.8
smoothed_edge = filter_depth(edge)
assert smoothed_edge[20, 19] < 0.35 and smoothed_edge[20, 20] > 0.75

color = Image()
color.width = color.height = 40
color.step, color.encoding = 120, "rgb8"
pixels = np.zeros((40, 40, 3), dtype=np.uint8)
pixels[10:30, 10:30] = 255
color.data = pixels.tobytes()
flat = depth_to_bgr(image(np.full((40, 40), 500, dtype=np.uint16)))
outlined = overlay_color_edges(flat.copy(), color, (40, 40))
assert np.any(np.all(outlined == (235, 235, 235), axis=2))
assert not np.array_equal(flat, outlined)
rgb_config = json.loads((Path(__file__).resolve().parents[2] / "config/episode_recorder.rgb_only.example.json").read_text())
assert len(rgb_config["frame_topics"]) == 3
assert all("/depth/" not in topic and "/aligned_depth_to_color/" not in topic
           for topic in rgb_config["topics"])
assert len(camera_topics(rgb_config, require_depth=False)) == 3
assert all(depth is None for _, depth in camera_topics(rgb_config, require_depth=False))
try:
    camera_topics(rgb_config)
except ValueError:
    pass
else:
    raise AssertionError("RGB-D 模式应拒绝缺少深度话题的配置")
print("腕部与顶部色标、双边滤波、小空洞填充和轮廓显示：通过")
