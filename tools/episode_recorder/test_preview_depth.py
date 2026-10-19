#!/usr/bin/env python3
"""验证深度预览的散点抑制和范围显示。"""

import numpy as np
from sensor_msgs.msg import Image

from preview_depth import FAR_COLOR_BGR, depth_to_bgr, overlay_color_edges


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
print("深度散点抑制、超距、无效像素和物体轮廓显示：通过")
