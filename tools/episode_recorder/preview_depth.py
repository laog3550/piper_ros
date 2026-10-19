"""Resolve RGB-D preview topics and turn depth images into a fixed-scale heatmap."""

import cv2
import numpy as np


CAMERA_MARKERS = ("camera_wrist_left", "camera_wrist_right", "camera_top")
DEPTH_MAX_METERS = 1.0


def camera_topics(config):
    pairs = []
    for marker in CAMERA_MARKERS:
        topics = [topic for topic in config["topics"] if f"/{marker}/" in topic]
        color = [topic for topic in topics if topic.endswith("/color/image_raw")]
        depth = [topic for topic in topics if topic.endswith("/depth/image_raw")
                 or topic.endswith("/aligned_depth_to_color/image_raw")]
        if len(color) != 1 or len(depth) != 1:
            raise ValueError(f"{marker} 必须恰有一路彩色和一路深度图话题")
        pairs.append((color[0], depth[0]))
    return pairs


def depth_to_bgr(message, max_dimension=None):
    width, height, step = message.width, message.height, message.step
    if message.encoding not in ("16UC1", "32FC1"):
        raise ValueError(f"不支持的深度图编码: {message.encoding}")
    bytes_per_pixel = 2 if message.encoding == "16UC1" else 4
    if (not width or not height or step < width * bytes_per_pixel
            or step % bytes_per_pixel or len(message.data) < step * height):
        raise ValueError("深度图尺寸或数据长度无效")
    dtype = (">" if message.is_bigendian else "<") + ("u2" if bytes_per_pixel == 2 else "f4")
    raw = np.frombuffer(message.data, dtype=dtype, count=step * height // bytes_per_pixel)
    depth = raw.reshape(height, step // bytes_per_pixel)[:, :width]
    meters = depth.astype(np.float32)
    if bytes_per_pixel == 2:
        meters *= 0.001
    if max_dimension and max(width, height) > max_dimension:
        scale = max_dimension / max(width, height)
        meters = cv2.resize(meters, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    valid = np.isfinite(meters) & (meters > 0)
    scaled = np.clip(np.where(valid, meters, 0), 0, DEPTH_MAX_METERS) * (254 / DEPTH_MAX_METERS)
    indices = np.where(valid, scaled.astype(np.uint8) + 1, 0).astype(np.uint8)
    colored = cv2.applyColorMap(indices, cv2.COLORMAP_TURBO)
    colored[~valid] = 0
    return colored
