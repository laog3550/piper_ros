"""Resolve RGB-D preview topics and turn depth images into a fixed-scale heatmap."""

import cv2
import numpy as np


CAMERA_MARKERS = ("camera_wrist_left", "camera_wrist_right", "camera_top")
DEPTH_MAX_METERS = (0.5, 0.5, 1.0)
FAR_COLOR_BGR = (52, 52, 52)
MAX_HOLE_PIXELS_AT_640 = 250


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


def filter_depth(meters):
    valid = np.isfinite(meters) & (meters > 0)
    if not valid.any():
        return np.zeros_like(meters, dtype=np.float32)
    working = np.where(valid, meters, 0).astype(np.float32)
    invalid = (~valid).astype(np.uint8)
    small_holes = np.zeros_like(invalid, dtype=bool)
    if invalid.any():
        count, labels, stats, _ = cv2.connectedComponentsWithStats(invalid, connectivity=8)
        max_area = max(16, round(MAX_HOLE_PIXELS_AT_640 * working.size / (640 * 480)))
        selected = np.zeros(count, dtype=bool)
        selected[1:] = stats[1:, cv2.CC_STAT_AREA] <= max_area
        small_holes = selected[labels]
        if small_holes.any():
            working = cv2.inpaint(working, small_holes.astype(np.uint8), 2, cv2.INPAINT_NS)
    smoothed = cv2.bilateralFilter(working, 5, 0.05, 2)
    smoothed[(~valid) & (~small_holes)] = 0
    return smoothed


def depth_to_bgr(message, max_dimension=None, max_meters=1.0):
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
    meters = filter_depth(meters)
    near = (meters > 0) & (meters <= max_meters)
    far = meters > max_meters
    scaled = np.clip(np.where(near, meters, 0), 0, max_meters) * (254 / max_meters)
    indices = np.where(near, scaled.astype(np.uint8) + 1, 0).astype(np.uint8)
    colored = cv2.applyColorMap(indices, cv2.COLORMAP_TURBO)
    colored[~near] = 0
    colored[far] = FAR_COLOR_BGR
    return colored


def overlay_color_edges(depth_bgr, color_message, depth_size):
    if color_message is None or (color_message.width, color_message.height) != depth_size:
        return depth_bgr
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4,
                "mono8": 1, "yuyv": 2, "yuv422_yuy2": 2}
    encoding = color_message.encoding
    count = channels.get(encoding)
    width, height, step = color_message.width, color_message.height, color_message.step
    if count is None or step < width * count or len(color_message.data) < step * height:
        return depth_bgr
    pixels = np.frombuffer(color_message.data, dtype=np.uint8, count=step * height)
    pixels = pixels.reshape(height, step)[:, :width * count].reshape(height, width, count)
    if encoding in ("mono8", "yuyv", "yuv422_yuy2"):
        gray = pixels[:, :, 0]
    else:
        conversion = {"rgb8": cv2.COLOR_RGB2GRAY, "bgr8": cv2.COLOR_BGR2GRAY,
                      "rgba8": cv2.COLOR_RGBA2GRAY, "bgra8": cv2.COLOR_BGRA2GRAY}[encoding]
        gray = cv2.cvtColor(pixels, conversion)
    if gray.shape != depth_bgr.shape[:2]:
        gray = cv2.resize(gray, (depth_bgr.shape[1], depth_bgr.shape[0]),
                          interpolation=cv2.INTER_AREA)
    edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 80, 160) > 0
    near = np.any(depth_bgr != FAR_COLOR_BGR, axis=2) & np.any(depth_bgr != 0, axis=2)
    depth_bgr[edges & near] = (235, 235, 235)
    return depth_bgr
