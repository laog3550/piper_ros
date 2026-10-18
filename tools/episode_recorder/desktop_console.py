#!/usr/bin/env python3
"""本地三路相机与 Episode 控制台。运行：python3 tools/episode_recorder/desktop_console.py --config config.json。"""

import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import json
from pathlib import Path
import socket
import sys
import threading
import time

import cv2
import numpy as np
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QComboBox, QGridLayout, QHBoxLayout, QHeaderView, QLabel,
    QLineEdit, QMainWindow, QMessageBox, QPushButton, QTableWidget,
    QTableWidgetItem, QTextEdit, QVBoxLayout, QWidget, QSizePolicy,
)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image


CAMERA_NAMES = ("左腕", "右腕", "顶部")
CAMERA_MARKERS = ("camera_wrist_left", "camera_wrist_right", "camera_top")
STATE_NAMES = {"idle": "空闲", "starting": "启动中", "recording": "录制中",
               "stopped": "已停止，待保存", "failed": "失败，待丢弃",
               "interrupted": "意外中断，待丢弃"}


def camera_topics(config):
    available = [topic for topic in config["topics"] if topic.endswith("/color/image_raw")]
    topics = []
    for marker in CAMERA_MARKERS:
        matches = [topic for topic in available if f"/{marker}/" in topic]
        if len(matches) != 1:
            raise ValueError(f"{marker} 必须恰有一路 color/image_raw 话题")
        topics.append(matches[0])
    return topics


def recorder_request(socket_path, request):
    with socket.socket(socket.AF_UNIX) as client:
        client.settimeout(30)
        client.connect(str(socket_path))
        client.sendall(json.dumps(request, ensure_ascii=False).encode())
        data = client.recv(1_000_000)
    if not data:
        raise OSError("录制服务未返回数据")
    result = json.loads(data)
    if not result.get("ok"):
        raise ValueError(result.get("error", "录制操作失败"))
    return result["result"]


def to_pixmap(message):
    width, height, step = message.width, message.height, message.step
    encodings = {
        "rgb8": (3, QImage.Format_RGB888), "bgr8": (3, QImage.Format_BGR888),
        "mono8": (1, QImage.Format_Grayscale8), "rgba8": (4, QImage.Format_RGBA8888),
        "bgra8": (4, QImage.Format_ARGB32),
    }
    if message.encoding in encodings:
        channels, image_format = encodings[message.encoding]
        if not width or not height or step < width * channels or len(message.data) < step * height:
            raise ValueError("图像尺寸或数据长度无效")
        pixels = np.frombuffer(message.data, dtype=np.uint8, count=step * height)
        image = QImage(pixels.data, width, height, step, image_format)
        return QPixmap.fromImage(image)
    if message.encoding in ("yuyv", "yuv422_yuy2"):
        if not width or not height or step < width * 2 or len(message.data) < step * height:
            raise ValueError("图像尺寸或数据长度无效")
        pixels = np.frombuffer(message.data, dtype=np.uint8, count=step * height)
        yuyv = pixels.reshape(height, step)[:, :width * 2].reshape(height, width, 2)
        rgb = cv2.cvtColor(yuyv, cv2.COLOR_YUV2RGB_YUY2)
        return QPixmap.fromImage(QImage(rgb.data, width, height, width * 3, QImage.Format_RGB888))
    raise ValueError(f"不支持的图像编码: {message.encoding}")


class Cameras:
    def __init__(self, config):
        self.node = rclpy.create_node("piper_collection_desktop_preview")
        self.topics = camera_topics(config)
        self.lock = threading.Lock()
        self.latest = [None] * 3
        self.sequence = [0] * 3
        self.received_at = [0.0] * 3
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        for index, topic in enumerate(self.topics):
            self.node.create_subscription(Image, topic,
                                          lambda msg, i=index: self.on_image(i, msg), qos)

    def on_image(self, index, message):
        with self.lock:
            self.latest[index] = message
            self.sequence[index] += 1
            self.received_at[index] = time.monotonic()

    def snapshot(self):
        with self.lock:
            return list(zip(self.latest, self.sequence, self.received_at))


class Console(QMainWindow):
    def __init__(self, config, cameras):
        super().__init__()
        self.cameras = cameras
        root = Path(config["output_dir"]).expanduser().resolve()
        self.socket_path = Path(config.get("socket_path", str(root / "recorder.sock"))).expanduser().resolve()
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.pending = {}
        self.state = "unknown"
        self.last_sequence = [0] * 3
        self.rendered = [0] * 3
        self.display_fps = [0] * 3
        self.fps_at = time.monotonic()
        self.setWindowTitle("Piper 数据采集控制台")
        self.resize(1500, 890)

        container = QWidget()
        self.setCentralWidget(container)
        layout = QVBoxLayout(container)
        self.state_label = QLabel("正在连接录制器…")
        layout.addWidget(self.state_label)
        camera_grid = QGridLayout()
        layout.addLayout(camera_grid, 3)
        self.images, self.camera_labels = [], []
        for index, name in enumerate(CAMERA_NAMES):
            title = QLabel(name)
            title.setStyleSheet("font-weight: bold; font-size: 16px")
            image = QLabel("等待图像…")
            image.setAlignment(Qt.AlignCenter)
            image.setMinimumWidth(300)
            image.setFixedHeight(300)
            image.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
            image.setStyleSheet("background: #10151d; color: white")
            info = QLabel(self.cameras.topics[index])
            info.setWordWrap(True)
            camera_grid.addWidget(title, 0, index)
            camera_grid.addWidget(image, 1, index)
            camera_grid.addWidget(info, 2, index)
            camera_grid.setColumnStretch(index, 1)
            self.images.append(image)
            self.camera_labels.append(info)

        fields = QGridLayout()
        layout.addLayout(fields)
        fields.addWidget(QLabel("任务名"), 0, 0)
        self.task = QLineEdit()
        self.task.setMaxLength(120)
        fields.addWidget(self.task, 0, 1)
        fields.addWidget(QLabel("语言指令"), 1, 0)
        self.instruction = QTextEdit()
        self.instruction.setMaximumHeight(74)
        fields.addWidget(self.instruction, 1, 1)
        fields.setColumnStretch(1, 1)

        controls = QHBoxLayout()
        layout.addLayout(controls)
        self.start = QPushButton("开始录制")
        self.stop = QPushButton("停止录制")
        self.outcome = QComboBox()
        self.outcome.addItems(("选择结果", "成功", "失败"))
        self.save = QPushButton("保存 Episode")
        self.discard = QPushButton("丢弃 Episode")
        for widget in (self.start, self.stop, self.outcome, self.save, self.discard):
            controls.addWidget(widget)
        self.message = QLabel("关闭本窗口不会中断录制；Rerun 可独立查看详细数据。")
        layout.addWidget(self.message)
        self.table = QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels(("编号", "任务", "语言指令", "结果", "开始时间", "路径"))
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        layout.addWidget(self.table, 2)

        self.start.clicked.connect(self.start_recording)
        self.stop.clicked.connect(lambda: self.action("stop"))
        self.save.clicked.connect(self.save_recording)
        self.discard.clicked.connect(self.discard_recording)
        self.outcome.currentIndexChanged.connect(self.update_buttons)
        self.update_buttons()

        self.frames_timer = QTimer(self)
        self.frames_timer.timeout.connect(self.draw_frames)
        self.frames_timer.start(33)
        self.work_timer = QTimer(self)
        self.work_timer.timeout.connect(self.check_futures)
        self.work_timer.start(100)
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.refresh)
        self.status_timer.start(1000)
        self.list_timer = QTimer(self)
        self.list_timer.timeout.connect(self.refresh_list)
        self.list_timer.start(5000)
        QTimer.singleShot(0, self.refresh)
        QTimer.singleShot(0, self.refresh_list)

    def submit(self, key, request):
        if key not in self.pending:
            self.pending[key] = self.pool.submit(recorder_request, self.socket_path, request)

    def refresh(self):
        if "action" not in self.pending:
            self.submit("status", {"action": "status"})

    def refresh_list(self):
        if "action" not in self.pending:
            self.submit("list", {"action": "list"})

    def action(self, name, **fields):
        if "action" in self.pending:
            return
        self.message.setText("正在执行…")
        self.submit("action", {"action": name, **fields})
        self.update_buttons()

    def start_recording(self):
        task, instruction = self.task.text().strip(), self.instruction.toPlainText().strip()
        if not task or not instruction or len(instruction) > 2000:
            self.message.setText("请填写任务名和语言指令（指令最多 2000 字）")
            return
        self.action("start", mode="task", task=task, skill=None,
                    metadata={"language_instruction": instruction})

    def save_recording(self):
        outcome = (None, "success", "failure")[self.outcome.currentIndex()]
        if outcome:
            self.action("save", outcome=outcome)

    def discard_recording(self):
        if QMessageBox.question(self, "确认丢弃", "确定丢弃当前 Episode？录制数据将被删除。") == QMessageBox.Yes:
            self.action("discard")

    def update_buttons(self):
        busy = "action" in self.pending
        self.start.setEnabled(not busy and self.state == "idle")
        self.stop.setEnabled(not busy and self.state == "recording")
        self.save.setEnabled(not busy and self.state == "stopped" and self.outcome.currentIndex() > 0)
        self.discard.setEnabled(not busy and self.state in ("recording", "stopped", "failed", "interrupted"))

    def check_futures(self):
        for key, future in list(self.pending.items()):
            if not future.done():
                continue
            del self.pending[key]
            try:
                result = future.result()
                if key == "status":
                    self.state = result["state"]
                    meta = result.get("metadata") or {}
                    detail = f" · {meta.get('task', '')} · {meta.get('error', '')}" if meta else ""
                    self.state_label.setText(f"录制状态：{STATE_NAMES.get(self.state, self.state)}{detail}")
                elif key == "list":
                    self.show_episodes(result["episodes"])
                else:
                    self.message.setText("操作完成")
                    if result.get("state") == "saved":
                        self.task.clear(); self.instruction.clear(); self.outcome.setCurrentIndex(0)
                    self.refresh(); self.refresh_list()
            except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
                if key == "status":
                    self.state = "unknown"
                    self.state_label.setText(f"录制服务未连接：{exc}")
                else:
                    self.message.setText(f"操作失败：{exc}")
            self.update_buttons()

    def show_episodes(self, episodes):
        self.table.setRowCount(len(episodes))
        for row, episode in enumerate(episodes):
            started = episode.get("started_at_unix_ns")
            when = datetime.fromtimestamp(started / 1e9).strftime("%Y-%m-%d %H:%M:%S") if started else ""
            outcome = {"success": "成功", "failure": "失败"}.get(episode.get("outcome"), "未标注")
            values = (episode.get("episode_number"), episode.get("task"),
                      episode.get("language_instruction"), outcome, when, episode.get("path"))
            for column, value in enumerate(values):
                self.table.setItem(row, column, QTableWidgetItem(str(value or "")))

    def draw_frames(self):
        now = time.monotonic()
        for index, (message, sequence, received_at) in enumerate(self.cameras.snapshot()):
            if sequence != self.last_sequence[index] and message is not None:
                self.last_sequence[index] = sequence
                try:
                    pixmap = to_pixmap(message)
                    size = self.images[index].size()
                    self.images[index].setPixmap(pixmap.scaled(size, Qt.KeepAspectRatio, Qt.FastTransformation))
                    self.rendered[index] += 1
                except (ValueError, cv2.error) as exc:
                    self.camera_labels[index].setText(f"{CAMERA_NAMES[index]}：{exc}")
            if now - received_at > 2 and received_at:
                self.camera_labels[index].setText(f"{CAMERA_NAMES[index]}：画面中断")
        if now - self.fps_at >= 1:
            interval = now - self.fps_at
            self.display_fps = [round(count / interval, 1) for count in self.rendered]
            for index, (_, _, received_at) in enumerate(self.cameras.snapshot()):
                if received_at and now - received_at <= 2:
                    self.camera_labels[index].setText(
                        f"{CAMERA_NAMES[index]}：预览 {self.display_fps[index]:.1f} fps · {self.cameras.topics[index]}")
            self.rendered = [0] * 3
            self.fps_at = now

    def closeEvent(self, event):
        self.frames_timer.stop(); self.status_timer.stop(); self.list_timer.stop(); self.work_timer.stop()
        self.pool.shutdown(wait=False, cancel_futures=True)
        super().closeEvent(event)


def main():
    parser = argparse.ArgumentParser(description="本地 Episode 数据采集控制台")
    parser.add_argument("--config", required=True, help="录制器 JSON 配置文件")
    args = parser.parse_args()
    with open(args.config, encoding="utf-8") as stream:
        config = json.load(stream)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    app = QApplication(sys.argv[:1])
    cameras = Cameras(config)

    def spin():
        try:
            rclpy.spin(cameras.node)
        except ExternalShutdownException:
            pass

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    window = Console(config, cameras)
    window.show()
    try:
        return app.exec_()
    finally:
        rclpy.shutdown()
        thread.join(timeout=2)
        cameras.node.destroy_node()


if __name__ == "__main__":
    sys.exit(main())
