#!/usr/bin/env python3
"""本地三路相机与 Episode 控制台。运行：python3 tools/episode_recorder/desktop_console.py --config config.json。"""

import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import json
import math
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
    QApplication, QComboBox, QFileDialog, QGridLayout, QHBoxLayout, QHeaderView, QLabel,
    QLineEdit, QMainWindow, QMessageBox, QPushButton, QSpinBox, QTableWidget,
    QTableWidgetItem, QTextEdit, QVBoxLayout, QWidget, QSizePolicy,
)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
from std_msgs.msg import String
from piper_msgs.msg import PiperArmVector
from preview_depth import DEPTH_MAX_METERS, camera_topics, depth_to_bgr, overlay_color_edges
from skill_web_console import (SKILLS, ARM_TOPICS, start_request, annotation_request,
                               collection_contract_diagnostics, provisional_collection_versions)


CAMERA_NAMES = ("左腕", "右腕", "顶部")
SKILL_LABELS = ("抓起香蕉并放入盒子中（单臂）", "将盒子的盖子翻过来（双臂）", "扣上盒子的盖子（双臂）")
TASK_TEXT = {
    "task": ("香蕉入盒与盒盖操作", "先用单臂抓起香蕉并放入盒中，再双臂翻过盒盖并扣上盒盖"),
    SKILLS[0]: ("香蕉入盒", "用{arm}臂抓起香蕉并放入盒中"),
    SKILLS[1]: ("翻转盒盖", "用双臂将盒子的盖子翻过来"),
    SKILLS[2]: ("扣上盒盖", "用双臂扣上盒子的盖子"),
}
STATE_NAMES = {"idle": "空闲", "starting": "启动中", "recording": "录制中",
               "stopped": "已停止，待保存", "failed": "失败，待丢弃",
               "interrupted": "意外中断，待丢弃"}


def skill_completion_issue(skill, annotations, frame_topics):
    """Describe why a successful atomic episode lacks one verified completion frame."""
    completions = [item for item in annotations if item.get("kind") == "skill_complete"]
    if len(completions) != 1 or completions[0].get("value") != skill:
        return "成功保存前须有且仅有一条与当前技能匹配的完成帧标注"
    item = completions[0]
    topic, stamp = item.get("source_topic"), item.get("header_stamp_ns")
    if (topic not in frame_topics or not isinstance(topic, str)
            or not topic.endswith("/color/image_raw") or type(stamp) is not int or stamp <= 0):
        return "完成帧缺少已录制彩色相机的有效 Header 引用"
    if item.get("verified_in_bag") is not True or item.get("frame_verification_error"):
        return "完成帧尚未通过 rosbag 核验；请在录制后补标"
    return None


def recorder_request(socket_path, request):
    with socket.socket(socket.AF_UNIX) as client:
        client.settimeout(30)
        client.connect(str(socket_path))
        client.sendall(json.dumps(request, ensure_ascii=False).encode())
        chunks = []
        while chunk := client.recv(1_000_000):
            chunks.append(chunk)
        data = b"".join(chunks)
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


def render_depth(message, color_message, index):
    bgr = depth_to_bgr(message, max_dimension=400, max_meters=DEPTH_MAX_METERS[index])
    overlay_color_edges(bgr, color_message, (message.width, message.height))
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


class Cameras:
    def __init__(self, config):
        self.node = rclpy.create_node("piper_collection_desktop_preview")
        pairs = camera_topics(config, require_depth=False)
        self.topics = [color for color, _ in pairs]
        self.depth_topics = [depth for _, depth in pairs]
        self.lock = threading.Lock()
        self.latest = [[None] * 3 for _ in range(2)]
        self.sequence = [[0] * 3 for _ in range(2)]
        self.received_at = [[0.0] * 3 for _ in range(2)]
        self.joints = {arm: None for arm in ARM_TOPICS}
        self.event = None
        self.frame_topics = set(config.get("frame_topics", ()))
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        for kind, topics in enumerate((self.topics, self.depth_topics)):
            for index, topic in enumerate(topics):
                if topic is None:
                    continue
                self.node.create_subscription(Image, topic,
                                              lambda msg, k=kind, i=index: self.on_image(k, i, msg), qos)
        for arm, topics in ARM_TOPICS.items():
            self.node.create_subscription(PiperArmVector, topics["slave_feedback"],
                                          lambda msg, side=arm: self.on_joint(side, msg), 10)
        self.node.create_subscription(String, "/collection/events", self.on_event, 10)

    def on_image(self, kind, index, message):
        with self.lock:
            self.latest[kind][index] = message
            self.sequence[kind][index] += 1
            self.received_at[kind][index] = time.monotonic()

    def frame_snapshot(self):
        with self.lock:
            return [list(zip(self.latest[kind], self.sequence[kind], self.received_at[kind]))
                    for kind in range(2)]

    def snapshot(self):
        with self.lock:
            received = list(self.received_at[0])
        now = time.monotonic()
        rows = []
        for i, topic in enumerate(self.topics):
            online = bool(received[i]) and now - received[i] < 3
            rows.append({"name": CAMERA_NAMES[i], "topic": topic, "online": online,
                         "age_sec": round(now - received[i], 2) if received[i] else None,
                         "error": None if online else ("话题无发布者" if not self.node.count_publishers(topic)
                                  else "尚未收到图像" if not received[i] else "图像已过期")})
        return rows

    def latest_reference(self, index):
        with self.lock:
            message = self.latest[0][index]
            received = self.received_at[0][index]
        if message is None or self.topics[index] not in self.frame_topics or time.monotonic() - received > 0.5:
            return None
        stamp = message.header.stamp
        value = stamp.sec * 1_000_000_000 + stamp.nanosec
        return (self.topics[index], value) if value > 0 else None

    def on_joint(self, arm, message):
        stamps = [s.sec * 1_000_000_000 + s.nanosec for s in message.source_stamps[:3]]
        positions = list(message.position[:6])
        usable = (len(stamps) == 3 and all(s > 0 for s in stamps)
                  and max(stamps) - min(stamps) <= 30_000_000 and len(positions) == 6
                  and all(math.isfinite(v) for v in positions)
                  and not any(message.missing[:3]) and not any(message.stale[:3])
                  and message.source_topic == f"/piper_direct/{arm}/slave_feedback")
        with self.lock:
            self.joints[arm] = {"received": time.monotonic(), "unix_ns": time.time_ns(),
                                "stamp_ns": max(stamps) if stamps else 0,
                                "positions": positions, "usable": usable}

    def joint_snapshot(self):
        with self.lock:
            joints = {arm: dict(row) if row else None for arm, row in self.joints.items()}
        now = time.monotonic()
        return {arm: {"online": bool(row and row["usable"] and now - row["received"] <= 1.5),
                      "age_sec": round(now - row["received"], 2) if row else None,
                      "position_rad": row["positions"] if row and row["usable"] and now - row["received"] <= 1.5 else None,
                      "error": "未收到反馈" if row is None else "反馈过期或来源无效"}
                for arm, row in joints.items()}

    def initial_joint_positions(self, required_arms):
        with self.lock:
            rows = {arm: dict(row) if row else None for arm, row in self.joints.items()}
        result = {}
        for arm in required_arms:
            row = rows[arm]
            if row is None or not row["usable"] or time.monotonic() - row["received"] > 1.5:
                raise ValueError(f"{arm} 臂没有可用的实时关节反馈")
        for arm, row in rows.items():
            if row and row["usable"] and time.monotonic() - row["received"] <= 1.5:
                result[arm] = {"topic": ARM_TOPICS[arm]["slave_feedback"],
                               "header_stamp_ns": row["stamp_ns"], "received_at_unix_ns": row["unix_ns"],
                               "joint_names": [f"joint{i}" for i in range(1, 7)],
                               "position_rad": row["positions"]}
        return result

    def on_event(self, message):
        try:
            event = json.loads(message.data)
        except json.JSONDecodeError:
            return
        with self.lock:
            self.event = event


class Console(QMainWindow):
    def __init__(self, config, cameras):
        super().__init__()
        self.cameras = cameras
        root = Path(config["output_dir"]).expanduser().resolve()
        self.socket_path = Path(config.get("socket_path", str(root / "recorder.sock"))).expanduser().resolve()
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.depth_pool = ThreadPoolExecutor(max_workers=2)
        self.pending = {}
        self.state = "unknown"
        self.last_sequence = [[0] * 3 for _ in range(2)]
        self.depth_draw_at = [0.0] * 3
        self.depth_jobs = [None] * 3
        self.rendered = [0] * 3
        self.display_fps = [0] * 3
        self.display_latency_samples = [[] for _ in range(3)]
        self.fps_at = time.monotonic()
        self.setWindowTitle("Piper 数据采集控制台")
        self.resize(1500, 1050)

        container = QWidget()
        self.setCentralWidget(container)
        layout = QVBoxLayout(container)
        self.state_label = QLabel("正在连接录制器…")
        layout.addWidget(self.state_label)
        camera_grid = QGridLayout()
        layout.addLayout(camera_grid, 3)
        self.images, self.depth_images, self.camera_labels, self.depth_labels = [], [], [], []
        for index, name in enumerate(CAMERA_NAMES):
            title = QLabel(name)
            title.setStyleSheet("font-weight: bold; font-size: 16px")
            image = QLabel("等待图像…")
            image.setAlignment(Qt.AlignCenter)
            image.setMinimumWidth(260)
            image.setFixedHeight(175)
            image.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
            image.setStyleSheet("background: #10151d; color: white")
            info = QLabel(self.cameras.topics[index])
            info.setWordWrap(True)
            depth_title = QLabel(f"深度 · 0–{DEPTH_MAX_METERS[index]:g} 米；灰色超距，黑色无效，白线为彩色轮廓")
            depth_image = QLabel("等待深度图…")
            depth_image.setAlignment(Qt.AlignCenter)
            depth_image.setMinimumWidth(260)
            depth_image.setFixedHeight(175)
            depth_image.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
            depth_image.setStyleSheet("background: #10151d; color: white")
            depth_info = QLabel(self.cameras.depth_topics[index] or "")
            depth_info.setWordWrap(True)
            if self.cameras.depth_topics[index] is None:
                depth_title.hide()
                depth_image.hide()
                depth_info.hide()
            camera_grid.addWidget(title, 0, index)
            camera_grid.addWidget(image, 1, index)
            camera_grid.addWidget(info, 2, index)
            camera_grid.addWidget(depth_title, 3, index)
            camera_grid.addWidget(depth_image, 4, index)
            camera_grid.addWidget(depth_info, 5, index)
            camera_grid.setColumnStretch(index, 1)
            self.images.append(image)
            self.depth_images.append(depth_image)
            self.camera_labels.append(info)
            self.depth_labels.append(depth_info)

        fields = QGridLayout()
        layout.addLayout(fields)
        fields.addWidget(QLabel("采集类型"), 0, 2)
        self.mode = QComboBox(); self.mode.addItem("完整任务", "task"); self.mode.addItem("原子技能", "skill")
        fields.addWidget(self.mode, 0, 3)
        fields.addWidget(QLabel("固定技能"), 1, 2)
        self.skill = QComboBox()
        for value, label in zip(SKILLS, SKILL_LABELS): self.skill.addItem(label, value)
        fields.addWidget(self.skill, 1, 3)
        fields.addWidget(QLabel("执行臂"), 2, 2)
        self.arm_mode = QComboBox()
        for value in ("LEFT", "RIGHT", "BIMANUAL"): self.arm_mode.addItem(value, value)
        fields.addWidget(self.arm_mode, 2, 3)
        fields.addWidget(QLabel("扰动编号"), 3, 2)
        self.perturbation = QLineEdit(); fields.addWidget(self.perturbation, 3, 3)
        fields.addWidget(QLabel("相机内参快照版本"), 4, 0)
        self.calibration = QLineEdit(); fields.addWidget(self.calibration, 4, 1)
        fields.addWidget(QLabel("数据约定版本"), 4, 2)
        self.data_convention = QLineEdit(); fields.addWidget(self.data_convention, 4, 3)
        defaults = provisional_collection_versions()
        self.calibration.setText(defaults.get("calibration_version", ""))
        self.data_convention.setText(defaults.get("data_convention_version", ""))
        self.calibration.setToolTip("记录在线 CameraInfo 内参快照；训练所需相机身份、安装状态与内参仍须独立验收")
        self.data_convention.setToolTip("记录原始数据顺序与单位；训练集归一化统计尚未生成")
        self.contract = QLabel("训练契约：待检查")
        self.contract.setWordWrap(True)
        fields.addWidget(self.contract, 5, 0, 1, 4)
        self.joint_labels = {}
        for offset, arm in enumerate(("left", "right"), 6):
            label = QLabel(f"{arm} 从臂：等待关节反馈")
            fields.addWidget(label, offset, 0, 1, 4)
            self.joint_labels[arm] = label
        fields.addWidget(QLabel("本条任务名"), 0, 0)
        self.task = QLineEdit()
        self.task.setMaxLength(120)
        self.task.setToolTip("原子技能填写本条技能；完整任务填写总体任务。同一技能会保留上次填写内容")
        fields.addWidget(self.task, 0, 1)
        fields.addWidget(QLabel("本条语言指令"), 1, 0)
        self.instruction = QTextEdit()
        self.instruction.setFixedHeight(42)
        self.instruction.setToolTip("原子技能只描述本条动作；切换技能时恢复各自的文本")
        fields.addWidget(self.instruction, 1, 1)
        fields.addWidget(QLabel("Episode 保存目录"), 2, 0)
        save_location = QHBoxLayout()
        self.save_dir = QLineEdit(str(root))
        self.save_dir.setToolTip("保存时在此目录下自动创建 episode_000001 等目录；每个 Episode 可单独修改")
        browse = QPushButton("选择文件夹…")
        browse.clicked.connect(self.choose_save_dir)
        save_location.addWidget(self.save_dir, 1)
        save_location.addWidget(browse)
        fields.addLayout(save_location, 2, 1)
        fields.addWidget(QLabel("本条时长上限（完整任务/原子技能）"), 3, 0)
        self.duration = QSpinBox()
        self.duration.setRange(1, 86400)
        self.duration.setValue(60)
        self.duration.setSuffix(" 秒（到时自动停止）")
        fields.addWidget(self.duration, 3, 1)
        fields.setColumnStretch(1, 1)

        data_use = QLabel(
            "数据用途：本窗口保存 RGB-D、按配置录制的机械臂/相机原始话题及任务结果，供回放和质量审计。"
            "原子技能 Episode 可在标定、夹爪映射和逐帧审计通过后导出为 π0.5 候选数据；"
            "完整任务与运行轨迹经人工核验后可整理为 Qwen 规划候选数据。"
            "保存的 Episode 不会自动成为训练样本。")
        data_use.setWordWrap(True)
        layout.addWidget(data_use)

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
        marks = QHBoxLayout(); layout.addLayout(marks)
        self.mark_start = QPushButton("开始下一技能")
        self.mark_start.hide()
        self.mark_complete = QPushButton("录制后补标完成帧")
        self.mark_success = QPushButton("标记当前技能成功")
        self.mark_failure = QPushButton("标记本条失败")
        self.mark_exception = QPushButton("标记异常")
        self.mark_recovery = QPushButton("标记恢复")
        self.mark_hold = QPushButton("开始保持（重复最后目标）")
        self.trigger_reason = QLineEdit(); self.trigger_reason.setPlaceholderText("异常或恢复原因")
        self.camera_ref = QComboBox(); self.camera_ref.addItem("无画面关联", None)
        for i, name in enumerate(CAMERA_NAMES): self.camera_ref.addItem(name, i)
        self.camera_ref.setCurrentIndex(3)
        self.review_offset = QLineEdit()
        self.review_offset.setPlaceholderText("完成时刻：录制开始后第几秒")
        self.review_offset.setMaximumWidth(230)
        for widget in (self.mark_start, self.mark_complete, self.mark_success, self.mark_failure,
                       self.mark_exception, self.mark_recovery, self.mark_hold, self.trigger_reason,
                       self.camera_ref, self.review_offset):
            marks.addWidget(widget)
        mark_help = QLabel("完整任务：录制中点击“标记当前技能成功”记录当前画面并进入下一技能。原子技能：可在成功时点击记录当前画面，再停止录制；停机核验通过后自动选择成功。到时自动取末帧，提前停止可补标。成功保存须有一条经 rosbag 核验的完成帧。")
        mark_help.setWordWrap(True)
        layout.addWidget(mark_help)
        self.annotation_label = QLabel("尚无标注")
        layout.addWidget(self.annotation_label)
        self.completion_label = QLabel("")
        self.completion_label.setWordWrap(True)
        layout.addWidget(self.completion_label)
        self.table = QTableWidget(0, 9)
        self.table.setHorizontalHeaderLabels(("编号", "类型", "技能", "任务", "语言指令", "结果", "录制时长", "开始时间", "路径"))
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        layout.addWidget(self.table, 2)

        self.start.clicked.connect(self.start_recording)
        self.stop.clicked.connect(lambda: self.action("stop"))
        self.save.clicked.connect(self.save_recording)
        self.discard.clicked.connect(self.discard_recording)
        self.outcome.currentIndexChanged.connect(self.update_buttons)
        self.mode.currentIndexChanged.connect(self.update_contract)
        self.skill.currentIndexChanged.connect(self.update_contract)
        self.arm_mode.currentIndexChanged.connect(self.update_contract)
        self.calibration.textChanged.connect(self.update_contract)
        self.data_convention.textChanged.connect(self.update_contract)
        self.mark_start.clicked.connect(self.mark_next_skill)
        self.mark_complete.clicked.connect(lambda: self.mark_annotation("skill_complete", self.active_skill))
        self.mark_success.clicked.connect(self.mark_skill_success)
        self.mark_failure.clicked.connect(self.mark_skill_failure)
        self.mark_exception.clicked.connect(lambda: self.mark_annotation("recovery_trigger", "异常：" + (self.trigger_reason.text().strip() or "人工触发")))
        self.mark_recovery.clicked.connect(lambda: self.mark_annotation("recovery_trigger", "恢复：" + (self.trigger_reason.text().strip() or "人工触发")))
        self.mark_hold.clicked.connect(self.toggle_hold)
        self.config = config
        self.active_skill = None
        self.active_mode = None
        self.annotations = []
        self.hold_segments = []
        self._text_context = None
        self._text_cache = {}
        self.update_contract()
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
        body = {"task": self.task.text(), "language_instruction": self.instruction.toPlainText(),
                "mode": self.mode.currentData(), "skill": self.skill.currentData(),
                "arm_mode": self.arm_mode.currentData(), "perturbation_id": self.perturbation.text(),
                "calibration_version": self.calibration.text(),
                "data_convention_version": self.data_convention.text(), "duration_sec": self.duration.value()}
        try:
            request = start_request(body, self.config, self.cameras)
        except ValueError as exc:
            self.message.setText(str(exc)); return
        self.action(request.pop("action"), **request)

    def update_contract(self):
        skill_mode = self.mode.currentData() == "skill"
        bimanual = self.skill.currentData() in SKILLS[1:]
        for widget in (self.skill, self.calibration, self.data_convention):
            widget.setEnabled(skill_mode and self.state == "idle")
        self.arm_mode.setEnabled(skill_mode and not bimanual and self.state == "idle")
        if bimanual and skill_mode:
            self.arm_mode.setCurrentIndex(2)
        elif self.arm_mode.currentData() == "BIMANUAL":
            self.arm_mode.setCurrentIndex(0)
        context = ("task",) if not skill_mode else ("skill", self.skill.currentData(),
                                                   self.arm_mode.currentData())
        if context != self._text_context:
            if self._text_context is not None:
                self._text_cache[self._text_context] = (self.task.text(), self.instruction.toPlainText())
            template = TASK_TEXT["task" if not skill_mode else self.skill.currentData()]
            arm = "左" if self.arm_mode.currentData() == "LEFT" else "右"
            recommended = (template[0], template[1].format(arm=arm))
            saved = self._text_cache.get(context, recommended)
            self.task.setText(saved[0])
            self.instruction.setPlainText(saved[1])
            self.task.setPlaceholderText(recommended[0])
            self.instruction.setPlaceholderText(recommended[1])
            self._text_context = context
        diagnostics = collection_contract_diagnostics(
            self.config, self.arm_mode.currentData(),
            self.calibration.text(), self.data_convention.text())
        declaration_status = ("原始事实声明齐全" if diagnostics["metadata_claimable"] else "原始事实声明尚不齐全")
        self.contract.setText(declaration_status + "；训练待审：" + "；".join(diagnostics["blockers"]))

    def mark_next_skill(self):
        done = [row for row in self.annotations if row.get("kind") == "skill_complete"]
        if len(done) < len(SKILLS): self.mark_annotation("skill_start", SKILLS[len(done)])

    def mark_skill_success(self):
        if self.state == "stopped" and self.active_mode == "skill":
            issue = self.success_completion_issue()
            if issue:
                self.message.setText(issue + "；请先补标，再选择成功")
            else:
                self.outcome.setCurrentIndex(1)
                self.message.setText("完成帧已核验；已选择本条原子技能成功，可保存 Episode")
        else:
            self.mark_annotation("skill_complete", self.active_skill)

    def mark_skill_failure(self):
        if self.state == "stopped" and self.active_mode == "skill":
            self.outcome.setCurrentIndex(2)
            self.message.setText("已选择本条原子技能失败；可保存 Episode")
        else:
            self.mark_annotation("outcome", "failure")

    def toggle_hold(self):
        open_hold = any(item.get("end_unix_ns") is None for item in self.hold_segments)
        if open_hold:
            self.mark_annotation("hold_end", "operator_release")
        else:
            self.mark_annotation("hold_start", "operator_keep_last_target")

    def mark_annotation(self, kind, value):
        try:
            body = {"kind": kind, "value": value, "camera_index": self.camera_ref.currentData()}
            if kind == "skill_complete" and self.state == "recording":
                if self.camera_ref.currentData() is None:
                    raise ValueError("请选择当前彩色相机画面")
                body["skill_result"] = "success"
            if kind == "skill_complete" and self.state == "stopped":
                if self.camera_ref.currentData() is None or not self.review_offset.text().strip():
                    raise ValueError("请选择相机并填写录制开始后的完成时刻（秒）")
                try:
                    body["review_offset_sec"] = float(self.review_offset.text().strip())
                except ValueError as exc:
                    raise ValueError("完成时刻须为秒数") from exc
            request = annotation_request(body, self.cameras)
        except ValueError as exc:
            self.message.setText(str(exc)); return
        self.action(request.pop("action"), **request)

    def save_recording(self):
        outcome = (None, "success", "failure")[self.outcome.currentIndex()]
        if outcome:
            if outcome == "success" and self.active_mode == "skill":
                issue = self.success_completion_issue()
                if issue:
                    self.message.setText(issue + "；请先补标，再保存")
                    return
            path = self.save_dir.text().strip()
            if not path:
                self.message.setText("请填写 Episode 保存目录")
                return
            self.action("save", outcome=outcome, save_dir=path)

    def choose_save_dir(self):
        selected = QFileDialog.getExistingDirectory(self, "选择 Episode 保存目录", self.save_dir.text())
        if selected:
            self.save_dir.setText(selected)

    def discard_recording(self):
        if QMessageBox.question(self, "确认丢弃", "确定丢弃当前 Episode？录制数据将被删除。") == QMessageBox.Yes:
            self.action("discard")

    def success_completion_issue(self):
        return skill_completion_issue(self.active_skill, self.annotations, self.cameras.frame_topics)

    def update_buttons(self):
        busy = "action" in self.pending
        editable = not busy and self.state == "idle"
        for widget in (self.mode, self.task, self.instruction, self.duration, self.perturbation):
            widget.setEnabled(editable)
        for widget in (self.skill, self.calibration, self.data_convention):
            widget.setEnabled(editable and self.mode.currentData() == "skill")
        self.arm_mode.setEnabled(editable and self.mode.currentData() == "skill"
                                 and self.skill.currentData() not in SKILLS[1:])
        self.start.setEnabled(not busy and self.state == "idle")
        self.stop.setEnabled(not busy and self.state == "recording")
        success_issue = (self.success_completion_issue() if self.state == "stopped"
                         and self.active_mode == "skill" else None)
        success_selected = self.outcome.currentIndex() == 1
        self.save.setEnabled(not busy and self.state == "stopped" and self.outcome.currentIndex() > 0
                             and not (success_selected and success_issue))
        self.completion_label.setText(
            f"原子技能完成帧：{success_issue}。选择彩色相机并填写完成时刻后点击补标。"
            if self.state == "stopped" and self.active_mode == "skill" and success_issue
            else "原子技能完成帧：已在 rosbag 中核验。"
            if self.state == "stopped" and self.active_mode == "skill" else "")
        self.discard.setEnabled(not busy and self.state in ("recording", "stopped", "failed", "interrupted"))
        recording = not busy and self.state == "recording"
        mode = self.active_mode if self.state in ("recording", "stopped") else self.mode.currentData()
        self.mark_start.setEnabled(recording and mode == "task"
                                   and not self.active_skill and len([a for a in self.annotations if a.get("kind") == "skill_complete"]) < len(SKILLS))
        skill_review = not busy and self.state == "stopped" and mode == "skill"
        completed = any(a.get("kind") == "skill_complete" for a in self.annotations)
        self.mark_complete.setEnabled(skill_review and not completed)
        self.review_offset.setEnabled(skill_review)
        self.camera_ref.setEnabled(recording or skill_review)
        self.mark_success.setEnabled((recording and bool(self.active_skill)) or skill_review)
        self.mark_failure.setEnabled((recording and mode == "task") or skill_review)
        for widget in (self.mark_exception, self.mark_recovery):
            widget.setEnabled(recording)
        open_hold = any(item.get("end_unix_ns") is None for item in self.hold_segments)
        self.mark_hold.setText("结束保持" if open_hold else "开始保持（重复最后目标）")
        self.mark_hold.setEnabled(recording and bool(self.active_skill))

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
                    self.active_mode = meta.get("mode")
                    self.active_skill = meta.get("active_skill") or (meta.get("skill") if meta.get("mode") == "skill" else None)
                    self.annotations = meta.get("annotations") or []
                    self.hold_segments = meta.get("hold_segments") or []
                    recorded_outcomes = [item.get("value") for item in self.annotations
                                         if item.get("kind") == "outcome"]
                    if self.outcome.currentIndex() == 0 and recorded_outcomes:
                        self.outcome.setCurrentIndex({"success": 1, "failure": 2}.get(recorded_outcomes[-1], 0))
                    if (meta.get("mode") == "task" and self.outcome.currentIndex() == 0
                            and len([item for item in self.annotations
                                     if item.get("kind") == "skill_complete"
                                     and item.get("skill_result") == "success"]) == len(SKILLS)
                            and not any(item.get("kind") == "outcome" and item.get("value") == "failure"
                                        for item in self.annotations)):
                        self.outcome.setCurrentIndex(1)
                    if (self.state == "stopped" and meta.get("mode") == "skill"
                            and self.outcome.currentIndex() == 0
                            and self.success_completion_issue() is None
                            and any(item.get("kind") == "skill_complete"
                                    and type(item.get("recorded_at_unix_ns")) is int
                                    for item in self.annotations)):
                        self.outcome.setCurrentIndex(1)
                    if self.annotations:
                        last = self.annotations[-1]
                        matched = last.get("matched_offset_sec")
                        position = f" · 匹配录制第 {matched:.3f} 秒" if isinstance(matched, (int, float)) else ""
                        self.annotation_label.setText(
                            f"标注 {len(self.annotations)} 条 · 最近：{last.get('kind')}={last.get('value')}{position}")
                    else:
                        self.annotation_label.setText("尚无标注")
                    detail = f" · {meta.get('task', '')}" if meta else ""
                    if self.state == "recording" and result.get("remaining_sec") is not None:
                        detail += f" · 剩余约 {math.ceil(result['remaining_sec'])} 秒"
                    if meta.get("stop_reason") == "duration_limit":
                        detail += " · 已到时自动停止"
                    if meta.get("auto_completion_error"):
                        detail += f" · 末帧自动标记失败：{meta['auto_completion_error']}；请手动补标"
                    if meta.get("auto_completion_source_warning"):
                        detail += f" · {meta['auto_completion_source_warning']}"
                    unresolved = [item.get("frame_verification_error") for item in self.annotations
                                  if item.get("frame_verification_error")]
                    if unresolved:
                        detail += f" · 技能完成帧核验失败：{unresolved[-1]}"
                    if meta.get("error"):
                        detail += f" · {meta['error']}"
                    self.state_label.setText(f"录制状态：{STATE_NAMES.get(self.state, self.state)}{detail}")
                elif key == "list":
                    self.show_episodes(result["episodes"])
                else:
                    if result.get("state") == "saved":
                        self.message.setText(f"已保存至 {result['path']}")
                    elif (result.get("annotation", {}).get("kind") == "skill_complete"
                          and self.active_mode == "skill" and self.state == "recording"):
                        self.message.setText("已记录原子技能完成时刻；请停止录制，等待完成帧核验")
                    else:
                        self.message.setText("操作完成")
                    if result.get("state") == "saved":
                        self.outcome.setCurrentIndex(0)
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
            duration = episode.get("recorded_duration_sec")
            values = (episode.get("episode_number"), episode.get("mode"), episode.get("skill"), episode.get("task"),
                      episode.get("language_instruction"), outcome,
                      f"{duration:.1f} 秒" if duration is not None else "", when, episode.get("path"))
            for column, value in enumerate(values):
                self.table.setItem(row, column, QTableWidgetItem(str(value or "")))

    def draw_frames(self):
        now = time.monotonic()
        snapshot = self.cameras.frame_snapshot()
        for kind, frames in enumerate(snapshot):
            images = self.images if kind == 0 else self.depth_images
            labels = self.camera_labels if kind == 0 else self.depth_labels
            for index, (message, sequence, received_at) in enumerate(frames):
                if kind == 1 and self.cameras.depth_topics[index] is None:
                    continue
                if kind == 1:
                    job = self.depth_jobs[index]
                    if job is not None and job.done():
                        self.depth_jobs[index] = None
                        try:
                            rgb = job.result()
                            pixmap = QPixmap.fromImage(QImage(rgb.data, rgb.shape[1], rgb.shape[0],
                                                              rgb.strides[0], QImage.Format_RGB888))
                            images[index].setPixmap(pixmap.scaled(images[index].size(),
                                                                  Qt.KeepAspectRatio, Qt.FastTransformation))
                        except (ValueError, cv2.error) as exc:
                            labels[index].setText(f"{CAMERA_NAMES[index]}：{exc}")
                    if (sequence != self.last_sequence[kind][index] and message is not None
                            and self.depth_jobs[index] is None and now - self.depth_draw_at[index] >= 0.2):
                        color_message, _, color_received_at = snapshot[0][index]
                        if now - color_received_at >= 0.25:
                            color_message = None
                        self.depth_jobs[index] = self.depth_pool.submit(render_depth, message, color_message, index)
                        self.depth_draw_at[index] = now
                        self.last_sequence[kind][index] = sequence
                    if now - received_at > 2 and received_at:
                        reason = "话题无发布者" if not self.cameras.node.count_publishers(self.cameras.depth_topics[index]) else "画面已过期"
                        labels[index].setText(f"{CAMERA_NAMES[index]}深度：{reason} · 最后帧龄 {now-received_at:.1f} s")
                    elif not received_at:
                        reason = "话题无发布者" if not self.cameras.node.count_publishers(self.cameras.depth_topics[index]) else "等待第一帧"
                        labels[index].setText(f"{CAMERA_NAMES[index]}深度：{reason} · {self.cameras.depth_topics[index]}")
                    continue
                if sequence != self.last_sequence[kind][index] and message is not None:
                    self.last_sequence[kind][index] = sequence
                    try:
                        pixmap = to_pixmap(message)
                        images[index].setPixmap(pixmap.scaled(images[index].size(),
                                                              Qt.KeepAspectRatio, Qt.FastTransformation))
                        self.rendered[index] += 1
                        samples = self.display_latency_samples[index]
                        samples.append((now - received_at) * 1000)
                        if len(samples) > 300: del samples[:-300]
                    except (ValueError, cv2.error) as exc:
                        labels[index].setText(f"{CAMERA_NAMES[index]}：{exc}")
                if now - received_at > 2 and received_at:
                    reason = "话题无发布者" if not self.cameras.node.count_publishers(self.cameras.topics[index]) else "画面已过期"
                    labels[index].setText(f"{CAMERA_NAMES[index]}彩色：{reason} · 最后帧龄 {now-received_at:.1f} s")
                elif not received_at:
                    reason = "话题无发布者" if not self.cameras.node.count_publishers(self.cameras.topics[index]) else "等待第一帧"
                    labels[index].setText(f"{CAMERA_NAMES[index]}彩色：{reason} · {self.cameras.topics[index]}")
        if now - self.fps_at >= 1:
            interval = now - self.fps_at
            self.display_fps = [round(count / interval, 1) for count in self.rendered]
            for kind, frames in enumerate(snapshot):
                for index, (_, _, received_at) in enumerate(frames):
                    if kind == 1 and self.cameras.depth_topics[index] is None:
                        continue
                    if received_at and now - received_at <= 2:
                        age_ms = round((now - received_at) * 1000)
                        if kind == 0:
                            self.camera_labels[index].setText(
                                f"彩色：预览 {self.display_fps[index]:.1f} fps · 接收帧龄 {age_ms} ms · {self.cameras.topics[index]}")
                        else:
                            self.depth_labels[index].setText(f"深度：接收帧龄 {age_ms} ms · {self.cameras.depth_topics[index]}")
            for arm, row in self.cameras.joint_snapshot().items():
                if row["online"]:
                    values = "、".join(f"{v:.3f}" for v in row["position_rad"])
                    self.joint_labels[arm].setText(f"{arm} 从臂六关节 rad：{values} · 帧龄 {row['age_sec']:.2f} s")
                else:
                    self.joint_labels[arm].setText(f"{arm} 从臂：{row['error']}")
            self.rendered = [0] * 3
            self.fps_at = now

    def closeEvent(self, event):
        self.frames_timer.stop(); self.status_timer.stop(); self.list_timer.stop(); self.work_timer.stop()
        self.pool.shutdown(wait=False, cancel_futures=True)
        self.depth_pool.shutdown(wait=False, cancel_futures=True)
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
