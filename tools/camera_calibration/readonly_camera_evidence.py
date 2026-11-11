#!/usr/bin/env python3
"""只读采集三相机证据并离线验收；不发布机器人命令、写参数或调用控制服务。"""

import argparse
from collections import deque
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
import zlib


ROLES = ("left_wrist", "right_wrist", "top")
ROOT = Path(__file__).resolve().parents[2]
INFO_KEYS = ("width", "height", "frame_id", "distortion_model", "d", "k", "r", "p")
ALLOWED_DEVICE_SERVICES = {
    "orbbec_camera_msgs/srv/GetDeviceInfo",
    "realsense2_camera_msgs/srv/DeviceInfo",
}


def digest(path):
    result = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def canonical(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"),
                      allow_nan=False).encode("utf-8")


def object_digest(value):
    return hashlib.sha256(canonical(value)).hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def write_json(path, value):
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def stamp(message):
    value = message.header.stamp
    if value.sec < 0 or not 0 <= value.nanosec < 1_000_000_000:
        raise ValueError("无效 Header 时间戳")
    return value.sec * 1_000_000_000 + value.nanosec


def info_dict(message):
    return {
        "header_stamp_ns": stamp(message), "frame_id": message.header.frame_id,
        "width": message.width, "height": message.height,
        "distortion_model": message.distortion_model,
        **{key: [float(value) for value in getattr(message, key)] for key in ("d", "k", "r", "p")},
        "binning_x": message.binning_x, "binning_y": message.binning_y,
        "roi": {key: getattr(message.roi, key) for key in
                ("x_offset", "y_offset", "width", "height", "do_rectify")},
    }


def profile(info):
    return {key: value for key, value in info.items() if key != "header_stamp_ns"}


def image_dict(message):
    return {"header_stamp_ns": stamp(message), "frame_id": message.header.frame_id,
            "width": message.width, "height": message.height, "encoding": message.encoding,
            "step": message.step, "is_bigendian": message.is_bigendian,
            "data_size": len(message.data)}


def png_rgb(meta, data):
    width, height, step = meta["width"], meta["height"], meta["step"]
    if meta["encoding"] != "rgb8" or step < width * 3 or len(data) != step * height:
        raise ValueError("RGB 数据布局不符，无法生成原像素 PNG")

    def chunk(kind, content):
        return (struct.pack(">I", len(content)) + kind + content
                + struct.pack(">I", zlib.crc32(kind + content) & 0xffffffff))

    rows = b"".join(b"\0" + data[y * step:y * step + width * 3] for y in range(height))
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(rows)) + chunk(b"IEND", b""))


def check_info(info, expected):
    errors, warnings = [], []
    for key in ("width", "height", "frame_id"):
        if info.get(key) != expected[key]:
            errors.append(f"CameraInfo {key} 不符")
    for key, size in (("k", 9), ("r", 9), ("p", 12), ("d", None)):
        values = info.get(key)
        if (not isinstance(values, list) or (size is not None and len(values) != size)
                or not values or any(type(x) not in (int, float) or not math.isfinite(x) for x in values)):
            errors.append(f"CameraInfo {key} 长度或数值无效")
    if errors:
        return errors, warnings
    k, p = info["k"], info["p"]
    if (k[0] <= 0 or k[4] <= 0 or abs(k[8] - 1) > 1e-9
            or not 0 <= k[2] < info["width"] or not 0 <= k[5] < info["height"]
            or any(abs(k[i]) > 1e-9 for i in (1, 3, 6, 7))
            or p[0] <= 0 or p[5] <= 0 or abs(p[10] - 1) > 1e-9):
        errors.append("CameraInfo K/P 不符合本项目 RGB 规格")
    if info.get("binning_x", 0) not in (0, 1) or info.get("binning_y", 0) not in (0, 1):
        errors.append("CameraInfo 使用了未在训练规格中声明的 binning")
    roi = info.get("roi", {})
    if (roi.get("x_offset", 0) != 0 or roi.get("y_offset", 0) != 0
            or roi.get("width", 0) not in (0, info["width"])
            or roi.get("height", 0) not in (0, info["height"])):
        errors.append("CameraInfo 使用了未在训练规格中声明的 ROI")
    lengths = {"plumb_bob": 5, "rational_polynomial": 8, "equidistant": 4}
    model = info.get("distortion_model")
    if model not in lengths or len(info["d"]) != lengths[model]:
        warnings.append(f"畸变模型 {model} 与 D[{len(info['d'])}] 的 ROS 语义须独立核实；禁止据此去畸变或反投影")
    return errors, warnings


def compare_baseline(info, baseline, tolerance):
    changed = []
    for key in INFO_KEYS:
        left, right = info.get(key), baseline.get(key)
        if key in ("d", "k", "r", "p"):
            if (not isinstance(left, list) or not isinstance(right, list) or len(left) != len(right)
                    or any(not math.isfinite(a) or abs(a - b) > tolerance for a, b in zip(left, right))):
                changed.append(key)
        elif left != right:
            changed.append(key)
    return changed


def usb_inventory(root=Path("/sys/bus/usb/devices")):
    result = []
    for device in sorted(root.iterdir()):
        if not (device / "idVendor").is_file():
            continue
        item = {"sysfs_device": device.name}
        for key in ("idVendor", "idProduct", "serial", "manufacturer", "product", "speed"):
            try:
                item[key] = (device / key).read_text(encoding="utf-8").strip()
            except OSError:
                item[key] = None
        result.append(item)
    return result


def installed_software():
    from ament_index_python.packages import get_package_prefix, get_package_share_directory

    result = {}
    for package, executable in (("orbbec_camera", "orbbec_camera_node"),
                                ("realsense2_camera", "realsense2_camera_node")):
        xml_path = Path(get_package_share_directory(package)) / "package.xml"
        binary = Path(get_package_prefix(package)) / "lib" / package / executable
        result[package] = {"version": ET.parse(xml_path).findtext("version"),
                           "package_xml_sha256": digest(xml_path), "executable_sha256": digest(binary),
                           "executable_path": str(binary)}
    return result


def validate_inputs(contract, mapping, baseline):
    if (contract.get("schema_version") != 1
            or contract.get("input_profile") != "raw_rgb_joint_absolute_position"
            or contract.get("mount_review_required") is not True
            or set(contract.get("cameras", {})) != set(ROLES)):
        raise ValueError("不是受支持的 RGB+关节相机证据契约")
    limits = contract["limits"]
    if (limits["width"] != 640 or limits["height"] != 480 or limits["encoding"] != "rgb8"
            or type(limits["max_rgb_info_skew_ns"]) is not int or limits["max_rgb_info_skew_ns"] <= 0
            or type(limits["min_messages_per_stream"]) is not int or limits["min_messages_per_stream"] < 2
            or not 0 <= limits["intrinsic_absolute_tolerance"] <= 1e-3):
        raise ValueError("无效的相机验收门限")
    for key in ("serial", "usb_serial", "camera_name"):
        values = [mapping[role][key] for role in ROLES]
        if any(not isinstance(value, str) or not value.strip() for value in values) or len(set(values)) != 3:
            raise ValueError(f"三相机 {key} 必须为不同的非空字符串")
    for role in ROLES:
        expected = contract["cameras"][role]
        if (expected["device_service_type"] not in ALLOWED_DEVICE_SERVICES
                or baseline["cameras"][role]["serial"] != mapping[role]["serial"]):
            raise ValueError(f"{role}：设备信息接口或历史序列号不符")


def stream_errors(records, minimum, name):
    errors = []
    if len(records) < minimum:
        errors.append(f"{name} 接收数不足 {minimum}")
    stamps = [item["header_stamp_ns"] for item in records]
    if any(type(value) is not int or value <= 0 for value in stamps):
        errors.append(f"{name} Header 时间戳无效")
    if any(right <= left for left, right in zip(stamps, stamps[1:])):
        errors.append(f"{name} Header 时间戳重复或倒退")
    return errors


def review_errors(review, capture, capture_sha):
    if review is None:
        return ["缺少绑定本次 RGB 画面的安装角色复核记录"]
    errors = []
    if (review.get("capture_sha256") != capture_sha or not review.get("reviewer", "").strip()
            or not review.get("installation_epoch_id", "").strip()
            or review.get("method") != "visual_role_confirmation"):
        errors.append("安装复核须绑定 capture 哈希、复核人、安装批次和 visual_role_confirmation 方法")
    try:
        when = datetime.fromisoformat(review.get("reviewed_at", "").replace("Z", "+00:00"))
        if when.tzinfo is None or when < datetime.fromisoformat(capture["ended_at"]):
            errors.append("安装复核时间须含时区且不早于本次采集结束")
    except ValueError:
        errors.append("安装复核时间格式无效")
    for role in ROLES:
        item = review.get("cameras", {}).get(role, {})
        sample = capture["cameras"][role].get("sample") or {}
        if (item.get("confirmed") is not True or not sample
                or item.get("rgb_png_sha256") != sample.get("rgb_png_sha256")
                or item.get("serial") != capture["cameras"][role]["device"].get("serial")
                or not item.get("observation", "").strip()):
            errors.append(f"{role} 缺少画面哈希、实际序列号及安装角色复核说明")
    return errors


def audit(contract, mapping, baseline, capture, capture_sha, review=None):
    validate_inputs(contract, mapping, baseline)
    limits = contract["limits"]
    result = {"schema_version": 1, "scope": "raw_rgb_joint_camera_evidence_only",
              "version_id": capture["version_id"], "capture_sha256": capture_sha,
              "training_camera_evidence_ready": False, "episode_training_eligible": "not_evaluated",
              "intrinsic_geometry_verified": False, "extrinsic_geometry_verified": False,
              "motion_planning_geometry_ready": False, "cameras": {}}
    for role in ROLES:
        item, expected = capture["cameras"][role], contract["cameras"][role]
        errors, warnings = [], []
        if item["device"].get("serial") != mapping[role]["serial"] or item["device"].get("success") is not True:
            errors.append("驱动实际设备信息未确认期望序列号")
        devices = [device for device in capture["usb_devices"] if device.get("serial") == mapping[role]["usb_serial"]]
        if len(devices) != 1:
            errors.append("USB 序列号未唯一在位")
        for kind, topic in (("rgb", "rgb_topic"), ("info", "info_topic")):
            endpoints = item["publishers"].get(kind, [])
            if endpoints != [expected["node"]]:
                errors.append(f"{expected[topic]} 发布者不唯一或节点身份不符")
        images, infos = item["images"], item["infos"]
        errors.extend(stream_errors(images, limits["min_messages_per_stream"], "RGB"))
        errors.extend(stream_errors(infos, limits["min_messages_per_stream"], "CameraInfo"))
        expected_shape = {**limits, "frame_id": expected["frame_id"]}
        for image in images:
            if (any(image.get(key) != expected_shape[key] for key in ("width", "height", "encoding", "frame_id"))
                    or image.get("is_bigendian") not in (0, 1) or image.get("step", 0) < limits["width"] * 3
                    or image.get("data_size") != image.get("step", 0) * limits["height"]):
                errors.append("RGB 尺寸、编码、frame_id 或数据布局不符")
                break
        profiles = {object_digest(profile(info)) for info in infos}
        if len(profiles) != 1:
            errors.append("本次 CameraInfo 参数不稳定或缺失")
        for info in infos:
            found, warning = check_info(info, expected_shape)
            errors.extend(found)
            warnings.extend(warning)
            changed = compare_baseline(info, baseline["cameras"][role], limits["intrinsic_absolute_tolerance"])
            if changed:
                errors.append(f"相对冻结内参发生变化：{','.join(changed)}；须新建版本复核")
        sample = item.get("sample")
        if sample is None:
            errors.append("缺少可复核的 RGB/CameraInfo 近时原始样本")
        elif (sample["image"] not in images or sample["info"] not in infos
              or abs(sample["image"]["header_stamp_ns"] - sample["info"]["header_stamp_ns"])
              > limits["max_rgb_info_skew_ns"]):
            errors.append("样本引用不存在或 RGB/CameraInfo Header 配对超限")
        result["cameras"][role] = {"automated_pass": not errors, "errors": sorted(set(errors)),
                                   "warnings": sorted(set(warnings)), "image_count": len(images),
                                   "camera_info_count": len(infos), "intrinsic_profile_count": len(profiles)}
    result["mount_review_errors"] = review_errors(review, capture, capture_sha)
    result["training_camera_evidence_ready"] = (
        all(item["automated_pass"] for item in result["cameras"].values()) and not result["mount_review_errors"])
    if review is not None:
        result["mount_review_sha256"] = object_digest(review)
        result["installation_epoch_id"] = review.get("installation_epoch_id")
    result["limitations"] = [
        "仅对本次短时三机 RGB 与 CameraInfo 证据负责；不证明整条 Episode 无丢帧或同步合格",
        "设备序列号来自驱动只读设备信息接口，并以发布者身份和 USB 枚举交叉检查；CameraInfo 本身没有序列号",
        "position_verified 是历史声明；本次安装身份须独立复核，不自动继承为当前验收",
        "非零 K、稳定内参与内参快照哈希均不证明标定板重投影精度、手眼或跨相机几何精度",
        "畸变模型警告仅在原始 RGB 不使用几何变换的输入路径下允许保留；几何用途须先解决模型语义",
    ]
    return result


def mount_review_template(capture, capture_sha):
    return {"capture_sha256": capture_sha, "reviewer": "", "reviewed_at": "",
            "installation_epoch_id": "", "method": "visual_role_confirmation",
            "cameras": {role: {"confirmed": False,
                                "serial": capture["cameras"][role]["device"].get("serial"),
                                "rgb_png_sha256": (capture["cameras"][role].get("sample") or {}).get("rgb_png_sha256"),
                                "observation": ""} for role in ROLES}}


def checked_file(directory, name):
    path = directory / name
    if Path(name).is_absolute() or ".." in Path(name).parts or path.is_symlink() or not path.is_file():
        raise ValueError(f"证据包路径无效：{name}")
    if not path.resolve().is_relative_to(directory.resolve()):
        raise ValueError(f"证据包路径越界：{name}")
    return path


def load_package(directory):
    manifest = read_json(directory / "manifest.json")
    files = manifest.get("files", {})
    required = {"capture.json", "report.json", "contract.json", "camera_map.json", "baseline.json", "collector.py"}
    if manifest.get("schema_version") != 1 or not required <= files.keys():
        raise ValueError("证据包清单不完整")
    actual = {str(path.relative_to(directory)) for path in directory.rglob("*") if path.is_file()
              and path != directory / "manifest.json"}
    if actual != set(files):
        raise ValueError("证据包文件集合与清单不一致")
    for name, expected in files.items():
        if digest(checked_file(directory, name)) != expected:
            raise ValueError(f"证据哈希不符：{name}")
    contract, mapping, baseline, capture = (read_json(directory / name) for name in
                                          ("contract.json", "camera_map.json", "baseline.json", "capture.json"))
    if (digest(directory / "camera_map.json") != contract["baseline_sources"]["camera_map_sha256"]
            or digest(directory / "baseline.json") != contract["baseline_sources"]["camera_info_sha256"]):
        raise ValueError("冻结来源与契约哈希不符")
    if (capture.get("collector_sha256") != digest(directory / "collector.py")
            or manifest.get("version_id") != capture.get("version_id")):
        raise ValueError("采集工具或版本来源不一致")
    for role in ROLES:
        sample = capture["cameras"][role].get("sample")
        if sample:
            raw = checked_file(directory, f"{role}.rgb.bin").read_bytes()
            png = checked_file(directory, f"{role}.rgb.png")
            if (hashlib.sha256(raw).hexdigest() != sample["rgb_data_sha256"]
                    or digest(png) != sample["rgb_png_sha256"] or png.read_bytes() != png_rgb(sample["image"], raw)):
                raise ValueError(f"{role} 图像数据、PNG 或样本哈希不一致")
    computed = audit(contract, mapping, baseline, capture, digest(directory / "capture.json"))
    if computed != read_json(directory / "report.json"):
        raise ValueError("离线重算结果与冻结报告不一致")
    return contract, mapping, baseline, capture


def capture_live(contract, duration):
    # Lazy imports keep package verification independent of ROS and camera drivers.
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.utilities import get_service
    from sensor_msgs.msg import CameraInfo, Image

    rclpy.init(args=[])
    node = rclpy.create_node("piper_readonly_camera_evidence", enable_rosout=False, start_parameter_services=False)
    capture = {"schema_version": 1, "started_at": datetime.now(timezone.utc).isoformat(),
               "duration_sec_requested": duration, "usb_devices": usb_inventory(), "cameras": {}}
    buffers, latest_images, samples, futures, clients, subscriptions = {}, {}, {}, {}, [], []
    for role in ROLES:
        capture["cameras"][role] = {"device": {"success": False, "serial": None},
                                      "publishers": {}, "images": [], "infos": []}
        buffers[role] = deque(maxlen=60)

    def on_info(message, role):
        value = info_dict(message)
        capture["cameras"][role]["infos"].append(value)
        buffers[role].append(value)
        if role in latest_images:
            image, data = latest_images[role]
            if abs(image["header_stamp_ns"] - value["header_stamp_ns"]) <= contract["limits"]["max_rgb_info_skew_ns"]:
                samples[role] = (image, value, data)

    def on_image(message, role):
        value = image_dict(message)
        capture["cameras"][role]["images"].append(value)
        data = bytes(message.data)
        latest_images[role] = (value, data)
        if buffers[role]:
            info = min(buffers[role], key=lambda item: abs(item["header_stamp_ns"] - value["header_stamp_ns"]))
            if abs(info["header_stamp_ns"] - value["header_stamp_ns"]) <= contract["limits"]["max_rgb_info_skew_ns"]:
                samples[role] = (value, info, data)

    try:
        for role, expected in contract["cameras"].items():
            for key, msg_type, callback in (("rgb_topic", Image, on_image), ("info_topic", CameraInfo, on_info)):
                subscriptions.append(node.create_subscription(
                    msg_type, expected[key], lambda message, r=role, cb=callback: cb(message, r), qos_profile_sensor_data))
            service_type = get_service(expected["device_service_type"])
            client = node.create_client(service_type, expected["device_service"])
            clients.append(client)
            futures[role] = (client, service_type, None)
        started = time.monotonic()
        while time.monotonic() - started < duration:
            for role, (client, service_type, future) in list(futures.items()):
                if future is None and client.service_is_ready():
                    futures[role] = (client, service_type, client.call_async(service_type.Request()))
            rclpy.spin_once(node, timeout_sec=0.05)
        for role, (client, service_type, future) in futures.items():
            if future is not None and future.done():
                response = future.result()
                raw = message_to_ordereddict(response)
                if "info" in raw:
                    serial, success = raw["info"]["serial_number"], raw["success"]
                else:
                    serial, success = raw.get("serial_number"), True
                capture["cameras"][role]["device"] = {
                    "success": success, "serial": serial, "service": contract["cameras"][role]["device_service"],
                    "service_type": contract["cameras"][role]["device_service_type"], "response": raw}
        for role, expected in contract["cameras"].items():
            for kind, topic in (("rgb", "rgb_topic"), ("info", "info_topic")):
                endpoints = node.get_publishers_info_by_topic(expected[topic])
                capture["cameras"][role]["publishers"][kind] = sorted(
                    item.node_namespace.rstrip("/") + "/" + item.node_name for item in endpoints)
        capture["ended_at"] = datetime.now(timezone.utc).isoformat()
        capture["duration_sec_actual"] = time.monotonic() - started
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return capture, samples


def capture_command(args):
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,119}", args.version):
        raise ValueError("版本名须为 1～120 个字母、数字、下划线、点或连字符")
    if not math.isfinite(args.duration) or not 2 <= args.duration <= 120:
        raise ValueError("采集时长须为 2～120 s")
    contract, mapping, baseline = (read_json(path) for path in (args.contract, args.camera_map, args.baseline))
    validate_inputs(contract, mapping, baseline)
    if (digest(args.camera_map) != contract["baseline_sources"]["camera_map_sha256"]
            or digest(args.baseline) != contract["baseline_sources"]["camera_info_sha256"]):
        raise ValueError("相机配置或历史内参与冻结契约哈希不符；请建立新版本契约")
    args.output.mkdir(parents=True, exist_ok=False)
    for source, name in ((args.contract, "contract.json"), (args.camera_map, "camera_map.json"),
                         (args.baseline, "baseline.json"), (Path(__file__), "collector.py")):
        shutil.copyfile(source, args.output / name)
    capture, samples = capture_live(contract, args.duration)
    capture["software"] = installed_software()
    capture["version_id"] = args.version
    capture["collector_sha256"] = digest(args.output / "collector.py")
    try:
        capture["repository_head"] = subprocess.check_output(
            ["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL).strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        capture["repository_head"] = None
    for role in ROLES:
        capture["cameras"][role]["sample"] = None
        if role not in samples:
            continue
        image, info, data = samples[role]
        raw_path, png_path = args.output / f"{role}.rgb.bin", args.output / f"{role}.rgb.png"
        try:
            png = png_rgb(image, data)
        except ValueError:
            continue
        raw_path.write_bytes(data)
        png_path.write_bytes(png)
        capture["cameras"][role]["sample"] = {"image": image, "info": info,
            "rgb_data_sha256": digest(raw_path), "rgb_png_sha256": digest(png_path)}
    write_json(args.output / "capture.json", capture)
    report = audit(contract, mapping, baseline, capture, digest(args.output / "capture.json"))
    write_json(args.output / "report.json", report)
    write_json(args.output / "manifest.json", {"schema_version": 1, "version_id": args.version,
        "purpose": "文件完整性清单；不是签名、几何验收或训练资格声明",
        "files": {path.name: digest(path) for path in sorted(args.output.iterdir()) if path.is_file()}})
    load_package(args.output)
    print(json.dumps({"output": str(args.output.resolve()), "manifest_sha256": digest(args.output / "manifest.json"),
                      "report": report}, ensure_ascii=False, indent=2))
    # Capture succeeds when the evidence is saved; missing acceptance is expressed in report.json.
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    capture = commands.add_parser("capture", help="只订阅 RGB/CameraInfo，调用只读设备信息接口，保存新证据目录")
    capture.add_argument("--output", type=Path, required=True, help="须为不存在的新目录；不会覆盖旧证据")
    capture.add_argument("--version", required=True, help="不可复用的采集版本名")
    capture.add_argument("--duration", type=float, default=8)
    capture.add_argument("--contract", type=Path, default=ROOT / "config/camera_training_contract.v1.json")
    capture.add_argument("--camera-map", type=Path, default=ROOT / "config/camera_map.json")
    capture.add_argument("--baseline", type=Path, default=ROOT / "config/collection_camera_info_snapshot.20261105.json")
    verify = commands.add_parser("verify", help="无 ROS 依赖：校验完整性并重算冻结报告；通过只代表证据包一致")
    verify.add_argument("package", type=Path)
    template = commands.add_parser("mount-review-template", help="生成已绑定来源哈希但尚未通过复核的安装记录模板")
    template.add_argument("package", type=Path)
    template.add_argument("--output", required=True, type=Path, help="证据包目录之外、不存在的新 JSON 文件")
    check = commands.add_parser("audit", help="无 ROS 依赖：输出相机训练证据状态；合格退出 0，否则退出 2")
    check.add_argument("package", type=Path)
    check.add_argument("--mount-review", type=Path, help="独立保存、绑定当前画面与安装批次的人工复核 JSON")
    args = parser.parse_args()
    try:
        if args.command == "capture":
            return capture_command(args)
        contract, mapping, baseline, capture = load_package(args.package)
        if args.command == "mount-review-template":
            if args.output.resolve().is_relative_to(args.package.resolve()):
                raise ValueError("安装复核记录必须独立保存，不能改动已冻结的证据包")
            write_json(args.output, mount_review_template(capture, digest(args.package / "capture.json")))
            print(json.dumps({"output": str(args.output.resolve()), "reviewed": False}, ensure_ascii=False))
            return 0
        if args.command == "verify":
            print(json.dumps({"integrity_verified": True, "geometry_verified": False,
                              "manifest_sha256": digest(args.package / "manifest.json")}, ensure_ascii=False))
            return 0
        review = read_json(args.mount_review) if args.mount_review else None
        report = audit(contract, mapping, baseline, capture, digest(args.package / "capture.json"), review)
        if args.mount_review:
            report["mount_review_file_sha256"] = digest(args.mount_review)
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report["training_camera_evidence_ready"] else 2
    except (OSError, ValueError, KeyError, TypeError, ImportError) as exc:
        print(f"相机证据失败：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
