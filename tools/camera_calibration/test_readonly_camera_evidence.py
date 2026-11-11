"""验证证据工具不会把缺失、混淆、篡改或未复核的来源升级为验收通过。"""

from copy import deepcopy
import hashlib
from pathlib import Path
import shutil
from types import SimpleNamespace

import pytest

import readonly_camera_evidence as evidence


@pytest.fixture
def inputs():
    contract = evidence.read_json(evidence.ROOT / "config/camera_training_contract.v1.json")
    mapping = evidence.read_json(evidence.ROOT / "config/camera_map.json")
    baseline = evidence.read_json(evidence.ROOT / "config/collection_camera_info_snapshot.20261105.json")
    capture = {"schema_version": 1, "version_id": "synthetic-test-only",
               "ended_at": "2026-01-01T00:00:00+00:00", "cameras": {}, "usb_devices": []}
    review = {"capture_sha256": "a" * 64, "reviewer": "合成测试",
              "reviewed_at": "2026-01-01T00:01:00+00:00", "installation_epoch_id": "synthetic-only",
              "method": "visual_role_confirmation", "cameras": {}}
    for role in evidence.ROLES:
        info = {key: deepcopy(baseline["cameras"][role][key]) for key in evidence.INFO_KEYS}
        infos, images = [], []
        for index in range(10):
            infos.append({**info, "header_stamp_ns": 1_000_000_000 + 33_333_333 * index})
            images.append({"header_stamp_ns": infos[-1]["header_stamp_ns"], "frame_id": info["frame_id"],
                           "width": 640, "height": 480, "encoding": "rgb8", "step": 1920,
                           "is_bigendian": 0, "data_size": 640 * 480 * 3})
        capture["cameras"][role] = {
            "device": {"serial": mapping[role]["serial"], "success": True},
            "publishers": {"rgb": [contract["cameras"][role]["node"]],
                           "info": [contract["cameras"][role]["node"]]},
            "infos": infos, "images": images,
            "sample": {"image": images[-1], "info": infos[-1], "rgb_data_sha256": "b" * 64,
                       "rgb_png_sha256": "c" * 64}}
        capture["usb_devices"].append({"serial": mapping[role]["usb_serial"]})
        review["cameras"][role] = {"serial": mapping[role]["serial"], "confirmed": True,
                                   "rgb_png_sha256": "c" * 64, "observation": "仅用于合成测试"}
    return contract, mapping, baseline, capture, review


def run_audit(inputs, review=True):
    contract, mapping, baseline, capture, reviewed = inputs
    return evidence.audit(contract, mapping, baseline, capture, "a" * 64, reviewed if review else None)


def test_ros_numeric_scalars_are_normalized_before_online_and_offline_audit(inputs):
    class ForeignScalar(float):
        pass

    info = deepcopy(inputs[3]["cameras"]["top"]["infos"][0])
    values = {key: [ForeignScalar(value) for value in info[key]] for key in ("d", "k", "r", "p")}
    message = SimpleNamespace(**values, width=640, height=480, distortion_model="plumb_bob",
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0), frame_id=info["frame_id"]),
        binning_x=0, binning_y=0,
        roi=SimpleNamespace(x_offset=0, y_offset=0, width=0, height=0, do_rectify=False))
    normalized = evidence.info_dict(message)
    assert all(type(value) is float for key in ("d", "k", "r", "p") for value in normalized[key])
    assert normalized == evidence.json.loads(evidence.canonical(normalized))
    assert not evidence.check_info(normalized, {"width": 640, "height": 480, "frame_id": info["frame_id"]})[0]


def test_rgb_only_review_can_pass_with_distortion_warning_without_claiming_geometry(inputs):
    report = run_audit(inputs)
    assert report["training_camera_evidence_ready"]
    assert report["cameras"]["left_wrist"]["warnings"]
    assert report["cameras"]["right_wrist"]["warnings"]
    assert not report["intrinsic_geometry_verified"]
    assert not report["extrinsic_geometry_verified"]
    assert not report["motion_planning_geometry_ready"]
    assert report["episode_training_eligible"] == "not_evaluated"


def test_historical_position_flag_does_not_confirm_current_installation(inputs):
    assert inputs[1]["position_verified"] is True
    report = run_audit(inputs, review=False)
    assert all(camera["automated_pass"] for camera in report["cameras"].values())
    assert not report["training_camera_evidence_ready"]


def test_generated_review_template_binds_sources_but_cannot_pass(inputs):
    review = evidence.mount_review_template(inputs[3], "a" * 64)
    assert review["capture_sha256"] == "a" * 64
    assert all(not item["confirmed"] for item in review["cameras"].values())
    assert all(item["rgb_png_sha256"] == "c" * 64 for item in review["cameras"].values())
    inputs = (*inputs[:4], review)
    assert not run_audit(inputs)["training_camera_evidence_ready"]


@pytest.mark.parametrize("change", ["wrong_serial", "missing_device", "duplicate_publisher", "wrong_node",
                                   "wrong_usb", "missing_info", "changed_intrinsics", "changed_roi",
                                   "wrong_frame", "bad_payload", "backward_stamp", "nonfinite_k", "unpaired_sample"])
def test_rejects_untrustworthy_camera_sources(inputs, change):
    camera = inputs[3]["cameras"]["left_wrist"]
    if change == "wrong_serial":
        camera["device"]["serial"] = inputs[1]["right_wrist"]["serial"]
    elif change == "missing_device":
        camera["device"]["success"] = False
    elif change == "duplicate_publisher":
        camera["publishers"]["rgb"] *= 2
    elif change == "wrong_node":
        camera["publishers"]["info"] = ["/unexpected"]
    elif change == "wrong_usb":
        inputs[3]["usb_devices"] = []
    elif change == "missing_info":
        camera["infos"] = []
    elif change == "changed_intrinsics":
        for info in camera["infos"]:
            info["k"] = list(info["k"])
            info["k"][0] += 0.1
    elif change == "changed_roi":
        for info in camera["infos"]:
            info["roi"] = {"x_offset": 4}
    elif change == "wrong_frame":
        camera["images"][0]["frame_id"] = "depth_optical_frame"
    elif change == "bad_payload":
        camera["images"][0]["data_size"] -= 1
    elif change == "backward_stamp":
        camera["images"][1]["header_stamp_ns"] = camera["images"][0]["header_stamp_ns"]
    elif change == "nonfinite_k":
        # Canonical serialization must refuse non-finite evidence instead of emitting JSON NaN.
        camera["infos"][0]["k"][0] = float("nan")
        with pytest.raises(ValueError):
            run_audit(inputs)
        return
    elif change == "unpaired_sample":
        camera["sample"]["info"] = {**camera["infos"][-1], "header_stamp_ns": 900_000_000_000}
    report = run_audit(inputs)
    assert not report["training_camera_evidence_ready"]
    assert not report["cameras"]["left_wrist"]["automated_pass"]


@pytest.mark.parametrize("change", ["stale_capture", "stale_image", "no_reviewer", "no_epoch", "before_capture"])
def test_mount_review_is_bound_to_this_capture_and_image(inputs, change):
    review = inputs[4]
    if change == "stale_capture":
        review["capture_sha256"] = "old"
    elif change == "stale_image":
        review["cameras"]["top"]["rgb_png_sha256"] = "old"
    elif change == "no_reviewer":
        review["reviewer"] = ""
    elif change == "no_epoch":
        review["installation_epoch_id"] = ""
    elif change == "before_capture":
        review["reviewed_at"] = "2025-12-31T23:00:00+00:00"
    report = run_audit(inputs)
    assert not report["training_camera_evidence_ready"]
    assert report["mount_review_errors"]


def make_package(directory, inputs):
    contract, mapping, baseline, capture, _ = inputs
    directory.mkdir()
    for source, name in ((evidence.ROOT / "config/camera_training_contract.v1.json", "contract.json"),
                         (evidence.ROOT / "config/camera_map.json", "camera_map.json"),
                         (evidence.ROOT / "config/collection_camera_info_snapshot.20261105.json", "baseline.json"),
                         (Path(evidence.__file__), "collector.py")):
        shutil.copyfile(source, directory / name)
    capture["collector_sha256"] = evidence.digest(directory / "collector.py")
    for role in evidence.ROLES:
        sample = capture["cameras"][role]["sample"]
        data = b"\x7f\x80\x81" * (640 * 480)
        png = evidence.png_rgb(sample["image"], data)
        (directory / f"{role}.rgb.bin").write_bytes(data)
        (directory / f"{role}.rgb.png").write_bytes(png)
        sample["rgb_data_sha256"] = hashlib.sha256(data).hexdigest()
        sample["rgb_png_sha256"] = hashlib.sha256(png).hexdigest()
    evidence.write_json(directory / "capture.json", capture)
    report = evidence.audit(contract, mapping, baseline, capture, evidence.digest(directory / "capture.json"))
    evidence.write_json(directory / "report.json", report)
    evidence.write_json(directory / "manifest.json", {"schema_version": 1, "version_id": capture["version_id"],
        "files": {path.name: evidence.digest(path) for path in directory.iterdir()}})
    return directory


def test_offline_package_replay_and_tamper_detection(tmp_path, inputs):
    package = make_package(tmp_path / "evidence", inputs)
    evidence.load_package(package)
    (package / "top.rgb.bin").write_bytes(b"corrupt")
    with pytest.raises(ValueError, match="哈希不符"):
        evidence.load_package(package)


def test_rehashed_false_report_still_fails_offline_recomputation(tmp_path, inputs):
    package = make_package(tmp_path / "evidence", inputs)
    report = evidence.read_json(package / "report.json")
    report["intrinsic_geometry_verified"] = True
    (package / "report.json").write_bytes(evidence.canonical(report))
    manifest = evidence.read_json(package / "manifest.json")
    manifest["files"]["report.json"] = evidence.digest(package / "report.json")
    (package / "manifest.json").write_bytes(evidence.canonical(manifest))
    with pytest.raises(ValueError, match="离线重算"):
        evidence.load_package(package)


def test_manifest_cannot_reference_outside_package(tmp_path):
    (tmp_path / "outside").write_text("source")
    package = tmp_path / "package"
    package.mkdir()
    with pytest.raises(ValueError, match="路径无效"):
        evidence.checked_file(package, "../outside")
