"""真实来源与人工审核候选导出的合成验收。"""

import hashlib
import json
from pathlib import Path
import sqlite3
import tempfile
import unittest

from qwen_lora_candidates import build_candidates, export_candidates
from runtime_trace import TraceError, TraceSession


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class CandidateTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.episode = self.root / "episode-1"
        bag = self.episode / "bag"
        bag.mkdir(parents=True)
        self.db = bag / "data_0.db3"
        with sqlite3.connect(self.db) as connection:
            connection.executescript("CREATE TABLE topics(id INTEGER PRIMARY KEY, type TEXT);"
                                     "CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER);"
                                     "INSERT INTO topics VALUES(1, 'sensor_msgs/msg/Image');"
                                     "INSERT INTO messages VALUES(7, 1);")
        self.uri = f"rosbag2://{self.db}#message=7"
        self.episode_manifest = self.episode / "episode.json"
        self.episode_manifest.write_text(json.dumps({"episode_id": "episode-1", "state": "saved",
                                                      "rosbag_relative_path": "bag"}), encoding="utf-8")
        self.trace = TraceSession(self.root / "traces", source_kind="real", run_id="run-1",
                                  models={"planner": "qwen-test", "policy": "pi-test"},
                                  links={"episode_id": "episode-1", "recovery_id": "a" * 32},
                                  episode_manifest_sha256=digest(self.episode_manifest))
        self.trace.record("plan_request", {"instruction": "把香蕉放入盒子",
                           "observation": {"images": {"top_rgb": {"uri": self.uri}}}},
                          ids={"request_id": "request-1"})
        self.trace.record("plan_result", {"plan": {"steps": [{"skill": "pick"}]}},
                          ids={"request_id": "request-1"})
        self.trace.record("plan_preflight", {"accepted": True})
        self.trace.finish("DONE")
        self.recovery = self.root / "recovery.json"
        self.recovery.write_text(json.dumps({"recovery_id": "a" * 32, "episode_id": "episode-1",
                                             "status": "complete", "training_approved": False}),
                                 encoding="utf-8")
        self.review = self.root / "review.json"
        self.review_data = {
            "schema_version": "piper_qwen_lora_review/1.0", "decision": "approve_candidate",
            "run_id": "run-1", "trace_manifest_sha256": digest(self.trace.manifest_path),
            "episode_manifest_sha256": digest(self.episode_manifest),
            "recovery_manifest_sha256": digest(self.recovery),
            "reviewer_id": "reviewer-1", "reviewed_at_unix_ns": 1,
            "approved_plan_result_seqs": [2], "verified_image_uris": [self.uri],
            "verified_image_sha256": {self.uri: digest(self.db)}}
        self.save_review()

    def save_review(self):
        self.review.write_text(json.dumps(self.review_data), encoding="utf-8")

    def build(self):
        return build_candidates(self.trace.path, self.episode_manifest, self.review, self.recovery)

    def test_reviewed_real_plan_exports_candidate_only(self):
        value = self.build()
        self.assertEqual(value["status"], "candidate_only")
        self.assertFalse(value["training_eligible"])
        self.assertFalse(value["real_success_verified"])
        self.assertEqual(value["recovery_id"], "a" * 32)
        self.assertEqual(value["samples"][0]["plan_result_seq"], 2)
        target = self.root / "candidate.json"
        export_candidates(self.trace.path, self.episode_manifest, self.review, target, self.recovery)
        self.assertEqual(json.loads(target.read_text())["status"], "candidate_only")
        with self.assertRaises(FileExistsError):
            export_candidates(self.trace.path, self.episode_manifest, self.review, target, self.recovery)

    def test_episode_or_trace_tamper_is_rejected(self):
        self.episode_manifest.write_text('{}', encoding="utf-8")
        with self.assertRaises(TraceError):
            self.build()

    def test_missing_or_wrong_review_is_rejected(self):
        self.review_data["decision"] = "reject"
        self.save_review()
        with self.assertRaises(TraceError):
            self.build()
        self.review_data["decision"] = "approve_candidate"
        self.review_data["verified_image_uris"] = []
        self.save_review()
        with self.assertRaises(TraceError):
            self.build()

    def test_image_and_recovery_binding_are_rejected(self):
        self.db.unlink()
        with self.assertRaises(TraceError):
            self.build()
        with sqlite3.connect(self.db) as connection:
            connection.executescript("CREATE TABLE topics(id INTEGER PRIMARY KEY, type TEXT);"
                                     "CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER);")
        with self.assertRaises(TraceError):
            self.build()
        with self.assertRaises(TraceError):
            build_candidates(self.trace.path, self.episode_manifest, self.review)

    def test_synthetic_and_incomplete_trace_are_rejected(self):
        synthetic = TraceSession(self.root / "synthetic", source_kind="synthetic",
                                 models={"planner": "fixture", "policy": "fixture"}, links={})
        synthetic.finish("DONE")
        with self.assertRaises(TraceError):
            build_candidates(synthetic.path, self.episode_manifest, self.review)
        incomplete = TraceSession(self.root / "incomplete", source_kind="real",
                                  models={"planner": "qwen", "policy": "pi"},
                                  links={"episode_id": "episode-1"},
                                  episode_manifest_sha256=digest(self.episode_manifest))
        with self.assertRaises(TraceError):
            build_candidates(incomplete.path, self.episode_manifest, self.review)


if __name__ == "__main__":
    unittest.main()
