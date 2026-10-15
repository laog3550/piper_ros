"""运行：python3 -m unittest discover -s tools/vla_contract -p 'test_openpi05_offline.py'。"""

import copy
import tempfile
import unittest
from pathlib import Path

from PIL import Image

from openpi05_offline import preflight
from validate_contract import synthetic


class OfflinePreflightTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.data = synthetic()
        self.data.pop("action_chunk")
        for name, frame in self.data["observation"]["images"].items():
            frame["uri"] = f"{name}.png"
            Image.new("RGB", (640, 480)).save(self.root / frame["uri"])
        self.now = self.data["observation"]["observation_timestamp_ns"] + 5_000_000

    def check(self, data=None, **kwargs):
        options = dict(now_ns=self.now, max_age_ns=10_000_000,
                       max_skew_ns=10_000_000, image_dir=self.root)
        options.update(kwargs)
        return preflight(data or self.data, **options)

    def test_validated_but_never_action_chunk(self):
        result = self.check()
        self.assertEqual(result["status"], "input_validated_bridge_required")
        self.assertNotIn("targets", result)
        self.assertEqual(result["age_ns"], 5_000_000)

    def test_reject_stale_and_future(self):
        for now in (self.now + 20_000_000, self.now - 20_000_000):
            with self.subTest(now=now), self.assertRaises(ValueError):
                self.check(now_ns=now)

    def test_reject_skew_version_and_bad_image(self):
        with self.assertRaises(ValueError):
            self.check(max_skew_ns=1)
        changed = copy.deepcopy(self.data)
        changed["observation"]["contract_version"] = "other"
        with self.assertRaises(ValueError):
            self.check(changed)
        Image.new("L", (640, 480)).save(self.root / "top_rgb.png")
        with self.assertRaises(ValueError):
            self.check()

    def test_reject_bimanual(self):
        changed = synthetic("BIMANUAL")
        changed.pop("action_chunk")
        with self.assertRaises(ValueError):
            self.check(changed)


if __name__ == "__main__":
    unittest.main()
