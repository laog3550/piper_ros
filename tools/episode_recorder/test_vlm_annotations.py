#!/usr/bin/env python3
"""VLM 规划与技能关联校验。运行：python3 -m unittest discover -s tools/episode_recorder -p 'test_vlm_annotations.py'。"""

import unittest

from vlm_annotations import validate_plan, validate_plan_link, validate_skill_context


class PlanLinkTests(unittest.TestCase):
    def setUp(self):
        self.plan = validate_plan({
            "long_task_description": "把杯子放入盒子", "object_ids": ["cup_1"],
            "location_ids": ["box_slot"],
            "steps": [{"skill": "grasp", "target_object_id": "cup_1",
                       "completion_criteria": "杯子离开桌面"},
                      {"skill": "place", "target_object_id": "cup_1",
                       "target_location_id": "box_slot", "completion_criteria": "杯子位于盒内"}],
        })
        self.context = validate_skill_context({
            "long_task_description": "把杯子放入盒子", "skill_instruction": "抓住杯子",
            "target_object_id": "cup_1", "expected_state_change": "杯子被夹持",
            "completion_criteria": "杯子离开桌面",
        })

    def test_matching_step(self):
        validate_plan_link(self.plan, self.context, "grasp")

    def test_reject_step_target_outside_scene_catalog(self):
        for key, value in (("target_object_id", "other_cup"),
                           ("target_location_id", "other_box")):
            with self.subTest(key=key):
                item = {"long_task_description": "把杯子放入盒子", "object_ids": ["cup_1"],
                        "location_ids": ["box_slot"],
                        "steps": [{"skill": "place", "target_object_id": "cup_1",
                                   "target_location_id": "box_slot", "completion_criteria": "杯子位于盒内"}]}
                item["steps"][0][key] = value
                with self.assertRaises(ValueError):
                    validate_plan(item)

    def test_reject_mismatched_step_fields(self):
        for key, value in (("long_task_description", "另一任务"),
                           ("target_object_id", "other_cup"),
                           ("target_location_id", "box_slot"),
                           ("completion_criteria", "杯子位于盒内")):
            with self.subTest(key=key):
                context = dict(self.context)
                context[key] = value
                with self.assertRaises(ValueError):
                    validate_plan_link(self.plan, context, "grasp")
        with self.assertRaises(ValueError):
            validate_plan_link(self.plan, self.context, "place")


if __name__ == "__main__":
    unittest.main()
