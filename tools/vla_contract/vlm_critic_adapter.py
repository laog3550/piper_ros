"""将离线 VLM 评估结果接入编排器，并始终要求人工复核。"""

from copy import deepcopy
from pathlib import Path

from PIL import Image

from tools.vlm_critic.critic import validate_input, validate_output


class OfflineVlmCriticAdapter:
    """sample_builder 提供执行前图像与几何摘要；evaluator 只返回离线结果。"""

    def __init__(self, sample_builder, evaluator, image_dir):
        self.sample_builder = sample_builder
        self.evaluator = evaluator
        self.image_dir = Path(image_dir).resolve()
        self.last_output = None

    def verify(self, skill, observation):
        self.last_output = None
        sample = validate_input(deepcopy(self.sample_builder(skill, observation)), self.image_dir)
        post_uri = observation["images"]["top_rgb"]["uri"]
        if "://" in post_uri:
            raise ValueError("VLM 评估需要不可变的本地图像")
        post_path = (self.image_dir / post_uri).resolve()
        if not post_path.is_relative_to(self.image_dir):
            raise ValueError("后帧路径越过指定目录")
        pre_path = Path(sample["pre_rgb"])
        if not pre_path.is_relative_to(self.image_dir) or pre_path == post_path:
            raise ValueError("前帧须位于指定目录且不同于后帧")
        with Image.open(post_path) as image:
            frame = observation["images"]["top_rgb"]
            if image.size != (frame["width"], frame["height"]):
                raise ValueError("后帧文件尺寸与观测不匹配")
        if (sample["sample_id"] != observation["observation_id"]
                or sample["skill_id"] != skill.name
                or sample["arm_mode"] != observation["arm_mode"]
                or sample["target_id"] != skill.target
                or sample["calibration_id"] != observation["calibration_version"]
                or sample["post_rgb"] != str(post_path)):
            raise ValueError("VLM 样本与当前技能或观测不匹配")
        output = validate_output(self.evaluator(sample), sample)
        self.last_output = output
        # 当前 critic 的唯一允许建议是 HUMAN_REVIEW；绝不映射成 SUCCESS/FAIL。
        return output["recommended_transition"]
