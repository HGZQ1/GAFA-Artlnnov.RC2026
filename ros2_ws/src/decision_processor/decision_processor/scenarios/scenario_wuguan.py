"""
scenario_wuguan.py
武馆场景决策逻辑
标签映射从 config.WUGUAN_CLASS_LABELS 读取
"""
from .base_scenario import BaseScenario
from ..config import (
    WUGUAN_VALID_CLASSES,
    WUGUAN_CONF_MIN,
    STOP_DISTANCE_M,
)


class ScenarioWuguan(BaseScenario):
    """武馆场景：识别并拾取端头"""

    def __init__(self):
        self.valid_classes = set(WUGUAN_VALID_CLASSES)
        self.min_confidence = WUGUAN_CONF_MIN

    def set_target_class(self, class_id: int):
        """只抓指定类别"""
        if class_id in WUGUAN_VALID_CLASSES:
            self.valid_classes = {class_id}

    def set_all_classes(self):
        self.valid_classes = set(WUGUAN_VALID_CLASSES)

    def evaluate_target(self, class_id, confidence, distance,
                        base_x, base_y) -> str:
        if class_id not in self.valid_classes:
            return 'IGNORE'
        if confidence < self.min_confidence:
            return 'IGNORE'
        if distance > STOP_DISTANCE_M:
            return 'APPROACH'
        return 'PICK'