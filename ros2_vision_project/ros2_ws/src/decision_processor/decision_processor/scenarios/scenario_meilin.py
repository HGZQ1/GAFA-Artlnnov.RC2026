"""
scenario_meilin.py
梅林场景决策逻辑

决策逻辑：
  - REAL_N → 可拾取（距离近→PICK，远→APPROACH）
  - FAKE_N / R1_* → 规避（发出AVOID指令触发绕道）
  - 其他类别 → 忽略

标签解析：从 config.MEILIN_CLASS_LABELS 读取 class_id→标签名映射
"""
import re
from .base_scenario import BaseScenario
from ..config import (
    MEILIN_CLASS_LABELS,
    MEILIN_REAL_PREFIX,
    MEILIN_FAKE_PREFIX,
    MEILIN_R1_LABELS,
    MEILIN_CONF_MIN,
    MEILIN_STOP_DIST,
)


class ScenarioMeilin(BaseScenario):
    """梅林场景：收集真KFS，规避假KFS和R1方块"""

    # 从标签名提取方块编号的正则：REAL_3 → 3, FAKE_12 → 12
    _BLOCK_NUM_RE = re.compile(r'_(\d+)$')

    def __init__(self):
        self.conf_min = MEILIN_CONF_MIN
        # 最近一次识别到的危险方块（供navigator使用）
        self.last_danger_block = None   # (block_num, label)
        self.last_real_block   = None   # (block_num, label)

    @classmethod
    def parse_label(cls, class_id: int) -> str:
        """class_id → 标签名，未知返回空字符串"""
        return MEILIN_CLASS_LABELS.get(class_id, '')

    @classmethod
    def extract_block_num(cls, label: str):
        """从标签名提取方块编号，失败返回None"""
        m = cls._BLOCK_NUM_RE.search(label)
        return int(m.group(1)) if m else None

    @classmethod
    def is_real(cls, label: str) -> bool:
        return label.startswith(MEILIN_REAL_PREFIX)

    @classmethod
    def is_fake(cls, label: str) -> bool:
        return label.startswith(MEILIN_FAKE_PREFIX)

    @classmethod
    def is_r1(cls, label: str) -> bool:
        return label in MEILIN_R1_LABELS

    @classmethod
    def is_dangerous(cls, label: str) -> bool:
        """假方块或R1都是危险的"""
        return cls.is_fake(label) or cls.is_r1(label)

    def evaluate_target(self, class_id, confidence, distance,
                        base_x, base_y) -> str:
        label = self.parse_label(class_id)
        if not label:
            return 'IGNORE'

        # 危险目标（假方块 / R1）→ 规避
        if self.is_dangerous(label):
            block_num = self.extract_block_num(label)
            self.last_danger_block = (block_num, label)
            return 'AVOID'

        # 真方块 → 正常决策
        if self.is_real(label):
            if confidence < self.conf_min:
                return 'IGNORE'
            block_num = self.extract_block_num(label)
            self.last_real_block = (block_num, label)

            if distance > MEILIN_STOP_DIST:
                return 'APPROACH'
            return 'PICK'

        return 'IGNORE'