"""
base_scenario.py
场景基类，定义所有比赛场景的通用接口
"""
from abc import ABC, abstractmethod


class BaseScenario(ABC):
    """
    比赛场景基类

    每个场景需要实现 evaluate_target() 方法，
    根据检测到的目标类别、置信度、距离等信息，
    返回决策动作字符串。
    """

    @abstractmethod
    def evaluate_target(self,
                        class_id: int,
                        confidence: float,
                        distance: float,
                        base_x: float,
                        base_y: float) -> str:
        """
        评估检测到的目标并返回决策动作

        Args:
            class_id:    YOLO 检测的类别ID
            confidence:  检测置信度 (0.0~1.0)
            distance:    目标距离（米，已做坡度补偿）
            base_x:      目标在底盘坐标系的 x 坐标（米，前方为正）
            base_y:      目标在底盘坐标系的 y 坐标（米，左方为正）

        Returns:
            决策动作字符串：
              'APPROACH' - 接近目标
              'PICK'     - 执行拾取
              'AVOID'    - 规避（危险目标）
              'IGNORE'   - 忽略（不相关目标或置信度不足）
        """
        raise NotImplementedError