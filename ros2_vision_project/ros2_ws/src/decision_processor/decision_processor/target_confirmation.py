"""
target_confirmation.py
连续帧确认算法，过滤 YOLO 误检
"""


class TargetConfirmation:
    """
    目标需要连续出现 confirm_frames 帧才被确认为有效目标
    目标连续消失 lost_frames 帧才被认定为丢失
    这样可以有效过滤单帧误检和短暂遮挡
    """

    def __init__(self, confirm_frames: int = 3,
                 lost_frames: int = 5):
        """
        Args:
            confirm_frames: 连续出现几帧才确认目标（建议3帧）
            lost_frames:    连续消失几帧才认定目标丢失（建议5帧）
        """
        self.confirm_frames = confirm_frames
        self.lost_frames    = lost_frames

        self._seen_count    = 0
        self._lost_count    = 0
        self.is_confirmed   = False

    def update(self, detected: bool) -> bool:
        """
        每帧调用一次
        Args:
            detected: 本帧是否检测到目标
        Returns:
            is_confirmed: 目标是否被确认有效
        """
        if detected:
            self._seen_count += 1
            self._lost_count  = 0
            if self._seen_count >= self.confirm_frames:
                self.is_confirmed = True
        else:
            self._lost_count += 1
            self._seen_count  = 0
            if self._lost_count >= self.lost_frames:
                self.is_confirmed = False

        return self.is_confirmed

    def reset(self):
        self._seen_count  = 0
        self._lost_count  = 0
        self.is_confirmed = False


def select_best_target(detections: list,
                       distances: list,
                       conf_threshold: float = 0.5) -> tuple | None:
    """
    从多个检测目标中选出最优目标
    策略：过滤置信度不足的，再选距离最近的
    
    Returns:
        (detection_dict, distance) 或 None
    """
    valid = [
        (det, dist)
        for det, dist in zip(detections, distances)
        if dist > 0 and det['confidence'] >= conf_threshold
    ]
    if not valid:
        return None
    # 选距离最近的目标（比赛中通常优先处理最近目标）
    return min(valid, key=lambda pair: pair[1])