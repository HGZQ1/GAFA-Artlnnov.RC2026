"""
meilin_path_planner.py
梅林 3×4 方块网格 BFS 路径规划器

规则:
  - 入口: 1,2,3    出口: 10,11,12
  - 只能前进(row+1) / 左(col-1) / 右(col+1), 不能后退
  - 真KFS可从其4方向相邻台阶拾取，无需进入KFS台阶本身
  - 只有路径必须经过该台阶时才将真KFS台阶纳入路径
  - 假KFS所在方块为障碍 (不可通行)

网格编号:
  1  2  3
  4  5  6
  7  8  9
  10 11 12
"""

import math
from collections import deque
from .config import (
    BLOCK_CENTERS, BLOCK_HEIGHTS_MM, BLOCK_GRID,
    MEILIN_ENTRY_BLOCKS, MEILIN_EXIT_BLOCKS,
    CLIMB_1, DESCEND_1,
    ENTRY_CLIMB_CMD, EXIT_DESCEND_CMD,
    MERLIN_TRIGGER_FROM_CENTER_M,
)


# ═══════════════════════════════════════
#   有向邻接表: 前进 / 左 / 右 (不能后退)
# ═══════════════════════════════════════

def _build_adjacency() -> dict:
    adj = {}
    for bid in range(1, 13):
        row, col = BLOCK_GRID[bid]
        neighbors = []
        for other in range(1, 13):
            r2, c2 = BLOCK_GRID[other]
            if r2 == row + 1 and c2 == col:      # 前进
                neighbors.append(other)
            elif r2 == row and c2 == col - 1:     # 向左
                neighbors.append(other)
            elif r2 == row and c2 == col + 1:     # 向右
                neighbors.append(other)
        adj[bid] = neighbors
    return adj


ADJACENCY = _build_adjacency()


def _all_neighbors(block: int) -> set:
    """4方向全邻（含上下左右，不受移动方向限制），用于拾取覆盖判断。"""
    r, c = BLOCK_GRID[block]
    result = set()
    for bid, (r2, c2) in BLOCK_GRID.items():
        if abs(r2 - r) + abs(c2 - c) == 1:
            result.add(bid)
    return result


# ═══════════════════════════════════════
#   覆盖感知 BFS
# ═══════════════════════════════════════

def _plan_with_coverage(entry: int, real_kfs: list, obstacles: set) -> list:
    """
    带 KFS 覆盖追踪的 BFS。

    真KFS无需强制经过：路过其4方向相邻台阶或台阶本身时即视为"已覆盖"（可拾取）。
    目标：所有真KFS被覆盖且到达出口台阶。

    例1: KFS=5, 路径经过4 → 4是5的右邻 → 在4处侧向拾取5，路径无需进入5
    例2: KFS=9, 路径经过6 → 6是9的上邻(前向) → 在6处前向拾取9，再继续经过9到12
    """
    exits = set(MEILIN_EXIT_BLOCKS) - obstacles

    if not real_kfs:
        return _bfs_simple(entry, exits, obstacles)

    kfs_list = list(real_kfs)
    n = len(kfs_list)

    # 预计算：每个台阶能覆盖哪些 KFS 的下标
    # 覆盖条件：台阶 blk 是 KFS 的4方向相邻或台阶本身（排除障碍）
    block_covers: dict = {}
    for idx, kfs in enumerate(kfs_list):
        coverable = (_all_neighbors(kfs) | {kfs}) - obstacles
        for blk in coverable:
            block_covers.setdefault(blk, set()).add(idx)

    # BFS 状态: (当前台阶, 已覆盖KFS下标集合)
    init_mask = frozenset(block_covers.get(entry, set()))
    init_state = (entry, init_mask)
    queue: deque = deque([(init_state, [entry])])
    visited: set = {init_state}

    while queue:
        (block, covered), path = queue.popleft()

        if block in exits and len(covered) == n:
            return path

        for nb in ADJACENCY.get(block, []):
            if nb in obstacles:
                continue
            new_covered = covered | frozenset(block_covers.get(nb, set()))
            new_state = (nb, new_covered)
            if new_state not in visited:
                visited.add(new_state)
                queue.append((new_state, path + [nb]))

    return []


def _bfs_simple(start: int, goals: set, obstacles: set) -> list:
    """无KFS约束的普通 BFS（无真KFS时使用）。"""
    if start in goals:
        return [start]
    queue: deque = deque([(start, [start])])
    visited = {start}
    while queue:
        node, path = queue.popleft()
        for nb in ADJACENCY.get(node, []):
            if nb in visited or nb in obstacles:
                continue
            new_path = path + [nb]
            if nb in goals:
                return new_path
            visited.add(nb)
            queue.append((nb, new_path))
    return []


# ═══════════════════════════════════════
#   公共接口
# ═══════════════════════════════════════

def plan_path(entry_block: int, real_kfs: list, fake_kfs: list) -> list:
    """
    规划梅林路径。

    真KFS可从4方向相邻台阶侧向/前向拾取，无需进入KFS台阶本身。
    只有当路径本就需要经过KFS台阶（如绕行不可行）时才将其纳入路径。

    Args:
        entry_block: 首选入口方块 (1/2/3)
        real_kfs:    真KFS所在方块列表
        fake_kfs:    假KFS所在方块列表 (障碍，不可通行)

    Returns:
        方块列表 [entry, ..., exit]，空列表=无可行路径
    """
    obstacles = set(fake_kfs)

    if entry_block not in obstacles:
        entries = [entry_block] + [e for e in MEILIN_ENTRY_BLOCKS
                                   if e != entry_block and e not in obstacles]
    else:
        entries = [e for e in MEILIN_ENTRY_BLOCKS if e not in obstacles]

    best = None
    for entry in entries:
        path = _plan_with_coverage(entry, real_kfs, obstacles)
        if path and (best is None or len(path) < len(best)):
            best = path

    return best or []


def get_pickup_info(path: list, real_kfs: list) -> dict:
    """
    为每个真KFS确定拾取台阶（路径中第一个与KFS 4方向相邻或等于KFS本身的台阶）。

    机器人到达该台阶后，面向KFS方块方向执行对齐和拾取。

    例1: path=[1,4,7,10], KFS=5 → 4是5的相邻台阶 → pickup[5]=4（侧向拾取）
    例2: path=[3,6,9,12], KFS=9 → 6是9的前邻台阶 → pickup[9]=6（前向拾取，拾取后经过9）

    Returns:
        {kfs_block: pickup_from_block}
    """
    pickup = {}
    for kfs in real_kfs:
        coverage_zone = _all_neighbors(kfs) | {kfs}
        for block in path:
            if block in coverage_zone:
                pickup[kfs] = block
                break
        if kfs not in pickup and path:
            pickup[kfs] = path[-1]  # 兜底
    return pickup


def get_transition_climb(from_block: int, to_block: int) -> int:
    """
    计算两个相邻方块之间的爬升/下降指令。

    Returns:
        CLIMB_1 / DESCEND_1 / 0
    """
    h_from = BLOCK_HEIGHTS_MM.get(from_block, 0)
    h_to   = BLOCK_HEIGHTS_MM.get(to_block, 0)
    diff = h_to - h_from
    if diff > 50:
        return CLIMB_1
    elif diff < -50:
        return DESCEND_1
    return 0


def compute_trigger_point(from_block: int, to_block: int) -> tuple:
    """
    计算爬升/下降触发点（距当前方块中心 MERLIN_TRIGGER_FROM_CENTER_M 处，朝向目标方块）。

    Returns:
        (x, y, yaw)
    """
    fx, fy = BLOCK_CENTERS[from_block]
    tx, ty = BLOCK_CENTERS[to_block]
    dx, dy = tx - fx, ty - fy
    dist = math.sqrt(dx * dx + dy * dy)
    if dist < 0.01:
        return fx, fy, 0.0
    nx, ny = dx / dist, dy / dist
    px = fx + MERLIN_TRIGGER_FROM_CENTER_M * nx
    py = fy + MERLIN_TRIGGER_FROM_CENTER_M * ny
    yaw = math.atan2(dy, dx)
    return px, py, yaw


def get_block_at(x: float, y: float) -> int:
    """判断坐标 (x,y) 在哪个方块上，返回方块标签或 0。"""
    from .config import BLOCK_HALF_SIZE_M
    for bid, (cx, cy) in BLOCK_CENTERS.items():
        if abs(x - cx) <= BLOCK_HALF_SIZE_M and abs(y - cy) <= BLOCK_HALF_SIZE_M:
            return bid
    return 0
