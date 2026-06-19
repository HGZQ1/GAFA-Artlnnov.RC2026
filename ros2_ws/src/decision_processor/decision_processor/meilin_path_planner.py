"""
meilin_path_planner.py
梅林 3x4 方块网格 BFS 路径规划器

规则 (来源 4.0.md):
  - 入口: 1,2,3    出口: 10,11,12
  - 只能前进(row+1) / 左(col-1) / 右(col+1), 不能后退
  - 真KFS所在方块为必经点 (路径必须经过)
  - 假KFS所在方块为障碍 (不可通行)
  - 空方块可通行
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
#   有向邻接表: 前 / 左 / 右 (不能后退)
# ═══════════════════════════════════════

def _build_adjacency() -> dict:
    adj = {}
    for bid in range(1, 13):
        row, col = BLOCK_GRID[bid]
        neighbors = []
        for other in range(1, 13):
            r2, c2 = BLOCK_GRID[other]
            if r2 == row + 1 and c2 == col:
                neighbors.append(other)
            elif r2 == row and c2 == col - 1:
                neighbors.append(other)
            elif r2 == row and c2 == col + 1:
                neighbors.append(other)
        adj[bid] = neighbors
    return adj


ADJACENCY = _build_adjacency()


# ═══════════════════════════════════════
#   BFS 核心
# ═══════════════════════════════════════

def _bfs(start: int, goals: set, obstacles: set) -> list:
    if start in goals:
        return [start]
    queue = deque([(start, [start])])
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


def _chain_bfs(waypoints: list, obstacles: set) -> list:
    """BFS through a sequence of mandatory waypoints."""
    full = []
    for i in range(len(waypoints) - 1):
        seg = _bfs(waypoints[i], {waypoints[i + 1]}, obstacles)
        if not seg:
            return []
        full = full + seg[1:] if full else seg
    return full


# ═══════════════════════════════════════
#   公共接口
# ═══════════════════════════════════════

def _plan_from_entry(entry: int, real_kfs: list, obstacles: set) -> list:
    """从指定入口规划路径, 返回方块列表或空列表."""
    exits = set(MEILIN_EXIT_BLOCKS) - obstacles

    if not real_kfs:
        return _bfs(entry, exits, obstacles)

    if len(real_kfs) == 1:
        seg1 = _chain_bfs([entry, real_kfs[0]], obstacles)
        if not seg1:
            return []
        seg2 = _bfs(seg1[-1], exits, obstacles)
        return seg1 + seg2[1:] if seg2 else []

    best = None
    k1, k2 = real_kfs[0], real_kfs[1]
    for order in [(k1, k2), (k2, k1)]:
        seg = _chain_bfs([entry, order[0], order[1]], obstacles)
        if not seg:
            continue
        tail = _bfs(seg[-1], exits, obstacles)
        if not tail:
            continue
        path = seg + tail[1:]
        if best is None or len(path) < len(best):
            best = path
    return best or []


def plan_path(entry_block: int, real_kfs: list, fake_kfs: list) -> list:
    """
    规划梅林路径.

    Args:
        entry_block: 首选入口方块 (1/2/3)
        real_kfs:    真KFS所在方块标签列表 (必经点)
        fake_kfs:    假KFS所在方块标签列表 (障碍)

    Returns:
        方块标签列表 [entry, ..., exit], 空列表=无可行路径
    """
    obstacles = set(fake_kfs)

    # 确定入口尝试顺序: 首选 → 其他可用入口
    if entry_block not in obstacles:
        entries = [entry_block] + [e for e in MEILIN_ENTRY_BLOCKS
                                   if e != entry_block and e not in obstacles]
    else:
        entries = [e for e in MEILIN_ENTRY_BLOCKS if e not in obstacles]

    best = None
    for entry in entries:
        path = _plan_from_entry(entry, real_kfs, obstacles)
        if path and (best is None or len(path) < len(best)):
            best = path

    return best or []


def get_pickup_info(path: list, real_kfs: list) -> dict:
    """
    确定每个真KFS的拾取信息.

    Returns:
        {kfs_block: pickup_from_block}
        pickup_from_block 是 path 中 kfs_block 前面那个方块
    """
    pickup = {}
    for kfs in real_kfs:
        if kfs in path:
            idx = path.index(kfs)
            if idx > 0:
                pickup[kfs] = path[idx - 1]
    return pickup


def get_transition_climb(from_block: int, to_block: int) -> int:
    """
    计算两个相邻方块之间的爬升/下降指令.

    Returns:
        CLIMB_1 / DESCEND_1 / 0
    """
    h_from = BLOCK_HEIGHTS_MM.get(from_block, 0)
    h_to = BLOCK_HEIGHTS_MM.get(to_block, 0)
    diff = h_to - h_from
    if diff > 50:
        return CLIMB_1
    elif diff < -50:
        return DESCEND_1
    return 0


def compute_trigger_point(from_block: int, to_block: int) -> tuple:
    """
    计算爬升/下降触发点 (距当前方块边缘40cm).

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
    """判断坐标 (x,y) 在哪个方块上, 返回方块标签或 0."""
    from .config import BLOCK_HALF_SIZE_M
    for bid, (cx, cy) in BLOCK_CENTERS.items():
        if abs(x - cx) <= BLOCK_HALF_SIZE_M and abs(y - cy) <= BLOCK_HALF_SIZE_M:
            return bid
    return 0
