#!/usr/bin/env python3
"""
generate_field_map.py
根据 Robocon 2026 官方场地尺寸生成左半场（蓝方）Nav2 2D OccupancyGrid 地图。

坐标系: 原点在半场底部左下角 (不含围栏), X 向右, Y 向上, 单位 mm
Zone 3 (对抗区) 在底部, Zone 1 (武馆区) 在顶部

用法: python3 generate_field_map.py --output <输出目录> [--resolution 0.02]
"""
import argparse
import os
import numpy as np
from PIL import Image
import yaml

# ═══════════════════════════════════════════════════════════════
#                    场地常量 (单位: mm)
# ═══════════════════════════════════════════════════════════════

BORDER = 50                      # 围栏宽度
HALF_WIDTH = 6000                # 半场宽度
FIELD_HEIGHT = 12000             # 场地高度 (不含围栏)
MAP_WIDTH = HALF_WIDTH + 2 * BORDER    # 6100
MAP_HEIGHT = FIELD_HEIGHT + 2 * BORDER  # 12100

# ── 区域划分 (Y 坐标, 从底部算起) ──
ZONE3_Y0 = 0                    # 对抗区底
ZONE3_Y1 = 2700                 # 对抗区顶 = 梅林区底
ZONE2_Y0 = 2700                 # 梅林区底
ZONE2_Y1 = 10000                # 梅林区顶 = 武馆区底
ZONE1_Y0 = 10000                # 武馆区底
ZONE1_Y1 = 12000                # 武馆区顶

# ═══════ Zone 1: 武馆区 (150mm 平台) ═══════

# R2 启动区 (我方机器人, 可通行)
R2_START_X0 = 0
R2_START_Y0 = ZONE1_Y1 - 800     # 11200
R2_START_X1 = 800
R2_START_Y1 = ZONE1_Y1           # 12000

# R1 启动区 (队友/对方, 不可通行)
R1_START_X0 = HALF_WIDTH - 1000  # 5000
R1_START_Y0 = ZONE1_Y1 - 1000   # 11000
R1_START_X1 = HALF_WIDTH         # 6000
R1_START_Y1 = ZONE1_Y1           # 12000

# 端头架 (左侧棕色长方形, 不可通行但需靠近拾取)
# R2 需要从右侧接近
TIP_RACK_X0 = 0
TIP_RACK_Y0 = ZONE1_Y0 + 30     # 10030
TIP_RACK_X1 = 300
TIP_RACK_Y1 = TIP_RACK_Y0 + 970  # 11000

# 长杆架 (中间上方, 不可通行, R1 拾取用)
# 从图纸: 中央 3200mm 区域 = 1200+800+1200, 长杆架是中间的 800mm
POLE_RACK_CENTER_X = 875 + 1200 + 400  # ≈ 2475
POLE_RACK_X0 = 875 + 1200       # 2075
POLE_RACK_Y0 = ZONE1_Y1 - 500   # 11500
POLE_RACK_X1 = POLE_RACK_X0 + 800  # 2875
POLE_RACK_Y1 = ZONE1_Y1 - 200   # 11800

# ═══════ Zone 2: 梅林区 (50mm 平台) ═══════
# 所有方块都是可通行的! 机器人可以走上去
# 5 等分: 1200mm × 5 = 6000mm, 方块在中间 3 列

BLOCK_SIZE = 1200
BLOCK_GRID_COLS = 3
BLOCK_GRID_ROWS = 4

# 水平: 左通道(1200) + 3×方块(3600) + 右通道(1200) = 6000
BLOCK_COL_X = [1200, 2400, 3600]     # 各列左边缘 x

# 垂直: 上通道(1200) + 4×方块(4800) + 下通道(1300) = 7300
# Row 0 顶部 y = 10000 - 1200 = 8800
BLOCK_ROW_Y_TOP = [8800, 7600, 6400, 5200]   # 各行顶部 y
BLOCK_ROW_Y_BOT = [7600, 6400, 5200, 4000]   # 各行底部 y

# 方块高度 [row][col] (mm), 行从上到下
BLOCK_HEIGHTS = [
    [400, 200, 400],   # Row 0 (靠近武馆区)
    [600, 400, 200],   # Row 1
    [400, 600, 400],   # Row 2
    [200, 400, 200],   # Row 3 (靠近对抗区)
]

# ═══════ Zone 2 → Zone 3 边界 ═══════
# Zone 3 平台 450mm, Zone 2 平台 50mm → 400mm 落差
# 坡道是唯一入口: 1500×1500mm, 右侧

SLOPE_X0 = HALF_WIDTH - 1500     # 4500
SLOPE_X1 = HALF_WIDTH            # 6000
SLOPE_Y0 = ZONE3_Y1 - 1500      # 1200
SLOPE_Y1 = ZONE3_Y1             # 2700

# ═══════ Zone 3: 对抗区 (450mm 平台) ═══════

# 亚克力护栏 (10mm 厚 × 4000mm 长, 不可通行)
# 位于对抗区上部, 黄色废弃区上方紧贴
BARRIER_X0 = 300
BARRIER_X1 = 300 + 4000          # 4300
BARRIER_Y0 = 2350
BARRIER_Y1 = 2370                # 20mm (最小可绘制)

# 武器废弃区 (黄色, 不可通行, 在护栏下方)
DISPOSAL_X0 = 300
DISPOSAL_X1 = 300 + 1500 + 2000  # 3800 (两段: 1500+2000)
DISPOSAL_Y0 = 2050
DISPOSAL_Y1 = 2350               # 300mm 高

# 九宫格 KFS (左侧白色长方体, 不可通行但需靠近放置 KFS)
# 俯视 footprint: 深度 320mm × 宽度 1620mm
KFS_X0 = 0
KFS_X1 = 320
KFS_Y0 = 300                     # 距底部 300mm
KFS_Y1 = 300 + 1620              # 1920

# 重试区 (右下角深蓝方块, 可通行)
RETRY_X0 = HALF_WIDTH - 1000     # 5000
RETRY_Y0 = 0
RETRY_X1 = HALF_WIDTH            # 6000
RETRY_Y1 = 1200

# ═══════════════════════════════════════════════════════════════
#                    OccupancyGrid 像素值
# ═══════════════════════════════════════════════════════════════
FREE = 254
OCCUPIED = 0
UNKNOWN = 205
SLOPE_VAL = 240      # 浅灰, 坡道 (可通行)


class FieldMapGenerator:
    def __init__(self, resolution_mm=20):
        self.res = resolution_mm
        self.w = int(np.ceil(MAP_WIDTH / self.res))
        self.h = int(np.ceil(MAP_HEIGHT / self.res))

    def px(self, mm):
        return int(round(mm / self.res))

    def fill(self, grid, x0, y0, x1, y1, val):
        """用半场坐标 (mm) 填充矩形区域"""
        c0 = self.px(x0 + BORDER)
        c1 = self.px(x1 + BORDER)
        r1 = self.px(MAP_HEIGHT - (y0 + BORDER))
        r0 = self.px(MAP_HEIGHT - (y1 + BORDER))
        r0, r1 = max(0, min(r0, r1)), min(self.h, max(r0, r1))
        c0, c1 = max(0, min(c0, c1)), min(self.w, max(c0, c1))
        grid[r0:r1, c0:c1] = val

    def generate(self):
        grid = np.full((self.h, self.w), UNKNOWN, dtype=np.uint8)

        # 1. 场地内部全部可通行
        self.fill(grid, 0, 0, HALF_WIDTH, FIELD_HEIGHT, FREE)

        # 2. 四面围栏
        b = BORDER
        self.fill(grid, -b, -b, HALF_WIDTH + b, 0, OCCUPIED)           # 底
        self.fill(grid, -b, FIELD_HEIGHT, HALF_WIDTH + b, FIELD_HEIGHT + b, OCCUPIED)  # 顶
        self.fill(grid, -b, -b, 0, FIELD_HEIGHT + b, OCCUPIED)          # 左
        self.fill(grid, HALF_WIDTH, -b, HALF_WIDTH + b, FIELD_HEIGHT + b, OCCUPIED)    # 右

        # ── Zone 1: 武馆区 ──
        self.fill(grid, R1_START_X0, R1_START_Y0, R1_START_X1, R1_START_Y1, OCCUPIED)
        self.fill(grid, TIP_RACK_X0, TIP_RACK_Y0, TIP_RACK_X1, TIP_RACK_Y1, OCCUPIED)
        self.fill(grid, POLE_RACK_X0, POLE_RACK_Y0, POLE_RACK_X1, POLE_RACK_Y1, OCCUPIED)

        # ── Zone 2: 梅林区 ──
        # 方块区域内部可通行, 但左右两侧画墙:
        # R2 必须从上方 3 个方块进入, 从下方 3 个方块离开
        # 不能从左右通道侧面进入方块区域
        block_x0 = BLOCK_COL_X[0]                          # 1200
        block_x1 = BLOCK_COL_X[-1] + BLOCK_SIZE            # 4800
        block_y_top = BLOCK_ROW_Y_TOP[0]                    # 8800
        block_y_bot = BLOCK_ROW_Y_BOT[-1]                   # 4000
        wall_thick = 30  # 墙壁厚度 (mm)
        # 左侧墙
        self.fill(grid, block_x0 - wall_thick, block_y_bot, block_x0, block_y_top, OCCUPIED)
        # 右侧墙
        self.fill(grid, block_x1, block_y_bot, block_x1 + wall_thick, block_y_top, OCCUPIED)

        # ── Zone 2/3 边界: 平台边缘 (400mm 落差 = 墙壁) ──
        # 只有坡道处可通行, 其余是不可跨越的台阶
        self.fill(grid, 0, ZONE3_Y1 - 30, SLOPE_X0, ZONE3_Y1 + 30, OCCUPIED)

        # ── 坡道 ──
        self.fill(grid, SLOPE_X0, SLOPE_Y0, SLOPE_X1, SLOPE_Y1, SLOPE_VAL)

        # ── Zone 3: 对抗区 ──
        # 亚克力护栏
        self.fill(grid, BARRIER_X0, BARRIER_Y0, BARRIER_X1, BARRIER_Y1, OCCUPIED)
        # 武器废弃区
        self.fill(grid, DISPOSAL_X0, DISPOSAL_Y0, DISPOSAL_X1, DISPOSAL_Y1, OCCUPIED)
        # 九宫格
        self.fill(grid, KFS_X0, KFS_Y0, KFS_X1, KFS_Y1, OCCUPIED)

        # Zone 3 底部边缘加强 (确保围栏可见)
        self.fill(grid, 0, -b, HALF_WIDTH, 0, OCCUPIED)

        return grid

    def save_map(self, grid, output_dir, name, resolution_m):
        os.makedirs(output_dir, exist_ok=True)
        pgm_path = os.path.join(output_dir, f'{name}.pgm')
        yaml_path = os.path.join(output_dir, f'{name}.yaml')

        Image.fromarray(grid).save(pgm_path)

        origin_x = -BORDER / 1000.0
        origin_y = -BORDER / 1000.0
        with open(yaml_path, 'w') as f:
            yaml.dump({
                'image': f'{name}.pgm',
                'resolution': resolution_m,
                'origin': [origin_x, origin_y, 0.0],
                'negate': 0,
                'occupied_thresh': 0.65,
                'free_thresh': 0.196,
            }, f, default_flow_style=False)

        print(f"  {pgm_path}  ({grid.shape[1]}×{grid.shape[0]} px)")

    def generate_waypoints(self, output_dir):
        wp = {
            'robot_start': {
                'x': round((R2_START_X0 + R2_START_X1) / 2 / 1000, 3),
                'y': round((R2_START_Y0 + R2_START_Y1) / 2 / 1000, 3),
                'yaw': -1.571,
            },
            'steps': {},
            'zones': {
                'zone1_weapons': {
                    'platform_mm': 150,
                    'y_min': ZONE1_Y0 / 1000,
                    'y_max': ZONE1_Y1 / 1000,
                },
                'zone2_forest': {
                    'platform_mm': 50,
                    'y_min': ZONE2_Y0 / 1000,
                    'y_max': ZONE2_Y1 / 1000,
                },
                'zone3_arena': {
                    'platform_mm': 450,
                    'y_min': ZONE3_Y0 / 1000,
                    'y_max': ZONE3_Y1 / 1000,
                },
            },
            'key_points': {
                'tip_rack_approach': {
                    'label': '端头架接近点',
                    'x': round((TIP_RACK_X1 + 400) / 1000, 3),
                    'y': round((TIP_RACK_Y0 + TIP_RACK_Y1) / 2 / 1000, 3),
                    'yaw': 3.14159,
                },
                'slope_entry': {
                    'label': '坡道入口 (Zone2侧)',
                    'x': round((SLOPE_X0 + SLOPE_X1) / 2 / 1000, 3),
                    'y': round(SLOPE_Y1 / 1000, 3),
                    'yaw': -1.571,
                },
                'kfs_approach': {
                    'label': '九宫格接近点',
                    'x': round((KFS_X1 + 500) / 1000, 3),
                    'y': round((KFS_Y0 + KFS_Y1) / 2 / 1000, 3),
                    'yaw': 3.14159,
                },
            },
        }

        for row in range(BLOCK_GRID_ROWS):
            for col in range(BLOCK_GRID_COLS):
                idx = row * BLOCK_GRID_COLS + col + 1
                name = f'step_{idx:02d}'
                cx = BLOCK_COL_X[col] + BLOCK_SIZE / 2
                cy = (BLOCK_ROW_Y_TOP[row] + BLOCK_ROW_Y_BOT[row]) / 2
                h = BLOCK_HEIGHTS[row][col]

                ax, ay, ayaw = self._approach(row, col, cx, cy)

                wp['steps'][name] = {
                    'label': f'梅林方块{idx}',
                    'row': row, 'col': col,
                    'center': {'x': round(cx/1000, 3), 'y': round(cy/1000, 3)},
                    'height_mm': h,
                    'approach_pose': {
                        'x': round(ax/1000, 3),
                        'y': round(ay/1000, 3),
                        'yaw': round(ayaw, 3),
                    },
                }

        path = os.path.join(output_dir, 'field_waypoints.yaml')
        with open(path, 'w') as f:
            yaml.dump(wp, f, default_flow_style=False, allow_unicode=True)
        print(f"  {path}")

    def _approach(self, row, col, cx, cy):
        d = 800
        if col == 0:
            return cx - d, cy, 0.0
        if col == BLOCK_GRID_COLS - 1:
            return cx + d, cy, 3.14159
        if row == BLOCK_GRID_ROWS - 1:
            return cx, cy - d, 1.5708
        if row == 0:
            return cx, cy + d, -1.5708
        return cx, cy - d, 1.5708


def main():
    parser = argparse.ArgumentParser(description='生成 Robocon 2026 左半场 Nav2 地图')
    parser.add_argument('--output', type=str, required=True)
    parser.add_argument('--resolution', type=float, default=0.02)
    args = parser.parse_args()

    res_mm = int(args.resolution * 1000)
    gen = FieldMapGenerator(resolution_mm=res_mm)

    print(f"分辨率: {args.resolution}m ({res_mm}mm/px), 地图: {gen.w}×{gen.h} px")
    print()

    print("生成左半场 (蓝方):")
    grid = gen.generate()
    gen.save_map(grid, args.output, 'left_half', args.resolution)
    gen.generate_waypoints(args.output)

    # 预览 PNG
    rgb = np.zeros((grid.shape[0], grid.shape[1], 3), dtype=np.uint8)
    rgb[grid == FREE] = [255, 255, 255]
    rgb[grid == OCCUPIED] = [0, 0, 0]
    rgb[grid == SLOPE_VAL] = [180, 200, 255]
    rgb[grid == UNKNOWN] = [128, 128, 128]
    preview = Image.fromarray(rgb).resize(
        (grid.shape[1] * 3, grid.shape[0] * 3), Image.Resampling.NEAREST)
    preview_path = os.path.join(args.output, 'left_half_preview.png')
    preview.save(preview_path)
    print(f"  {preview_path} (3x 放大预览)")


if __name__ == '__main__':
    main()
