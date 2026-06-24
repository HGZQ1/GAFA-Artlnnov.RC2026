#!/usr/bin/env python3
"""
view_map.py  —  用 Open3D 可视化 PCD/PLY 地图
用法:
  python3 view_map.py                          # 默认查看右半场
  python3 view_map.py right                    # 右半场
  python3 view_map.py left                     # 左半场
  python3 view_map.py /path/to/custom.pcd      # 自定义路径
"""
import sys, os
import numpy as np
import open3d as o3d

MAP_DIR = os.path.join(os.path.dirname(__file__),
                       '..', 'map')

def _resolve(arg):
    if os.path.isabs(arg) or arg.endswith('.pcd') or arg.endswith('.ply'):
        return arg
    if arg == 'left':
        return os.path.join(MAP_DIR, 'left_half.pcd')
    return os.path.join(MAP_DIR, 'right_half.pcd')   # default: right

side = sys.argv[1] if len(sys.argv) > 1 else 'right'
path = _resolve(side)

print(f'加载: {path}')
pcd = o3d.io.read_point_cloud(path)
pts = np.asarray(pcd.points)

print(f'点数 : {len(pts):,}')
print(f'X    : [{pts[:,0].min():.2f}, {pts[:,0].max():.2f}]  宽={pts[:,0].ptp():.1f}m')
print(f'Y    : [{pts[:,1].min():.2f}, {pts[:,1].max():.2f}]  长={pts[:,1].ptp():.1f}m')
print(f'Z    : [{pts[:,2].min():.2f}, {pts[:,2].max():.2f}]  高={pts[:,2].ptp():.1f}m')

# 按高度着色 (低→蓝, 高→红)
z = pts[:, 2]
z_n = (z - z.min()) / max(z.ptp(), 0.01)
colors = np.column_stack([z_n, np.zeros_like(z_n), 1 - z_n])
pcd.colors = o3d.utility.Vector3dVector(colors)

# 添加坐标轴 (原点 = 机器人启动位置)
frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0,0,0])

print('\n操作提示:')
print('  鼠标左键拖动  旋转')
print('  鼠标右键拖动  平移')
print('  滚轮          缩放')
print('  R             重置视角')
print('  Q / Esc       退出')

o3d.visualization.draw_geometries(
    [pcd, frame],
    window_name=f'RC2026 地图 — {os.path.basename(path)}',
    width=1280, height=720,
    point_show_normal=False,
)
