#!/usr/bin/env python3
"""
pcd_to_gridmap.py
将 FAST-LIO 输出的 3D PCD 点云转换为 Nav2 可用的 2D 占据栅格地图
用法: python3 pcd_to_gridmap.py <input.pcd> <output_dir> [--resolution 0.05] [--height_min 0.1] [--height_max 2.0]
"""
import argparse
import numpy as np
import struct
import os
import yaml
from PIL import Image


def read_pcd(filepath):
    """读取 PCD 文件,返回 Nx3 numpy 数组"""
    with open(filepath, 'rb') as f:
        header_lines = []
        while True:
            line = f.readline().decode('ascii', errors='ignore').strip()
            header_lines.append(line)
            if line.startswith('DATA'):
                break

        fields = []
        sizes = []
        types = []
        counts = []
        num_points = 0
        data_format = 'ascii'

        for line in header_lines:
            parts = line.split()
            if parts[0] == 'FIELDS':
                fields = parts[1:]
            elif parts[0] == 'SIZE':
                sizes = [int(s) for s in parts[1:]]
            elif parts[0] == 'TYPE':
                types = parts[1:]
            elif parts[0] == 'COUNT':
                counts = [int(c) for c in parts[1:]]
            elif parts[0] == 'POINTS':
                num_points = int(parts[1])
            elif parts[0] == 'DATA':
                data_format = parts[1]

        x_idx = fields.index('x') if 'x' in fields else 0
        y_idx = fields.index('y') if 'y' in fields else 1
        z_idx = fields.index('z') if 'z' in fields else 2

        points = []
        if data_format == 'ascii':
            for _ in range(num_points):
                line = f.readline().decode('ascii', errors='ignore').strip()
                if not line:
                    continue
                vals = line.split()
                x = float(vals[x_idx])
                y = float(vals[y_idx])
                z = float(vals[z_idx])
                points.append([x, y, z])
        elif data_format == 'binary':
            point_size = sum(s * c for s, c in zip(sizes, counts))
            for _ in range(num_points):
                data = f.read(point_size)
                if len(data) < point_size:
                    break
                offset = 0
                vals = []
                for s, t, c in zip(sizes, types, counts):
                    for _ in range(c):
                        if t == 'F' and s == 4:
                            val = struct.unpack('f', data[offset:offset+4])[0]
                        elif t == 'F' and s == 8:
                            val = struct.unpack('d', data[offset:offset+8])[0]
                        elif t == 'U' and s == 4:
                            val = struct.unpack('I', data[offset:offset+4])[0]
                        elif t == 'U' and s == 1:
                            val = struct.unpack('B', data[offset:offset+1])[0]
                        else:
                            val = 0
                        vals.append(val)
                        offset += s
                points.append([float(vals[x_idx]), float(vals[y_idx]), float(vals[z_idx])])

    return np.array(points)


def pcd_to_gridmap(pcd_path, output_dir, resolution=0.05,
                   height_min=0.1, height_max=2.0):
    print(f"读取 PCD: {pcd_path}")
    points = read_pcd(pcd_path)
    print(f"总点数: {len(points)}")

    mask = (points[:, 2] >= height_min) & (points[:, 2] <= height_max)
    filtered = points[mask]
    print(f"高度过滤后 ({height_min}~{height_max}m): {len(filtered)} 点")

    if len(filtered) == 0:
        print("错误: 过滤后没有点,请调整 height_min/height_max")
        return

    x_min, y_min = filtered[:, 0].min(), filtered[:, 1].min()
    x_max, y_max = filtered[:, 0].max(), filtered[:, 1].max()

    padding = 1.0
    x_min -= padding
    y_min -= padding
    x_max += padding
    y_max += padding

    width = int(np.ceil((x_max - x_min) / resolution))
    height = int(np.ceil((y_max - y_min) / resolution))
    print(f"地图尺寸: {width} x {height} 像素 ({x_max-x_min:.1f} x {y_max-y_min:.1f} m)")

    grid = np.full((height, width), 205, dtype=np.uint8)

    ix = ((filtered[:, 0] - x_min) / resolution).astype(int)
    iy = ((filtered[:, 1] - y_min) / resolution).astype(int)
    ix = np.clip(ix, 0, width - 1)
    iy = np.clip(iy, 0, height - 1)
    grid[iy, ix] = 0

    img = Image.fromarray(np.flipud(grid))

    os.makedirs(output_dir, exist_ok=True)
    pgm_path = os.path.join(output_dir, 'arena.pgm')
    yaml_path = os.path.join(output_dir, 'arena.yaml')

    img.save(pgm_path)

    map_yaml = {
        'image': 'arena.pgm',
        'resolution': resolution,
        'origin': [float(x_min), float(y_min), 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
    }
    with open(yaml_path, 'w') as f:
        yaml.dump(map_yaml, f, default_flow_style=False)

    print(f"地图已保存:")
    print(f"  PGM: {pgm_path}")
    print(f"  YAML: {yaml_path}")
    print(f"  原点: ({x_min:.2f}, {y_min:.2f})")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='PCD to Nav2 OccupancyGrid Map')
    parser.add_argument('input', help='输入 PCD 文件路径')
    parser.add_argument('output', help='输出目录')
    parser.add_argument('--resolution', type=float, default=0.05, help='地图分辨率 (m/pixel)')
    parser.add_argument('--height_min', type=float, default=0.1, help='障碍物最低高度 (m)')
    parser.add_argument('--height_max', type=float, default=2.0, help='障碍物最高高度 (m)')
    args = parser.parse_args()
    pcd_to_gridmap(args.input, args.output, args.resolution, args.height_min, args.height_max)
