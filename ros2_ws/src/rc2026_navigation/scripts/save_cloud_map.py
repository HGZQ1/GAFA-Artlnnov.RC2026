#!/usr/bin/env python3
"""
save_cloud_map.py
订阅 FAST-LIO 的 /cloud_registered 话题，累积点云，保存为 PCD 文件。
用法: python3 save_cloud_map.py [--duration 30] [--output ~/map.pcd]

启动后会持续收集点云，到达 duration 秒后自动保存并退出。
也可以随时 Ctrl+C 提前保存。
"""
import argparse
import os
import struct
import signal
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class CloudAccumulator(Node):
    def __init__(self, duration, output_path):
        super().__init__('cloud_accumulator')
        self.output_path = output_path
        self.duration = duration
        self.points = []
        self.msg_count = 0
        self.start_time = None

        self.sub = self.create_subscription(
            PointCloud2, '/cloud_registered', self.cloud_cb, 10)
        self.get_logger().info(
            f'开始收集点云, 持续 {duration} 秒, 保存到 {output_path}')

    def cloud_cb(self, msg):
        if self.start_time is None:
            self.start_time = self.get_clock().now()

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed >= self.duration:
            self.save_and_exit()
            return

        pts = self.parse_pointcloud2(msg)
        if len(pts) > 0:
            self.points.append(pts)
            self.msg_count += 1
            total = sum(len(p) for p in self.points)
            if self.msg_count % 10 == 0:
                self.get_logger().info(
                    f'已收集 {self.msg_count} 帧, {total} 点, '
                    f'{elapsed:.1f}/{self.duration}s')

    def parse_pointcloud2(self, msg):
        """从 PointCloud2 消息提取 xyz 坐标"""
        point_step = msg.point_step
        data = msg.data

        x_off = y_off = z_off = 0
        for field in msg.fields:
            if field.name == 'x':
                x_off = field.offset
            elif field.name == 'y':
                y_off = field.offset
            elif field.name == 'z':
                z_off = field.offset

        n_points = msg.width * msg.height
        points = np.zeros((n_points, 3), dtype=np.float32)

        for i in range(n_points):
            base = i * point_step
            points[i, 0] = struct.unpack_from('f', data, base + x_off)[0]
            points[i, 1] = struct.unpack_from('f', data, base + y_off)[0]
            points[i, 2] = struct.unpack_from('f', data, base + z_off)[0]

        mask = np.isfinite(points).all(axis=1)
        return points[mask]

    def save_and_exit(self):
        if not self.points:
            self.get_logger().error('没有收集到任何点云!')
            rclpy.shutdown()
            return

        all_points = np.vstack(self.points)
        self.get_logger().info(f'保存 {len(all_points)} 点到 {self.output_path}')
        self.write_pcd(self.output_path, all_points)
        self.get_logger().info('保存完成!')
        rclpy.shutdown()

    def write_pcd(self, path, points):
        """写入 ASCII PCD 文件"""
        with open(path, 'w') as f:
            f.write('# .PCD v0.7 - Point Cloud Data file format\n')
            f.write('VERSION 0.7\n')
            f.write('FIELDS x y z\n')
            f.write('SIZE 4 4 4\n')
            f.write('TYPE F F F\n')
            f.write('COUNT 1 1 1\n')
            f.write(f'WIDTH {len(points)}\n')
            f.write('HEIGHT 1\n')
            f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
            f.write(f'POINTS {len(points)}\n')
            f.write('DATA ascii\n')
            for p in points:
                f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=30.0,
                        help='收集时长(秒), 默认30秒')
    parser.add_argument('--output', type=str,
                        default=os.path.expanduser('~/FAST_LIO/PCD/scans.pcd'),
                        help='输出 PCD 路径')
    args = parser.parse_args()

    rclpy.init()
    node = CloudAccumulator(args.duration, args.output)

    def signal_handler(sig, frame):
        node.get_logger().info('收到 Ctrl+C, 保存已收集的点云...')
        node.save_and_exit()

    signal.signal(signal.SIGINT, signal_handler)

    rclpy.spin(node)


if __name__ == '__main__':
    main()
