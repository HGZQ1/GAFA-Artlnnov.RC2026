#!/usr/bin/env python3
"""
simple_teleop.py
简易键盘遥控 — 输入字母后回车发送速度指令

操作:
  w/s — 前进/后退
  a/d — 左移/右移
  q/e — 左转/右转
  x   — 停止
  +/- — 加速/减速
  Ctrl+C — 退出
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SimpleTeleop(Node):
    def __init__(self):
        super().__init__('simple_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed = 0.3
        self.turn = 0.5
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.timer = self.create_timer(0.1, self._publish)

    def _publish(self):
        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        msg.angular.z = self.wz
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleTeleop()

    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print('='*40)
    print('  RC2026 简易遥控')
    print('='*40)
    print(f'  速度: {node.speed:.2f} m/s  转速: {node.turn:.2f} rad/s')
    print()
    print('  w — 前进    s — 后退')
    print('  a — 左移    d — 右移')
    print('  q — 左转    e — 右转')
    print('  x — 停止')
    print('  + — 加速    - — 减速')
    print('  Ctrl+C — 退出')
    print('='*40)

    try:
        while True:
            cmd = input('> ').strip().lower()
            if cmd == 'w':
                node.vx = node.speed
                node.vy = 0.0
                node.wz = 0.0
            elif cmd == 's':
                node.vx = -node.speed
                node.vy = 0.0
                node.wz = 0.0
            elif cmd == 'a':
                node.vx = 0.0
                node.vy = node.speed
                node.wz = 0.0
            elif cmd == 'd':
                node.vx = 0.0
                node.vy = -node.speed
                node.wz = 0.0
            elif cmd == 'q':
                node.vx = 0.0
                node.vy = 0.0
                node.wz = node.turn
            elif cmd == 'e':
                node.vx = 0.0
                node.vy = 0.0
                node.wz = -node.turn
            elif cmd == 'x':
                node.vx = 0.0
                node.vy = 0.0
                node.wz = 0.0
            elif cmd == '+' or cmd == '=':
                node.speed = min(node.speed + 0.1, 2.0)
                node.turn = min(node.turn + 0.2, 3.14)
                print(f'  速度: {node.speed:.2f}  转速: {node.turn:.2f}')
            elif cmd == '-':
                node.speed = max(node.speed - 0.1, 0.1)
                node.turn = max(node.turn - 0.2, 0.1)
                print(f'  速度: {node.speed:.2f}  转速: {node.turn:.2f}')
            else:
                continue
            print(f'  vx={node.vx:.2f}  vy={node.vy:.2f}  wz={node.wz:.2f}')
    except (KeyboardInterrupt, EOFError):
        node.vx = 0.0
        node.vy = 0.0
        node.wz = 0.0
        import time
        time.sleep(0.2)
        print('\n停止')

    node.destroy_node()
    rclpy.shutdown()
