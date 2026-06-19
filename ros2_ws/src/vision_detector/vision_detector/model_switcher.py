#!/usr/bin/env python3
"""
model_switcher.py
运行时模型切换工具
按 回车 触发，输入模型名后通过 ROS2 参数服务热切换模型

使用方式：
  ros2 run vision_detector model_switcher
"""
import sys
import os
import threading
import termios
import tty
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


WEIGHTS_DIR = os.path.join(
    os.path.expanduser('~'),
    'GAFA-Artlnnov.RC2026-main',
    'ros2_vision_project',
    'ros2_ws',
    'src',
    'vision_detector',
    'weights'
)


class ModelSwitcherNode(Node):

    def __init__(self):
        super().__init__('model_switcher_node')

        self.set_param_cli = self.create_client(
            SetParameters,
            '/vision_detector/set_parameters')

        self.get_logger().info(
            '模型切换工具已启动\n'
            '  按 回车 切换模型\n'
            '  按 Ctrl+C 退出\n'
            f'  模型目录：{WEIGHTS_DIR}')

        self._list_models()

    def _list_models(self):
        """列出可用模型"""
        if os.path.exists(WEIGHTS_DIR):
            models = [f for f in os.listdir(WEIGHTS_DIR)
                      if f.endswith('.pt')]
            if models:
                print(f"\n可用模型（{WEIGHTS_DIR}）：")
                for m in sorted(models):
                    print(f"  - {m}")
            else:
                print(f"\n⚠ weights/ 目录下没有 .pt 文件")
        print()

    def switch_model(self, model_name: str):
        """发送模型切换请求"""
        model_name = model_name.strip()
        if not model_name:
            print("模型名不能为空")
            return

        # 自动补全 .pt 后缀
        if not model_name.endswith('.pt'):
            model_name += '.pt'

        # 支持绝对路径和文件名两种格式
        if os.path.isabs(model_name):
            model_path = model_name
        else:
            model_path = os.path.join(WEIGHTS_DIR, model_name)

        if not os.path.exists(model_path):
            print(f"✗ 文件不存在：{model_path}")
            self._list_models()
            return

        if not self.set_param_cli.wait_for_service(timeout_sec=2.0):
            print("✗ 检测节点未响应，确认 vision_detector 已启动")
            return

        # 构造参数设置请求
        req = SetParameters.Request()
        param = Parameter()
        param.name = 'model_path'
        param.value = ParameterValue(
            type=ParameterType.PARAMETER_STRING,
            string_value=model_path)
        req.parameters = [param]

        print(f"正在切换到：{model_name} ...")
        future = self.set_param_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() and future.result().results[0].successful:
            print(f"✓ 模型切换成功：{model_name}")
        else:
            reason = (future.result().results[0].reason
                      if future.result() else '超时')
            print(f"✗ 模型切换失败：{reason}")


def get_char():
    """读取单个字符（原始终端模式）"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def input_thread(node: ModelSwitcherNode):
    """后台线程：监听键盘输入"""
    print("监听中... (Ctrl+M 切换模型，Ctrl+C 退出)")
    try:
        while rclpy.ok():
            ch = get_char()

            # Ctrl+M = ASCII 13（回车）在原始模式下
            # 用 Ctrl+M 的实际值：\r 或 chr(13)
            if ch == '\r' or ord(ch) == 13:
                # 切换回正常终端模式获取输入
                fd = sys.stdin.fileno()
                old = termios.tcgetattr(fd)
                termios.tcsetattr(fd, termios.TCSADRAIN, old)

                # 检测是否是 Ctrl+M（通过前一个字符判断）
                # 实际上在原始模式下 Ctrl+M 和 Enter 都是 \r
                # 改用更明确的 Ctrl+M 检测
                continue

            # Ctrl+M = chr(13) 但在某些终端是 \x0d
            # 更可靠的方式：直接检测 \x0d（十进制13）
            if ord(ch) == 13:
                _prompt_model_switch(node)

            # Ctrl+C = chr(3)
            elif ord(ch) == 3:
                print("\n退出模型切换工具")
                rclpy.shutdown()
                break

    except Exception as e:
        pass


def _prompt_model_switch(node):
    """恢复终端正常模式，提示输入模型名"""
    # 恢复正常终端
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    try:
        print("\n" + "─" * 40)
        print("切换模型（直接回车取消）")
        node._list_models()
        model_name = input("输入模型名：").strip()
        print("─" * 40)

        if model_name:
            node.switch_model(model_name)
        else:
            print("已取消")
    except (KeyboardInterrupt, EOFError):
        print("\n已取消")


def main(args=None):
    rclpy.init(args=args)
    node = ModelSwitcherNode()

    # 后台线程监听键盘
    t = threading.Thread(
        target=keyboard_listener,
        args=(node,),
        daemon=True)
    t.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def keyboard_listener(node: ModelSwitcherNode):
    """监听 Ctrl+M（ASCII 13）"""
    import sys, termios, tty

    fd = sys.stdin.fileno()

    print("\n监听键盘... 按 Ctrl+M 切换模型，Ctrl+C 退出\n")

    try:
        while rclpy.ok():
            old = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                ch = sys.stdin.buffer.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)

            byte_val = ch[0] if ch else 0

            if byte_val == 13:        # Ctrl+M 或 Enter
                _prompt_model_switch(node)
            elif byte_val == 3:       # Ctrl+C
                print("\n退出")
                rclpy.shutdown()
                break
    except Exception:
        pass


if __name__ == '__main__':
    main()
