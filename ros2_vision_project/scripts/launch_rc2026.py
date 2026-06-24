#!/usr/bin/env python3
"""
RC2026 全系统交互启动脚本
一个终端完成所有参数输入，自动生成并执行 ros2 launch 指令

用法:
    source scripts/setup_env.sh
    python3 scripts/launch_rc2026.py
"""
import os
import sys
import subprocess
import threading
import time

R  = '\033[1;31m'
G  = '\033[1;32m'
Y  = '\033[1;33m'
C  = '\033[1;36m'
W  = '\033[1;37m'
DIM= '\033[2m'
N  = '\033[0m'

def clr(text, color): return f'{color}{text}{N}'


def ask(prompt, options=None, default=None):
    """
    交互输入，支持首字母缩写。
    options 中每个选项的首字母自动作为有效缩写（前提是无冲突）。
    """
    # 构建首字母缩写映射
    shorthand = {}
    if options:
        for opt in options:
            key = opt[0].lower()
            if key not in shorthand:
                shorthand[key] = opt

    # 构建提示后缀
    if options:
        def fmt(o):
            # 以 首字母(剩余) 格式显示，默认值高亮
            s = f'{o[0]}({o[1:]})' if len(o) > 1 else o
            return clr(s, Y) if o == default else s
        suffix = ' [' + '/'.join(fmt(o) for o in options) + ']'
    elif default is not None:
        suffix = f' [{clr(default, Y)}]'
    else:
        suffix = ''

    while True:
        try:
            raw = input(f'  {prompt}{suffix}: ').strip()
        except (EOFError, KeyboardInterrupt):
            print('\n已取消。')
            sys.exit(0)

        val = raw.lower()

        # 空输入 → 默认值
        if not val:
            if default is not None:
                return default
            print(f'  {clr("⚠", R)} 不能为空')
            continue

        # 精确匹配
        if options and val in options:
            return val

        # 首字母缩写匹配
        if options and val in shorthand:
            return shorthand[val]

        # 无选项约束 → 直接返回原始输入
        if not options:
            return raw

        # 不合法
        if options:
            print(f'  {clr("⚠", R)} 有效输入: ' +
                  ', '.join(f'{o[0]}({o[1:]})' if len(o) > 1 else o for o in options))
        else:
            print(f'  {clr("⚠", R)} 请输入值')


def section(title):
    print(f'\n{clr("▶  " + title, C)}')


def main():
    os.system('clear')
    print(f"""
{Y}╔══════════════════════════════════════════════════════╗
║        ROBOCON 2026  RC2026 全系统交互启动           ║
╚══════════════════════════════════════════════════════╝{N}""")

    # ════════════════════════════════════════════════════
    #   1. 半场
    # ════════════════════════════════════════════════════
    section('半场')
    field_side = ask('场地半场', ['left', 'right'], 'left')

    # ════════════════════════════════════════════════════
    #   2. 测试区域
    # ════════════════════════════════════════════════════
    section('测试区域')
    print(f'  {DIM}full=完整比赛  weapon=武馆  merlin=梅林  confront=对抗区{N}')
    test_area = ask('测试区域', ['full', 'weapon', 'merlin', 'confront'], 'full')

    # ════════════════════════════════════════════════════
    #   3. KFS 配置（所有模式均需要）
    # ════════════════════════════════════════════════════
    section('KFS 配置')
    kfs_color = ask('KFS 颜色', ['blue', 'red'], 'blue')
    kfs_real  = ask('真 KFS 台阶编号 (空格分隔, 如: 5 8)',  default='5 8')
    kfs_fake  = ask('假 KFS 台阶编号 (空格分隔, 如: 2 11)', default='2 11')

    # ════════════════════════════════════════════════════
    #   4. 功能开关
    #      完整比赛：状态机与串口默认全开
    #      区域测试：状态机默认开，串口默认关
    # ════════════════════════════════════════════════════
    section('功能开关')
    if test_area == 'full':
        gc_default     = 'true'
        serial_default = 'true'
    else:
        gc_default     = 'true'
        serial_default = 'false'

    use_gc        = ask('启动比赛状态机 (game_controller)', ['true', 'false'], gc_default)
    enable_serial = ask('启动串口通信 (连接 STM32)',          ['true', 'false'], serial_default)

    # ════════════════════════════════════════════════════
    #   5. 组装参数
    # ════════════════════════════════════════════════════
    params = [
        ('field_side',          field_side),
        ('test_area',           test_area),
        ('kfs_color',           kfs_color),
        ('kfs_real',            kfs_real),
        ('kfs_fake',            kfs_fake),
        ('use_game_controller', use_gc),
        ('enable_serial',       enable_serial),
    ]

    # ════════════════════════════════════════════════════
    #   6. 预览
    # ════════════════════════════════════════════════════
    arg_lines = ' \\\n    '.join(f'{k}:={v}' for k, v in params)
    preview   = f'ros2 launch rc2026_bringup full_system.launch.py \\\n    {arg_lines}'

    print(f'\n{G}┌──────────────────── 即将执行 ──────────────────────┐{N}')
    for line in preview.split('\n'):
        print(f'{G}│{N}  {W}{line}{N}')
    print(f'{G}└────────────────────────────────────────────────────┘{N}')

    # 摘要行
    serial_clr = G if enable_serial == 'true' else DIM
    gc_clr     = G if use_gc        == 'true' else DIM
    print(
        f'\n  半场 {clr(field_side.upper(), Y)}  '
        f'区域 {clr(test_area, Y)}  '
        f'颜色 {clr(kfs_color, Y)}  '
        f'真={clr(kfs_real, G)}  假={clr(kfs_fake, R)}\n'
        f'  状态机 {clr(use_gc, gc_clr)}  '
        f'串口 {clr(enable_serial, serial_clr)}'
    )

    # ════════════════════════════════════════════════════
    #   7. 确认并启动
    # ════════════════════════════════════════════════════
    print()
    confirm = ask('确认启动', ['y', 'n'], 'y')
    if confirm != 'y':
        print('已取消。')
        sys.exit(0)

    print(f'\n{G}[RC2026] 启动中...{N}\n')

    argv = ['ros2', 'launch', 'rc2026_bringup', 'full_system.launch.py']
    for k, v in params:
        argv.append(f'{k}:={v}')

    # 构建 KFS 话题消息 (game_controller 在 full 模式下通过话题接收)
    kfs_real_csv  = kfs_real.strip().replace(' ', ',')
    kfs_fake_csv  = kfs_fake.strip().replace(' ', ',')
    kfs_topic_msg = f'real:{kfs_real_csv} fake:{kfs_fake_csv} color:{kfs_color}'

    proc = subprocess.Popen(argv, env=os.environ.copy())

    def _send_kfs():
        """等节点就绪后自动发 KFS 配置话题."""
        time.sleep(8)
        if proc.poll() is not None:
            return
        cmd = [
            'ros2', 'topic', 'pub', '--times', '3',
            '/game/kfs_input', 'std_msgs/msg/String',
            f'{{data: "{kfs_topic_msg}"}}'
        ]
        subprocess.run(cmd, env=os.environ.copy(),
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f'\n{G}[RC2026] KFS 配置已发送: {kfs_topic_msg}{N}')
        print(f'{Y}[RC2026] 等待启动信号 — 按下机器人物理启动按钮{N}')
        print(f'{DIM}         或手动: ros2 topic pub --once /game/start_signal '
              f'std_msgs/msg/UInt8 \'{{data: 1}}\'{N}')

    threading.Thread(target=_send_kfs, daemon=True).start()

    try:
        proc.wait()
    except KeyboardInterrupt:
        print(f'\n{Y}[RC2026] 正在关闭...{N}')
        proc.terminate()
        proc.wait()


if __name__ == '__main__':
    main()
