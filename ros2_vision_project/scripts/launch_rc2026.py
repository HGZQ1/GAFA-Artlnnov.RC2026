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
    shorthand = {}
    if options:
        for opt in options:
            key = opt[0].lower()
            if key not in shorthand:
                shorthand[key] = opt

    if options:
        def fmt(o):
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

        if not val:
            if default is not None:
                return default
            print(f'  {clr("⚠", R)} 不能为空')
            continue

        if options and val in options:
            return val

        if options and val in shorthand:
            return shorthand[val]

        if not options:
            return raw

        if options:
            print(f'  {clr("⚠", R)} 有效输入: ' +
                  ', '.join(f'{o[0]}({o[1:]})' if len(o) > 1 else o for o in options))
        else:
            print(f'  {clr("⚠", R)} 请输入值')


def section(title):
    print(f'\n{clr("▶  " + title, C)}')


def _start_debug_viewers(proc, usb_video_idx=0):
    """调试模式：在后台自动启动相机预览窗口。"""
    # D435i 彩色图像 (等待3秒让 RealSense 节点就绪)
    time.sleep(3)
    if proc.poll() is not None:
        return
    try:
        subprocess.Popen(
            ['ros2', 'run', 'rqt_image_view', 'rqt_image_view',
             '/camera/camera/color/image_raw'],
            env=os.environ.copy(),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        print(f'\n{G}[调试] D435i 预览已启动 → rqt_image_view /camera/camera/color/image_raw{N}')
    except FileNotFoundError:
        print(f'\n{Y}[调试] rqt_image_view 未找到，跳过 D435i 预览{N}')

    # USB 相机 OpenCV 预览 (再等2秒，避免与 fine_align_node 竞争设备)
    time.sleep(2)
    if proc.poll() is not None:
        return
    usb_viewer = (
        f'import cv2, sys\n'
        f'cap = cv2.VideoCapture({usb_video_idx}, cv2.CAP_V4L2)\n'
        f'if not cap.isOpened(): print("[USB相机] 无法打开 /dev/video{usb_video_idx}"); sys.exit(1)\n'
        f'print("[USB相机] 预览已开启 — 按 Q 关闭")\n'
        f'while True:\n'
        f'    ret, frame = cap.read()\n'
        f'    if not ret: break\n'
        f'    cv2.imshow("USB相机 (/dev/video{usb_video_idx})", frame)\n'
        f'    if cv2.waitKey(1) & 0xFF == ord("q"): break\n'
        f'cap.release()\n'
        f'cv2.destroyAllWindows()\n'
    )
    try:
        subprocess.Popen(
            ['python3', '-c', usb_viewer],
            env=os.environ.copy(),
        )
        print(f'{G}[调试] USB相机预览已启动 → OpenCV /dev/video{usb_video_idx}{N}')
        print(f'{DIM}       (fine_align_node 的 debug_gui 窗口在 /fine_align/enable 触发后显示){N}')
    except Exception as e:
        print(f'{Y}[调试] USB相机预览启动失败: {e}{N}')


def _send_kfs(proc, kfs_topic_msg):
    """等节点就绪后自动发 KFS 配置话题。"""
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


def main():
    os.system('clear')
    print(f"""
{Y}╔══════════════════════════════════════════════════════╗
║        ROBOCON 2026  RC2026 全系统交互启动           ║
╚══════════════════════════════════════════════════════╝{N}""")

    # ════════════════════════════════════════════════════
    #   0. 选择启动模式
    # ════════════════════════════════════════════════════
    section('启动模式')
    print(f'  {DIM}competition=比赛模式(自动全场景/状态机/串口)  debug=调试模式(手动选区+相机预览){N}')
    mode = ask('启动模式', ['competition', 'debug'], 'competition')

    # ════════════════════════════════════════════════════
    #   比赛模式
    # ════════════════════════════════════════════════════
    if mode == 'competition':
        section('半场')
        field_side = ask('场地半场', ['left', 'right'], 'left')

        section('KFS 配置')
        kfs_color = ask('KFS 颜色', ['blue', 'red'], 'blue')
        kfs_real  = ask('真 KFS 台阶编号 (空格分隔, 如: 5 8)',  default='5 8')
        kfs_fake  = ask('假 KFS 台阶编号 (空格分隔, 如: 2 11)', default='2 11')

        test_area     = 'full'
        use_gc        = 'true'
        enable_serial = 'true'
        debug_gui     = 'false'

    # ════════════════════════════════════════════════════
    #   调试模式
    # ════════════════════════════════════════════════════
    else:
        section('半场')
        field_side = ask('场地半场', ['left', 'right'], 'left')

        section('测试区域')
        print(f'  {DIM}full=完整比赛  weapon=武馆  merlin=梅林  confront=对抗区{N}')
        test_area = ask('测试区域', ['full', 'weapon', 'merlin', 'confront'], 'full')

        section('KFS 配置')
        kfs_color = ask('KFS 颜色', ['blue', 'red'], 'blue')
        kfs_real  = ask('真 KFS 台阶编号 (空格分隔, 如: 5 8)',  default='5 8')
        kfs_fake  = ask('假 KFS 台阶编号 (空格分隔, 如: 2 11)', default='2 11')

        section('功能开关')
        gc_default     = 'true'
        serial_default = 'false' if test_area != 'full' else 'true'

        use_gc        = ask('启动比赛状态机 (game_controller)', ['true', 'false'], gc_default)
        enable_serial = ask('启动串口通信 (连接 STM32)',          ['true', 'false'], serial_default)
        debug_gui     = 'true'

    # ════════════════════════════════════════════════════
    #   组装参数
    # ════════════════════════════════════════════════════
    params = [
        ('field_side',          field_side),
        ('test_area',           test_area),
        ('kfs_color',           kfs_color),
        ('kfs_real',            kfs_real),
        ('kfs_fake',            kfs_fake),
        ('use_game_controller', use_gc),
        ('enable_serial',       enable_serial),
        ('debug_gui',           debug_gui),
    ]

    # ════════════════════════════════════════════════════
    #   预览
    # ════════════════════════════════════════════════════
    arg_lines = ' \\\n    '.join(f'{k}:={v}' for k, v in params)
    preview   = f'ros2 launch rc2026_bringup full_system.launch.py \\\n    {arg_lines}'

    mode_label = clr('比赛模式', R) if mode == 'competition' else clr('调试模式', C)
    print(f'\n{G}┌──────────────────── 即将执行 ({mode_label}{G}) ─────────────────┐{N}')
    for line in preview.split('\n'):
        print(f'{G}│{N}  {W}{line}{N}')
    print(f'{G}└────────────────────────────────────────────────────┘{N}')

    serial_clr = G if enable_serial == 'true' else DIM
    gc_clr     = G if use_gc        == 'true' else DIM
    extra = f'  {clr("相机预览已开启", C)}' if debug_gui == 'true' else ''
    print(
        f'\n  半场 {clr(field_side.upper(), Y)}  '
        f'区域 {clr(test_area, Y)}  '
        f'颜色 {clr(kfs_color, Y)}  '
        f'真={clr(kfs_real, G)}  假={clr(kfs_fake, R)}\n'
        f'  状态机 {clr(use_gc, gc_clr)}  '
        f'串口 {clr(enable_serial, serial_clr)}'
        f'{extra}'
    )

    if debug_gui == 'true':
        print(f'\n  {DIM}调试模式将自动启动:'
              f'\n    · rqt_image_view → D435i (/camera/camera/color/image_raw)'
              f'\n    · OpenCV 窗口    → USB相机 (/dev/video0)'
              f'\n    · fine_align_node debug_gui 窗口(精对齐激活后显示){N}')

    # ════════════════════════════════════════════════════
    #   确认并启动
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

    kfs_real_csv  = kfs_real.strip().replace(' ', ',')
    kfs_fake_csv  = kfs_fake.strip().replace(' ', ',')
    kfs_topic_msg = f'real:{kfs_real_csv} fake:{kfs_fake_csv} color:{kfs_color}'

    proc = subprocess.Popen(argv, env=os.environ.copy())

    threading.Thread(target=_send_kfs, args=(proc, kfs_topic_msg), daemon=True).start()

    if debug_gui == 'true':
        threading.Thread(target=_start_debug_viewers, args=(proc, 0), daemon=True).start()

    try:
        proc.wait()
    except KeyboardInterrupt:
        print(f'\n{Y}[RC2026] 正在关闭...{N}')
        proc.terminate()
        proc.wait()


if __name__ == '__main__':
    main()
