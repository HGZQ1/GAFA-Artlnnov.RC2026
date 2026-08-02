import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8

import serial
from serial.tools import list_ports


BAUDRATE = 115200
ENTER_EXTERNAL_LEARN = bytes.fromhex("68 07 00 FF 20 1F 16")
EXIT_EXTERNAL_LEARN = bytes.fromhex("68 07 00 FF 21 20 16")

TEMPLATES = {
    "KEY1": bytes.fromhex(
        "68 7B 00 00 22 7A 48 51 8C 01 4D 47 4E 6B 52 8C 01 46 4B 4A 4A 4E 6B 52 67 4D 8D 01 4B BB 01 4E FF 0B 7E 45 49 94 01 4D 47 4E 6B 4C 8E 01 51 43 4A 4E 4A 6B 4D 6A 4E 8F 01 4A B8 01 50 FC 0B 80 01 46 4A 91 01 51 44 4A 6F 4A 91 01 4D 47 52 43 4E 6B 51 68 46 94 01 51 B8 01 44 83 0C 80 01 46 51 8C 01 51 44 4D 6A 51 8C 01 4D 47 49 4B 4D 69 49 73 4E 8C 01 50 B5 01 50 12 16"
    ),
    "KEY2": bytes.fromhex(
        "68 7B 00 00 22 7E 45 4D 90 01 52 43 4D 6B 4D 8E 01 51 43 51 43 4E 6B 4D 8E 01 51 8C 01 51 44 4D C8 0C 7F 48 49 90 01 52 42 4E 6B 4D 8D 01 51 44 4E 47 4E 6B 4E 8D 01 50 90 01 4A 47 4E C8 0C 83 01 43 51 8C 01 4A 48 4D 6B 51 8C 01 4D 48 49 48 50 68 51 8D 01 4D 8D 01 51 43 51 C7 0C 82 01 44 4D 90 01 4D 48 4D 69 50 8D 01 50 44 4C 48 49 6C 4D 90 01 51 8B 01 50 44 50 C0 16"
    ),
    "KEY3": bytes.fromhex(
        "68 7A 00 00 22 7F 43 4E 90 01 4D 48 49 6C 51 8D 01 51 44 4D 47 4A 91 01 51 B5 01 4A 6C 51 68 51 FB 0B 7C 47 4D 90 01 4E 47 4D 69 4D 90 01 4D 47 4D 47 4D 8E 01 4D B8 01 4D 6A 50 68 51 FC 0B 80 01 44 4D 90 01 51 43 4D 6C 4D 8D 01 4D 47 50 44 51 8A 01 53 B2 01 4D 69 4C 6C 50 FD 0B 7B 48 4D 90 01 50 44 51 68 49 91 01 50 44 51 44 50 8B 01 4F B5 01 4D 6C 51 68 4A 83 16"
    ),
}

START_SIGNAL_NAMES = {
    1: '启动指令',
}

R1_SIGNAL_NAMES = {
    2: '进入梅林',
    3: '合体后释放KFS指令',
}


def frame_payload(frame):
    # 68 len_l len_h 00 22 payload checksum 16
    return frame[5:-2]


def decode_durations(data):
    durations = []
    index = 0
    while index < len(data):
        value = data[index]
        if value >= 0x80 and index + 1 < len(data):
            durations.append(value | (data[index + 1] << 8))
            index += 2
        else:
            durations.append(value)
            index += 1
    return durations


def duration_symbol(duration):
    if duration < 95:
        return "S"
    if duration < 180:
        return "M"
    if duration < 800:
        return "L"
    return "G"


def feature_sequence(frame):
    return "".join(duration_symbol(value) for value in decode_durations(frame_payload(frame)))


def frame_to_hex(frame):
    return " ".join(f"{byte:02X}" for byte in frame)


def checksum_ok(frame):
    return len(frame) >= 7 and frame[-1] == 0x16 and ((sum(frame[3:-2]) & 0xFF) == frame[-2])


def read_frame(port):
    while rclpy.ok():
        byte = port.read(1)
        if not byte:
            return None
        if byte[0] == 0x68:
            break

    length_bytes = port.read(2)
    if len(length_bytes) != 2:
        return None

    frame_length = length_bytes[0] | (length_bytes[1] << 8)
    if frame_length < 7 or frame_length > 512:
        return None

    rest = port.read(frame_length - 3)
    if len(rest) != frame_length - 3:
        return None

    return bytes([0x68]) + length_bytes + rest


def levenshtein(a, b):
    previous = list(range(len(b) + 1))
    for row_index, a_char in enumerate(a, 1):
        current = [row_index]
        for col_index, b_char in enumerate(b, 1):
            current.append(
                min(
                    previous[col_index] + 1,
                    current[-1] + 1,
                    previous[col_index - 1] + (0 if a_char == b_char else 1),
                )
            )
        previous = current
    return previous[-1]


UNIT_FEATURES = {
    "KEY1": "MSSLSSSMSLSSSSSMSMSLSLS",
    "KEY2": "MSSLSSSMSLSSSSSMSLSLSSS",
    "KEY3": "MSSLSSSMSLSSSSSLSLSMSMS",
}


def unit_match_score(sequence, unit):
    best_distance = 999
    hit_count = 0
    unit_len = len(unit)
    min_window = max(1, unit_len - 3)
    max_window = unit_len + 3

    for window_len in range(min_window, max_window + 1):
        if len(sequence) < window_len:
            continue
        for start in range(0, len(sequence) - window_len + 1):
            distance = levenshtein(sequence[start:start + window_len], unit)
            if distance < best_distance:
                best_distance = distance
            if distance <= 1:
                hit_count += 1

    return float(best_distance), hit_count


def compare_score(frame, key_name):
    return unit_match_score(feature_sequence(frame), UNIT_FEATURES[key_name])


def classify(frame, max_score, min_gap, preferred_key="", preferred_margin=0.0):
    scored = []
    for name in UNIT_FEATURES:
        distance, hits = compare_score(frame, name)
        scored.append((distance, -hits, name, hits))

    scores = sorted(
        scored,
        key=lambda item: (item[0], item[1], item[2]),
    )
    best_score, _best_hits_sort, best_name, best_hits = scores[0]
    second_score = scores[1][0] if len(scores) > 1 else 9999.0
    if preferred_key in UNIT_FEATURES:
        for distance, _neg_hits, name, hits in scores:
            if name != preferred_key:
                continue
            if (distance <= max_score
                    and hits > 0
                    and distance - best_score <= preferred_margin):
                return preferred_key, scores
            break

    accepted = best_score <= max_score and (second_score - best_score) >= min_gap and best_hits > 0
    return best_name if accepted else "UNKNOWN", scores


def scores_to_text(scores):
    return ", ".join(
        f"{name}:d{distance:.0f}/h{hits}"
        for distance, _neg_hits, name, hits in scores
    )


def available_ports_text():
    ports = list(list_ports.comports())
    if not ports:
        return "未发现串口设备"
    return "; ".join(f"{item.device}({item.description})" for item in ports)


class IrKeyReceiver(Node):
    def __init__(self):
        super().__init__('ir_key_receiver')

        self.declare_parameter('port', '')
        self.declare_parameter('baud', BAUDRATE)
        self.declare_parameter('max_score', 1.0)
        self.declare_parameter('min_gap', 1.0)
        self.declare_parameter('preferred_key', '')
        self.declare_parameter('preferred_margin', 0.0)
        self.declare_parameter('repeat_guard_s', 1.0)
        self.declare_parameter('publish_unknown_raw', False)
        self.declare_parameter('module_name', 'main')
        self.declare_parameter('key1_start_value', 1)
        self.declare_parameter('key2_signal', 2)
        self.declare_parameter('key3_signal', 3)

        self.port_name = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.max_score = float(self.get_parameter('max_score').value)
        self.min_gap = float(self.get_parameter('min_gap').value)
        self.preferred_key = str(self.get_parameter('preferred_key').value).strip().upper()
        self.preferred_margin = float(self.get_parameter('preferred_margin').value)
        self.repeat_guard_s = float(self.get_parameter('repeat_guard_s').value)
        self.publish_unknown_raw = bool(self.get_parameter('publish_unknown_raw').value)
        self.module_name = str(self.get_parameter('module_name').value).strip() or 'main'
        self.key1_start_value = int(self.get_parameter('key1_start_value').value)
        self.key_to_signal = {
            'KEY2': int(self.get_parameter('key2_signal').value),
            'KEY3': int(self.get_parameter('key3_signal').value),
        }

        self.start_publisher = self.create_publisher(UInt8, '/game/start_signal', 10)
        self.r1_publisher = self.create_publisher(UInt8, '/game/r1_signal', 10)
        self._last_key = None
        self._last_pub_time = 0.0

        self.get_logger().info(
            f'红外模块[{self.module_name}]信号映射: '
            f'KEY1->/game/start_signal:{self.key1_start_value}, '
            f'KEY2->/game/r1_signal:{self.key_to_signal["KEY2"]}, '
            f'KEY3->/game/r1_signal:{self.key_to_signal["KEY3"]}, '
            f'preferred={self.preferred_key or "-"} '
            f'margin={self.preferred_margin:.1f}')

    def run(self):
        if not self.port_name:
            self.get_logger().error(
                '未设置红外模块串口参数 ir_port/port。当前可用串口: '
                + available_ports_text())
            return 2

        try:
            with serial.Serial(self.port_name, self.baud, timeout=0.15) as port:
                self.get_logger().info(
                    f'红外学习模块已打开: {self.port_name}, baud={self.baud}')
                self._receive_loop(port)
        except serial.SerialException as exc:
            self.get_logger().error(
                f'打开红外模块串口失败: {self.port_name}, baud={self.baud}, error={exc}')
            self.get_logger().error('当前可用串口: ' + available_ports_text())
            return 2
        return 0

    def _receive_loop(self, port):
        last_arm_time = 0.0
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
                now = time.time()
                if now - last_arm_time >= 0.8:
                    port.write(ENTER_EXTERNAL_LEARN)
                    last_arm_time = now

                frame = read_frame(port)
                if frame is None:
                    continue

                if not checksum_ok(frame):
                    self.get_logger().warn(
                        f'红外坏帧 len={len(frame)} hex={frame_to_hex(frame)}')
                    continue

                command = frame[4] if len(frame) > 4 else None
                if command != 0x22:
                    continue

                key_name, scores = classify(
                    frame,
                    self.max_score,
                    self.min_gap,
                    self.preferred_key,
                    self.preferred_margin)
                score_text = scores_to_text(scores)

                if key_name == 'UNKNOWN':
                    best_score, _neg_hits, best_name, best_hits = scores[0]
                    second_score = scores[1][0] if len(scores) > 1 else 9999.0
                    if best_score > self.max_score:
                        reason = f'最佳匹配{best_name}距离{best_score:.0f}超过阈值{self.max_score:.0f}'
                    elif best_hits <= 0:
                        reason = f'最佳匹配{best_name}没有命中特征单元'
                    else:
                        reason = (
                            f'最佳/次佳分差{second_score - best_score:.1f}'
                            f'小于阈值{self.min_gap:.1f}')
                    self.get_logger().warn(
                        f'红外未知按键 len={len(frame)} {reason} scores[{score_text}]')
                    if self.publish_unknown_raw:
                        self.get_logger().warn(f'raw: {frame_to_hex(frame)}')
                else:
                    self._publish_key(key_name, score_text)

                port.write(ENTER_EXTERNAL_LEARN)
                last_arm_time = time.time()
        finally:
            try:
                port.write(EXIT_EXTERNAL_LEARN)
            except serial.SerialException:
                pass
            if rclpy.ok():
                self.get_logger().info('红外学习模块已停止')

    def _publish_key(self, key_name, score_text):
        now = time.time()
        if self._last_key == key_name and now - self._last_pub_time < self.repeat_guard_s:
            self.get_logger().debug(f'红外 {key_name} 重复触发已抑制')
            return

        if key_name == 'KEY1':
            if self.key1_start_value < 0:
                self.get_logger().info(
                    f'红外模块[{self.module_name}] {key_name} 已识别但配置为忽略 '
                    f'scores[{score_text}]')
                return
            msg = UInt8()
            msg.data = max(0, min(255, self.key1_start_value))
            self.start_publisher.publish(msg)
            self._last_key = key_name
            self._last_pub_time = now
            signal_name = START_SIGNAL_NAMES.get(msg.data, f'未知启动信号({msg.data})')
            self.get_logger().info(
                f'收到红外模块[{self.module_name}]启动信号 {key_name}: {signal_name}, '
                f'/game/start_signal data={msg.data} scores[{score_text}]')
            return

        signal = self.key_to_signal.get(key_name, -1)
        if signal < 0:
            self.get_logger().info(
                f'红外模块[{self.module_name}] {key_name} 已识别但配置为忽略 '
                f'scores[{score_text}]')
            return

        msg = UInt8()
        msg.data = max(0, min(255, signal))
        self.r1_publisher.publish(msg)
        self._last_key = key_name
        self._last_pub_time = now
        signal_name = R1_SIGNAL_NAMES.get(msg.data, f'未知R1信号({msg.data})')
        self.get_logger().info(
            f'收到红外模块[{self.module_name}]R1机器人指令 {key_name}: {signal_name}, '
            f'/game/r1_signal data={msg.data} scores[{score_text}]')


def main(args=None):
    rclpy.init(args=args)
    node = IrKeyReceiver()
    try:
        return_code = node.run()
    except KeyboardInterrupt:
        return_code = 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return return_code


if __name__ == '__main__':
    raise SystemExit(main())
