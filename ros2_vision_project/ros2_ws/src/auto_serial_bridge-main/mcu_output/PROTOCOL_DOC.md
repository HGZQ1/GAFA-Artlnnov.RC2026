> 生成时间：2026-04-08T22:28:19+08:00
# MCU ↔ ROS 串口通信协议文档

> **Auto-generated** — 由 `scripts/codegen.py` 根据 `config/protocol.yaml` 生成，请勿手动修改。

---

## 全局参数

| 参数 | 值 |
| :--- | :--- |
| 波特率 | `115200` |
| 帧头字节 1 | `0x5a` |
| 帧头字节 2 | `0xa5` |
| 校验算法 | `CRC8` |
| 强制握手 | `否` |
| 协议哈希（握手用）| `0x2669E2D8` |

---

## 帧格式

每帧结构如下（小端序）：

| 字节位置 | 字段 | 说明 |
| :------: | :--- | :--- |
| 0 | Header1 | 固定 `0x5a` |
| 1 | Header2 | 固定 `0xa5` |
| 2 | ID | 消息 ID，见下表 |
| 3 | Len | 数据段字节数 |
| 4 … 4+Len-1 | Data | 各字段按结构体内存布局排列 |
| 4+Len | Checksum | CRC8，覆盖 ID + Len + Data，多项式 `0x31` |

---

## 电控 → ROS（电控主动发送）

### `Handshake` — ID `0xff`

- **ROS 话题**：`/task/handshake`
- **ROS 消息类型**：`std_msgs/msg/UInt32`
- **数据段字节数（Len）**：`4`

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `protocol_hash` | `uint32_t` | 4 |
| **4** | *(CRC8)* | `uint8_t` | 1 |

### `GripperStatus` — ID `0x10`

- **ROS 话题**：`/feedback/gripper`
- **ROS 消息类型**：`std_msgs/msg/UInt8`
- **数据段字节数（Len）**：`1`

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `status` | `uint8_t` | 1 |
| **1** | *(CRC8)* | `uint8_t` | 1 |

### `AssemblyStatus` — ID `0x11`

- **ROS 话题**：`/feedback/assembly`
- **ROS 消息类型**：`std_msgs/msg/UInt8`
- **数据段字节数（Len）**：`1`

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `status` | `uint8_t` | 1 |
| **1** | *(CRC8)* | `uint8_t` | 1 |

### `EncoderFeedback` — ID `0x20`

- **ROS 话题**：`/feedback/encoder`
- **ROS 消息类型**：`geometry_msgs/msg/Twist`
- **数据段字节数（Len）**：`12`

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `delta_x` | `float` | 4 |
| 4 | `delta_y` | `float` | 4 |
| 8 | `delta_theta` | `float` | 4 |
| **12** | *(CRC8)* | `uint8_t` | 1 |

---

## ROS → 电控（电控被动接收）

### `Heartbeat` — ID `0x00`

- **ROS 话题**：`/task/heartbeat`
- **ROS 消息类型**：`std_msgs/msg/UInt32`
- **数据段字节数（Len）**：`4`
- **默认生成行为**：`on_receive_Heartbeat()` 会自动调用 `send_Heartbeat(pkt)`，按原样回同一个 `count` 作为 ACK。

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `count` | `uint32_t` | 4 |
| **4** | *(CRC8)* | `uint8_t` | 1 |

### `Handshake` — ID `0xff`

- **ROS 话题**：`/task/handshake`
- **ROS 消息类型**：`std_msgs/msg/UInt32`
- **数据段字节数（Len）**：`4`

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `protocol_hash` | `uint32_t` | 4 |
| **4** | *(CRC8)* | `uint8_t` | 1 |

### `CmdVel` — ID `0x01`

- **ROS 话题**：`/serial/chassis_cmd`
- **ROS 消息类型**：`geometry_msgs/msg/Twist`
- **数据段字节数（Len）**：`24`

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `turn_angle` | `float` | 4 |
| 4 | `turn_wheels` | `float` | 4 |
| 8 | `forward_dist` | `float` | 4 |
| 12 | `drive_wheels` | `float` | 4 |
| 16 | `pickup_action` | `float` | 4 |
| 20 | `search_rotate` | `float` | 4 |
| **24** | *(CRC8)* | `uint8_t` | 1 |

### `SlopeInfo` — ID `0x03`

- **ROS 话题**：`/serial/slope_info`
- **ROS 消息类型**：`geometry_msgs/msg/Vector3`
- **数据段字节数（Len）**：`12`

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `angle_deg` | `float` | 4 |
| 4 | `horiz_dist` | `float` | 4 |
| 8 | `height_diff` | `float` | 4 |
| **12** | *(CRC8)* | `uint8_t` | 1 |

### `MeilinCmd` — ID `0x04`

- **ROS 话题**：`/serial/meilin_cmd`
- **ROS 消息类型**：`geometry_msgs/msg/Twist`
- **数据段字节数（Len）**：`20`

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `next_block` | `float` | 4 |
| 4 | `climb_mode` | `float` | 4 |
| 8 | `slope_angle` | `float` | 4 |
| 12 | `block_height` | `float` | 4 |
| 16 | `detour` | `float` | 4 |
| **20** | *(CRC8)* | `uint8_t` | 1 |

### `ClimbCmd` — ID `0x05`

- **ROS 话题**：`/serial/climb_cmd`
- **ROS 消息类型**：`geometry_msgs/msg/Twist`
- **数据段字节数（Len）**：`24`

| 字节偏移 | 字段名 | C 类型 | 字节数 |
| :------: | :----- | :----- | :----: |
| 0 | `climb_mode` | `float` | 4 |
| 4 | `slope_deg` | `float` | 4 |
| 8 | `speed_factor` | `float` | 4 |
| 12 | `torque_factor` | `float` | 4 |
| 16 | `nav_mode` | `float` | 4 |
| 20 | `height_diff` | `float` | 4 |
| **24** | *(CRC8)* | `uint8_t` | 1 |

---

*文档由构建系统自动生成，版本以协议哈希为准。*
