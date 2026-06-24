# RC2026 导航与决策系统 v4.0 设计文档

> 基于 4.0.md 需求分析，结合当前代码库实际状态编写
> 更新日期：2026-05-27

---

## 一、当前项目结构与各文件职责

### 1.1 ROS2 包一览

```
ros2_ws/src/
├── rc2026_bringup/          # 系统启动与配置管理
├── rc2026_navigation/       # 导航栈 (FAST-LIO + Nav2)
├── vision_detector/         # YOLOv8 视觉检测
├── decision_processor/      # 决策系统
├── cmd_vel_bridge/          # Nav2速度→底盘PID→串口指令
└── auto_serial_bridge-main/ # 通用串口桥 (协议自动生成)
```

### 1.2 各文件职责详解

#### rc2026_bringup（系统启动）

| 文件 | 职责 |
|------|------|
| `launch/robot_bringup.launch.py` | 全系统启动入口：URDF + 相机 + YOLO + 决策 + 串口 + 导航(可选) |
| `launch/sensor_only.launch.py` | 仅传感器启动（调试用） |
| `config/robot_params.yaml` | 底盘/相机/雷达/PID/IMU 全部物理参数集中管理 |
| `config/MID360s_config.json` | Livox Mid-360S 雷达网络配置 |
| `config/measurement_params.yaml` | 测量参数 |
| `map/arena.yaml + arena.pgm` | 全场地图 (12m×6m) |
| `map/field_waypoints.yaml` | 关键坐标点：12个梅林方块中心、起点、端头架、九宫格、坡道入口 |
| `rviz/navigation.rviz` | RViz 导航可视化配置 |
| `urdf/rc2026_robot.urdf.xacro` | 机器人 URDF 模型 |

#### rc2026_navigation（导航栈）

| 文件 | 职责 |
|------|------|
| `launch/fastlio.launch.py` | 启动 FAST-LIO2 定位 |
| `launch/nav2.launch.py` | 启动 Nav2（无AMCL，用FAST-LIO定位）+ TF桥接 + pointcloud→laserscan |
| `config/nav2/planner.yaml` | Nav2 全参数：planner/controller/costmap/behavior/bt_navigator |
| `config/fastlio2/mapping.yaml` | FAST-LIO 建图参数 |
| `config/fastlio2/localization.yaml` | FAST-LIO 定位参数 |

#### vision_detector（视觉检测）

| 文件 | 职责 |
|------|------|
| `vision_detector/detector_node.py` | ROS2 检测节点：订阅图像 → YOLO推理 → 发布目标 |
| `vision_detector/yolov8_detector.py` | YOLOv8 推理封装 |
| `vision_detector/model_switcher.py` | 运行时切换 YOLO 模型（wuqi.pt ↔ kfs.pt） |
| `vision_detector/utils.py` | 工具函数 |

#### decision_processor（决策系统）

| 文件 | 职责 | 是否需改造 |
|------|------|-----------|
| `processor_node.py` | 决策主节点：接收感知数据 → 场景评估 → 输出底盘/机械臂指令 | **大改** |
| `robot_decision.py` | 5状态FSM：SEARCHING→ALIGNING→MOVING→ARRIVED→PICKING | **保留**，作为子状态机 |
| `config.py` | 所有参数集中管理（坐标、类别、阈值、路径等） | **扩展** |
| `meilin_navigator.py` | 梅林方块位置跟踪与导航状态机 | **大改** |
| `motion_planner.py` | 运动规划：对齐→前进 + 坡度感知 + 梯形速度 | 保留 |
| `kalman_filter.py` | 2D 卡尔曼滤波（目标位置平滑） | 保留 |
| `target_confirmation.py` | 目标确认（多帧确认/丢失检测） | 保留 |
| `tf_manager.py` | TF 坐标变换管理 | 保留 |
| `imu_processor.py` | IMU 数据处理（坡度/姿态估计） | 保留 |
| `odometry_fusion.py` | 里程计融合（编码器+IMU） | 保留 |
| `scenarios/scenario_wuguan.py` | 武馆场景：目标评估（类别/置信度→PICK/IGNORE） | 保留 |
| `scenarios/scenario_meilin.py` | 梅林场景：真假KFS评估（REAL→PICK，FAKE→AVOID） | 保留 |
| `scenarios/base_scenario.py` | 场景基类 | 保留 |

#### cmd_vel_bridge（底盘控制桥）

| 文件 | 职责 |
|------|------|
| `bridge_node.py` | `/cmd_vel` → PID闭环 → `/serial/chassis_cmd`，支持仿真模式 |
| `chassis_kinematics.py` | 全向轮/麦轮逆运动学（底盘速度→四轮转速） |
| `pid_controller.py` | PID 控制器实现 |
| `wheel_odom_publisher.py` | 轮式里程计发布 |

#### auto_serial_bridge（串口桥）

| 文件 | 职责 |
|------|------|
| `config/protocol.yaml` | 串口协议定义（消息ID/字段/方向/话题映射） |

---

## 二、信息传递流（数据流图）

### 2.1 当前数据流

```
┌─────────────┐     ┌──────────────┐     ┌─────────────────┐
│ Livox Mid360S│────→│  FAST-LIO2   │────→│  Nav2 导航栈     │
│  (点云+IMU)  │     │ (定位+建图)   │     │ (路径规划+控制)  │
└─────────────┘     └──────┬───────┘     └────────┬────────┘
                           │                       │
                    /Odometry               /cmd_vel
                    /cloud_registered              │
                           │                       ▼
                           │              ┌─────────────────┐
                           │              │  cmd_vel_bridge  │
                           │              │  (PID闭环控制)    │
                           │              └────────┬────────┘
                           │               /serial/chassis_cmd
┌─────────────┐            │                       │
│  RealSense  │            │                       ▼
│  D435i      │     ┌──────▼───────┐     ┌─────────────────┐
│ (RGB+深度)  │────→│  YOLO 检测    │     │  serial_bridge   │
└─────────────┘     └──────┬───────┘     │  (串口通信)       │
                           │             └────────┬────────┘
                  /vision/raw_target               │ UART
                           │                       ▼
                    ┌──────▼───────┐     ┌─────────────────┐
                    │ 决策处理器    │     │    STM32        │
                    │(processor_   │     │  (电机+机械臂)   │
                    │  node)       │────→│                 │
                    └──────────────┘     └─────────────────┘
                           ▲
                    /imu/processed
                    /odom/fused
                    /meilin/nav_state
```

### 2.2 v4.0 目标数据流（新增红色部分）

```
┌──────────────────────────────────────────────────────────────┐
│                    比赛总控 game_controller (新增)             │
│  IDLE → WAIT_INPUT → WAIT_START → WEAPON_GRAB →             │
│  WEAPON_ASSEMBLE → ENTER_MEILIN → MEILIN_TRAVERSE →         │
│  KFS_PICKUP → EXIT_MEILIN → STOP                            │
│                                                              │
│  - 通过 Nav2 ActionClient 发送导航目标                         │
│  - 控制 YOLO 模型切换 (wuqi.pt ↔ kfs.pt)                     │
│  - 4分10秒全局超时                                            │
│  - R1 信号处理                                                │
└──────────────┬───────────────────────────┬───────────────────┘
               │                           │
    NavigateToPose                /vision/switch_model
    (Nav2 Action)                          │
               │                           ▼
               ▼                  ┌─────────────────┐
     ┌─────────────────┐         │  YOLO 检测       │
     │  Nav2 导航栈     │         │  (模型自动切换)   │
     │  → /cmd_vel      │         └────────┬────────┘
     └────────┬────────┘                   │
              │                   /vision/raw_target
              ▼                            │
     ┌─────────────────┐         ┌────────▼────────┐
     │  cmd_vel_bridge  │         │ processor_node  │
     │  → /serial/      │         │ (视觉子决策)     │
     │    chassis_cmd   │         └────────┬────────┘
     └────────┬────────┘          /serial/arm_gripper_cmd
              │                   /serial/climb_cmd
              ▼                            │
     ┌─────────────────┐                   │
     │  serial_bridge   │◄─────────────────┘
     │  ↕ STM32         │
     └─────────────────┘
              ▲
       /feedback/gripper          (新增: 夹取完成信号)
       /feedback/assembly         (新增: 组装完成信号)
       /feedback/stm32_start      (新增: 开始按钮信号)
       /feedback/chassis_vel      (已有: PID反馈)
```

### 2.3 关键话题列表

#### Jetson → STM32 (TX)

| 话题 | 消息ID | 内容 | 使用阶段 |
|------|--------|------|---------|
| `/serial/chassis_cmd` | 0x01 | vx, vy, omega, mode | 全程 |
| `/serial/slope_info` | 0x03 | 坡度角, 水平距离, 高度差 | 梅林 |
| `/serial/meilin_cmd` | 0x04 | 下一方块, 爬升模式, 坡度, 高度差 | 梅林 |
| `/serial/climb_cmd` | 0x05 | 爬升模式, 坡度, 速度系数, 力矩系数 | 梅林 |
| `/serial/arm_joint_cmd` | 0x06 | 6轴关节角度 | 武馆/KFS拾取 |
| `/serial/arm_gripper_cmd` | 0x07 | 0=打开 1=关闭 2=半开 | 武馆/KFS拾取 |

#### STM32 → Jetson (RX)

| 话题 | 消息ID | 内容 | 使用阶段 |
|------|--------|------|---------|
| `/feedback/gripper` | 0x10 | 0=空载 1=已抓取 2=已完成组装 3=错误 | 武馆 |
| `/feedback/assembly` | 0x11 | 0=未组装 1=组装中 2=组装完成 3=失败 | 武馆 |
| `/feedback/encoder` | 0x20 | delta_x, delta_y, delta_theta | 全程 |
| `/feedback/arm_status` | 0x22 | 0=空闲 1=运动中 2=已抓取 3=已释放 4=错误 | 全程 |
| `/feedback/chassis_vel` | 0x24 | vx_actual, vy_actual, omega_actual | 全程 |
| `/feedback/wheel_odom` | 0x25 | pos_x, pos_y, yaw, vx, vy, omega | 全程 |

---

## 三、需要新建的模块

### 3.1 比赛总控状态机 game_controller.py（新建）

**位置**: `decision_processor/decision_processor/game_controller.py`

**职责**: 比赛全流程编排，是所有其他模块的调度者

**状态定义**:

```
IDLE             系统启动，硬件初始化中
WAIT_INPUT       等待操作员输入梅林KFS标签
WAIT_START       等待开始信号（下位机按钮 或 终端输入'q'）
─── 以下为正式比赛，开始4分钟计时 ───
NAV_TO_WEAPON    导航到端头架坐标 (0.65, 10.45)
WEAPON_ALIGN     视觉粗对齐端头 (用深度相机)
WEAPON_GRAB      向下位机发送夹取指令，等待成功信号
NAV_TO_ASSEMBLY  导航到组装点 (5.6, 11.6)
WAIT_ASSEMBLY    等待R1组装完成信号 → 松开夹爪
WAIT_ENTER_CMD   等待R1发送"进入梅林"信号
NAV_TO_MEILIN    导航到梅林入口 (3.0, 10.0)
MEILIN_TRAVERSE  梅林方块穿越（由 meilin_controller 子状态机接管）
EXIT_MEILIN      从出口方块导航离开梅林区域
STOP             比赛结束或超时，底盘停止，发送"复位"指令
```

**关键接口**:
- Nav2 `NavigateToPose` ActionClient：发送导航目标，等完成
- `/vision/switch_model` 发布器：切换YOLO模型
- `/feedback/gripper` 订阅：夹取结果
- `/feedback/assembly` 订阅：组装结果
- 全局超时：比赛开始后 4分10秒 → 强制进入 STOP

**设计要点**:
- 导航阶段：game_controller 发 Nav2 目标，cmd_vel 由 Nav2 控制
- 视觉对齐阶段：game_controller 暂停 Nav2，cmd_vel 交给 processor_node
- 两者不同时控制 /cmd_vel，通过状态切换避免冲突

**实现方式**:
```python
class GamePhase(Enum):
    IDLE = auto()
    WAIT_INPUT = auto()
    WAIT_START = auto()
    NAV_TO_WEAPON = auto()
    WEAPON_ALIGN = auto()
    WEAPON_GRAB = auto()
    NAV_TO_ASSEMBLY = auto()
    WAIT_ASSEMBLY = auto()
    WAIT_ENTER_CMD = auto()
    NAV_TO_MEILIN = auto()
    MEILIN_TRAVERSE = auto()
    EXIT_MEILIN = auto()
    STOP = auto()

class GameController(Node):
    def __init__(self):
        # Nav2 action client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # 状态
        self.phase = GamePhase.IDLE
        self.match_start_time = None
        self.MATCH_TIMEOUT = 250.0  # 4分10秒
        # 赛前输入
        self.real_kfs_blocks = []
        self.fake_kfs_blocks = []
```

### 3.2 梅林网格路径规划器 meilin_path_planner.py（新建）

**位置**: `decision_processor/decision_processor/meilin_path_planner.py`

**职责**: 基于3×4网格和KFS信息，规划梅林穿越路径

**网格模型**:
```
        col0        col1        col2
row0:   [1]         [2]         [3]          入口行
        (1.8,8.2)   (3.0,8.2)   (4.2,8.2)

row1:   [4]         [5]         [6]
        (1.8,7.0)   (3.0,7.0)   (4.2,7.0)

row2:   [7]         [8]         [9]
        (1.8,5.8)   (3.0,5.8)   (4.2,5.8)

row3:   [10]        [11]        [12]         出口行
        (1.8,4.6)   (3.0,4.6)   (4.2,4.6)
```

**移动规则**:
- 前进: (row, col) → (row+1, col)
- 左移: (row, col) → (row, col-1)
- 右移: (row, col) → (row, col+1)
- 禁止后退 (row-1)

**算法: 两段BFS + 枚举必经点顺序**

```python
BLOCK_TO_GRID = {1:(0,0), 2:(0,1), 3:(0,2),
                 4:(1,0), 5:(1,1), 6:(1,2),
                 7:(2,0), 8:(2,1), 9:(2,2),
                 10:(3,0),11:(3,1),12:(3,2)}

ENTRY_BLOCKS = {1, 2, 3}
EXIT_BLOCKS  = {10, 11, 12}

def get_neighbors(block, obstacles):
    """可达的相邻方块 (前/左/右, 排除障碍)"""
    row, col = BLOCK_TO_GRID[block]
    moves = []
    if row + 1 <= 3: moves.append((row+1, col))  # 前
    if col - 1 >= 0: moves.append((row, col-1))   # 左
    if col + 1 <= 2: moves.append((row, col+1))   # 右
    return [grid_to_block(r,c) for r,c in moves
            if grid_to_block(r,c) not in obstacles]

def bfs(start, targets, obstacles):
    """BFS: start → targets 中任一节点的最短路径"""
    queue = deque([(start, [start])])
    visited = {start}
    while queue:
        node, path = queue.popleft()
        if node in targets:
            return path
        for nb in get_neighbors(node, obstacles):
            if nb not in visited:
                visited.add(nb)
                queue.append((nb, path + [nb]))
    return None  # 不可达

def plan_meilin_path(entry_block, real_kfs_blocks, fake_kfs_blocks):
    """
    规划梅林穿越路径
    entry_block: 入口方块 (1/2/3, 默认2)
    real_kfs_blocks: [5] 或 [3, 8] 等，必经点
    fake_kfs_blocks: [8] 等，障碍
    返回: [2, 5, 4, 7, 10] 依次经过的方块序列
    """
    obstacles = set(fake_kfs_blocks)
    waypoints = list(real_kfs_blocks)

    if len(waypoints) == 0:
        # 无必经点，直接找最短路到出口
        return bfs(entry_block, EXIT_BLOCKS, obstacles)

    if len(waypoints) == 1:
        # 入口→KFS→出口
        seg1 = bfs(entry_block, {waypoints[0]}, obstacles)
        seg2 = bfs(waypoints[0], EXIT_BLOCKS, obstacles)
        return seg1 + seg2[1:]  # 去重连接点

    if len(waypoints) == 2:
        # 枚举两种顺序，取总长最短
        best = None
        for order in [(0,1), (1,0)]:
            w = [waypoints[order[0]], waypoints[order[1]]]
            seg1 = bfs(entry_block, {w[0]}, obstacles)
            seg2 = bfs(w[0], {w[1]}, obstacles)
            seg3 = bfs(w[1], EXIT_BLOCKS, obstacles)
            if seg1 and seg2 and seg3:
                path = seg1 + seg2[1:] + seg3[1:]
                if best is None or len(path) < len(best):
                    best = path
        return best
```

**输出附加信息**: 路径确定后自动推导每段的爬升/下降指令

```python
def compute_climb_commands(path):
    """根据路径和高度表，生成每一步的爬升/下降指令"""
    commands = []
    for i in range(len(path) - 1):
        cur, nxt = path[i], path[i+1]
        h_cur = BLOCK_HEIGHTS[cur]
        h_nxt = BLOCK_HEIGHTS[nxt]
        diff = h_nxt - h_cur
        if diff > 0.15:
            cmd = 'CLIMB_2'   # 爬升40cm (如从地面到1/3号)
        elif diff > 0.05:
            cmd = 'CLIMB_1'   # 爬升20cm
        elif diff < -0.15:
            cmd = 'DESCEND_2' # 下降40cm (如从11号离开)
        elif diff < -0.05:
            cmd = 'DESCEND_1' # 下降20cm
        else:
            cmd = 'FLAT'
        commands.append((cur, nxt, cmd, diff))
    return commands
```

### 3.3 梅林穿越控制器 meilin_controller.py（新建，替代当前 meilin_navigator.py）

**位置**: `decision_processor/decision_processor/meilin_controller.py`

**职责**: 控制机器人在梅林方块间移动，执行KFS拾取

**与当前 meilin_navigator.py 的区别**:
- 当前版本：用里程计累计距离判断到达
- 新版本：用 **FAST-LIO 全局定位** (map→base_link TF) 判断在哪个方块

**方块判定逻辑** (来自4.0.md 第5条):
```python
def get_current_block(self, robot_x, robot_y):
    """根据全局坐标判断当前在哪个方块"""
    for block_id, (cx, cy) in BLOCK_CENTERS.items():
        if abs(robot_x - cx) < 0.6 and abs(robot_y - cy) < 0.6:
            return block_id
    return None  # 不在任何方块上
```

**子状态机**:
```
ENTER          从入口进入第一个方块 (默认2号)
NAV_TO_NEXT    导航到路径上下一个方块的中心坐标
APPROACH_EDGE  到达当前方块边缘40cm处，发送爬升/下降指令
WAIT_CLIMB     等待下位机完成爬升/下降动作
ON_BLOCK       到达方块顶部，检查是否为KFS拾取点
KFS_ALIGN      面向KFS方块，视觉精对齐
KFS_PICKUP     向下位机发送拾取指令，等待完成
NAV_TO_EXIT    路径走完，导航到出口
DONE           梅林穿越完成
```

**导航方式**:
- 方块间移动：直接用 Nav2 `NavigateToPose` 导航到下一个方块中心坐标
- 接近方块边缘时：切换为低速直线行驶 + 发爬升/下降指令
- KFS拾取：底盘停止，视觉对齐后由 processor_node 控制精调

### 3.4 赛前输入节点 match_input.py（新建）

**位置**: `decision_processor/decision_processor/match_input.py`

**职责**: 启动后阻塞等待操作员输入，然后发布到话题

```python
class MatchInput(Node):
    def __init__(self):
        self.pub = self.create_publisher(String, '/match/config', 10)

    def run_input(self):
        print("=" * 40)
        print("  RC2026 赛前配置")
        print("=" * 40)
        real = input("请输入真KFS所在台阶 (逗号分隔, 如 5 或 3,8): ")
        fake = input("请输入假KFS所在台阶 (逗号分隔, 如 8): ")
        # 发布配置
        msg = String()
        msg.data = json.dumps({"real_kfs": [...], "fake_kfs": [...]})
        self.pub.publish(msg)
        print(f"配置已发布: 真KFS={real}, 假KFS={fake}")
        print("等待开始信号... (按q+回车 或 下位机按钮)")
        # 等待启动
        while True:
            cmd = input()
            if cmd.strip().lower() == 'q':
                break
        start_msg = String()
        start_msg.data = "START"
        self.create_publisher(String, '/match/start', 10).publish(start_msg)
```

---

## 四、需要改造的现有模块

### 4.1 processor_node.py 改造

**当前问题**: processor_node 同时负责场景管理、cmd_vel 输出和目标处理，与 Nav2 的 cmd_vel 冲突。

**改造方案**:
- processor_node **不再直接发布** `/serial/chassis_cmd`，改为发布 `/decision/visual_cmd`
- 当 game_controller 处于视觉对齐阶段时，由 game_controller 将 `/decision/visual_cmd` 转发到 `/cmd_vel`
- 当 game_controller 处于导航阶段时，Nav2 控制 `/cmd_vel`，processor_node 的 cmd 被忽略

**具体改动**:
1. 删除 `self.chassis_pub` 对 `/serial/chassis_cmd` 的直接发布
2. 新增 `self.visual_cmd_pub` 发布到 `/decision/visual_cmd`
3. 场景切换改为由 game_controller 触发（不再自动切换）
4. 新增 `/decision/align_result` 发布器（通知 game_controller 对齐完成）

### 4.2 meilin_navigator.py → 废弃

被 `meilin_controller.py` + `meilin_path_planner.py` 替代。

**废弃原因**:
- 基于里程计的距离判断不够精确
- 只支持3条固定直线路径
- 不支持动态障碍物（假KFS）标记
- 不支持必经点约束

### 4.3 config.py 扩展

新增以下参数（4.0.md 要求"坐标点放在统一文件"）:

```python
# ══════════════════════════════════════════════════════════════
#   比赛流程坐标点（左半场）
# ══════════════════════════════════════════════════════════════
WAYPOINTS = {
    'start':          {'x': 1.4,  'y': 11.6, 'yaw': -1.5708},
    'weapon_rack':    {'x': 0.65, 'y': 10.45,'yaw':  3.1416},
    'assembly_point': {'x': 5.6,  'y': 11.6, 'yaw':  0.0},
    'meilin_entry':   {'x': 3.0,  'y': 10.0, 'yaw': -1.5708},
    'meilin_climb_2': {'x': 3.0,  'y':  9.2, 'yaw': -1.5708},
}

# 梅林方块中心坐标
BLOCK_CENTERS = {
    1:  (1.8, 8.2),   2:  (3.0, 8.2),   3:  (4.2, 8.2),
    4:  (1.8, 7.0),   5:  (3.0, 7.0),   6:  (4.2, 7.0),
    7:  (1.8, 5.8),   8:  (3.0, 5.8),   9:  (4.2, 5.8),
    10: (1.8, 4.6),  11:  (3.0, 4.6),  12:  (4.2, 4.6),
}

# ══════════════════════════════════════════════════════════════
#   爬升/下降指令定义 (4.0.md 标准)
# ══════════════════════════════════════════════════════════════
CLIMB_COMMANDS = {
    'CLIMB_1':   {'height_cm': 20, 'desc': '爬升20cm (相邻方块上升/从2号进入)'},
    'CLIMB_2':   {'height_cm': 40, 'desc': '爬升40cm (从1或3号进入)'},
    'DESCEND_1': {'height_cm': 20, 'desc': '下降20cm (相邻方块下降/从10或12离开)'},
    'DESCEND_2': {'height_cm': 40, 'desc': '下降40cm (从11号离开)'},
}

# 边缘触发距离 (到达当前台阶边缘多少米时发送爬升/下降指令)
CLIMB_TRIGGER_DIST = 0.40

# ══════════════════════════════════════════════════════════════
#   比赛超时
# ══════════════════════════════════════════════════════════════
MATCH_TOTAL_SECONDS = 250  # 4分10秒
```

### 4.4 robot_bringup.launch.py 改造

新增节点:
```python
# ── 比赛总控 (延迟6秒，等所有子系统就绪) ──
game_controller_node = TimerAction(
    period=6.0,
    actions=[Node(
        package='decision_processor',
        executable='game_controller',
        name='game_controller',
        output='screen',
    )],
)

# ── 赛前输入 (最先启动) ──
match_input_node = Node(
    package='decision_processor',
    executable='match_input',
    name='match_input',
    output='screen',
)
```

---

## 五、串口协议更新

### 5.1 需要新增的消息

| 名称 | ID | 方向 | 话题 | 内容 | 用途 |
|------|-----|------|------|------|------|
| `GameCmd` | 0x08 | TX | `/serial/game_cmd` | action(u8): 0=停止 1=复位 2=开始 | 比赛控制 |
| `ClimbAction` | 0x09 | TX | `/serial/climb_action` | action(u8): 0=FLAT 1=CLIMB_1 2=CLIMB_2 3=DESCEND_1 4=DESCEND_2 | 精确爬升/下降指令 |
| `R1Signal` | 0x12 | RX | `/feedback/r1_signal` | signal(u8): 0=无 1=组装完成 2=进入梅林 | R1通信 |
| `StartButton` | 0x13 | RX | `/feedback/start_button` | pressed(u8): 1=按下 | 开始按钮 |
| `ClimbFeedback` | 0x23 | RX | `/feedback/climb_status` | status(u8): 0=空闲 1=爬升中 2=完成 3=失败 | 爬升完成确认 |

### 5.2 需要修改的现有消息

| 名称 | 修改内容 |
|------|---------|
| `MeilinCmd` (0x04) | 新增字段: climb_action(u8) 对应 CLIMB_1/2/DESCEND_1/2 |

### 5.3 更新后完整串口通信表

#### TX: Jetson → STM32

| ID | 名称 | 字段 | 说明 |
|----|------|------|------|
| 0x00 | Heartbeat | count(u32) | 心跳 |
| 0x01 | CmdVel | vx, vy, omega, mode, pickup_action, search_rotate (6×f32) | 底盘速度指令 |
| 0x03 | SlopeInfo | angle_deg, horiz_dist, height_diff (3×f32) | 坡度信息 |
| 0x04 | MeilinCmd | next_block, climb_mode, slope_angle, block_height, detour (5×f32) | 梅林控制 |
| 0x05 | ClimbCmd | climb_mode, slope_deg, speed_factor, torque_factor, nav_mode, height_diff (6×f32) | 爬坡控制 |
| 0x06 | ArmJointCmd | joint_1~6 (6×f32) | 机械臂关节 |
| 0x07 | ArmGripperCmd | action(u8): 0=开 1=关 2=半开 | 夹爪控制 |
| **0x08** | **GameCmd** | **action(u8): 0=停 1=复位 2=开始** | **比赛控制 (新增)** |
| **0x09** | **ClimbAction** | **action(u8): 0~4 对应5种指令** | **精确爬升 (新增)** |

#### RX: STM32 → Jetson

| ID | 名称 | 字段 | 说明 |
|----|------|------|------|
| 0x10 | GripperStatus | status(u8) | 夹爪状态 |
| 0x11 | AssemblyStatus | status(u8) | 组装状态 |
| **0x12** | **R1Signal** | **signal(u8): 0=无 1=组装完成 2=进入梅林** | **R1通信 (新增)** |
| **0x13** | **StartButton** | **pressed(u8): 1=按下** | **开始按钮 (新增)** |
| 0x20 | EncoderFeedback | delta_x, delta_y, delta_theta (3×f32) | 编码器增量 |
| 0x21 | ArmFeedback | joint_1~6 (6×f32) | 机械臂反馈 |
| 0x22 | ArmStatus | status(u8) | 机械臂状态 |
| **0x23** | **ClimbFeedback** | **status(u8): 0=空闲 1=爬升中 2=完成 3=失败** | **爬升反馈 (新增)** |
| 0x24 | ChassisVelFeedback | vx, vy, omega (3×f32) | 底盘速度反馈 |
| 0x25 | WheelOdom | pos_x, pos_y, yaw, vx, vy, omega (6×f32) | 轮式里程计 |

---

## 六、比赛全流程时序

```
时间轴
─────────────────────────────────────────────────────────────────→
│                                                                │
│ [赛前准备]          [比赛 4分钟]                    [超时保护]   │
│                                                                │
│  启动系统           按下开始                                    │
│  ↓                  ↓                                          │
│  硬件初始化         ┌─ 开始计时 ──────────────── 4:10 ──┐      │
│  ↓                  │                                    │      │
│  输入KFS标签        │  NAV_TO_WEAPON (≈5s)               │      │
│  ↓                  │  ↓                                 │      │
│  等待开始信号       │  WEAPON_ALIGN + GRAB (≈8s)         │      │
│                     │  ↓                                 │      │
│                     │  NAV_TO_ASSEMBLY (≈10s)            │      │
│                     │  ↓                                 │      │
│                     │  WAIT_ASSEMBLY (等R1)              │      │
│                     │  ↓                                 │      │
│                     │  WAIT_ENTER_CMD (等R1)             │      │
│                     │  ↓                                 │      │
│                     │  NAV_TO_MEILIN (≈5s)               │      │
│                     │  ↓                                 │      │
│                     │  MEILIN_TRAVERSE (≈90s)            │      │
│                     │    进入→爬升→方块间导航→            │      │
│                     │    KFS视觉对齐→拾取→               │      │
│                     │    继续导航→出口下降                │      │
│                     │  ↓                                 │      │
│                     │  EXIT_MEILIN (≈5s)                 │      │
│                     │  ↓                                 │      │
│                     └─ STOP (发送复位) ──────────────────┘      │
│                                                                │
```

---

## 七、改进建议

### 7.1 导航层

1. **Nav2 DWB 参数调优**: 当前 `xy_goal_tolerance: 0.05m` 对梅林方块导航太严格（方块中心精度不需要5cm），建议梅林阶段临时放宽到 0.15m
2. **动态切换 costmap 参数**: 武馆区域用大 inflation_radius（安全优先）；梅林区域用小 inflation_radius（空间紧凑）
3. **导航超时处理**: 每个 `NavigateToPose` 设置单独超时（如武馆导航10s，梅林方块间3s），超时触发恢复行为或跳过

### 7.2 决策层

1. **cmd_vel 仲裁**: 必须有一个明确的机制决定谁在控制 /cmd_vel。建议用 `cmd_vel_mux`（话题复用器），game_controller 通过切换优先级决定 Nav2 还是视觉决策控制底盘
2. **模型预热**: YOLO 模型切换需要 1-2 秒加载时间，建议在导航到目标点的过程中提前切换模型，而不是到达后再切
3. **失败恢复**: 每个阶段需要定义失败后的处理：
   - 武器夹取失败 → 重试1次 → 跳过进入梅林
   - KFS拾取失败 → 重试1次 → 跳过该KFS继续导航
   - 导航超时 → 取消当前目标，尝试备选路径

### 7.3 梅林路径规划

1. **实时重规划**: 如果视觉在行进中发现某个"空方块"实际上有障碍物（赛场变化），应能实时重新规划路径
2. **入口选择**: 默认从2号进入，但如果真KFS在1号或3号方块，应自动选择对应入口
3. **拾取位置计算**: 4.0.md 要求"在相邻台阶拾取"，需要计算最佳拾取位置——面向目标KFS台阶中心、距离当前台阶边缘40cm

### 7.4 串口通信

1. **指令确认机制**: 关键指令（爬升、夹取、复位）需要 STM32 回复确认，超时重发
2. **紧急停止**: 新增紧急停止指令（独立于 GameCmd），由硬件按钮直接触发
3. **R1通信方式**: 如果 R1 和 R2 不共享 ROS2 网络，需要通过 STM32 中转信号（串口→STM32→无线→R1_STM32→R1），或考虑使用 ESP-NOW/蓝牙等短距无线方案

---

## 八、更新后项目文件结构

```
decision_processor/decision_processor/
├── __init__.py
├── config.py                    # [扩展] 新增比赛坐标、爬升指令、超时参数
├── game_controller.py           # [新建] 比赛总控状态机
├── match_input.py               # [新建] 赛前输入节点
├── meilin_path_planner.py       # [新建] 梅林3×4网格BFS路径规划
├── meilin_controller.py         # [新建] 梅林穿越控制器 (替代meilin_navigator)
├── processor_node.py            # [改造] 不再直接控制底盘，改为视觉子决策
├── robot_decision.py            # [保留] 5状态子FSM
├── motion_planner.py            # [保留] 运动规划+坡度感知
├── kalman_filter.py             # [保留] 目标位置滤波
├── target_confirmation.py       # [保留] 多帧确认
├── tf_manager.py                # [保留] TF坐标变换
├── imu_processor.py             # [保留] IMU数据处理
├── odometry_fusion.py           # [保留] 里程计融合
├── meilin_navigator.py          # [废弃] 被 meilin_controller 替代
└── scenarios/
    ├── __init__.py
    ├── base_scenario.py         # [保留]
    ├── scenario_wuguan.py       # [保留]
    └── scenario_meilin.py       # [保留]
```

---

## 九、开发优先级

| 优先级 | 模块 | 预计工时 | 依赖 |
|--------|------|---------|------|
| P0 | game_controller.py (骨架+Nav2 ActionClient) | 4h | 无 |
| P0 | config.py 坐标扩展 | 0.5h | 无 |
| P1 | meilin_path_planner.py (BFS) | 2h | config.py |
| P1 | meilin_controller.py (穿越子状态机) | 3h | path_planner |
| P2 | match_input.py (赛前输入) | 1h | game_controller |
| P2 | processor_node.py 改造 (cmd_vel 仲裁) | 2h | game_controller |
| P2 | protocol.yaml 新增消息 | 0.5h | 无 |
| P3 | 失败恢复与超时处理 | 2h | 全部 |
| P3 | 联调测试 | 4h | 全部 |
