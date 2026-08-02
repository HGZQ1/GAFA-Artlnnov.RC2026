# RC2026 状态机文档

本文档描述比赛全流程状态机（`game_controller.py`）与视觉伺服五状态机（`robot_decision.py`）的完整运行逻辑。

---

# 一、整体比赛状态机 (`game_controller.py` / `GamePhase`)

主状态机由 `GameController` 以 10Hz `_tick()` 驱动，分为 **武馆流程 → 梅林子状态机 → 对抗区流程** 三大段。

## 1.1 武馆流程

```
WAIT_INPUT
   │ 终端/话题输入完成 (真假KFS台阶号)
   ▼
WAIT_START
   │ 收到启动信号 (按钮 或 终端输入'q')
   │ → 记录比赛开始时间, 重置导航
   ▼
NAV_TO_WEAPON
   │ Nav2导航到武器架, 到达(_nav_done)
   ▼
ALIGN_WEAPON  ◄── 此阶段会被 processor_node 接管 (VISUAL_SERVO_PHASES)
   │ 视觉伺服五状态机跑完到 ARRIVED (decision_state_id == ARRIVED)
   │ → 停车, 进入 GRAB_WEAPON
   ▼
GRAB_WEAPON
   │ 持续停车, 等待 GRAB_WEAPON_SETTLE_S 后发送"拾取武器端头"动作组
   │ 收到下位机动作完成反馈 (action_status == DONE)
   │ → 重置导航
   ▼
NAV_TO_ASSEMBLY
   │ 导航到组装点, 到达
   ▼
WAIT_ASSEMBLY
   │ 一次性发送"底盘锁死"指令(等R1组装)
   │ 收到 R1 信号 = ENTER_MERLIN
   │ → 同时视为组装完成, 切换/预加载 kfs.pt, 发送"释放武器端头"动作组
   ▼
RELEASE_WEAPON
   │ 等待 PHASE_SWITCH_WAIT_S 秒(给下位机执行时间)                              PHASE_SWITCH_WAIT_S = 阶段切换缓冲时间
   ▼
NAV_TO_MERLIN_ENTRY
   │ 导航到梅林入口, 到达
   ▼
SWITCH_TO_MERLIN
   │ 停车, 等待 PHASE_SWITCH_WAIT_S 秒
   │ → 梅林子状态机置为 M_INIT
   ▼
MERLIN_PHASE  ────────────► (展开见 1.2 梅林子状态机)
```

## 1.2 梅林子状态机 (`MerlinStep`，仅在 `MERLIN_PHASE` 内运行)

```
M_INIT
   │ 调用 meilin_path_planner.plan_path(入口, 真KFS列表, 假KFS列表)
   │ 规划出一条避开假KFS、途经真KFS的方块路径
   │ 若真KFS在1/2/3入口台阶:
   │   - 记录 pre_entry_pickups
   │   - 强制从对应入口进入
   │   - 入口KFS不再作为梅林内部拾取目标
   │ 失败 → 直接 STOP（无可行路径）
   ▼
M_ENTRY_NAV
   │ Nav2导航到入口爬升触发点, 到达
   │ 若存在未拾取的入口KFS:
   │   → kfs_target=该入口KFS, 进入 M_ALIGN_KFS
   │ 否则:
   │   → 进入 M_ENTRY_CLIMB
   ▼
M_ENTRY_CLIMB
   │ 发送入口爬升指令(/serial/meilin_cmd, ENTRY_CLIMB_CMD)
   │ 等待 MERLIN_CLIMB_WAIT_S 秒(等爬升机构动作完成)
   │ → path_idx=0
   ▼
M_ON_BLOCK ◄─────────────────────────────────────────┐
   │ 判断当前方块cur:                                  │
   │  ① 若cur是某真KFS的"拾取触发方块"且未拾取        │
   │     → kfs_target=该KFS, 进 M_PICKUP_NAV          │
   │  ② 若cur是路径最后一块(出口) → 进 M_EXIT_NAV     │
   │  ③ 否则 → 进 M_NAV_TO_TRIGGER                    │
   │                                                    │
   ├─入口前拾取分支 (由 M_ENTRY_NAV 直接进入)            │
   │  M_ALIGN_KFS                                       │
   │     │ 在入口前坐标点对齐目标入口KFS                 │
   │     │ 后续复用 M_ARM_LIFT → M_FINE_ALIGN → M_PICKUP_KFS
   │     │ M_ARM_LIFT 比较高度时当前高度按入口地面0mm处理 │
   │     │ M_PICKUP_KFS完成后:
   │     │   - 若还有入口KFS → 回 M_ENTRY_NAV
   │     │   - 若当前入口就是路径入口 → M_ENTRY_CLIMB
   │     │   - 否则 → 回 M_ENTRY_NAV 导航到最终入口
   │                                                    │
   ├─① M_PICKUP_NAV                                    │
   │     │ 计算面向KFS方块的触发点(compute_trigger_point) │
   │     │ Nav2导航到该点, 到达后停车                   │
   │     ▼                                             │
   │  M_ALIGN_KFS  ◄── 视觉伺服模式 ALIGN_KFS 接管      │
   │     │ decision_state_id == ARRIVED (D435I粗对齐完成)│
   │     │ → 停车, 进入 M_ARM_LIFT                      │
   │     ▼                                             │
   │  M_ARM_LIFT   ◄── 机械臂精对齐子流程① (见 1.4)     │
   │     │ 比较 BLOCK_HEIGHTS[kfs_block] 与 cur高度      │
   │     │ → 发送"机械臂抬升1或2"动作组                  │
   │     │ action_status==DONE → 关闭D435I视觉伺服      │
   │     │   (离开ALIGN_KFS后自动停止, 见1.4说明)        │
   │     │ → 进入 M_FINE_ALIGN                          │
   │     ▼                                             │
   │  M_FINE_ALIGN ◄── 机械臂精对齐子流程② (见 1.4)     │
   │     │ 开启USB相机精对齐(/fine_align/enable)         │
   │     │ 持续微调底盘左右位置(/fine_align/cmd→chassis) │
   │     │ status==DONE(连续5帧居中) 或 超时15s          │
   │     │ → 关闭USB相机, 发送"拾取KFS"动作组             │
   │     ▼                                             │
   │  M_PICKUP_KFS                                     │
   │     │ action_status==DONE → 标记该KFS已拾取        │
   │     │   若下一方块就是该KFS方块 → M_SEND_CLIMB     │
   │     │   否则 → M_NAV_TO_TRIGGER                    │
   │     │ 超时(MERLIN_PICKUP_WAIT_S) → 跳过, M_NAV_TO_TRIGGER │
   │     │ 入口前拾取时完成/超时后按"入口前拾取分支"规则跳转 │
   │                                                    │
   ├─③ M_NAV_TO_TRIGGER                                │
   │     │ 计算 cur→next 之间的爬升/下降触发点           │
   │     │ Nav2导航到该点, 到达                          │
   │     ▼                                             │
   │  M_SEND_CLIMB                                     │
   │     │ get_transition_climb(cur,next) 得到爬升/下降指令 │
   │     │ 非0则发送 /serial/meilin_cmd                 │
   │     │   linear.x=next_block, linear.y=climb_mode,  │
   │     │   angular.x=height_diff_mm                   │
   │     ▼                                             │
   │  M_CLIMB_WAIT                                     │
   │     │ 等待 MERLIN_CLIMB_WAIT_S 秒                  │
   │     ▼                                             │
   │  M_NAV_TO_CENTER                                  │
   │     │ Nav2导航到next方块中心, 到达                  │
   │     │ → path_idx += 1                             │
   └─────┴──────────────────────────────────────────────┘ 回到 M_ON_BLOCK

M_EXIT_NAV (②触发)
   │ 导航到出口方块中心
   ▼
M_EXIT_DESCEND
   │ 发送出口下降指令(EXIT_DESCEND_CMD)
   │ 等待 MERLIN_CLIMB_WAIT_S 秒
   ▼
M_DONE
   │ 打印已拾取KFS集合, 重置导航
   │ → 退出 MERLIN_PHASE, 进入对抗区
   ▼
(返回主状态机) NAV_TO_MERLIN_EXIT
```

## 1.3 对抗区流程

```
NAV_TO_MERLIN_EXIT  ── 导航到梅林出口集结点, 到达 ──► NAV_TO_EXIT_MERLIN
NAV_TO_EXIT_MERLIN  ── 导航到场地出口, 到达        ──► NAV_TO_CONFRONT_ENTRY
NAV_TO_CONFRONT_ENTRY
   │ 导航到对抗区入口, 到达后【不停车】直接连续导航到KFS放置点
   ▼
NAV_TO_KFS_PLACE
   │ 导航到放置点, 到达
   ▼
PLACE_KFS
   │ 等0.3s稳定 → 发送"放置KFS(抬升40cm+放置)"动作组
   │ action_status==DONE → 重置导航
   ▼
NAV_TO_CONFRONT_WAIT
   │ 导航到对抗区等待点, 到达
   ▼
WAIT_MERGE
   │ 停车等待, 收到 R1 信号 = MERGE
   ▼
MERGE_WITH_R1
   │ 停车, 一次性发送"底盘锁死"
   │ (预留: 后续加R2对齐R1的导航逻辑)
   ▼
STOP  ── 持续发布chassis停止指令
```

## 1.4 机械臂精对齐子流程 (`M_ARM_LIFT` / `M_FINE_ALIGN`)

KFS拾取专用，衔接 D435I 粗对齐(`M_ALIGN_KFS`)与最终拾取(`M_PICKUP_KFS`)，由 `fine_align_node.py`（USB相机"三棱检测"算法封装）与 `game_controller.py` 协同完成。详细算法见 `triple_edge_align.py`（核心图像处理逻辑未改动）。

### ① `M_ARM_LIFT`（机械臂抬升）

```python
def _m_arm_lift(self):
    if not self._arm_lift_sent:
        kfs_h = BLOCK_HEIGHTS[self.kfs_target]
        cur_h = BLOCK_HEIGHTS[self.path[self.path_idx]]
        # 抬升1: 抓取比当前台阶高的KFS; 抬升2: 抓取比当前台阶低的KFS
        action = ACTION_ARM_LIFT_1 if kfs_h > cur_h else ACTION_ARM_LIFT_2
        发送 /serial/action_group_cmd = action
        self._arm_lift_sent = True
    elif action_status == ACTION_STATUS_DONE:
        self._set_merlin_step(MerlinStep.FINE_ALIGN)
```
- **动作**：根据 `BLOCK_HEIGHTS` 比较目标KFS方块与当前方块的高度差，下发 `ACTION_ARM_LIFT_1`(7,抬升+前伸+摄像头/吸盘转向下，用于抓高台阶) 或 `ACTION_ARM_LIFT_2`(8，同上但抬升幅度不同，用于抓低台阶)
- **D435I视觉伺服关闭**：`_merlin_step` 离开 `ALIGN_KFS` 后，`processor_node._on_game_phase` 的 `pub_phase` 覆写逻辑（仅在 `ALIGN_KFS` 时生效）不再触发，五状态机自动停止驱动——**无需额外代码关闭深度相机视觉伺服**
- **退出条件**：收到 `/feedback/action_group == ACTION_STATUS_DONE` → 进入 `M_FINE_ALIGN`

### ② `M_FINE_ALIGN`（USB相机精对齐）

```python
def _m_fine_align(self):
    if not self._fine_align_sent:
        发送 /fine_align/enable = FINE_ALIGN_ENABLE_BLUE 或 FINE_ALIGN_ENABLE_RED (按 self._kfs_color)
        self._fine_align_sent = True
        self._fine_align_start = now
    # 持续转发 /fine_align/cmd → /serial/chassis_cmd (微调左右位置)
    if self._fine_align_status == FINE_ALIGN_STATUS_DONE \
       or now - self._fine_align_start >= FINE_ALIGN_TIMEOUT_S:
        发送 /fine_align/enable = FINE_ALIGN_DISABLE  # 关闭USB相机
        发送 /serial/action_group_cmd = ACTION_PICKUP_KFS
        self._set_merlin_step(MerlinStep.PICKUP_KFS)
```
- **开启**：发布 `/fine_align/enable`（`FINE_ALIGN_ENABLE_BLUE`/`FINE_ALIGN_ENABLE_RED`，按比赛前手动输入的 `kfs_color` 选择滤色目标）
- **持续微调**：`fine_align_node` 以原 `triple_edge_align.py` 算法（颜色分割→边缘检测→透视/居中两级减速）计算底盘左右微调速度，发布到 `/fine_align/cmd`（`Twist.linear.y`），`game_controller` 原样转发到 `/serial/chassis_cmd`
- **完成判定**：`fine_align_node` 连续 `FINE_ALIGN_CONFIRM_FRAMES`(5) 帧居中 → 发布 `/fine_align/status = FINE_ALIGN_STATUS_DONE`
- **兜底**：`FINE_ALIGN_TIMEOUT_S`(15s) 超时仍未对齐 → 视为完成，继续后续流程，避免卡死
- **退出**：关闭USB相机(`FINE_ALIGN_DISABLE`)，发送"拾取KFS"动作组 → `M_PICKUP_KFS`

### `/fine_align/*` 话题与常量一览

| 话题/常量 | 类型 | 含义 |
|---|---|---|
| `/fine_align/enable` (game_controller → fine_align_node) | `std_msgs/UInt8` | `FINE_ALIGN_DISABLE`(0)=关闭 `FINE_ALIGN_ENABLE_BLUE`(1)=开启,目标蓝色KFS `FINE_ALIGN_ENABLE_RED`(2)=开启,目标红色KFS |
| `/fine_align/cmd` (fine_align_node → game_controller) | `geometry_msgs/Twist` | 仅用 `linear.y`（横移微调速度 m/s，符号按REP-103: +y=左），由 `game_controller` 转发到 `/serial/chassis_cmd` |
| `/fine_align/status` (fine_align_node → game_controller) | `std_msgs/UInt8` | `FINE_ALIGN_STATUS_ALIGNING`(0)=对齐中 `FINE_ALIGN_STATUS_DONE`(1)=已完成(连续5帧居中) `FINE_ALIGN_STATUS_NO_TARGET`(2)=未检测到目标 |
| `ACTION_ARM_LIFT_1`(7) / `ACTION_ARM_LIFT_2`(8) | `/serial/action_group_cmd` | 机械臂抬升1(抓高台阶KFS)/抬升2(抓低台阶KFS)，对应 `protocol.yaml` ActionGroupCmd 7/8 |
| `kfs_color`（终端输入，比赛前手动设置一次） | `KFS_COLOR_BLUE`(1) / `KFS_COLOR_RED`(2) | 己方KFS颜色，决定 `/fine_align/enable` 发送哪个颜色目标 |

**全局兜底**：任意阶段若 `比赛已用时 >= MATCH_TIMEOUT_S` → 取消导航、停车、发 `GAME_CMD_RESET` → 强制 `STOP`。

**与视觉伺服的握手**：`/game/phase` 发出当前阶段（`MERLIN_PHASE`+`ALIGN_KFS`时对外发布为 `'ALIGN_KFS'`）；`processor_node` 收到 `ALIGN_WEAPON`/`ALIGN_KFS` 时启动五状态机，并通过 `/decision/state_id` 把 `ARRIVED`/其他状态回传给 `game_controller` 判断何时收尾。

---

# 二、视觉伺服五状态机 (`robot_decision.py` / `RobotState`)

只在 `_visual_servo_active()`（即 `/game/phase ∈ {ALIGN_WEAPON, ALIGN_KFS}`）时被驱动，每次收到 `/vision/raw_target` 或超时(0.5s无目标)调用一次 `update()`。

```
┌─────────────┐  连续3帧确认到目标(confirmed)   ┌─────────────┐
│ SEARCHING   │ ──────────────────────────────► │  ALIGNING   │
│ 原地旋转搜索 │   event='TARGET_LOCKED'          │ 转向对准目标 │
│ all_picked? │                                  │             │
│ →ALL_DONE   │ ◄────────────────────────────── │             │
└─────────────┘  连续5帧丢失(confirmed=False)    └──────┬──────┘
      ▲            event='TARGET_LOST'                  │
      │                                     |align_angle|<=阈值(默认5°)
      │                                     event='ALIGNED'
      │                                                  ▼
      │                                          ┌─────────────┐
      │            连续5帧丢失                    │   MOVING    │
      │ ◄─────────────────────────────────────── │ 直线前进逼近 │
      │            event='TARGET_LOST'            └──────┬──────┘
      │                                                   │
      │                                  distance <= stop_dist(默认0.20m)
      │                                  event='ARRIVED'
      │                                                   ▼
      │                                          ┌─────────────┐
      │                                          │   ARRIVED   │
      │                                          │ 沉降等待     │
      │                                          │ settle_time │
      │                                          └──────┬──────┘
      │                            !pause_at_arrived 且 settle完成
      │                            event='PICK_START'
      │                                                   ▼
      │                                          ┌─────────────┐
      │  pick_elapsed>=pick_duration              │   PICKING   │
      │  或 收到夹爪反馈"已抓取"                   │ 视觉伺服关闭 │
      │  pick_count++; event='PICK_DONE'          │ 执行抓取动作 │
      └────────────────────────────────────────── │ 输出arm_dist │
        若 pick_count>=总数 → all_picked=True      └─────────────┘
        event再叠加'ALL_DONE'(下一次SEARCHING时触发)
```

## 各状态详细逻辑（带中文注释）

### ① `SEARCHING`（搜索）
```python
if self.state == RobotState.SEARCHING:
    if self.all_picked:
        # 全部目标已拾取完毕，只触发一次ALL_DONE事件，之后保持静止
        if not self._all_done_fired:
            self._all_done_fired = True
            self.event = 'ALL_DONE'
        return cmd   # cmd为全0停止指令
    if confirmed:
        # TargetConfirmation连续3帧检测到合法目标 → 锁定目标，进入对准
        self.state = RobotState.ALIGNING
        self.event = 'TARGET_LOCKED'
    else:
        # 还没锁定目标 → 原地旋转搜索
        cmd['search_rotate'] = 1.0
```
- **输入**：`detected`（本帧是否检测到合法目标）
- **输出**：`search_rotate=1.0`（驱动底盘原地旋转扫视）或 `all_picked` 时的全停指令
- **退出条件**：confirmation 模块连续3帧确认 → ALIGNING

### ② `ALIGNING`（对准/转向）
```python
elif self.state == RobotState.ALIGNING:
    if not confirmed:
        # 连续5帧没看到目标 → 放弃，回到搜索
        self._reset_tracking()
        self.state = RobotState.SEARCHING
        self.event = 'TARGET_LOST'
    elif not detected:
        # confirmation还没超时，但本帧恰好没检测到 → 保持原指令等待
        pass
    else:
        # 本帧检测到目标，用实时align_angle判断是否已对准
        if abs(align_angle) > self.align_thr_deg:
            # 偏角超过阈值(默认5°) → 调用运动规划器算转向量
            plan = self.planner.plan(base_x, base_y)
            cmd['turn_angle']  = plan.turn_deg     # 还需转多少度
            cmd['turn_wheels'] = plan.turn_wheels  # 转向轮需转多少圈
        else:
            # 已对准 → 进入前进阶段
            self.state = RobotState.MOVING
            self.event = 'ALIGNED'
```
- **输入**：`align_angle`（相机水平偏角，正=目标在左需左转，负=右转）、`base_x/base_y`（底盘系坐标，给规划器算转向量）
- **输出**：`turn_angle`（角度指令）、`turn_wheels`（对应转向轮圈数，发给STM32执行精确转向）
- **退出条件**：`|align_angle| <= align_thr_deg` → MOVING；目标连续丢失5帧 → SEARCHING

### ③ `MOVING`（前进逼近）
```python
elif self.state == RobotState.MOVING:
    if not confirmed:
        self._reset_tracking()
        self.state = RobotState.SEARCHING
        self.event = 'TARGET_LOST'
    elif not detected:
        pass
    else:
        # (注: REALIGN二次对准逻辑当前被注释掉，前进过程中不再检查角度)
        if distance <= self.stop_dist:
            # 距离已小于停止距离(默认0.20m) → 到达，开始沉降计时
            self._arrival_start = now
            self.state = RobotState.ARRIVED
            self.event = 'ARRIVED'
        else:
            # 还没到 → 计算前进量
            plan = self.planner.plan(base_x, base_y)
            cmd['forward_dist']  = plan.forward_m       # 前进距离(m)
            cmd['drive_wheels']  = plan.drive_wheels    # 驱动轮圈数
```
- **输入**：`distance`（俯仰角修正后的水平距离 `dist_h`）、`base_x/base_y`
- **输出**：`forward_dist`（前进距离）、`drive_wheels`（驱动轮圈数）
- **退出条件**：`distance <= stop_dist` → ARRIVED；丢失目标 → SEARCHING
- **设计取舍**：注释掉的 `REALIGN` 分支原本是"前进中若偏角过大(超过2倍阈值)则回到ALIGNING重新对准"，目前被禁用，意味着前进过程中**不会**因为偏角变化而中断。

### ④ `ARRIVED`（到达/沉降）
```python
elif self.state == RobotState.ARRIVED:
    elapsed = now - self._arrival_start
    self.settle_remain = max(0, self.settle_time - elapsed)  # ARRIVAL_SETTLE_S
    if not self.pause_at_arrived and elapsed >= self.settle_time:
        # 沉降时间到 且 没有被game_controller暂停
        self._pick_start = now
        self.arm_target_dist = arm_distance   # 记录机械臂目标距离, 供PICKING使用
        self.state = RobotState.PICKING
        self.event = 'PICK_START'
```
- **作用**：到达后稳定一小段时间(`ARRIVAL_SETTLE_S`)，让车身震动/惯性消失再抓取
- **`pause_at_arrived` 标志**：由 `processor_node._on_game_phase` 设置——进入视觉伺服阶段时设为 `True`。意味着**默认情况下到达后会卡在ARRIVED不自动进PICKING**，需要 `game_controller` 检测到 `decision_state_id==ARRIVED` 后自行处理（例如 `ALIGN_WEAPON`阶段是 `game_controller` 自己发"拾取武器"动作组，并不依赖 `PICKING` 状态）
- **输出**：无运动指令（全停），`settle_remain` 用于状态行显示倒计时

### ⑤ `PICKING`（拾取执行）
```python
elif self.state == RobotState.PICKING:
    cmd['pickup_action'] = 1.0
    cmd['arm_distance']  = self.arm_target_dist
    self.pick_elapsed = now - self._pick_start

    # 退出条件：超时(兜底) 或 下位机夹爪反馈"已抓取"(提前结束)
    if self.pick_elapsed >= self.pick_duration or self._pick_feedback_done:
        self.pick_done_by_feedback = self._pick_feedback_done
        self.pick_count += 1
        self.event = 'PICK_DONE'
        if self.pick_count >= WUGUAN_TOTAL_WEAPONS:
            self.all_picked = True
        self._pick_feedback_done = False
        self._reset_tracking()
        self.state = RobotState.SEARCHING   # 完成后回到搜索, 寻找下一个目标
```
- **输出**：`pickup_action=1.0`、`arm_distance`（供机械臂/抓取机构使用，对应 `/arm/target_pos`，在 PICK_START 那一帧发布一次）
- **退出条件**（满足其一即可）：
  1. `pick_elapsed >= pick_duration`（默认10s，兜底超时，防止夹爪反馈丢失导致卡死）
  2. 收到下位机夹爪反馈 `/feedback/gripper == GRIPPER_STATUS_GRABBED(1, 已抓取)`，由 `processor_node._on_gripper_feedback` 调用 `decision.notify_gripper_grabbed()` 置位 `_pick_feedback_done`，下一次 `update()` 时提前结束计时
  - 退出后 `pick_done_by_feedback` 记录本次是按哪种条件退出的（用于 `PICK_DONE` 事件日志中区分"超时"/"夹爪反馈"）
  - 计数+1 → `_reset_tracking()` → 回 SEARCHING 找下一个

#### 关闭视觉伺服 (PICKING期间)

`processor_node._on_raw_target` 在函数开头新增：
```python
if self.decision.state == RobotState.PICKING:
    return
```

**原因**：进入 PICKING 后机械臂会平移/抓取，导致相机视野变化或被抓取物遮挡相机，此时继续跑 TF变换→卡尔曼→场景判定 只会产生误检，并可能反复发布错误的 `/arm/target_pos`。

**效果**：
- ARRIVED→PICKING 切换的**那一帧**（函数入口时 `state` 仍为 `ARRIVED`）会完整跑完，触发一次 `_publish_arm_target(...)`——即"对齐完成瞬间，用当时的TF结果算一次抓取目标并下发"
- 之后 PICKING 期间所有 `/vision/raw_target` 帧被直接拦截，不再处理视觉、不重复发布 `/arm/target_pos`、不刷新状态行
- PICKING 的推进/退出完全由 `_timer_cb`（10Hz，`detected=False` 的 `update()`）和 `/feedback/gripper` 回调驱动
- PICKING 结束后 `_reset_tracking()` + 回 SEARCHING，自然恢复正常视觉处理

#### 夹爪反馈话题

| 话题 | 类型 | 含义 |
|---|---|---|
| `/feedback/gripper` | `std_msgs/UInt8` | `GripperStatus`：0=空载 1=已抓取(`GRIPPER_STATUS_GRABBED`) 2=已完成组装 3=错误 |

`processor_node._on_gripper_feedback`：仅当 `decision.state == PICKING` 且 `data == GRIPPER_STATUS_GRABBED` 时调用 `decision.notify_gripper_grabbed()`，其余状态/数值忽略。

## 通用辅助模块

| 模块 | 作用 |
|---|---|
| `TargetConfirmation`(confirm_frames=3, lost_frames=5) | 防抖：连续3帧检测到才"确认"，连续5帧丢失才"放弃"，避免单帧误检/漏检导致状态抖动 |
| `KalmanFilter2D`(dt=0.05) | 对 `(base_x, base_y)` 做卡尔曼平滑，减少深度噪声导致的运动指令抖动 |
| `is_valid_detection` | 跳变检测：若新位置相对上一帧跳变过大，视为误检直接丢弃该帧 |
| `MotionPlanner.plan(x,y)` | 把目标在底盘系下的坐标转换成 `turn_deg/turn_wheels/forward_m/drive_wheels`（基于轮径、轮距几何计算） |

## `cmd_dict` → 实际下发字段总表

| cmd_dict 键 | 含义 | 仅在哪个状态非0 |
|---|---|---|
| `search_rotate` | 原地旋转搜索 | SEARCHING |
| `turn_angle` / `turn_wheels` | 历史key名；当前 `turn_angle` 数值作为 yaw rate(deg/s) 下发 | ALIGNING |
| `forward_dist` / `drive_wheels` | 历史key名；当前分别作为 vx/vy(m/s) 下发 | MOVING |
| `pickup_action` / `arm_distance` | 拾取动作标志 / 机械臂目标距离 | PICKING |

`_build_chassis_cmd` 目前只把 `forward_dist→linear.x(vx m/s)`、`drive_wheels→linear.y(vy m/s)`、`turn_angle→angular.z(wz deg/s)` 打包进 `/serial/chassis_cmd`；`pickup_action`/`arm_distance`/`turn_wheels`/`search_rotate` **没有被映射进Twist**，如笔记里"串口通信映射"部分所写，这部分协议字段还需在 `processor_node` 里补充对应逻辑。
