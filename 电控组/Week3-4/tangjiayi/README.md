# 电控组 Week3-4 · 整机拼装与总线舵机控制

| | |
| --- | --- |
| 姓名 / 拼音 | 唐嘉怡 / `tangjiayi` |
| 学号 | 20263633 |
| 周次 | 第 3、4 周（舵机点亮 + 整机装配） |
| 平台 | Ubuntu 22.04 + ROS 2 Humble + 地瓜派 |
| 提交内容 | 任务 1 截图、任务 2 代码 + bag + 截图、任务 3 代码 + 动作 yaml + 整机照片、任务 4 流程 |

---

## 0. 完成状态

| 关卡 | 分值 | 状态 | 材料位置 |
| --- | --- | --- | --- |
| 任务 1 · 点亮一个舵机 | 15 | ✅ 单机已点亮并标定（4 只） | `01_中立位标定.png`、`01_电池电压.png` |
| 任务 2 · ROS2 接管（Action） | 50 | ✅ 实机验证通过 | `02_*.png`、`src/`、`bag/run1/`、录屏 P1~P4 |
| 任务 3 · 整机 + 动作组 | 25 | ✅ 装配完成；带载连跑受热保护限制 | `03_*.png`、`config/actions/`、录屏 P5 |
| 任务 4 · 安全下电 | 10 | ✅ 流程已实现并实测 | 第 11 节、`tools/safe_poweroff.py`、`04_安全下电_归位卸力.png` |
| 加分项 | +10 | ✅ 6 项全部实现 | 见第 13 节 |

> 说明：本 README 里的截图和终端输出都来自真实运行，没有加工过。
> 代码部分已写完并在本地做过静态验证（`colcon test` 汇总 28 项、0 failures，其中 `pytest` 用例 23 个，其余为两个包的包格式/代码风格检查；4 个 yaml 全部可解析）。

---

## 1. 架构图

### 1.1 包与节点

```
                    ┌──────────────────────────────────────────┐
                    │  cquick_servo_interfaces  (ament_cmake)  │
                    │  ├─ msg/ServoCommand.msg                 │
                    │  └─ action/MoveServo.action              │
                    └──────────────────┬───────────────────────┘
                                       │ 依赖（编译期 + 运行期）
                    ┌──────────────────▼───────────────────────┐
  ros2 launch  ────>│  cquick_servo  (ament_python)            │
                    │  ├─ servo_driver     ← Action Server     │
                    │  ├─ move_client      ← Action Client     │
                    │  ├─ sequence_player  ← 读 yaml 打 goal   │
                    │  ├─ launch/servo.launch.py               │
                    │  └─ config/{servo_params.yaml, actions/} │
                    └──────────────────┬───────────────────────┘
                                       │ pyserial
                              USB 转 TTL（115200 8-N-1）
                                       │
                            24 路舵机控制板 → 总线舵机 1~10（两条腿各 5 个关节）
```

### 1.2 一次 goal 的完整数据流

```
move_client / sequence_player / ros2 action send_goal
        │  MoveServo.Goal{id, pwm, duration_ms}
        ▼
goal_callback          ① 校验：pwm 在 pwm_min~pwm_max？id 在 servo_ids？
        │              ② 串口通不通？看门狗锁没锁？→ 不合格直接 REJECT
        ▼  ACCEPT
handle_accepted_callback  ③ 该 ID 空闲 → 开线程跑；该 ID 在动 → 抢占，新 goal 排队
        ▼
execute_callback
   ④ PULR（恢复扭力，不发舵机纹丝不动）
   ⑤ "#001P1700T1000!" → write + flush → 控制板 → 舵机
   ⑥ 每 200ms 读一次 PRAD 发 feedback，同时检查 cancel / 抢占 / 看门狗
   ⑦ 动作走完再读一次 PRAD，与目标差 ≤ ±30 才算到位；超了自动重发一次
        ▼
MoveServo.Result{success, pwm_readback, message}
        │
        └─ 同时把这条指令 echo 到 /servo/command → ros2 bag 录下来 → bag play 能复现
```

### 1.3 ROS 接口一览

| 类型 | 名字 | 消息/服务类型 | 方向 | 干什么 |
| --- | --- | --- | --- | --- |
| Action | `/move_servo` | `cquick_servo_interfaces/action/MoveServo` | 外部 → 驱动 | **唯一的控制入口** |
| Topic | `/servo/command` | `cquick_servo_interfaces/msg/ServoCommand` | 双向（驱动订阅 + echo） | 内部调试总线，bag 回放走这条 |
| Topic | `/servo/state` | `std_msgs/msg/String`（JSON） | 驱动发布 5 Hz | 状态 + 心跳，bag 里能看时间线 |
| Topic | `/servo/heartbeat` | `std_msgs/msg/Empty` | 上位机 → 驱动 | 喂看门狗，1 秒不来就全卸力 |
| Service | `/servo/confirm_resume` | `std_srvs/srv/Trigger` | 人工调用 | 看门狗跳闸后确认恢复 |

---

## 2. 参数表

全部参数在 `config/servo_params.yaml`，驱动代码里**只出现在两处**：声明 + `_param()` 读取。

| 参数 | 类型 | 默认值 | 为什么是这个默认值 |
| --- | --- | --- | --- |
| `port` | string | `/dev/cq_servo` | udev 固定设备名。细则给的默认是 `/dev/ttyUSB0`，但插着摄像头时它会被挤成 ttyUSB1，所以改成按 VID/PID 绑定的固定名（加分项 +2）。没装规则时 `port:=/dev/ttyUSB0` 覆盖即可 |
| `baudrate` | int | `115200` | 众灵总线舵机的出厂波特率。串口助手默认 9600，所以「收到一堆乱码」99% 是这一个参数 |
| `pwm_min` | int | `1050` | **逐点扫出来的实测下限**：目标 1000 可到（读回 1008），950 和 900 都卡在 1007 不动 → 硬下限约 1007，取 1050 留 45 步余量。细则默认 900 比硬件还宽，区间内有 110 步是「发了也白发、等 2 秒再 ABORTED」 |
| `pwm_max` | int | `2100` | 同上，实测上限。给 2500 会先把结构顶变形，舵机才开始报警 |
| `default_duration_ms` | int | `800` | 900 步（900→1800）用 800ms 走完，肉眼看是平滑的；给 200ms 会「啪」一下甩过去 |
| `servo_ids` | int[] | `[1,2,3,4,5,6,7,8,9,10]` | 整机 10 个关节（两条腿各 5 个）。不在表里的 ID 直接 REJECT，挡住「ID 记错」这种低级错 |
| `startup_reset` | bool | `true` | 上电全部回 1500，装配时省得一根根敲指令 |
| `pwm_center` | int | `1500` | 中立位。取消动作、上电复位都回这里 |
| `center_duration_ms` | int | `800` | 回中位的用时。给小了会猛跳撞结构 |
| `feedback_period_ms` | int | `200` | 细则要求 ≥5 Hz，200ms 正好压线；给 100ms 反馈更顺但日志更长 |
| `pos_tolerance` | int | `30` | PRAD 与目标差多少算到位。±30 约 3°，肉眼看不出来，再紧就会被舵机自身回差搞成「永远不到位」 |
| `retry_times` | int | `1` | 未到位自动重发次数。1 次够用：真的是机械卡住，重发 100 次也没用 |
| `watchdog_enabled` | bool | `true` | 看门狗总开关（加分项 +3） |
| `watchdog_timeout_ms` | int | `1000` | 细则要求 1 秒。上位机卡死 1 秒内卸力，舵机还来不及把结构顶热 |
| `reconnect_period_ms` | int | `1000` | 串口掉线后的重试间隔（加分项 +3）。1 秒一次，插回去最多等 1 秒就恢复 |
| `allow_preempt` | bool | `true` | 同一 ID 允许新 goal 抢占旧动作（加分项 +4） |

参数校验（`add_on_set_parameters_callback`）拒绝的情况：
`pwm_min/pwm_max/pwm_center` 不在 500~2500、`pwm_min ≥ pwm_max`、`pwm_center` 跑出安全区间、
`baudrate` 不在白名单 `[9600, 19200, 38400, 57600, 115200, 230400]`、各种时长/阈值越界、
`servo_ids` 为空或含 0/255、`port` 为空。拒绝时 `reason` 里写清楚哪个值不行、合法范围多少。

---

## 3. 接口契约

### 3.1 `MoveServo` Action

**Goal**（字段从 `ServoCommand` 摊平过来，为了命令行好拼）

| 字段 | 类型 | 含义 | 什么时候校验 |
| --- | --- | --- | --- |
| `id` | `uint8` | 舵机 ID，1~254 | `goal_callback`：不在 `servo_ids` → REJECT |
| `pwm` | `uint16` | 目标 PWM | `goal_callback`：不在 `pwm_min~pwm_max` → REJECT |
| `duration_ms` | `uint16` | 从当前位置走多久；`0` = 用 `default_duration_ms` | `goal_callback`：> 60000 → REJECT |

**Feedback**（≥ 5 Hz）

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `pwm_current` | `uint16` | 当前实际位置，优先取 `PRAD` 读回值；读不到就用时间线性插值估算（不编数） |
| `progress` | `float32` | 已用时间 / `duration_ms`，0.0~1.0 |

**Result**

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `success` | `bool` | 到位且 PRAD 校验通过才为 `true` |
| `pwm_readback` | `uint16` | 动作结束后 `PRAD` 读回值；读不到为 0 |
| `message` | `string` | 一句话说清，取值见下表 |

`message` 字典：

| 值 | 谁给的 | 含义 |
| --- | --- | --- |
| `ok` | 正常完成 | 到位 |
| `canceled` | 用户 C`trl+C` | 已回中位，`success=false` |
| `preempted: ...` | 同一 ID 来了新 goal | 旧动作从当前位置退出，不回中位、不跳变 |
| `watchdog_locked` | 看门狗 | 已卸力，需 `/servo/confirm_resume` |
| `serial_down` | 串口掉线 | 节点还活着，插回去自动恢复 |
| `not_in_position: 目标 1700 读回 1752 差 +52 > 30` | PRAD 校验 | 重发过还是没到位 |

### 3.2 为什么是 Action 而不是话题

- 话题是「发出去就不管了」，拿不到「到没到位」；
- Action 自带 goal / feedback / result / cancel 四件事，正好对应「发指令 / 看进度 / 要结论 / 中途打断」；
- 舵机动作是**有明确终点**的操作，天然该用 Action。

`/servo/command` 只作为内部调试总线存在（给 bag 用），**不承担控制职责** —— 评审用 `ros2 action send_goal` 打 goal 才是验收路径。

---

## 4. 一条启动命令

```bash
ros2 launch cquick_servo servo.launch.py
```

换串口（这条是验收点）：

```bash
ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB1
```

带客户端 / 带动作组：

```bash
ros2 launch cquick_servo servo.launch.py demo:=true
ros2 launch cquick_servo servo.launch.py player:=true loop_times:=3
```

启动之后终端会打这几行，看到就说明链路通了：

```
[INFO] [servo_driver]: servo_driver 启动：action=/move_servo port=/dev/cq_servo baud=115200
                         安全区间=(1050, 2100) ids=[1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
[INFO] [servo_driver]: 串口已连接：/dev/cq_servo@115200（8-N-1）
[INFO] [servo_driver]: startup_reset：[1..10] 全部回中位 1500，用时 800ms
```

串口打不开时只打 `WARN` 并由 `reconnect_period_ms`（1 s）重试，**节点不会退出** —— 这是「掉线重连」加分项的落地方式。

命令行打一条 goal（也是验收方式）：

```bash
ros2 action send_goal /move_servo cquick_servo_interfaces/action/MoveServo \
    "{id: 1, pwm: 1700, duration_ms: 1000}" --feedback
```

---

## 5. 安全区间是怎么试出来的

`pwm_min=1050 / pwm_max=2100` 不是拍脑袋，也不是抄电气范围（500~2500）。

**做法**：单舵机、不装结构 → 装结构后复测。

1. 先给 1500，确认舵机在中位；
2. 每次向一侧加 50，停下听声音、摸温度、看连杆有没有顶到打印件；
3. 第一次出现「连杆已经贴住限位但舵机还在发力（嗡嗡声）」的那个值，**往回退 50**，记为边界；
4. 另一侧同理；
5. 把两侧边界写进 `pwm_min` / `pwm_max`，再跑一遍全范围确认不再有堵转声。

**实测过程（11-13 22:00 扫出）**：从 1500 向下每 100 打一点，1400→读回 1412、1300→1313、1200→1212、1100→1113、1000→1008 全部 `SUCCEEDED`；**950 和 900 都卡在 1007 不动**（`ABORTED`，`not_in_position: 差 +57/+107 > 30`）。向上 2100→2089 `SUCCEEDED`。由此确定硬下限 ≈ 1007、上限 ≈ 2089，取 `pwm_min:1050 / pwm_max:2100` 各留余量。

> 这条也是「限位不是拍脑袋填的、是扫出来的」的完整证据链；`not_in_position` 一开始很像代码 bug，实际证明的是 PRAD 到位校验在正确工作。

---

## 6. 运行时改参数，当帧生效

**关键设计**：所有判断都走 `self._param(name)` 实时 `get_parameter()`，`__init__` 里**不缓存副本**。
这一条是整节的根：缓存到 `self.pwm_max` 是本周最高频的坑，改完参数行为不变，还极难查。

### 6.1 

| 层 | 范围 | 来源 | 能不能在线改 |
| --- | --- | --- | --- |
| **电气极限** | 500 ~ 2500 | `PWM_ELECTRIC_MIN/MAX` 硬编码 | 不能，改要重编译 |
| **安全区间** | 1050 ~ 2100 | `servo_params.yaml` / launch 参数 | 能，`ros2 param set` |

外层防**烧舵机**（超出舵机能接受的 PWM 就物理损伤），内层防**顶结构**（是本机实测行程，换机器要重扫）。
下面 ④ 打的是外层，②③ 打的是内层。

### 6.2 实测

```bash
ros2 param set /servo_driver pwm_max 1600      # ② 改小上限
ros2 param get /servo_driver pwm_max           # ③ 立刻回读
ros2 param set /servo_driver pwm_max 100000    # ④ 超电气极限
ros2 param set /servo_driver pwm_min 2200      # ⑤ min > max
ros2 param set /servo_driver pwm_max 2100      # ⑥ 恢复
```

真实输出（`02_param_set现场.png`）：

```
--- ① 当前上限 ---
Integer value is: 2100
--- ② 在线改到 1600，不重启 ---
Set parameter successful
--- ③ 立刻回读（证明代码没缓存副本）---
Integer value is: 1600
--- ④ 100000 远超电气极限 500~2500，必须被拦 ---
Setting parameter failed: pwm_max=100000 超出电气极限 500~2500
--- ⑤ min 不能大于 max ---
Setting parameter failed: pwm_min(2200) 必须小于 pwm_max(1600)
--- ⑥ 恢复 2100 ---
Set parameter successful
Integer value is: 2100
```

每一条在证明什么：

- **② `successful`**：校验全过才写入。`_on_set_parameters()` 里返回 `successful=False` 时整组参数**原子回滚**，不会出现「改了一半」的中间态。
- **③ 回读 1600**：这是最关键的一行。如果代码把值缓存到 `self.pwm_max`，`get` 可能照样返回 1600，但判断还在用 2100 —— 第 10 节 ① 就是这个坑，当时排查了很久。回读等于当场证明**没有第二份副本**。
- **④ 被拦**：`pwm_max=100000` 连外层都没过。注意文案里的 `500~2500` 是**电气极限**，不是 1050~2100，这两个数字不一样是故意的。
- **⑤ 被拦**：`min >= max` 也要拦，否则窗口反向，任何 pwm 都判不过。文案里 `pwm_max(1600)` 带的是**上一步刚改成的值**，等于顺手又证明了一次参数是实时读的。
- **⑥ 恢复**：改回去能回到 2100，说明拒绝是「不写入」而不是「写坏了再回滚」。

参数改小之后，同一个 pwm 会不会被拒？判断顺序是
`watchdog_locked → serial_down → id 不合法 → **pwm 超区间** → duration 超限`。
`02_限位保护.png` 里那种 `Goal was rejected` 就是这个区间判断打的，pwm 2500 / pwm 600 两条都落在窗口外。

---

## 7. bag：录了什么、怎么回放

**只录两个话题，不用 `-a`**（全录会把板卡上所有节点的数据都塞进去，几百 MB）：

```bash
# 动作开始前就开录，跑完再停
ros2 bag record -o bag/run1 /servo/command /servo/state --max-cache-size 16777216
```

**为什么回放能动**：驱动节点订阅 `/servo/command`，且这个话题的回调与 Action 走**同一个底层发送函数**
（`servo_protocol.build_move_command()` → 同一个 `_write_frame()`）。
两条路径分叉的话，回放出来的动作一定和当时现场的对不上。

**回放**：

```bash
ros2 bag info bag/run1
ros2 bag play bag/run1
```

**`ros2 bag info` 实测（见 `02_bag_info.png`，11-13 22:32）**：

| 字段 | 值 |
| --- | --- |
| Bag size | 61.4 KiB |
| Duration | 51.2 s |
| Messages | 260 |
| 话题 | `/servo/state`（`std_msgs/msg/String`）257 条、`/servo/command`（`cquick_servo_interfaces/msg/ServoCommand`）3 条 |

**关于时长 51.2 s：这一段是**有意不剪**的。同一次录制里包含完整的一次「发 goal → 中途 Ctrl+C 取消 → 收到 `CANCELED`」，
取消之后到下一次发 goal 之间的等待本身就是演示的一部分，剪掉就只剩"能动"、看不到"能停"，而 `CANCELED` 语义正是任务 2 要证明的东西之一。
回放时注意：`info` 里 260 条消息中真正带指令的只有 3 条，其余 257 条是 5 Hz 的状态心跳，所以 51.2 s 里大部分是静止等待，不是无效数据。

bag 文件完整可回放（`bag/run1/run1_0.db3`，sqlite3 存储），`/servo/command` 3 条对应当时的 3 次 goal，条数对得上。

一个容易踩的坑（代码里已经处理）：驱动节点**既发布又订阅** `/servo/command`）。
如果话题回调里也走 `_send_move(echo=True)`，就会自己喂自己 → 无限循环。
所以回调里只调 `_write_frame()`，不回发。

---

## 8. 接线说明

```
电池（先量电压！5~8.4V）
   ├── 正极 ──> 24 路控制板 V+ ──> 舵机红线
   └── 负极 ──> 控制板 GND ──┬──> 舵机棕/黑线
                             └──> USB 转 TTL GND   ← 必须共地，缺这根数据一定收不到
电脑 ──USB──> USB 转 TTL ──TX/RX──> 控制板串口
                                    （信号线：橙/白，接控制板通道信号脚）
```

| 线色 | 含义 | 注意 |
| --- | --- | --- |
| 橙 / 白 | 信号 | 接通道信号脚 |
| 红 | 正极 5~8.4V | **绝不能接 12V** |
| 棕 / 黑 | 负极 GND | 与板卡、TTL 共地 |

串口四个参数：**115200 / 8 / 1 / 无校验**，写作 `115200 8-N-1`。

> 实测记录（2026-11-20）：
> - 电池：**7.98 V**，UNI-T UT33D+ 直流 20 V 档，2S 1500 mAh 45C，折合约 3.99 V/cell，属正常区间（截图 `01_电池电压.png`）。低于 7.0 V 不建议再带载跑动作组。
> - USB 转 TTL：**CH340**（`1a86:7523`），与 `udev` 规则里写的 VID:PID 一致；`dmesg` 中驱动名为 `ch341-uart`。

---

## 9. 动作 yaml 

放在 `config/actions/`，和 `servo_params.yaml` 同一层。**换一套动作 = 加一个 yaml，一行 python 不改。**

```yaml
name: stand_up
steps:
  # 同一个 step 里的 joints 是【同时下发】：驱动端按 ID 并发执行，两个关节一起走
  - name: 01_前后腿同时抬到预备位
    settle_ms: 150            # 两个 result 都回来后，再多等 150ms 让结构消余振
    joints:
      - {id: 1, pwm: 1700, duration_ms: 700}
      - {id: 2, pwm: 1300, duration_ms: 700}

  # 必须先动的关节单独开一个 step。不靠调 duration_ms 硬凑先后
  - name: 02_中间关节跟上
    settle_ms: 150
    joints:
      - {id: 3, pwm: 1600, duration_ms: 600}
```

设计说明：

| 问题 | 怎么解决 |
| --- | --- |
| 一步之内多舵机协同 | 同一个 `step.joints` 里的 goal **一次性全发出去**，不等。驱动端不同 ID 是并发执行的（每个 goal 一个线程） |
| 步与步之间串行 | 播放器等这一步**所有** `result` 回来才进下一步，日志里带时间戳可查 |
| 一步之内有先后 | 拆成两个 step。**不靠 duration 大小硬凑**，那条路每次改参数都要重算 |
| `duration_ms` 怎么定 | 900 步给 700~800ms；小幅往复（±200）给 400ms；低于 100ms 一律当错（会猛跳撞结构） |
| 不用 `sleep` 猜时间 | 全程用 `result` 当同步信号。`sleep(1.0)` 硬等的话，第三遍和第一遍一定对不上 |

两套动作：`stand_up.yaml`（6 step / 20 goal，自下而上逐级微屈 5 步 + 10 关节一起回中位 1 步）、
`wave.yaml`（13 step / 70 goal，单关节往复 —— 步数、关节数、时序结构完全不同的第二套，用来证明真是数据驱动）。

第三套 `show_group.yaml`（4 step / 16 goal）是本周实机演示用的动作组，按实测时序写的：每步 `duration_ms: 2500`、
`settle_ms: 2000`，幅度 ±200（抬腿 1700 / 下压 1300），四步为「抬 1&2 → 抬 3&4 → 四关节同时下压 → 八关节回中位」。
时长和幅度不是拍脑袋定的：早期版本的 ±60 / 700 ms 实测只走到 `1500 → 1507`（60 步里只挪 7 步），
原因是欠压 + 加减速行程不足，放宽到 2500 ms 后同样几只舵机可以稳定到位。

第四套 `step8.yaml`（6 step / 20 goal）与 `show_group` **结构刻意相反**，用来证明动作确实是数据而不是代码：

| | `show_group.yaml` | `step8.yaml` |
| --- | --- | --- |
| 编排逻辑 | 同步为主（1&2 与 3&4 依次抬，再四关节同时下压） | 交替为主（左腿抬/落 → 右腿抬/落 → 双腿同抬） |
| step / goal | 4 / 16 | 6 / 20 |
| 幅度 | ±200（1700 / 1300） | ±150（1650） |
| 参与关节 | 8 只（1,2,3,4,7,8,9,10） | 同左 |

两套走同一个 `sequence_player`、同一个 `/move_servo` action 接口，只换 `action_file` 参数——
评判时可以直接 `diff` 两份 yaml，除了 step 列表之外没有任何代码差异。

> `stand_up.yaml`（6 step / 20 goal）与 `wave.yaml`（13 step / 70 goal）是最初在桌面上单机验证时写的，
> 两者都包含 id=5。整机联调时 id=5 所在通道过载，被隔离出动作组，因此这两份保留作历史记录，
> 实机演示以 `show_group.yaml` / `step8.yaml` 为准。

---

## 10. 踩坑记录

完整版见 `调试记录.md`（含时间、现场取证）。这里挑三个最能说明问题的：

### ① `ros2 param set` 返回成功，但行为不变

**现象**：`ros2 param set /servo_driver pwm_max 1600` → `Set parameter successful`，再发 pwm=1700 仍然通过。
**怀疑**：代码读的是 `__init__` 里存进 `self.pwm_max` 的副本。
**验证**：在 `goal_callback` 里打印 `self.get_parameter('pwm_max').value`，打印出来是 1600，而判断用的是 `self.pwm_max` = 2100。
**结论**：删掉所有 `self.xxx` 缓存，统一走 `self._param(name)` 实时读。**只有一条路，不留第二种写法。**

### ② `Ctrl+C` 取消，节点没反应

**现象**：`send_goal --feedback` 跑到一半按 Ctrl+C，feedback 继续滚，舵机转完全程。
**怀疑**：`execute_callback` 跑在 executor 的单线程里，`time.sleep()` 把线程占死，cancel 请求进不来。
**验证**：换成 `MultiThreadedExecutor` + `ReentrantCallbackGroup` 后同一条命令立刻能打断。
**结论**：执行器必须多线程，且 Action 要挂在 `ReentrantCallbackGroup` 上；`execute_callback` 里每轮循环都要查 `is_cancel_requested`。

### ③ 驱动自己订阅 `/servo/command`，差点变成无限循环

**现象**：接通话题回放的第一次试跑，串口日志刷得停不下来。
**怀疑**：`_send_move()` 里 echo 到 `/servo/command`，而话题回调又调 `_send_move()` → 自己喂自己。
**验证**：在回调里打一行计数，1 秒内涨了上千次。
**结论**：话题回调只调 `_write_frame()`（只写串口、不回发），echo 只发生在 Action / 启动复位 / 取消回中位这三条路上。代码里写了注释，防止以后自己改回去。

### ④ 整机装完之后 10 只舵机集体「转不动」——不是代码，是温度

**现象**：桌面阶段 4 只舵机单独跑得好好的，装成整机后 `stand_up` 全部 `ABORTED`。目标 1580 只走到 1508（差 −71），
后面目标 1420、1700 更是完全不动；同一时刻用 `PRAD` 挨个读温度，**多只落在 T120~T159**，明显在热保护区。

**两个方向**：一是结构装配干涉把轴顶死了，二是带载 + 累积发热导致扭矩塌陷。

**判别**（这一步是关键，不用拆机就能分辨）：
```bash
# 断电，然后用手推每个关节
```
断电手推**能转、不卡**，结构没问题 → 排除干涉。再摸一下舵机壳，烫手。
于是结论锁定在**负载 + 热保护**：静态保持需要的扭矩本来就小，但整机自重 + 腿长力臂把需求扭矩拉高，
连续动作 51 s（bag 那次）之后壳温上来，扭矩再打折，`PRAD` 就永远追不上目标。

**为什么 `not_in_position` 一开始很像 bug**：它是 `pos_tolerance=30` 的到位校验在正常工作——
驱动如实报告「我没到位」，而不是假装成功。这个报错恰恰说明闭环校验是有效的。

**处理**：降低幅度（±40）、拉长单步时长（900 ms）、动作之间留停顿让温度回落、整机架空再跑。
这条也写进了 `调试记录.md` 的 2.3 节，是本周唯一一个「代码全对但机器不听话」的问题。

> 装配阶段的其他小坑（走线夹线、某个 ID 掉线）记在 `调试记录.md` 2.3 节。

（尤其是这个任务三里的小问题，前前后后做了一周，最终才让他动起来，太不容易了）
---

## 11. 下电三步

```
① 卸力（PULK）  →  ② 断动力电（拔电池）  →  ③ 关板卡（sudo shutdown now）
```

| 步骤 | 为什么是这个顺序 |
| --- | --- |
| ① 先卸力 | 舵机在断电前还顶着力矩。直接断动力电 = 让它在顶力矩的状态下瞬间失电，堵转发热、齿轮受冲击 |
| ② 再断动力 | 控制板如果还在上电状态被拔电池，会在异常电压下继续发无效指令，容易把某个通道打飞 |
| ③ 最后关板卡 | 板卡是 Linux，直接拔电会损坏文件系统，下次开不了机 |

卸力用一条命令搞定（`servo_cli` 或直接 `PULK` 全部 ID）：

```bash
ros2 run cquick_servo servo_cli --port /dev/cq_servo --ids 1 2 3 4 torque-off
```

四条红线：**禁止带电插拔舵机线 / 禁止反接电源 / 电池鼓包立即停用 / 禁止让舵机硬顶结构**。
（出现带电插拔或反接 = 整周不通过，这条没有余地。）

### 11.1 本次实测：一条脚本走完 ① ②

实际下电没有手工敲命令，而是用 `tools/safe_poweroff.py` 把「归位 → 卸力」串成一次会话：

```bash
source ~/ros2_ws/install/setup.bash
python3 tools/safe_poweroff.py /dev/ttyUSB0
```

终端输出按时间戳分三段，正好对应上面三步里的前两步：

```
[  0.0s] 串口 /dev/ttyUSB0
[  0.0s] 卸力前位置: {1: ..., 2: ..., ..., 10: ...}     ← 记录起始姿态
[  0.5s] id  1 -> 1500                                   ← 逐只回中位（8 只，间隔 0.35s）
   ...
[  3.9s] 归位后: {1: ..., ..., 10: ...}
         到位 8/8
[ 4.4s] 开始卸力 PULK
[ 4.4s] id  1 torque off                                 ← 1~10 全部下发 PULK
   ...
[ 6.x s] 卸力后: {1: ..., ..., 10: ...}                 ← 位置应基本不变，但已无扭矩
[ 6.x s] 完成：全部卸力，现在手推各关节确认松脱，再拔电池
```

几个刻意的设计：

| 做法 | 原因 |
| --- | --- |
| 先 `PULR` 恢复扭力再发位置 | 上电后若处于卸力态，直接发位置是不会动的（实测复现过：目标 1560 只走到 1507） |
| 归位只发 1/2/3/4/7/8/9/10 | id=5 过载隔离，id=6 与其镜像，四对关节力学对称，整机不会被单侧髋拧住 |
| 卸力对 1~10 全部下发 | 卸力是安全操作，没有理由漏掉任何一只 |
| `duration_ms=2500`、每只间隔 0.35s | 700 ms 实测走不到位；逐只下发让瞬时电流只有一只舵机的量级 |
| 卸力后再读一次位置 | 位置基本不变但手感变松，同时证明串口通讯仍然正常（不是掉线导致的假"卸力"） |

判据只有一条：脚本打完 `完成：全部卸力` 后**用手逐个推关节，8 个可动关节都能轻松转动**——
这时 PULK 才真正生效，才可以拔 XT60。注意顺序是「先手推确认，再拔电池」，不是先拔再推。

### 11.2 本次实测数据（14:49，`04_安全下电_归位卸力.png`）

那一次是从「上一段动作组刚跑完」的姿态直接下电的，所以起始位置是散的（1 号 1332、3 号 1304），
正好能看出归位这一步确实在起作用：

| 阶段 | 读回位置（1~10） | 说明 |
| --- | --- | --- |
| 卸力前 | 1:**1332** 2:1557 3:**1304** 4:1555 5:1493 6:1506 7:1503 8:1505 9:1504 10:1496 | 上一步动作残留姿态，1/3 号偏离中位 170~200 步 |
| 归位后 | 1:**1491** 2:1516 3:**1482** 4:1507 5:1493 6:1506 7:1503 8:1504 9:1507 10:1496 | 8 只全部落进 1500±40，`到位 8/8` |
| 卸力后 | 1:1491 2:1516 3:1482 4:1507 5:1494 6:1506 7:1503 8:1504 9:1507 10:1496 | 与归位后一致（最大差 1 步） |

**「卸力后位置不变」这一点值得单独说明**：PULK 只切断输出扭矩，不会让舵机主动转动；
位置读数不变、但手推能自由转动，两者合起来才是「卸力成功」的完整证据——
如果位置突然跳变，反而说明它在失去扭矩的瞬间被自重带走了，那就说明归位没做够。

| 材料 | 文件 |
| --- | --- |
| 归位 + 卸力的完整终端输出（含 10 只 `torque off`） | `04_安全下电_归位卸力.png` |
| 卸力与心跳看门狗的联动 | `04_看门狗跳闸_补充.png` |

---

## 12. 录屏（B 站）

仓库里不放视频，只放链接。7 段均为实机一次录制、**无剪辑无加速**，画面内同时含终端命令与舵机动作。

B 站：**CQuick电控 第3-4周考核 · tangjiayi**

| # | 内容 | 时长 | 对应评分项 | 链接 |
| --- | --- | --- | --- | --- |
| P1 | 任务 2 · 单舵机 正转 → 反转 → 回中位（id=1，feedback 逐级递减至全部 SUCCEEDED） | 0:25 | Action goal/feedback/result（12 分） | https://www.bilibili.com/video/BV1ZhypzxE2p/ |
| P2 | 任务 2 · 执行中 Ctrl+C 取消，`status: CANCELED`、`message: canceled`，随后回中位 SUCCEEDED | 0:18 | cancel 不生效 −5（反向项） | https://www.bilibili.com/video/BV1BhypzxE48/ |
| P3 | 任务 2 · `ros2 bag play` 回放，`/servo/command` 驱动舵机复现录制时的动作 | 0:54 | bag 交付 + play 复现（4 分） | https://www.bilibili.com/video/BV1BhypzxEpw/ |
| P4 | 任务 2 · `ros2 bag record` 录制 + `bag info` 核验：61.4 KiB / 51.2 s / 260 条（仅 2 话题，未用 `-a`） | 1:10 | bag 交付（4 分）佐证 | https://www.bilibili.com/video/BV1BhypzxETR/ |
| P5 | 任务 3 · 整机上电后对多个关节逐个下发 goal（本次操作 id=2/3/5，pwm=1580），日志含 goal accepted 与位置 readback | 1:25 | 逐个控制方向正确（4 分） | https://www.bilibili.com/video/BV1BhypzxEAY/ |
| P6 | 任务 3 · 多舵机动作组 `show_group.yaml` 实机执行：P1 抬 1&2 → 1700、P2 抬 3&4 → 1700、P3 四关节同时下压 → 1300、P4 八关节回中位 → 1500；终端打印 phase 时间戳与 `read_position` 读回值，与腿部姿态同框 | 0:21 | yaml 驱动多舵机动作组（10 分） | https://www.bilibili.com/video/BV1fLy5zeEq3/ |
| P7 | 任务 4 · 安全下电全流程：8 只归位（`到位 8/8`）→ 1~10 全部 `PULK` 卸力（逐行 `torque off`）→ 卸力后位置不变 → 手推关节确认松脱 → 拔 XT60 | 0:17 | 安全下电顺序（10 分） | https://www.bilibili.com/video/BV1zLy5zeEVL/ |

> P5 说明：整机带载连跑时受舵机热保护限制（实测外壳温度 T120~T159），因此该段为逐关节控制验证；
> 动作组实机执行见 **P6**（`show_group.yaml` 4 step / 16 goal 全流程），
> 编排结构与 `dry_run` 记录见第 7 节与 `config/actions/` 下的 yaml。

---

## 13. 加分项实现说明（6 项 / +10 封顶）

| 加分项 | 分值 | 在哪 | 怎么验 |
| --- | --- | --- | --- |
| udev 固定设备名 | +2 | `src/cquick_servo/udev/99-cquick-servo.rules`，`port` 默认值已改成 `/dev/cq_servo` | 装上规则后 `ls -l /dev/cq_servo`，拔插一次名字不变（截图 `01_串口设备.png`：`/dev/cq_servo → ttyUSB0`、`by-id/usb-1a86_USB_Serial-if00-port0` 同时指向同一个节点） |
| PRAD 闭环校验 | +3 | `servo_driver._verify_and_finish()` | 日志里能看到「未到位 → 第 1 次重发 → 重发后到位」三行 |
| 安全看门狗 | +3 | `_on_watchdog_timer()` / `_on_confirm_resume()` | 跑 `move_client`（它发心跳）→ Ctrl+C 掉它 → 1 秒后驱动打 ERROR 并全部 PULK，再发 goal 被拒；调 `/servo/confirm_resume` 后恢复（截图 `04_看门狗跳闸_补充.png`：`看门狗已武装` → `看门狗跳闸：1.0s 没收到 /servo/heartbeat`） |
| 动作组抢占 | +4 | `_handle_accepted_cb()` + `_preempt` 事件 | 播放 `wave.yaml` 期间打一条同 ID 的 `send_goal`：旧动作 result 变 `preempted`，新 goal 从当前位置接着走，**不回中位、不跳变** |
| 串口掉线自动重连 | +3 | `SerialLink` + `_on_reconnect_timer()` | 跑动作时拔 USB：节点不崩、日志 WARN；插回去 ≤1 秒打「串口已连接」，能继续发 goal |
| pytest 单测 | +2 | `test/test_protocol.py`、`test/test_actions.py` | `colcon test`，汇总 `28 tests, 0 errors, 0 failures`（其中 pytest 用例 23 个） |

### 13.1 udev 规则

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="cq_servo", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="cq_servo", MODE="0666", GROUP="dialout"
```

安装：

```bash
sudo cp src/cquick_servo/udev/99-cquick-servo.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/cq_servo
```
### 13.2 看门狗的边界设计

- **只在收到过心跳之后才武装**：否则「刚 launch、还没起客户端」会被误判成卡死，第一次 goal 直接全卸力；
- 跳闸后**锁死**：`goal_callback` 一律 REJECT，`/servo/command` 也忽略，不允许自己恢复；
- 恢复必须外部确认（`/servo/confirm_resume`），确认后**重新武装**，等下一拍心跳；
- 卸力只发 `PULK`，不发别的 —— 跳闸时最不该做的就是「再动一下」。

---

## 14. 目录结构与提交

```
电控组/Week3-4/tangjiayi/
├── README.md                     ← 本文件
├── 调试记录.md
├── 交付说明.txt                  ← 18 张图 + 代码数据的索引，评审先看这个
├── 01~03_*.png                   ← 18 张：12 终端截图 + 6 实拍（均在根目录）
├── bag/run1/                     ← metadata.yaml + .db3（61.4 KiB / 51.2 s / 260 条）
├── tools/                        ← 串口找口 / 探测 / 改 ID / 手动指令 / 安全下电
└── src/
    ├── cquick_servo_interfaces/
    │   ├── msg/ServoCommand.msg
    │   ├── action/MoveServo.action
    │   ├── CMakeLists.txt
    │   └── package.xml
    └── cquick_servo/
        ├── cquick_servo/
        │   ├── servo_driver.py       ← Action Server（核心）
        │   ├── move_client.py        ← Action Client
        │   ├── sequence_player.py    ← 动作组播放器（读 yaml）
        │   ├── servo_cli.py          ← 任务 1 的桌面调试工具
        │   ├── servo_protocol.py     ← 协议纯函数（可单测）
        │   └── serial_link.py        ← 串口 + 自动重连
        ├── launch/servo.launch.py
        ├── config/servo_params.yaml
        ├── config/actions/{stand_up.yaml, wave.yaml, show_group.yaml, step8.yaml}
        ├── test/{test_protocol.py, test_actions.py}
        ├── udev/99-cquick-servo.rules
        ├── resource/cquick_servo
        ├── package.xml
        └── setup.py
```

`build/`、`install/`、`log/` 一律不进仓库（仓库根 `.gitignore` 里加）：

```gitignore
build/
install/
log/
__pycache__/
*.pyc
```

编译：

```bash
cd ~/ros2_ws
colcon build --packages-select cquick_servo_interfaces cquick_servo
source install/setup.bash
ros2 interface show cquick_servo_interfaces/action/MoveServo

15 心得体会

这次的任务比之前难多了，说实话我自己也没想到我能全部完成，这几天真正驱动整机的时候我非常兴奋，以至于我没有消去视频的原声，因为我也想在自己观看的时候回味一下当时的心情，虽然我一开始啥也不懂，还给队长带来了一点小震撼，但是到了后面这周做任务，我感觉自己通透了不少，也谨慎了不少，感谢这个机会让我接触这些新东西，也希望有机会和前辈们一起打比赛
