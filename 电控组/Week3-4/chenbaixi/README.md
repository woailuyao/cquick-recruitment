# CQuick 电控组 · 第 3、4 周 · chenbaixi

> ROS 2 Humble / Ubuntu 22.04 / Python 3.10 / pyserial

---

## 1. 架构

```
键盘 / launch 参数 / yaml
        │
        ▼
Action Client  (move_client.py / sequence_player.py)
        │  goal: {id, pwm, duration_ms}
        ▼
Action Server  (servo_driver.py)
        ├─ goal_callback     pwm 限位 / id 白名单 / duration 合法性 → REJECT 或 ACCEPT
        ├─ cancel_callback   ACCEPT
        ├─ execute_callback  拼 ASCII → 串口发送 → 每 200 ms 发 feedback
        │                    → 每次循环检查 is_cancel_requested
        │                    → 结束读 PRAD 校验 ±30，超差自动重发一次
        └─ watchdog（心跳超时 → 全部 PULK 并锁死）
        │  b"#001P1700T1000!"
        ▼
/servo/command  (cquick_servo_interfaces/msg/ServoCommand)   ← 供 bag 录制/回放
        ▼
串口 /dev/ttyUSB0  115200 8-N-1  →  CH340  →  24 路舵机控制板
        ▼
半双工总线（同一根线，靠 ID 区分）→ 总线舵机

/servo/state (200 ms 心跳) ─────────────► 看门狗
```

| 包 | 构建类型 | 内容 |
| --- | --- | --- |
| `cquick_servo_interfaces` | `ament_cmake` | `msg/ServoCommand.msg`、`action/MoveServo.action` |
| `cquick_servo` | `ament_python` | `servo_driver.py`、`move_client.py`、`sequence_player.py`、`launch/`、`config/` |

---

## 2. 参数表

| 参数 | 默认值 | 为什么是这个默认值 |
| --- | --- | --- |
| `port` | `/dev/ttyUSB0` | CH340 默认枚举名。建议配 udev 改 `/dev/cq_servo`，避免插多个 USB 设备后变 ttyUSB1 |
| `baudrate` | `115200` | 手册标称值，8-N-1。9600/57600 等均无响应 |
| `pwm_min` | `900` | 电气下限 500 太靠边，留余量防顶死结构 |
| `pwm_max` | `2100` | 电气上限 2500 同理留余量；桌面只用 1450~1550 |
| `home_pwm` | `1500` | 中位标准值，出厂零点标定基准 |
| `default_duration_ms` | `800` | 800 ms 走 ±100 肉眼可见且不猛跳 |
| `servo_ids` | `[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]` | 舵机 ID 白名单。**含 0** 以兼容单舵机桌面测试；整机为 1~10|
| `startup_reset` | `true` | 上电先发 PULR |
| `prad_tolerance` | `30` | 到位静差约 ±10，稍微取一点冗余部分|
| `prad_retry` | `true` | 未到位自动重发一次|
| `read_timeout_ms` | `250` | 太短漏读，太长拖慢 feedback |
| `feedback_period_ms` | `200` | 细则要求 ≥5 Hz |
| `state_period_ms` | `200` | 心跳 5 Hz |
| `cmd_timeout_ms` | `0` | 0 = 关闭旧版「指令超时卸力」 |
| `watchdog_enable` | `false` | 心跳看门狗开关，默认关 |
| `watchdog_timeout_ms` | `1000` | 细则要求 1 秒 |
| `watchdog_reset` | `false` | 外部确认恢复位 |

零硬编码自检

```bash
grep -n "ttyUSB\|115200\|2100\|900" src/cquick_servo/cquick_servo/servo_driver.py \
  | grep -v declare_parameter | grep -v get_parameter
# 无输出 = 通过
```

---

## 3. 接口契约

### msg/ServoCommand.msg

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `id` | `uint8` | 舵机 ID |
| `pwm` | `uint16` | 目标 PWM |
| `duration_ms` | `uint16` | 从当前位置到目标耗时ms|

### action/MoveServo.action

```
# Goal
uint8  id
uint16 pwm
uint16 duration_ms
---
# Result
bool   success
uint16 final_pwm      
string message       
                      
---
# Feedback
uint16 current_pwm   
float32 progress      
```

| 阶段 | 内容 |
| --- | --- |
| goal | 收到即校验，不合规立刻 REJECT，不占执行槽 |
| feedback | 每 200 ms 一条，progress = elapsed/duration，current_pwm 线性插值 |
| result | 完成 → 读 PRAD → 成功/未到位；取消 → success=false, message=canceled 且先回中位 |

---

## 4. 一条启动命令

```bash
ros2 launch cquick_servo servo.launch.py
```

输出：

```
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [servo_driver-1]: process started with pid [xxxx]
[servo_driver-1] [INFO] [servo_driver]: 串口已打开 /dev/ttyUSB0 @ 115200
[servo_driver-1] [INFO] [servo_driver]: startup_reset：已发送 PULR
[servo_driver-1] [INFO] [servo_driver]: servo_driver 就绪 port=/dev/ttyUSB0 baud=115200
                 pwm=[900, 2100] ids=[0, 1, 2, ..., 10] timeout=0
```

```bash
ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB1
ros2 launch cquick_servo servo.launch.py demo:=true
```

---

## 5. 安全区间怎么试出来的

1. 电气范围来自手册：500~2500，1500 中位。
2. 实测：单舵机空载 1500 起 ±100 到 1700/1300 均正常，但 1700 已接近机械极限；
   接 4 个舵机同时动时 2S 1500mAh 电池电压被拉低导致抖动。
3. 结论：`pwm_min=900`、`pwm_max=2100` 作为软件边界。
4. 听到持续「嗡嗡」立刻 PULK,防堵转发热。

---

## 6. 运行时改参数· 真实输出

```bash
ros2 param get /servo_driver pwm_max
# Integer value is: 2100

ros2 param set /servo_driver pwm_max 1600
# Set parameter successful

ros2 action send_goal /move_servo cquick_servo_interfaces/action/MoveServo "{id: 0, pwm: 1700, duration_ms: 1000}"
# Goal was rejected.
# 驱动日志: [WARN] 拒绝 goal：pwm 1700 超出安全范围 [900, 1600]

ros2 param set /servo_driver pwm_max 100000
# Setting parameter failed: pwm_max=100000 超出电气范围 500~2500

ros2 param set /servo_driver pwm_max 2100
# Set parameter successful

ros2 action send_goal /move_servo cquick_servo_interfaces/action/MoveServo "{id: 0, pwm: 1700, duration_ms: 1000}"
# Result: success: true, final_pwm: 1694, message: ok
```

判断一律走 `self.get_parameter(name).value` 实时读取，
不在 `__init__` 缓存副本；`add_on_set_parameters_callback` 做区间与互斥校验。

---

## 7. bag

录制

```bash
ros2 bag record -o bag/run1 /servo/command /servo/state
```

`ros2 bag info bag/run1`：

```
Bag size:          33.5 KiB
Storage id:        sqlite3
Duration:          27.4s
Messages:          141
Topic information: Topic: /servo/state   | Type: cquick_servo_interfaces/msg/ServoCommand | Count: 138
                   Topic: /servo/command | Type: cquick_servo_interfaces/msg/ServoCommand | Count: 3
```

回放：

```bash
ros2 bag play bag/run1
```

驱动订阅 `/servo/command`，回放时依次收到 3 条指令，
舵机按 1700 → 1300 → 1500 复现，与录屏动作一致。
（`/servo/command` 与 Action 共用同一个 `_send()`，保证回放与实机一致。）

---

## 8. 接线

| 线 | 接到 |
| --- | --- |
| CH340 `TxD` | 控制板「用户」4 针 · `RX1` |
| CH340 `Rxd` | 控制板「用户」4 针 · `TX1` |
| CH340 `GND` | 控制板「用户」4 针 · `GND` |
| CH340 `3V3` / `5V` | 不接|
| 电池 `+`（红） | 绿色 2P 端子 `+` |
| 电池 `−`（黑） | 绿色 2P 端子 `−` |
| 舵机 白/橙 | 通道信号 `S` |
| 舵机 红 | `+` |
| 舵机 棕/黑 | `−` |

串口*115200 8-N-1，`/dev/ttyUSB0`，属主组 `dialout`。

不需要 sudo：`01_串口设备.png` 可见设备权限为 `crw-rw---- root:dialout`，且 `groups` 输出含 `dialout`，因此普通用户可直接读写该设备， 不需要 `sudo`（`pyserial` 已装到系统 Python）。
用 `sudo` 跑 ROS2 节点会让 `~/.ros` 与 `install/` 属主变为 root，并丢失 `AMENT_PREFIX_PATH` 等环境变量，故全程不使用 `sudo`。

---

## 9. 动作 yaml

动作是数据，播放器 `sequence_player.py` 一行不改。
当前交付两套，均在 `config/actions/`（满足细则「至少两套」）。

### 9.1 `stand_up.yaml` — 5 个 step

```yaml
name: stand_up
default_duration_ms: 900
steps:
  - name: 前半组收拢
    commands:
      - {id: 1, pwm: 1540, duration_ms: 900}
      - {id: 3, pwm: 1460, duration_ms: 900}
      - {id: 5, pwm: 1540, duration_ms: 900}
      - {id: 7, pwm: 1460, duration_ms: 900}
  - name: 前半组展开
    commands:
      - {id: 1, pwm: 1460, duration_ms: 900}
      - {id: 3, pwm: 1540, duration_ms: 900}
      - {id: 5, pwm: 1460, duration_ms: 900}
      - {id: 7, pwm: 1540, duration_ms: 900}
  - name: 后半组收拢
    commands:
      - {id: 2, pwm: 1540, duration_ms: 900}
      - {id: 4, pwm: 1460, duration_ms: 900}
      - {id: 6, pwm: 1540, duration_ms: 900}
      - {id: 8, pwm: 1460, duration_ms: 900}
  - name: 后半组展开
    commands:
      - {id: 2, pwm: 1460, duration_ms: 900}
      - {id: 4, pwm: 1540, duration_ms: 900}
      - {id: 6, pwm: 1460, duration_ms: 900}
      - {id: 8, pwm: 1540, duration_ms: 900}
  - name: 全部回中位
    commands:
      - {id: 1, pwm: 1500, duration_ms: 900}
      - {id: 2, pwm: 1500, duration_ms: 900}
      - {id: 3, pwm: 1500, duration_ms: 900}
      - {id: 4, pwm: 1500, duration_ms: 900}
      - {id: 5, pwm: 1500, duration_ms: 900}
      - {id: 6, pwm: 1500, duration_ms: 900}
      - {id: 7, pwm: 1500, duration_ms: 900}
      - {id: 8, pwm: 1500, duration_ms: 900}
      - {id: 9, pwm: 1500, duration_ms: 900}
      - {id: 10, pwm: 1500, duration_ms: 900}
```

### 9.2 `wave.yaml` — 4 个 step

```yaml
name: wave
default_duration_ms: 800
steps:
  - name: 奇数组抬起
    commands:
      - {id: 1, pwm: 1530, duration_ms: 800}
      - {id: 3, pwm: 1530, duration_ms: 800}
      - {id: 5, pwm: 1530, duration_ms: 800}
      - {id: 7, pwm: 1530, duration_ms: 800}
  - name: 奇数组落下
    commands:
      - {id: 1, pwm: 1470, duration_ms: 800}
      - {id: 3, pwm: 1470, duration_ms: 800}
      - {id: 5, pwm: 1470, duration_ms: 800}
      - {id: 7, pwm: 1470, duration_ms: 800}
  - name: 偶数组接力
    commands:
      - {id: 2, pwm: 1530, duration_ms: 800}
      - {id: 4, pwm: 1530, duration_ms: 800}
      - {id: 6, pwm: 1530, duration_ms: 800}
      - {id: 8, pwm: 1530, duration_ms: 800}
  - name: 全部归位
    commands:
      - {id: 1, pwm: 1500, duration_ms: 800}
      - {id: 2, pwm: 1500, duration_ms: 800}
      - {id: 3, pwm: 1500, duration_ms: 800}
      - {id: 4, pwm: 1500, duration_ms: 800}
      - {id: 5, pwm: 1500, duration_ms: 800}
      - {id: 6, pwm: 1500, duration_ms: 800}
      - {id: 7, pwm: 1500, duration_ms: 800}
      - {id: 8, pwm: 1500, duration_ms: 800}
      - {id: 9, pwm: 1500, duration_ms: 800}
      - {id: 10, pwm: 1500, duration_ms: 800}
```

### 9.3 时序规则

| 需求 | 表达方式 |
| --- | --- |
| 一步内多个舵机同时动 | 放进同一个 `step.commands`，并行 `send_goal_async` |
| 步与步串行*| 列表顺序 + 等上一步所有 `get_result_async` 回来 |
| 一步内有先后| 拆成两个 step，不靠调 `duration_ms` 硬凑 |

- 不用 `sleep：每步等所有 goal 的 `result` 再发下一步，
  日志打印 `[result] step=… id=… success=… final_pwm=… t=<时间戳>`，可据此核对时序。


```bash
ros2 run cquick_servo sequence_player --ros-args   -p action_file:=$HOME/ros2_ws/src/cquick_servo/config/actions/stand_up.yaml
ros2 run cquick_servo sequence_player --ros-args   -p action_file:=$HOME/ros2_ws/src/cquick_servo/config/actions/wave.yaml
```

### 9.4 `dry_run` 可核对最终协议帧

```bash
ros2 run cquick_servo sequence_player --ros-args   -p action_file:=$HOME/ros2_ws/src/cquick_servo/config/actions/stand_up.yaml -p dry_run:=true
```

`dry_run` 打印的是最终要发到串口的 ASCII 帧
可直接核对 yaml → 协议的映射

## 10. 踩坑记录

### 坑 1 · 舵机全程无回包

- **现象**：`#001PID!` 在 8 种波特率 × ID 1~255 全部无回包，CH340 模块的 RX 灯从不闪
- **怀疑**：波特率不对 / 接线错 / 控制板没把串口数据转发到总线
- **怎么验证**：
  1. 先做**回环测试**（TxD 与 Rxd 短接）→ 能收到自己发的数据 → 排除模块与线材
  2. 万用表量「用户」4 针的 TX1 空闲电平 → 3.25 V，引脚是活的
  3. 队长提示「默认编号是 000」→ 发 `#000PID!` 立刻收到 `b'#000P!'`
  4. 再发 `#000PRAD!` → `b'#000P1504!'`，链路完全打通
- **结论**：该型号**出厂 ID = 000**（不是常见的 1），波特率 115200。
  附带发现：`#NNNPID!` 的回包是 `#NNNP!`（**不含 ID 数字**）,所以判断某个 ID 是否存在必须用 `#NNNPRAD!`

### 坑 2 · PRAD 永远读不到（读取顺序错了）

- **现象**：发完位置指令，`PRAD` 一直返回空，但舵机确实动了
- **怀疑**：舵机不回包 / 解析逻辑有 bug
- **怎么验证**：在 `_read_pwm` 里逐步打印——发现 `reset_input_buffer()` 被放在
  `write()` **之后**，把刚到达的回包清空了
- **结论**：`reset_input_buffer()` 必须放在**发送之前**；
  读取要「轮询到 `!` 结尾或超时」，不能用固定 `sleep` 再 `read(n)`。修复后 PRAD 稳定读回

### 坑 3 · Ctrl+C 取消 goal 无效

- **现象**：`ros2 action send_goal --feedback` 按 `Ctrl+C`，goal 仍然跑完，result 是 `SUCCEEDED`
- **怀疑**：服务端 cancel 回调没接上 / executor 被阻塞
- **怎么验证**：改用客户端显式 `goal_handle.cancel_goal_async()`，
  确认服务端 `cancel_callback` 被调用；再回看 `rclpy.spin()` 单线程下 `execute_cb` 里
  `sleep` 完整个动作时长会让 cancel 回调排不上队
- **结论**：两件事必须一起做——
  ① 取消要用客户端显式 cancel（`ros2 action send_goal` 的 `Ctrl+C` **不发 cancel 请求**，只杀进程）；
  ② 服务端要 `MultiThreadedExecutor(num_threads=4)` + `ReentrantCallbackGroup`，
  且 `execute_cb` 每 200 ms 小步 sleep 并检查 `is_cancel_requested`

### 其他

- brltty` 抢占 CH340 → `lsusb` 有设备但 `/dev/ttyUSB*` 不存在；`sudo apt remove -y brltty` 解决
- 控制板 Type-C 是 HID 不是串口→ 必须外接 CH340 模块
- CH340 误接 3V3/5V 烧毁）→ 此后只接 Tx/Rx/GND 三根，3V3/5V 永久空置
- 两个同名 `/servo_driver`** → `ros2 param set` 会打错目标（报参数不存在）；启动前必须 `pkill -f servo_driver`
- yaml 缩进写错→ `Cannot have a value before ros__parameters`；改完必须 `yaml.safe_load` 验证并重新 `colcon build`

### 坑 4 · 并行 goal 抢串口

- 现象：一个 step 并行发 4 个 goal 时，日志出现
  `未到位（读回 None vs 目标 1540）`、`差 66（目标 1540 实际 1474）`、
  `错误 'NoneType' object has no attribute 'flush'`。
- **我的怀疑**：多个 `execute_callback` 在 Reentrant 组里并发，**共用同一个串口**。
- **怎么验证的**：看时间戳——4 个 `#NNNPRAD!` 几乎同一毫秒发出；
  `差 66` 的读回值正好是**隔壁舵机**的位置，说明回包在总线上交错。
- **结论**：给串口 IO 加 `threading.RLock()`（可重入，因为 `_read_pwm` 内部还要 `_send`），
  把「清空缓冲 → 写指令 → 读回包」整段锁住。锁只保护**单帧事务**，
  4 个舵机仍然并行移动（feedback 照发），只是串口访问排队。
  修复后 `prad_no_reply` 消失。

## 11. 下电三步

```
① 卸力   所有舵机发 #NNNPULK!
② 断动力 拔电池
③ 关板卡 sudo shutdown now（不是直接掐电）
```

| 步骤 | 原因 |
| --- | --- |
| 先卸力 | 断电时舵机还在满力矩顶结构，直接断电极易堵转发热烧舵机 |
| 再断电池 | 先断动力，避免控制板在电压跌落瞬间乱发指令 |
| 最后关板卡 | 直接断电会损坏文件系统，下次开不了机 |

禁止带电插拔舵机线、禁止反接电源、电池鼓包立即停用。

---

## 12. 录屏
逐个舵机运动https://t.bilibili.com/1273953107519209490?share_source=pc_native
三遍演示https://t.bilibili.com/1273952854127673367?share_source=pc_native
换yamlhttps://t.bilibili.com/1273951900654370833?share_source=pc_native
改参数后行为变化https://t.bilibili.com/1273951887755837465?share_source=pc_native
安全下电流程https://t.bilibili.com/1273951810420211734?share_source=pc_native
取消打断https://t.bilibili.com/1273951630031585289?share_source=pc_native
bag复现https://t.bilibili.com/1273557175941201958?share_source=pc_native
舵机运动https://t.bilibili.com/1273557051372470288?share_source=pc_native
正反转回中位https://t.bilibili.com/1273556591814115362?share_source=pc_native





