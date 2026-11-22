# CQuick 电控组 Week3-4 · 整机拼装与舵机控制

- 姓名拼音：wangxiaotong
- 环境：VMware + Ubuntu 22.04 + ROS 2 Humble，Python 3.10，pyserial 3.5
- 串口：`/dev/cquick_servo`（udev 固定名），115200 8-N-1

**硬件链路**：USB 转 TTL（CH340，`lsusb` = `1a86:7523`）→ 24 路总线舵机控制板 → 总线舵机 ×10

**舵机映射（10 颗全部在线，PRAD 为 `scan_all.py` 实测回读）**

| ID | 部位 | PRAD 回读 | ID | 部位 | PRAD 回读 |
| --- | --- | --- | --- | --- | --- |
| 1 | 左脚 | 1496 | 6 | 左后膝 | 1503 |
| 2 | 右脚 | 1510 | 7 | 5 与 9 的连接处 | 1504 |
| 3 | 右前膝 | 1484 | 8 | 6 与 10 的连接处 | 1506 |
| 4 | 左前膝 | 1484 | 9 | 髋部 | 1511 |
| 5 | 右后膝 | 1503 | 10 | 髋部 | 1505 |

---

## 1. 架构

```
                        ┌─────────────────────────────────────┐
   ros2 action send_goal│                                     │
  ─────────────────────►│  cquick_servo / servo_driver (Node) │
                        │                                     │
  /servo/command ──────►│  _command_cb ──┐                    │
   (ServoCommand)       │                ├─► _send_move() ────┼──► /dev/cquick_servo ──► 控制板 ──► 舵机
  /move_servo ─────────►│  _execute_cb ──┘      (唯一出口)     │        115200 8-N-1
   (MoveServo)          │                                     │
                        │  _publish_state (5 Hz) ─► /servo/state (ServoState × 10)
                        │  _watchdog_tick (5 Hz) ─► 1 s 无心跳则全 PULK
                        │  _try_reopen_serial (0.5 Hz) ─► 串口掉线自动重开
                        └─────────────────────────────────────┘
                                  ▲
                                  │ Action /move_servo
                        ┌─────────┴───────────┐
                        │ sequence_player     │ 读 config/actions/*.yaml
                        │ (动作组播放器)       │ step 内并发、step 间等 result
                        └─────────────────────┘
```

**两个包**

| 包 | 类型 | 可执行文件 | 作用 |
| --- | --- | --- | --- |
| `cquick_servo_interfaces` | ament_cmake | — | `msg/ServoCommand`、`msg/ServoState`、`action/MoveServo` |
| `cquick_servo` | ament_python | `servo_driver` | 驱动节点：串口、Action、话题、状态、看门狗、重连 |
| `cquick_servo` | ament_python | `sequence_player` | 动作组播放器：读 yaml，调 Action 播动作组 |

`ros2 pkg executables cquick_servo` 实际输出：

```
cquick_servo sequence_player
cquick_servo servo_driver
```

**Action / 话题**

| 名字 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/move_servo` | `cquick_servo_interfaces/action/MoveServo` | 上位机 → 驱动 | 单舵机移动，带 feedback 与 cancel |
| `/servo/command` | `cquick_servo_interfaces/msg/ServoCommand` | 上位机 → 驱动 | 即发即忘，与 Action 共用同一个底层发送函数 `_send_move()` |
| `/servo/state` | `cquick_servo_interfaces/msg/ServoState` | 驱动 → 上位机 | 每个 ID 一条，5 Hz |
| `/servo/heartbeat` | `std_msgs/msg/Empty` | 上位机 → 驱动 | 看门狗心跳 |

---

## 2. 参数表

全部通过 `declare_parameter` 声明，代码里**任何地方都不缓存**，一律 `self._param(name)` 实时读。

| 参数 | 默认值 | 为什么是这个值 |
| --- | --- | --- |
| `port` | `/dev/cquick_servo` | udev 固定名挡住 `ttyUSB0/1/2` 漂移；换机器只改这一处 |
| `baudrate` | `115200` | 手册标称 115200 8-N-1，实测直连即可通 |
| `pwm_min` | `900` | 电气范围是 500–2500，但结构上低于 900 就开始顶限位，留余量 |
| `pwm_max` | `2100` | 同上，反向取 2100 |
| `center_pwm` | `1500` | 手册中立位，装舵盘前必须先标定到这个值 |
| `default_duration_ms` | `800` | yaml 不写时长时的兜底（注意：本机固件会忽略 <1000 的时间，见 §10.4） |
| `servo_ids` | `[1, 2, 3, 4, 5, 6, 7, 8, 9, 10]` | 由 `scan_all.py` 实测扫描得出，10 颗全部在线 |
| `startup_reset` | `true` | 启动时把全部 ID 推回 1500，保证每次启动姿态一致 |
| `serial_timeout` | `0.2` | 单帧读取超时；太小会截断回显，太大会拖慢单次移动 |
| `state_rate_hz` | `5.0` | 5 Hz × 10 个 ID = `/servo/state` 约 50 Hz |
| `feedback_hz` | `5.0` | 任务书要求 feedback ≥5 Hz，取最小值省总线 |
| `prad_tolerance` | `30` | 实测 PRAD 偏差 ≤8，取 30 给零点漂移和结构回弹留余量 |
| `prad_retry` | `1` | 超差重发一次；再多会拖长一次 goal 的时间，意义不大 |
| `watchdog_enabled` | `true` | 安全功能默认开 |
| `watchdog_timeout` | `1.0` | 任务书要求 1 s 无心跳卸力 |

启动时 `_self_check()` 会把 15 个参数打印成一行，并核对四条一致性关系（`pwm_min < pwm_max`、`center_pwm` 落在区间内、`servo_ids` 非空无重复、`baudrate` 是常用档位）；不一致直接打 `FAIL`，而不是带病启动。证据见 `02_参数自检.png`。

---

## 3. 接口契约

`MoveServo.action`

```
# Goal
uint8  id            # 舵机逻辑 ID，1..254
uint16 pwm           # 目标位置，必须落在 [pwm_min, pwm_max]
uint16 duration_ms   # 期望转动时长，毫秒
---
# Result
bool   success       # 到位且通过 PRAD 校验为 true
uint16 final_pwm     # 结束时的实际位置（PRAD 回读；取消时为回中位的值）
string message       # 失败原因，如「未到位：目标 1900 实际 1506（差 -394，阈值 ±30）」
---
# Feedback
uint16 current_pwm   # 按 duration 线性插值出的当前位置（不是实测值）
float32 progress     # 0.0 → 1.0
```

**Goal 为什么是摊平的？**  
因为现场用命令行发：

```bash
"{id: 1, pwm: 1700, duration_ms: 1000}"
```

如果套一层 `{command: {...}}`，多一层括号就多一次打错的机会。摊平之后一条命令只有一个括号层。

**feedback 什么时候发？**  
进入 `_execute_cb` 后立刻发第一帧，之后每 200 ms 一帧（=5 Hz），直到结束或取消。`current_pwm` 是**插值预测**，不是舵机实测——实测只在结束时用 PRAD 校验一次。

**result 什么时候回？**  
三种：PRAD 校验通过 → `success=true`；超差且重发次数用尽 → `success=false` 并在 `message` 里写明差值；被取消 → `success=false`、`message="canceled"`、状态 `CANCELED`。

---

## 4. 一条启动命令

```bash
ros2 launch cquick_servo servo.launch.py
```

启动输出（实测）：

```
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [servo_driver-1]: process started with pid [xxxxx]
[servo_driver-1] [INFO] [servo_driver]: 串口已打开：/dev/cquick_servo @ 115200 8-N-1
[servo_driver-1] [INFO] [servo_driver]: 启动回中位：[1, 2, 3, 4, 5, 6, 7, 8, 9, 10] -> 1500
```

**launch 参数**

| 参数 | 默认 | 作用 |
| --- | --- | --- |
| `port` | `/dev/cquick_servo` | 覆盖 yaml 里的端口：`ros2 launch ... port:=/dev/ttyUSB1` |
| `demo` | `false` | `demo:=true` 额外起一个进程发一条示范指令，用于一键演示 |

```bash
ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB1
ros2 launch cquick_servo servo.launch.py demo:=true
```

---

## 5. 安全区间与来历

`pwm_min=900`、`pwm_max=2100`。

- 手册给的电气范围是 **500–2500**，那是“不会烧”的范围，不是“能转”的范围。
- 整机膝部的动作组实测行程集中在 **1300–1700**（见 §9），上下各留 200 以上余量，取 **900–2100**。
- 更靠边界的值（如 2500）会让电机顶着机械限位持续堵转，电流升高、外壳发烫，位置也不会再变，所以不用。
- `pwm_min >= pwm_max` 时参数回调直接拒绝（`Setting parameter failed: pwm_min 必须小于 pwm_max`）。

---

## 6. 运行时改参数（六步实录）

真实终端输出：

```
$ ros2 param set /servo_driver pwm_max 800
Setting parameter failed: pwm_min 必须小于 pwm_max

$ ros2 param set /servo_driver pwm_max 6000000000000
Setting parameter failed: pwm_max=6000000000000 超出电气范围 500~2500

$ ros2 param set /servo_driver pwm_max 1900
Set parameter successful

$ ros2 param get /servo_driver pwm_max
Integer value is: 1900
```

改完立刻对新指令生效（不用重启），越界指令被丢弃：

```
$ ros2 topic pub --once /servo/command cquick_servo_interfaces/msg/ServoCommand \
    "{id: 1, pwm: 1950, duration_ms: 1000}"
（驱动端）[WARN] [servo_driver]: pwm=1950 超出 [900, 1900]，丢弃
```

三点说明：

1. 回调里**先把 `pwm_min` / `pwm_max` / `center_pwm` 合并成一份 pending 再校验**。因为 `ros2 param set` 一次只送一个参数，拿新 `pwm_max` 和旧 `pwm_min` 比，才不会误拒。
2. 校验失败返回 `successful=False`，参数**不落盘**——所以“拒绝”之后要用 `param get` 确认还是旧值。
3. 越界判断发生在 `_command_cb` 里，用的是**当前**参数值，所以改完立刻对新指令生效。

---

## 7. bag 里录了什么 / 怎么回放

```bash
ros2 bag record -o bag/run1 /servo/command /servo/state
```

只录两个话题，**不用 `-a`**——`-a` 会把 `/move_servo/_action/feedback` 之类也录进去，任务书明确只要求这两个。

录制内容：连续发三条 `ServoCommand`（1700 → 1300 → 1500），时长 20 秒左右。

`ros2 bag info bag/run1` 实测输出：

```
Files:             task2_0.db3
Bag size:          25.5 KiB
Storage id:        sqlite3
Duration:          20.401095971s
Messages:          106
Topic information: Topic: /servo/command | Type: cquick_servo_interfaces/msg/ServoCommand | Count: 3
                   Topic: /servo/state   | Type: cquick_servo_interfaces/msg/ServoState   | Count: 103
```

- `Count: 3` 对应三条指令，与录屏里按下的三次一一对得上。
- `Count: 103 / 20.4 s ≈ 5 Hz` **正是设计值**：`/servo/state` 的频率 = `state_rate_hz(5 Hz) × servo_ids 里 ID 的个数`，录制这份 bag 时只接了 ID 1 一颗舵机，所以是 5 Hz。整机 10 颗都接上后同一时长会是约 50 Hz × 20 s，话题与录制方式完全不变。

回放：

```bash
ros2 bag play bag/run1
```

回放后舵机按录制顺序复现三次动作，`/servo/state` 同步更新，可另开终端 `ros2 topic echo /servo/command` 对照三条指令。

---

## 8. 接线与实测电压

```
USB 转 TTL (CH340)          24 路控制板                    舵机
  TXD ───────────────────► RX1                        ┌── 1 号通道
  RXD ◄─────────────────── TXD                        │
  GND ──────────────────── GND ────────────── GND ───┤
                              VIN(+6~8.4V) ───── 电池正┴── 电源正

TTL 模块的 5V 不接，控制板由电池独立供电。
```

- **电池实测 7.87 V**（在 6.0–8.4 V 范围内；中途充过几次电，每次上机前重测）。
- 板子 PC 侧是**独立 UART**（丝印 `RX1` / `TXD` 分开），所以走**交叉接线**（TXD→RX1、RXD→TXD、GND→GND），不是单线半双工并接。
- **udev 固定名**（`/etc/udev/rules.d/99-cquick-servo.rules`）：

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
  SYMLINK+="cquick_servo", MODE="0666", GROUP="dialout"
```

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add     # 注意：默认的 change 动作不会新建符号链接
```

效果：拔插之后 `ttyUSB0` 会漂到 `ttyUSB1`/`ttyUSB2`，但 `/dev/cquick_servo` 永远指向当前那颗。

---

## 9. 动作 yaml

`config/actions/stand_up.yaml`（片段）：

```yaml
# 动作组A：站起与蹲下
# 3 右前膝 / 4 左前膝 / 5 右后膝 / 6 左后膝
# 实测：右侧（3、6）pwm 增大 = 向后收；左侧（4、5）方向相反
# 下蹲：前膝往前，后膝往后；左右两侧镜像，所以 3/6 同向、4/5 同向
name: stand_up
strict: true
steps:
  - name: 预备
    duration_ms: 1500
    joints:
      - {id: 1, pwm: 1500}
      - {id: 2, pwm: 1500}
      - {id: 3, pwm: 1500}
      - {id: 4, pwm: 1500}
      - {id: 5, pwm: 1500}
      - {id: 6, pwm: 1500}
      - {id: 7, pwm: 1500}
      - {id: 8, pwm: 1500}
      - {id: 9, pwm: 1500}
      - {id: 10, pwm: 1500}

  - name: 蹲下
    duration_ms: 2000
    joints:
      - {id: 3, pwm: 1700}
      - {id: 4, pwm: 1300}
      - {id: 5, pwm: 1300}
      - {id: 6, pwm: 1700}

  - name: 蹲住
    duration_ms: 1000
    joints:
      - {id: 3, pwm: 1700}
      - {id: 4, pwm: 1300}
      - {id: 5, pwm: 1300}
      - {id: 6, pwm: 1700}

  - name: 起立
    duration_ms: 1000
    joints:
      - {id: 3, pwm: 1500}
      - {id: 4, pwm: 1500}
      - {id: 5, pwm: 1500}
      - {id: 6, pwm: 1500}
```

`strict: true` 表示任何一个 joint 失败就整组中止，不会带着一条腿跑到下一步。

`config/actions/wave.yaml`（第二条动作组，验证“换动作只改 yaml”）：

```yaml
name: wave
description: 右前腿抬起挥两下
steps:
  - name: 抬腿
    joints:
      - {id: 1, pwm: 1250, duration_ms: 1400}
      - {id: 2, pwm: 1750, duration_ms: 1400}
  - name: 挥动
    joints:
      - {id: 1, pwm: 1700, duration_ms: 1400}
      - {id: 2, pwm: 1300, duration_ms: 1400}
  - name: 挥回
    joints:
      - {id: 1, pwm: 1250, duration_ms: 1400}
      - {id: 2, pwm: 1750, duration_ms: 1400}
  - name: 放下
    joints:
      - {id: 1, pwm: 1500, duration_ms: 1400}
      - {id: 2, pwm: 1500, duration_ms: 1400}
```

规则：

- **一个 step 内的 joint 同时下发**——`for` 循环里 `send_goal_async` 连发不等待，实测 4 个目标在 **9.3 ms 内全部发出**（`SEND` 日志时间戳差）。
- **要先后就拆 step**，step 之间靠 `result` 同步：`_pending` 计数减到 0 才进下一 step，全程没有 `sleep` 等动作完成。
- `duration_ms` 两种写法都支持：`stand_up.yaml` 写在 **step 级**（整步统一个时长），`wave.yaml` 写在 **joint 级**（逐关节覆盖）。回退顺序是 `joint.duration_ms → step.duration_ms → 1000`，最后一层是为了绕开固件忽略 `T<1000` 的坑（见 §10.4）。
- 换动作只改 yaml，**不改 python**——这是“动作写死在代码里 −4 分”要防的。`wave.yaml` 就是第二条动作组，用来证明这一点。

---

## 10. 踩坑记录（四个真实的）

### 10.1 yaml 改对了，跑起来还是旧端口

**现象**：`config/servo_params.yaml` 已经写成 `port: /dev/cquick_servo`，`grep install/...` 也确认编进去了，启动日志却还是 `/dev/ttyUSB0`。

**怀疑**：以为是 build 没更新，`rm -rf build install` 重来一遍，一样。

**怎么验证**：读 launch 文件——

```python
parameters=[params_file, {'port': port}]
```

列表里**后面的元素覆盖前面的**，字典里的 `port` 来自 `DeclareLaunchArgument('port', default_value='/dev/ttyUSB0')`。所以 yaml 根本没机会生效。

**结论**：参数有四层来源，优先级从低到高是「代码默认值 < yaml < launch 的 parameters 字典 < 命令行 `--ros-args -p`」。把 launch 的默认值和 yaml 改成一致才解决。

### 10.2 feedback 从第一帧就等于目标值

**现象**：`send_goal` 打出来的 feedback，`current_pwm` 第一帧就是 1700（=目标），`progress` 却在正常涨。

**怀疑**：插值公式写错。

**怎么验证**：读 `_execute_cb` 的语句顺序——`_send_move()` 先执行，它内部把 `self._last_pwm[id] = target` 写成了 1700；之后 `_feedback_loop` 才读 `start_pwm = self._last_pwm.get(id, center)`，起点=终点。

**结论**：起点必须在 `_send_move` **之前**取，并作为参数传进 feedback 循环。改完之后 feedback 从 1500 平滑涨到 1700。

### 10.3 拔掉 TTL，驱动毫无反应

**现象**：拔掉 USB 转 TTL，驱动不报错、不退出，`/servo/state` 还在照常发。

**怀疑**：串口句柄没检测到断开。

**怎么验证**：在拔掉状态下发一条指令——立刻报 `[Errno 5] Input/output error`。

**结论**：Linux 上已打开的 fd 不会因为设备拔出而失效，`is_open` 仍是 `True`，**只有真正读/写才会炸**。所以加了 `_try_reopen_serial` 定时器：写失败 → 置 `_ser=None` → 每 2 秒重试 → 插回后自动恢复（日志 `串口已重新打开：/dev/cquick_servo @ 115200 8-N-1`）。

### 10.4 动作组里写 800 ms，舵机一动不动

**现象**：单个 Action goal 能动能到位；动作组 `stand_up` 播下去，四颗膝部舵机纹丝不动，日志全是「未到位：目标 1300 实际 1478（差 +188）」，重发一次后 abort。

**怀疑**：先怀疑机械卡死，手掰能动 → 排除；再怀疑 ID 串了，`scan_all.py` 扫出 10 颗全在线 → 排除。

**怎么验证**：在 `_write()` 前加一行 `SEND {frame}` 日志，抓到驱动实际发出去的是 `#003P1300T800!`。

用 pyserial 直发做对比：

| 指令 | 1.2 s 后回读 | 2.5 s 后回读 |
| --- | --- | --- |
| `#003P1300T800!` | `#003P1488!` | 仍 `1488`（**没动**） |
| `#003P1300T1500!` | — | `#003P1315!`（到位） |

**结论**：**固件会静默忽略 `T < 1000` 的移动指令**，不报错、不回显失败，就是不动。同时查出第二个 bug：`_parse_joint` 从 `joint` 取 `duration_ms`，而 yaml 把它写在 `step` 级，所以永远退回默认 800。

修复两处：

1. `_parse_joint(joint, step_duration=0)`：`int(joint.get('duration_ms', 0) or step_duration or 1000)`
2. 驱动入口兜底：`duration_ms = max(duration_ms, 1000)`

---

## 11. 下电三步

```bash
# ① 卸力：开一个终端持续发心跳，然后 Ctrl-C 驱动，finally 里 emergency_unload() 对全部 ID 发 PULK
ros2 topic pub -r 5 /servo/heartbeat std_msgs/msg/Empty "{}"

# ② 断开动力电：拔电池，不是拔 USB
# ③ 关机
sudo shutdown now
```

**每一步的原因**

1. **先卸力**：带电状态下舵机保持扭矩，直接拔电池会让结构在残余扭矩下突然塌下来，可能夹手、也可能把齿轮别坏。PULK 之后舵机是自由状态，结构由人扶着放平。
2. **再断动力电**：动力电（6–8.4 V）和逻辑电（USB 5 V）是两路。先断动力电，控制板还能靠 USB 活着，你能看到日志确认所有舵机都松了；反过来先拔 USB，控制板掉电，舵机状态就查不了了。
3. **最后关机**：`shutdown now` 让系统正常落盘。直接断电可能损坏 SD 卡或者让 bag 文件写坏。

---

## 12. 加分项实现位置

| 加分项 | 分值 | 实现位置 | 证据 |
| --- | --- | --- | --- |
| udev 固定设备名 | +2 | `/etc/udev/rules.d/99-cquick-servo.rules` + yaml 的 `port` | `ls -l /dev/cquick_servo` |
| PRAD 闭环重发 | +3 | `servo_driver.py` → `_verify_and_finish()` | 驱动日志 `回读 1506, 目标 1900, 差 -394 超 ±30 → 重发第 1 次` |
| 安全看门狗 | +3 | `servo_driver.py` → `_watchdog_tick()` / `_heartbeat_cb()` | 停心跳 1 s 内全 PULK |
| 串口掉线自动重连 | +3 | `servo_driver.py` → `_try_reopen_serial()` + `create_timer(2.0, ...)` | 拔插后 `串口已重新打开` |
| pytest 单测 | +2 | `test/test_protocol.py` | `python3 -m pytest test/test_protocol.py -v` → 25 passed |
| 动作组抢占 | +4 | `sequence_player.py` → `_on_preempt()`（第 104 行）+ `self._preempted` 中断判据（第 197 行的 `wait` 条件里） | 播放中发 `/sequence/preempt` → 全组目标被 `cancel_goal_async()`，日志 `抢占：已取消 N 个未完成目标` |

---

## 13. 录屏链接（B 站）

| 内容 | 链接 |
| --- | --- |
| 任务 2 · 正反转 / 取消打断 / param_set / 限位保护 | https://b23.tv/wQrZUjg |
| 任务 2 · bag record / info / play | https://b23.tv/ExGahC6 |
| 任务 3 · 动作组 A（`stand_up`，`repeat:=3`）+ 动作组 B（`wave`） | https://b23.tv/yzDVBbX |
| 任务 4 · 安全下电（PULK → 断动力电 → 关机） | https://b23.tv/0yAbHXd |
