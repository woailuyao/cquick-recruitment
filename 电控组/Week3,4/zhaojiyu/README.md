# CQuick 电控组 · 第 3、4 周 · 整机拼装与舵机控制

姓名：赵基煜（zhaojiyu）

---

## 1. 架构图

```
                     ┌─────────────────────────────────────────┐
   ros2 action       │  cquick_servo                           │
   send_goal ───────>│  servo_driver.py  = Action Server       │
   /move_servo       │    ├─ 参数校验（goal_callback）          │
                     │    ├─ 串口收发（send_command / query）   │
                     │    └─ PRAD 读回校验                     │
                     │                                         │
   /servo/command ──>│  （订阅，bag 回放入口）                  │
                     │                                         │
                     │  /servo/state ──> （发布，供监视/录制）  │
                     └────────────────┬────────────────────────┘
                                      │ USB 转 TTL 115200 8-N-1
                                      ▼
                            24 路控制板 ── 白座 ──> 10 个总线舵机
```

| 包 | 构建类型 | 内容 |
|---|---|---|
| `cquick_servo_interfaces` | ament_cmake | `msg/ServoCommand`、`action/MoveServo` |
| `cquick_servo` | ament_python | `servo_driver`、`move_client`、`sequence_player`、launch、config |

| 名字 | 类型 |
|---|---|
| `/move_servo` | Action（`cquick_servo_interfaces/action/MoveServo`） |
| `/servo/command` | 话题（`ServoCommand`），订阅 + 镜像发布，bag 回放入口 |
| `/servo/state` | 话题（`ServoCommand`），当前状态 |

---

## 2. 参数表

| 参数 | 类型 | 值 | 为什么是这个值 |
|---|---|---|---|
| `port` | string | `/dev/ttyUSB0` | 实测设备号；插拔后会变，用 `ls /dev/ttyUSB*` 确认 |
| `baudrate` | int | `115200` | 与舵机总线一致，8-N-1 无流控 |
| `pwm_min` | int | `900` | 结构实测的安全下限（电气下限是 500） |
| `pwm_max` | int | `2100` | 结构实测的安全上限（电气上限是 2500） |
| `default_duration_ms` | int | `1000` | **本机舵机对小于 1000ms 的指令直接忽略**，所以下限就是 1000 |
| `servo_ids` | int[] | `[1,2,3,4,5,6,7,8,9,10]` | 整机共 10 个舵机，扫描确认 ID 为 1~10 |
| `startup_reset` | bool | `false` | 本机构的中位姿态**不是全体 1500**，启动复位会让关节互相挤压 |
| `shutdown_pulk` | bool | `true` | 退出时给所有舵机 `PULK` 卸力，对应任务 4 的第一步 |

---

## 3. 接口契约

**`MoveServo.Goal`**

| 字段 | 类型 | 含义 |
|---|---|---|
| `id` | uint8 | 舵机 ID（本机 1~10） |
| `pwm` | uint16 | 目标 PWM |
| `duration_ms` | uint16 | 走完这段用多久；0 表示用 `default_duration_ms` |

**`MoveServo.Feedback`**（5 Hz）

| 字段 | 含义 |
|---|---|
| `current_pwm` | 当前 PWM（按线性插值估算） |
| `progress` | 0.0 ~ 1.0 |

**`MoveServo.Result`**

| 字段 | 含义 |
|---|---|
| `success` | 是否成功到位 |
| `final_pwm` | PRAD 读回的实际位置 |
| `message` | `ok` / `canceled` / 未到位原因 |

字段是**摊平**的，好处是命令行一行就能拼出来：

```bash
ros2 action send_goal /move_servo cquick_servo_interfaces/action/MoveServo "{id: 1, pwm: 1700, duration_ms: 1000}" --feedback
```

---

## 4. 一条启动命令

```bash
ros2 launch cquick_servo servo.launch.py
```

输出：

```
[servo_driver-1] [INFO] ... 串口打开成功: /dev/ttyUSB0 @ 115200 8-N-1
[servo_driver-1] [INFO] ... servo_driver 就绪，Action: /move_servo
```

换端口：`ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB1`

带客户端：`ros2 launch cquick_servo servo.launch.py demo:=true`

---

## 5. 安全区间与来历

`pwm_min = 900`、`pwm_max = 2100`，**不是**电气极限（500~2500）。

来历：逐个舵机从 1500 往两边加量，直到关节开始顶结构或听到嗡嗡声，回退 100 作为余量。最终整机在 900~2100 内动作不撞、不堵转。

另外实测：舵机存在约 **±100 PWM 的死区**，小于这个量的位移指令会被忽略，所以动作幅度不能太小。

---

## 6. 运行时改参数（当帧生效）

```
① ros2 param get /servo_driver pwm_max      → 2100
② send_goal pwm=1700                        → success: true
③ ros2 param set /servo_driver pwm_max 1600 → Set parameter successful
④ send_goal pwm=1700                        → Goal rejected
   服务端：Goal REJECTED: pwm 1700 超出安全范围 [900, 1600]
⑤ ros2 param set /servo_driver pwm_max 100000
   → Setting parameter failed: pwm_max 必须在 500~2500 之间，收到 100000
⑥ ros2 param set /servo_driver pwm_max 2100 → 成功，再发 1700 → success
```

关键是 ③→④ **中间没有重启节点**，行为当场变了。

---

## 7. bag

录了 `/servo/command` 和 `/servo/state` 两个话题：

```
Files: run1_0.db3
Bag size: 25.5 KiB
Duration: 24.5 s
Messages: 24
Topic: /servo/state   | Count: 21
Topic: /servo/command | Count: 3
```

回放：

```bash
ros2 bag play ~/cq_ws/bag/run1
```

结果：服务端依次打出 3 条 TX，整机按 1700 → 1300 → 1500 复现动作。

---

## 8. 接线

| 从 | 到 | 说明 |
|---|---|---|
| 电脑 USB | USB 转 TTL | CH340 |
| USB-TTL **TXD** | 控制板 **RX1** | 交叉 |
| USB-TTL **RXD** | 控制板 **TX1** | 交叉 |
| USB-TTL **GND** | 控制板 **GND** | 共地，必须 |
| USB-TTL 5V | — | 不接 |
| 电池 | 控制板绿色端子 | 红 +、黑 − |

电池实测：**7.5V**（2S），带载不掉压。
串口参数：**115200 8-N-1**，无流控。

---

## 9. 动作 yaml

一套动作 = 若干 step；step 内多个 move 表达“这一步要哪些关节一起动”，step 之间靠 result 串行。

```yaml
name: stand_up
description: 起立 —— 从下往上逐层抬起，回位时从上往下
steps:
  - name: 第1层（1、4 镜像）
    moves:
      - {id: 1, pwm: 1640, duration_ms: 1000}
      - {id: 4, pwm: 1360, duration_ms: 1000}
  - name: 第2层（3、2 同向）
    moves:
      - {id: 3, pwm: 1635, duration_ms: 1000}
      - {id: 2, pwm: 1635, duration_ms: 1000}
```

播放：

```bash
ros2 run cquick_servo sequence_player --ros-args -p action_file:=.../stand_up.yaml -p repeat:=3
```

**换动作只改 yaml，不改一行 python。**

### 关节映射与极性（实测）

| 层（下→上） | 舵机 | 极性 |
|---|---|---|
| 第 1 层 | 1、4 | **相反**（1640 / 1360） |
| 第 2 层 | 3、2 | 相同（1635） |
| 第 3 层 | 5、6 | 相同（1630） |
| 第 4 层 | 7、8 | 相同（1625） |
| 第 5 层 | 9、10 | **相同**（1380 / 1380） |

极性是逐层实测出来的，原因是左右腿装配朝向不同。

---

## 10. 踩坑记录（详见 `调试记录.md`）

1. **舵机 ID 是 0 不是 1** —— 文档里的 `#001` 只是举例；
2. **`PULK` 不是“临时卸力”而是“失能”** —— `PULR` 唤不回，必须断电重启；
3. **动作组 `duration_ms` 不能低于 1000** —— 900 被忽略、1000 正常；
4. **极性必须逐层实测** —— 第 1 层异值对称、第 5 层同值对称；
5. **控制板同一时刻只能驱动一路动作** —— 并发下发两路都不动，播放器改为逐个下发；
6. **手写的 ament_python 包缺 `setup.cfg`** —— 入口脚本会装到 `bin/` 而不是 `lib/<pkg>/`，`ros2 run` 找不到；
7. **改完 python 必须 `colcon build`** —— 否则跑的还是 install 里的旧副本，`grep` 源码永远查得到改动。

---

## 11. 下电三步

```
① 先卸力 :  Ctrl+C 让节点发 PULK，或手动发 #xxxPULK!
② 断动力 :  拔电池
③ 关板卡 :  sudo shutdown now
```

原因：

- **先卸力**：舵机断电前还在顶力矩，直接断电极易堵转发热；
- **先断动力**：避免控制板在电压异常时乱发指令；
- **最后关板卡**：直接拔电会损坏文件系统，下次开不了机。

---

## 12. 录屏（B 站）

https://www.bilibili.com/video/BV1meyGzCEwG/

合集里包含本考核的全部录屏：舵机转动、正转/反转/回中位、取消打断、改参数后行为变化、`ros2 bag play` 复现动作、整机逐个动作、动作组连跑三遍 + 换 yaml、安全下电。
