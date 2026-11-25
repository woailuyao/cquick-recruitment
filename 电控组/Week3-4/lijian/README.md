# lijian · 电控组 Week3,4

## 0. 任务 1 录屏

- 舵机四连发与转动：https://b23.tv/MSgQIdK

## 1. 架构图

```text
[ros2 action send_goal / move_client]
        │ goal
        ▼
[servo_driver]  ── Action Server /move_servo
        │  ├─ goal_callback 校验 id/pwm/duration
        │  ├─ execute_callback 发串口 + 发 feedback + 读 PRAD
        │  └─ cancel_callback 接受取消
        │
        ├── publish /servo/command（bag 录制/回放）
        ├── publish /servo/state（心跳，0.5s）
        │
        ▼ 串口 115200 8-N-1
[USB 转 TTL CH340] → [24 路控制板] → [总线舵机 ID=1]
```

- 包：`cquick_servo_interfaces`（msg/action）、`cquick_servo`（驱动/客户端/launch/config）
- Action：`/move_servo`，类型 `cquick_servo_interfaces/action/MoveServo`
- 话题：`/servo/command`、`/servo/state`，类型 `cquick_servo_interfaces/msg/ServoCommand`

## 2. 参数表

| 参数 | 默认值 | 为什么是这个默认值 |
|---|---|---|
| `port` | `/dev/ttyUSB0` | 实验室 CH340 默认枚举为 ttyUSB0；可用 launch `port:=` 覆盖 |
| `baudrate` | `115200` | 总线舵机协议固定 115200 8-N-1 |
| `pwm_min` | `900` | 任务 2 阶段先用保守默认；任务 3 装配后按结构实测调整 |
| `pwm_max` | `2100` | 同上；电气范围 500~2500，安全区间必须小于电气范围 |
| `default_duration_ms` | `800` | 客户端没给 duration 时用，800ms 平滑不猛跳 |
| `servo_ids` | `[1]` | 当前桌面只接 ID=1 的舵机；装配后按实际 ID 列表改 |
| `startup_reset` | `true` | 启动时先 PULR 再回 1500，避免上电姿态未知 |
| `watchdog_enabled` | `false` | 安全看门狗总开关；演示期间关，加分项现场 `param set` 打开 |
| `watchdog_timeout_s` | `1.0` | 空闲多久触发看门狗，考核要求 1 秒 |

## 3. 接口契约

### `MoveServo.action`

**Goal**
| 字段 | 类型 | 含义 |
|---|---|---|
| `id` | uint8 | 舵机 ID，必须在 `servo_ids` 里 |
| `pwm` | uint16 | 目标 PWM，必须在 `pwm_min~pwm_max` |
| `duration_ms` | uint16 | 从当前位置走到目标用时，0 表示用 `default_duration_ms` |

**Feedback**（≥5Hz，实际 10Hz）
| 字段 | 类型 | 含义 |
|---|---|---|
| `current_pwm` | uint16 | 当前插值 PWM |
| `progress` | float32 | 0~1 进度 |

**Result**
| 字段 | 类型 | 含义 |
|---|---|---|
| `success` | bool | 是否到位（PRAD 差 ≤30） |
| `final_pwm` | uint16 | 最终 PRAD 读回 |
| `message` | string | `ok (diff=..)` / `未到位: ...` / `canceled` / `串口写失败` |

## 4. 一条启动命令

```bash
ros2 launch cquick_servo servo.launch.py
```

输出：

```text
[servo_driver-1] [INFO] ... [servo_driver]: serial opened: /dev/ttyUSB0 @ 115200
[servo_driver-1] [INFO] ... [servo_driver]: startup reset done for ids=[1]
[servo_driver-1] [INFO] ... [servo_driver]: servo_driver ready
```

换端口：

```bash
ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB1
```

## 5. 安全区间

当前 `pwm_min=900`、`pwm_max=2100`，是任务 2 桌面阶段的保守默认值。**任务 3 装配后**，手动转每个关节确认不卡，再逐步试 `pwm_min/pwm_max`，把结构实际能到的范围写进 `servo_params.yaml`。电气范围 500~2500 不能直接用。

## 6. 运行时改参数演示

```text
① ros2 param get /servo_driver pwm_max
   Integer value is: 2100
② 发 pwm=1700 → SUCCEEDED
③ ros2 param set /servo_driver pwm_max 1600
   Set parameters successful
④ 再发 pwm=1700 → Goal rejected，日志 reject: pwm=1700 not in [900,1600]
⑤ ros2 param set /servo_driver pwm_max 100000
   Set parameters failed: pwm_max=100000 超出 500~2500
⑥ ros2 param set /servo_driver pwm_max 2100 → 再发 1700 → SUCCEEDED
```

关键点：`goal_callback` 每次实时 `self.get_parameter("pwm_max").value`，不读 `__init__` 的副本，所以改完不重启就生效。

## 7. bag

```text
$ ros2 bag info ~/servo_ws/bag/run1
Files: run1_0.db3
Bag size: 25.5 KiB
Duration: 23.999868098s
Messages: 52
Topic: /servo/command | Count: 3
Topic: /servo/state   | Count: 49
```

回放：

```bash
ros2 bag play ~/servo_ws/bag/run1
```

回放结果：舵机复现 1700 → 1300 → 1500 三个动作，与录屏一致。录屏链接见第 12 节。

## 8. 接线

| 线色 | 含义 | 接哪 |
|---|---|---|
| 橙/白 | 信号 | 控制板通道信号脚 |
| 红 | 正极 5–8.4V | 电池正 |
| 棕/黑 | GND | 电池负 |

**必须共地**：USB 转 TTL GND、控制板 GND、电池负极连一起。电池实测电压：**7.58v**。

## 9. 动作 yaml

任务 3 补充。播放器读 `config/actions/*.yaml`，一个 step 内多舵机并行，step 之间等 result 再发下一个。

## 10. 踩坑记录

1. **`brltty` 抢占 CH340，`/dev/ttyUSB0` 生成后立刻消失**  
   现象：`lsusb` 有 CH340，但 `/dev/ttyUSB*` 没有。  
   怀疑：`dmesg` 看到 `brltty sets config #1`。  
   验证：`sudo dmesg | grep -iE "ch341|brltty"`。  
   结论：`sudo apt remove brltty`，重插 USB，ttyUSB0 稳定。

2. **VMware 下 CH340 握手超时，`Errno 5` / `probe failed -110`**  
   现象：`/dev/ttyUSB0` 在，但打开报 `Input/output error`。  
   怀疑：VMware USB 控制器兼容性。  
   验证：`dmesg` 出现 `failed to send control message: -110`。  
   结论：切换 VMware USB 控制器模式（2.0 ↔ 3.0），重载 `ch341`，重新连接设备后恢复。

3. **新舵机 ID=000，扫描脚本扫不到**  
   现象：`scan_id.py` 只扫 1~254，扫不到。  
   怀疑：ID 不在扫描范围。  
   验证：手动发 `#000PID!` 回 `b'#000P!'`。  
   结论：扫描范围加 0 和 255；255 是广播响应，回显 ID 仍是 000，不另记。

4. **1505 → 1500 看起来“没动”**  
   现象：发 1500 舵机不动。  
   怀疑：舵机没执行。  
   验证：`#001PRAD!` 读回 1505，本来就在 1500 附近。  
   结论：差 5 个 PWM 单位几乎不转，发 1300/1700 才能看到明显转动。

5. **新终端没 source，`Package 'cquick_servo' not found`**  
   现象：`ros2 launch` 找不到包。  
   怀疑：环境没 source。  
   验证：`ros2 pkg prefix cquick_servo` 报错。  
   结论：每个新终端先 `cd ~/servo_ws && source install/setup.bash`。

6. **bag Duration 超 30 秒**  
   现象：第一次录 32.999s，文档要求 ≤30s。  
   怀疑：开录后等太久才发 goal。  
   验证：`ros2 bag info` 看 Duration。  
   结论：开录后立刻连发三条 goal，发完立刻停录，重录 23.999s。

## 11. 下电三步

1. **先卸力**：所有舵机发 `#xxxPULK!`。原因：断电前舵机还顶着力矩，直接断电极易堵转发热。
2. **再断动力电**：拔电池。原因：先断动力，避免控制板在异常电压下乱发指令。
3. **最后关板卡**：`sudo shutdown now`。原因：直接拔电会损坏文件系统。

## 12. 录屏链接（B 站）

- 任务 1 舵机四连发与转动：https://b23.tv/MSgQIdK
- 任务 2 正反转回中位：https://b23.tv/Bh9zNFL
- 任务 2 取消打断：https://b23.tv/IfOwhjH
- 任务 2 改参数生效：https://b23.tv/6IrrRcU
- 任务 2 bag 回放：https://b23.tv/mY56A3d
- 任务 3 动作组演示：https://b23.tv/W813nP6
- 任务 4 安全下电：https://b23.tv/Jx14vyK

## 13. 任务 3：整机装配与动作组

- **整机**：双足，10 个总线舵机，ID 1~10（1-5 左腿，6-10 右腿），无上半身配重。
- **照片**：`03_整机照片.png`、`03_接线特写.png`
- **ID 扫描**：`03_ID扫描.png`
- **动作组日志**：`03_多舵机动作_关键帧.png`

**演示方式**：整机只有双腿、没有上半身配重，在地面无法稳定站立，因此采用**悬空法**（手扶腰部、双脚离桌）验证动作组：
- 重点验证**多舵机并行下发**、**步与步之间的 result 同步**、**YAML 数据驱动**；
- `config/actions/squat.yaml`：双膝下蹲—起立，10 个舵机一个 step 内并行、三个 step 串行；
- `config/actions/march.yaml`：左右腿交替抬腿，演示单腿独立控制；
- 两套动作连跑三遍结果一致；换 yaml 不需要改任何 Python 代码。

**关节分配**（实测）：

| 舵机 | 位置 | 说明 |
|---|---|---|
| 1 / 6 | 髋（侧摆/旋转） | 保持基准，防腿外撇 |
| 2 / 7 | 大腿前后摆 | 下蹲时前屈 |
| 3 / 8 | 膝关节 | 下蹲时后折 |
| 4 / 9 | 小腿/踝 | 保持基准 |
| 5 / 10 | 脚踝 | 保持基准 |

**悬空 vs 桌面的差异**：悬空无负载，PRAD 读回在 ±30 内，result 为 `ok`；放在桌面时脚被地面顶住、整机重量压在支撑腿上，舵机转不到目标角度，PRAD 差超过 30，result 如实报“未到位”。这说明闭环校验读的是真实位置。

## 14. 任务 4：安全下电

顺序：**PULK 卸力 → 断动力电（拔电池）→ 关板卡（sudo shutdown now）**。

- **先卸力**：断电前舵机还顶着力矩，直接断电极易堵转发热；
- **再断动力电**：避免控制板在异常电压下乱发指令；
- **最后关板卡**：直接拔电会损坏文件系统。

录屏：见第 12 节任务 4 链接。

## 15. 加分项

| 加分项 | 分值 | 实现 | 验证 |
|---|---|---|---|
| udev 固定设备名 | +2 | `/etc/udev/rules.d/99-cq-servo.rules` 把 CH340 固定成 `/dev/cq_servo` | `ls -l /dev/cq_servo` |
| pytest 单测 | +2 | `test/test_command.py` 测 `build_move_cmd()` | `colcon test` 3 项通过 |
| PRAD 闭环重发 | +3 | 未到位自动重发一次，日志打「重发一次」 | 堵住舵机发 goal，日志出现 attempt=2 |
| 安全看门狗 | +3 | `watchdog_enabled` 打开后空闲 5 秒 PULK 锁死，`/servo/recover` 恢复 | 打开参数 → 等 6 秒 → goal 被拒 → 恢复 |
| 动作组抢占 | +4 | 同一 ID 新 goal 打断旧 goal | 长动作中发新 goal，日志「抢占触发」 |
| 串口掉线重连 | +3 | 写失败置空串口，定时器每 2 秒重连 | 拔插 USB，节点不崩 |