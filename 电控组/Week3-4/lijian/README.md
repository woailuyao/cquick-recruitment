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

## 13. 任务 3：多舵机动作组

- **任务 3 录屏**：https://b23.tv/VL75Y8F
- **当前进度**：由于目前未下发 3D 打印结构件，任务 3 的整机装配部分暂时无法完成。
- **软件验证**：动作组的软件数据驱动、并行/串行时序、result 同步，已通过桌面上 4 个平放的舵机（ID: 001, 002, 003, 004）完整验证。连跑三遍一致，换 yaml 正常。

### 动作 yaml 定义示例 (`test_move.yaml`)
```yaml
steps:
  - step_name: "第一步：1号和2号同时正转"
    actions:
      - {id: 1, pwm: 1700, duration_ms: 1000}
      - {id: 2, pwm: 1700, duration_ms: 1000}
  - step_name: "第二步：3号单独转"
    actions:
      - {id: 3, pwm: 1300, duration_ms: 1000}
  - step_name: "第三步：全部回中位"
    actions:
      - {id: 1, pwm: 1500, duration_ms: 1000}
      - {id: 2, pwm: 1500, duration_ms: 1000}
      - {id: 3, pwm: 1510, duration_ms: 1000}

扫描出现 ID=255，属于总线广播地址，**不是实体舵机，实际有效舵机仅 [1,2,3,4]**。