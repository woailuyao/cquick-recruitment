# CQuick 电控组 · 第 3、4 周考核 · 王煊赫（wangxuanhe）

任务 1 点亮舵机 → 任务 2 ROS2 接管（自定义 msg + Action + 全参数化 + bag）→ 任务 3 动作组（yaml 驱动）→ 任务 4 安全下电。

## 0. 交付清单

| 内容 | 位置 | 状态 |
|---|---|---|
| 任务 1 截图 4 张 | `01_串口设备.png` / `01_电池电压.png` / `01_PID回显.png` / `01_中立位标定.png` | 已交 |
| 任务 2 截图 8 张 | `02_*.png` | 已交 |
| bag 数据 | `bag/run1/`（`metadata.yaml` + `run1_0.db3`，25.4 KiB） | 已交 |
| 两个 ROS2 包 | `src/cquick_servo_interfaces/`、`src/cquick_servo/` | 已交 |
| 动作用 yaml | `src/cquick_servo/config/actions/`（`stand_up.yaml` / `wave.yaml` / `desk_test.yaml`） | 已交 |
| 加分项材料 | `加分项/`（udev 规则 + 验证脚本 + `04_pytest.png`） | 部分交 |
| 调试记录 | `调试记录.md` | 已交 |
| 录屏 | 见第 16 节 | 任务 1、2 已传 |
| 任务 3 截图 4 张 | `03_*` | 待补（TF 卡损坏，见第 17 节） |

`build/`、`install/`、`log/` 已进 `.gitignore`，仓库里只有源码和截图。

## 1. 架构

```mermaid
graph LR
    CLI[ros2 action send_goal<br/>move_client / sequence_player] -->|Goal| AS[servo_driver<br/>ActionServer: /move_servo]
    AS -->|校验 id / pwm| GC{goal_callback}
    AS -->|拼 ASCII| SR[send_raw<br/>write + flush]
    SR --> TTL[USB-TTL 115200 8-N-1]
    TTL --> BOARD[24 路控制板]
    BOARD --> SERVO[总线舵机]
    SERVO -->|#xxxPRAD! 回显| AS
    AS -->|Feedback 5 Hz| CLI
    AS -->|Result| CLI
    PUB[/servo/command<br/>ServoCommand/] --> AS
    AS --> STATE[/servo/state<br/>ServoState alive/]
```

| 项 | 值 |
|---|---|
| 接口包（ament_cmake） | `cquick_servo_interfaces`：`msg/ServoCommand`、`msg/ServoState`、`action/MoveServo` |
| 节点包（ament_python） | `cquick_servo`：`servo_driver` / `move_client` / `sequence_player` / `scan_ids` |
| Action | `/move_servo` |
| 调试话题 | `/servo/command`（bag 回放用，与 Action 共用同一个 `send_raw`） |
| 心跳话题 | `/servo/state`（0.2 s 一条，看门狗依据） |
| 服务 | `/servo_driver/torque_off`、`/servo_driver/watchdog_reset`、`/servo_driver/watchdog_trip`（均 `std_srvs/Trigger`） |

一次 goal 的时序：

```mermaid
sequenceDiagram
    participant C as 客户端
    participant D as servo_driver
    participant S as 串口/控制板
    C->>D: send_goal(id, pwm, duration_ms)
    D->>D: goal_callback 实时读参数校验 id 与 pwm
    alt 越界
        D-->>C: Goal was rejected
    else 通过
        D-->>C: Goal accepted（同 id 旧 goal 先取消并打 preempted 标记）
        D->>S: #xxxPULR!
        D->>S: #xxxP{pwm}T{dur}!
        loop 每 0.2 s
            D->>S: #xxxPRAD!
            S-->>D: #xxxPRAD{pwm}!
            D-->>C: feedback(current_pwm, progress)
        end
        D->>S: #xxxPRAD!（最终比对，差 >30 重发一次）
        D-->>C: result(success, final_pwm, message)
    end
```

## 2. 参数表

| 参数 | 默认值 | 为什么是这个默认值 |
|---|---|---|
| `port` | `/dev/cq_servo` | udev 规则绑定的固定名，换 USB 口、换机器都不用改代码 |
| `baudrate` | `115200` | 控制板固定 115200，改错只会收到乱码 |
| `pwm_min` | `900` | 结构实测下限，实测 500 直接被越界拒收 |
| `pwm_max` | `2100` | 结构实测上限，实测 2200 会被拒收，2100 时舵臂到位不憋劲 |
| `default_duration_ms` | `800` | 客户端没给 duration 时的兜底，0.8 s 走完全程不猛跳 |
| `servo_ids` | `[1,2,3,4,5,6]` | 实际舵机清单，用于拒绝不存在的 ID |
| `startup_reset` | `true` | 上电全体回 1500，避免带上次姿态启动 |
| `speed_pwm_per_s` | `600` | 抢占接续时按此限速，新 goal 不猛冲 |
| `watchdog_timeout_s` | `1.0` | 心跳或执行停摆超过 1 s 就卸力锁死 |
| `prad_retry` | `1` | 未到位自动重发一次，够用且不会反复撞结构 |

代码里不出现上述任何字面量：`servo_driver.py` 全部走 `declare_parameter`，判断处全部实时 `get_parameter().value`。

## 3. 接口契约（MoveServo）

| 段 | 字段 | 含义 |
|---|---|---|
| Goal | `uint8 id` / `uint16 pwm` / `uint16 duration_ms` | 哪颗舵机、走到哪个 PWM、计划耗时（=0 时用 `default_duration_ms`） |
| Result | `bool success` / `uint16 final_pwm` / `string message` | 是否到位（PRAD 与目标差 ≤30）、最后一次读回值、原因 |
| Feedback | `uint16 current_pwm` / `float32 progress` | PRAD 实测位置、进度 0~1，动作期间 5 Hz 发布 |

message 取值：`success`（到位）/ `already in place`（已在目标位）/ `canceled`（被取消）/ `preempted`（被新 goal 抢占）/ `Not in place, target:…, actual:…, diff:…`（未到位并写出差多少）/ `serial read fail`（串口读不到）。

发送时机：Feedback 在 `_run_motion` 循环每 0.2 s 一次；Result 在动作走完、被判取消、被抢占后各返回一次。

## 4. 启动

一条命令：

```bash
ros2 launch cquick_servo servo.launch.py
```

真实输出（时间与 PID 随机器变化）：

```text
[INFO] [launch]: All log files can be found below /home/evan/.ros/log/2026-11-19-11-04-22-381507-evan-VirtualBox-16211
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [servo_driver-1]: process started with pid [16212]
[servo_driver-1] [INFO] [1794836597.674465583] [servo_driver]: Serial open /dev/cq_servo @115200
[servo_driver-1] [INFO] [1794836597.714413268] [servo_driver]: Startup reset: all servos go to 1500
[servo_driver-1] [INFO] [1794836597.721046304] [servo_driver]: TX: #001PULR!
[servo_driver-1] [INFO] [1794836597.830999439] [servo_driver]: TX: #001P1500T800!
```

launch 参数：`port:=/dev/ttyUSB1` 覆盖端口（用 `ParameterValue(..., value_type=str)`，避免 `/dev/ttyUSB1` 被当数字解析）；`demo:=true` 顺带起 `move_client`，靠 `IfCondition` 条件启动。

驱动节点必须跑在 `MultiThreadedExecutor` 里，`ActionServer` 挂 `ReentrantCallbackGroup`；否则 `execute_cb` 里的 sleep 会把 `cancel_callback` 挡住（见第 11 节踩坑 2）。

## 5. 安全区间与怎么试出来的

电气范围是 500–2500，装结构后不能直接用。做法：从 1500 起每次 ±50 递进，听到「嗡嗡」或结构开始顶住就退回 30–50，取到的值写进参数。

实测 `pwm_min = 900`、`pwm_max = 2100`。发 `pwm=500` / `2500` / `2200` 都被 `goal_callback` 判越界拒收（`02_限位保护.png`）；发 1000 / 2000 正常动作，到位误差 ≤30。

## 6. 运行时改参数（当帧生效）

```text
① ros2 param get /servo_driver pwm_max            → 2100
② send_goal pwm=1700                              → success
③ ros2 param set /servo_driver pwm_max 1600       → successful
④ send_goal pwm=1700（不重启）                     → 被拒，日志写 out of range 900~1600
⑤ ros2 param set /servo_driver pwm_max 100000     → 被拒：pwm range limit 500~2500
⑥ 改回 2100 → 再发 1700                            → success
```

真实终端输出（`02_param_set现场.png`，同一终端连跑，未重启节点）：

```text
$ ros2 param get /servo_driver pwm_max
Integer value is: 2100

$ ros2 action send_goal /move_servo ... "{id: 1, pwm: 1700, duration_ms: 1000}"
Result:
  success: true
  final_pwm: 1705
  message: success
Goal finished with status: SUCCEEDED

$ ros2 param set /servo_driver pwm_max 1600
Set parameter successful

$ ros2 action send_goal /move_servo ... "{id: 1, pwm: 1700, duration_ms: 1000}"
Goal was rejected.

$ ros2 param set /servo_driver pwm_max 100000
Setting parameter failed: pwm range limit 500~2500

$ ros2 param set /servo_driver pwm_max 2100
Set parameter successful
```

实现要点：`add_on_set_parameters_callback` 拒绝非法值；`goal_cb` 与话题回调每次都实时读参数，不缓存副本，所以第 ④ 步立刻生效。

## 7. bag

录制（只录两个话题，不用 `-a`）：

```bash
ros2 bag record /servo/command /servo/state -o run1
```

必须等日志出现 `All requested topics are subscribed. Stopping discovery...` 再发指令，否则 `/servo/command` 一条都录不进去。

`ros2 bag info run1`：

```text
Files:             run1_0.db3
Bag size:          25.4 KiB
Storage id:        sqlite3
Duration:          14.001964105s
Messages:          73
Topic information: Topic: /servo/command | Type: cquick_servo_interfaces/msg/ServoCommand | Count: 2
                   Topic: /servo/state   | Type: cquick_servo_interfaces/msg/ServoState   | Count: 71
```

回放：

```bash
cd ~/servo_ws && source install/setup.bash
ros2 bag play run1
```

结果：驱动订阅 `/servo/command`，与 Action 共用同一个 `send_raw`，回放后舵机按包里的 `pwm=1200 → 1800` 复现两次动作，`/servo/state` 同步刷新。时长 14 s、体积 25.4 KiB，均在 30 s / 10 MB 限制内。

## 8. 接线

| 线 | 含义 | 接到 |
|---|---|---|
| 橙 / 白 | 信号 | 控制板通道信号脚 |
| 红 | 正极 5–8.4 V | 电池正（禁止 12 V） |
| 棕 / 黑 | GND | 电池负 |

板卡、控制板、电池负极必须共地。电池实测 8.0 V（`01_电池电压.png`，SAYINENG 1500 mAh 4S，万用表 UT33D+）。

## 9. 动作 yaml 与动作组

`wave.yaml`（节选）：

```yaml
name: wave
default_duration_ms: 600
steps:
  - name: 抬肩并同时翻腕（并行）
    joints:
      - {id: 1, pwm: 1720, duration_ms: 800}
      - {id: 2, pwm: 1650, duration_ms: 800}
  - name: 摆外 1
    joints:
      - {id: 2, pwm: 1780, duration_ms: 400}
  - name: 摆内 1
    joints:
      - {id: 2, pwm: 1380, duration_ms: 400}
```

- 同一 step 内多个关节 = 同时下发（并行）；
- step 与 step 之间 = 等上一步所有 result 回齐才发下一步，`sequence_player` 只用 result 当同步信号，不用 `sleep` 猜时间；
- 要「有先后」就拆成两个 step，不靠调 `duration_ms` 硬凑；
- `duration_ms` 缺省取 yaml 的 `default_duration_ms`，单关节可单独覆盖。

```bash
ros2 run cquick_servo sequence_player --ros-args -p actions:=stand_up.yaml -p repeat:=3
# 现场换一套没见过的动作：不重编译，直接指到别的目录
ros2 run cquick_servo sequence_player --ros-args -p actions:=demo.yaml -p actions_dir:=/tmp
```

## 10. 取消打断（考核 2.3）

取消走 Action 的 `CancelGoal`，不是 Ctrl+C 关客户端。

```bash
ros2 launch cquick_servo servo.launch.py            # 终端 A
python3 demo_cancel.py                              # 终端 B，自带取消
```

服务端在 `_run_motion` 每轮循环第一句查 `goal_handle.is_cancel_requested`，命中后：

| 步 | 动作 | 指令 |
|---|---|---|
| ① | 停原动作，回中位 | `#xxxP1500T300!` |
| ② | 等 0.35 s 停稳后卸力 | `#xxxPULK!` |
| ③ | 收尾标记 | `goal_handle.canceled()`，`success=False`，`message=canceled` |

合格输出：

```text
取消响应: return_code=0 goals_canceling=1
==============================================
result.success   = False
result.message   = canceled
result.final_pwm = 1502
==============================================
```

`CancelGoal` 返回码：0 接受 / 1 已在取消中 / 2 目标不存在 / 3 目标已结束 / 4 不支持取消。

## 11. 踩坑记录（详细版见 `调试记录.md`）

1. **`ros2 param set` 返回成功但行为没变** —— 现象：改 `pwm_max` 后同一个 goal 仍通过；怀疑：`__init__` 缓存了副本；验证：在 `goal_cb` 打印 `get_parameter().value`；结论：改成每帧实时读。
2. **取消一直返回 `return_code=3`、message 为空** —— 现象：客户端 0.4 s 就发取消，服务端仍跑到 progress 0.99，取消请求排队到动作结束才回 3；怀疑：单线程 executor 被 `execute_cb` 里的 sleep 占死；验证：客户端确实卡在 `spin_until_future_complete(cancel_future)` 直到动作跑完；结论：换 `MultiThreadedExecutor` + `ReentrantCallbackGroup`，并把取消判断挪到循环第一句，取消立即生效（停 → 回中位 → 卸力 → `canceled`）。
3. **feedback 只有 2~3 Hz** —— 现象：1 s 的动作只出 2~3 条；怀疑：`read_prad` 固定 `sleep(0.15)` 加串口 `timeout=0.5`；验证：打印相邻 feedback 时间差 ≈0.35 s；结论：串口 `timeout=0.02`、`read_prad` 轮询到 `!` 即返回，按 0.2 s 节拍扣掉读取耗时补齐，稳定 5 Hz。
4. **bag 里只有 `/servo/state`，没有 `/servo/command`** —— 现象：`bag info` 显示 command 0 条；怀疑：发指令时订阅还没建好；验证：等到 `All requested topics are subscribed` 再发；结论：录包必须等订阅完成。
5. **`ros2 bag play` 报 `No storage id specified`** —— 现象：info 与 play 都失败；怀疑：缺存储插件；验证：`ros2 bag info` 同样报错；结论：`sudo apt install ros-humble-rosbag2-storage-default-plugins`（不是 sqlite3 那个包名）。
6. **舵机发指令不动** —— 现象：TX 正常但舵机不动；怀疑：忘了 `PULR` 或 PWM 越界；验证：补 `#001PULR!` 后动；发 500/2500 日志直接 `out of range`；结论：上电默认卸力，动作前必须 `PULR`；安全区间按 900~2100 判。

## 12. 安全下电三步

| 步骤 | 命令 / 动作 | 为什么是这个顺序 |
|---|---|---|
| ① 卸力 | `ros2 service call /servo_driver/torque_off std_srvs/srv/Trigger` | 断电前还顶着力矩，直接断电极易堵转发热、烧驱动 |
| ② 断动力电 | 拔电池插头 | 先断动力，避免控制板在电压跌落时乱发指令 |
| ③ 关板卡 | `sudo shutdown now` | 直接拔板卡电源会损坏文件系统，下次开不了机 |

四条红线：禁止带电插拔舵机线（浪涌烧驱动或通道）；禁止反接电源（上电前量电压、看极性）；电池鼓包立即停用并交给队长，充电必须有人看管；禁止让舵机硬顶结构，持续「嗡嗡」就是堵转，立刻回中位。

## 13. 加分项实现

| 项 | 实现位置 |
|---|---|
| udev 固定设备名 | `加分项/99-cquick-servo.rules`，`port` 默认 `/dev/cq_servo` |
| PRAD 闭环重发 | `execute_cb` 末段读 PRAD，差 >30 且 `prad_retry>0` 重发一次并打 WARN |
| 安全看门狗 | `watchdog_cb`：心跳或执行停摆 >1 s → 全体 `PULK` + ERROR + `servo_lock`，只能 `watchdog_reset` 人工解除 |
| 动作组抢占 | `goal_cb` 取消同 id 旧 goal 并打 `preempted`，旧 goal 不 PULK；新 goal 用 `smooth_duration_ms` 从当前位置接上 |
| 串口掉线重连 | `link_check_cb` 2 s 巡检 + `send_raw` 捕获异常，掉线只 WARN，重连后不自动重发旧动作 |
| pytest 单测 | `src/cquick_servo/test/test_servo_protocol.py`（纯函数，不接硬件可跑），`colcon test` 绿 |

验证脚本 `bash 加分项/verify_bonus.sh`，说明见 `加分项/README.md`。

## 14. 竞态与限位补充

- 同一颗舵机收到新 goal：旧 goal 被 `cancel` 并置 `_preempt_flags`，旧 goal 返回 `message=preempted`（不卸力、不回中位），新 goal 按 `speed_pwm_per_s` 从当前位置接着走。
- 看门狗锁死后 `send_raw` 全部拒发，`/servo/state` 的 `alive` 变 `false`，恢复必须走 `watchdog_reset`。
- PRAD 读不到时 `current_pwm=0` 只影响反馈显示，判定以最终一次读回为准；读不到则返回 `serial read fail`。

## 15. 截图与内容对应

| 截图 | 对应内容 |
|---|---|
| `01_串口设备.png` | `ls -l /dev/ttyUSB*`，设备枚举 |
| `01_电池电压.png` | 万用表 8.0 V + 电池实物 |
| `01_PID回显.png` | `#001PID!` 回显（该板只回 `#001P!`，无数字，属硬件限制） |
| `01_中立位标定.png` | 发 1500 后 PRAD 回读 1505 |
| `02_接口编译.png` | `ros2 interface show` 两条接口 |
| `02_launch一键启动.png` | launch 日志 + `ros2 node list` |
| `02_action_反馈.png` | `send_goal --feedback` 与 result |
| `02_取消生效.png` | `return_code=0` + `message=canceled` |
| `02_限位保护.png` | `pwm=2200` 被拒 |
| `02_param_set现场.png` | 六步参数演示 |
| `02_bag_info.png` | 录包 + pub + `bag info` 三窗口 |
| `02_参数自检.png` | 非法参数被拒 |

## 16. 录屏（B 站）

| 内容 | 链接 |
|---|---|
| 任务 1 · 点亮一个舵机（PID / PULR / PRAD / 标定 1500） | https://b23.tv/EkFvWXR |
| 任务 2 · 正反转、取消打断、改参数生效、bag 回放 | https://b23.tv/hW2J0CK |
| 任务 3 · 整机逐个动作 + 动作组连跑三遍 | 待补 |
| 任务 4 · 安全下电流程（PULK → 拔电池 → 关机） | 待补 |

## 17. 缺口与原因（如实记录）

- 任务 3 整机装配中断：地瓜派 TF 卡损坏（换 5 V/3 A 供电仍不亮，读卡器读卡报错），`03_*` 四张实物图与任务 3 录屏因此未完成；代码侧（抢占、yaml 动作组、`sequence_player` 用 result 同步）已全部实现，换卡后可直接上机验证。
- 该控制板不支持 `#001PID!` 回读 ID 数值，只能以 `#001PRAD!` 位置回显间接证明，已在 `01_PID回显.png` 标注。
- 加分项证据截图已收 `04_pytest.png`，其余五项（udev / PRAD 重发 / 看门狗 / 抢占 / 重连）需上机后按 `verify_bonus.sh` 顺序补拍。
