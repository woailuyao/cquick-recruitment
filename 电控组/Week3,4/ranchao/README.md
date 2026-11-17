# 电控组 Week3,4 · 舵机控制（ranchao）

本目录是任务 1～4 的提交目录。当前进度：**任务 2 软件侧完成，任务 1/3 受硬件限制未完成**（原因见文末「已知问题」）。

---

## 1. 架构

```
                    ┌─────────────────────────┐
   move_client ────▶│  Action: /move_servo     │
   (客户端)          │  cquick_servo_interfaces │
                    │  /action/MoveServo       │
                    └───────────┬─────────────┘
                                │ goal / feedback / result
                    ┌───────────▼─────────────┐
                    │  servo_driver（节点）     │
                    │  package: cquick_servo   │
                    └───────┬─────────┬───────┘
                            │         │
              /servo/command│         │/servo/state
              (调试/回放入口)│         │(执行期 5Hz 状态)
                            ▼         ▼
                      串口 /dev/ttyUSB0 @115200
                            │
                        舵机控制板 → 舵机
```

| 项 | 值 |
|---|---|
| 接口包 | `cquick_servo_interfaces`（ament_cmake） |
| 节点包 | `cquick_servo`（ament_python） |
| Action 名 | `/move_servo` |
| 订阅话题 | `/servo/command`（`ServoCommand`） |
| 发布话题 | `/servo/state`（`ServoCommand`，执行期 5Hz） |
| 启动 | `ros2 launch cquick_servo servo.launch.py` |

## 2. 参数表

来自 `config/servo_params.yaml`：

| 参数 | 默认值 | 为什么是这个值 |
|---|---|---|
| `port` | `/dev/ttyUSB0` | CH340 在 Ubuntu 下固定枚举为 ttyUSB0；校验必须 `/dev/` 开头 |
| `baudrate` | `115200` | 总线舵机出厂默认波特率 |
| `pwm_min` | `900` | 机械下限。取 500 会顶到舵机内限位，堵转发热，所以留了余量 |
| `pwm_max` | `2100` | 机械上限，同上，对称留余量 |
| `default_duration_ms` | `800` | 一次常规动作的过渡时间，太快会抖，太慢演示不直观 |
| `allowed_baudrates` | `[9600 … 460800]` | 白名单。早期版本是 `if baud == 115200` 硬编码，改成参数后可运行时校验 |
| `servo_ids` | `[1,2,3,4,5,6]` | 6 个关节的 ID 范围 |
| `startup_reset` | `true` | 启动时先 `PULR` 恢复扭力并回中位，避免上电瞬间保持上一次角度 |

## 3. 接口契约（`MoveServo.action`）

```
# Goal
cquick_servo_interfaces/ServoCommand command
  uint8  id            # 舵机 ID
  uint16 pwm           # 目标脉宽，500~2500
  uint16 duration_ms   # 过渡时间，≤60000
---
# Result
bool   success         # 是否确认到位
uint16 final_pwm       # PRAD 读回的实际脉宽
string message         # 失败原因，一句话
---
# Feedback
uint16 current_pwm     # 当前目标脉宽
float32 progress       # 0.0 → 1.0
```

| 字段 | 什么时候发 |
|---|---|
| Goal 校验 | `goal_callback` 里做三重校验（ID 在 `servo_ids` 内、pwm 在 `[pwm_min, pwm_max]`、duration ≤ 60000），不合格直接 REJECT |
| Feedback | `execute_callback` 循环里每 0.2s 发一次（5Hz），`progress = elapsed / duration` |
| Result | 执行结束时一次性发：`PRAD` 读回位置，偏差 ≤30 判成功；串口打不开或读不到回显则 `success=false` 并在 `message` 写清原因 |

## 4. 启动

```bash
source /opt/ros/humble/setup.bash
source ~/cquick_ws/install/setup.bash
ros2 launch cquick_servo servo.launch.py
```

真实输出：

```
[INFO] [servo_driver]: 串口已打开: /dev/ttyUSB0 @ 115200
[INFO] [servo_driver]: 启动初始化已发出 #001PULR!，回中位
[INFO] [servo_driver]: servo_driver 已启动, Action 名称: /move_servo
```

串口打不开时不会崩，会打一条警告后继续启动，等硬件插上自动重连：

```
[WARN] [servo_driver]: 串口打不开: /dev/ttyUSB0: [Errno 2] No such file or directory
[WARN] [servo_driver]: 没接硬件时正常，插上后自动重连（此提示只报一次）
```

## 5. 安全区间

`pwm_min=900` / `pwm_max=2100`，中位 1500。

取值依据：总线舵机标称行程 500~2500，但两侧各留约 400 的余量，防止舵机顶到机械限位后堵转。运行时 `ros2 param set /servo_driver pwm_max 1600` 会立刻生效——参数回调里对 `pwm_min < pwm_max` 和范围都做了校验。

## 6. 运行时改参数

见 `02_param_set现场.png`、`02_参数自检.png`、`02_限位保护.png`。

```
$ ros2 param set /servo_driver pwm_max 1600
Set parameter successful

$ ros2 action send_goal /move_servo ... pwm=1700
Goal 被 REJECT：pwm 超出 [900, 1600]
```

## 7. bag

`bag/run1/` 里是 `metadata.yaml` + `servo_bag_0.db3`（25 KB）。

`ros2 bag info` 输出：

```
Files:             servo_bag_0.db3
Bag size:          25.5 KiB
Storage id:        sqlite3
Duration:          11.76973455s
Messages:          10
Topic: /servo/command | Type: cquick_servo_interfaces/msg/ServoCommand | Count: 6
Topic: /servo/state   | Type: cquick_servo_interfaces/msg/ServoCommand | Count: 4
```

只录了 `/servo/command` 和 `/servo/state` 两个话题（没加 `-a`）。

回放：

```bash
ros2 bag play bag/run1
```

**回放结果：节点能收到 `/servo/command` 并把指令发到串口，但当前舵机不动作**（见文末已知问题 1），所以「舵机重做一遍动作」这一条没验证成功。

## 8. 接线

| 线路 | 接法 |
|---|---|
| CH340 → 控制板 | `TXD`→`RX`，`RXD`→`TX`（交叉），`GND`→`GND`，`5V` 空着 |
| 电池 → 控制板 | 红线→`+`，黑线→`−`（绿色 2pin 螺丝端子），拧紧后手扯不掉 |
| 舵机 → 控制板 | 插编号 `0` 的 3 针座 |

电池电压：**未实测**（实验室没借到万用表），标称 2S 7.4V，满电 8.4V。上电前应实测确认在 5.0~8.4V。

## 9. 动作 yaml

任务 3 的 `config/actions/` 和 `sequence_player.py` **未完成**（硬件未通过验证，见文末）。

## 10. 踩坑记录（详见 `调试记录.md`）

1. brltty 抢占 CH340，`/dev/ttyUSB0` 插上 0.6 秒就消失
2. VMware USB 转接反复 `-110` 超时
3. `/servo/state` 只在 goal 开始时发一条，循环里漏了 `publish`
4. `ros2 bag record` 后立刻 Ctrl+C，`metadata.yaml` 写不完，bag 读不出来
5. `ros2 launch` 给节点注入 `--ros-args`，`move_client` 里 `int('--ros-args')` 崩
6. 波特率硬编码，改参数不生效

## 11. 下电三步

1. **先发 `PULK` 卸力** —— 让舵机不再顶住，避免下电瞬间齿轮受冲击
2. **断电池，再拔信号线** —— 带电插拔会打火，可能击穿 IO
3. **最后拔 USB** —— 保证断电顺序是「功率 → 信号 → 逻辑」

## 12. 录屏

待补（B 站链接）。四段：正反转回中位 / 取消打断 / 改参数后行为变化 / bag 回放。

---

## 已知问题（重要）

**1. 硬件协议不匹配**

考核文档要求的是**总线舵机**（有 ID、能 `PID` 读 ID、能 `PRAD` 读位置、能 `PULR/PULK` 控制扭力）。实际拿到的是**普通 PWM 舵机 + 24 路 PWM 控制板**：

| 考核要求 | 实际硬件 |
|---|---|
| `#001PID!` 读 ID | PWM 舵机没有 ID |
| `#001PRAD!` 读回位置 | 没有位置反馈 |
| `#001PULR!` / `#001PULK!` | 没有扭力控制概念 |

所以节点发出去的总线协议指令，控制板不认，舵机不动作。已尝试过：总线协议、PWM ASCII 协议（`#1P1500T1000`）、厂家二进制帧（`$A090#`）、6 个波特率档位，均无响应。

**2. 受此影响未完成的材料**

- 任务 1：`01_PID回显.png`、`01_中立位标定.png`、`01_电池电压.png`、舵机转动录屏
- 任务 2：四段录屏
- 任务 3：整机拼装、ID 扫描、动作组
- 任务 4：下电录屏

**已向带队学长反馈设备型号问题。**
