# 电控组 Week3,4 · 舵机控制（ranchao）

本目录是任务 1～4 的提交目录。当前进度：**任务 1、任务 2 完成（截图/代码/bag 见下）；任务 3 代码与数据就绪，整机动作卡在电池供电不足；任务 4 未录**（原因见文末「已知问题」）。

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
| `servo_ids` | `[2, 4]` | 整机总线舵机 ID 白名单。整机 12 个舵机扫描出 1~9 在线，后因总线负载过重多数掉线，最终稳定在线的是 2 和 4，动作组按这两个做 |
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

电池电压：**实测过，但当时没拍照**。事后想补拍，万用表借不到（用不了），就一直没补上，如实记录不做假数据。标称 2S 7.4V，满电 8.4V。

## 9. 动作 yaml

任务 3 的代码与数据已就绪，位于 `src/cquick_servo/cquick_servo/action_player.py` 和 `src/cquick_servo/config/actions/`：

- `action_player.py`：播放器节点。读 yaml → 展开成 goal 序列 → **等上一个 result 回来才发下一个**（不用 sleep 猜时间，result 是服务端给的准确同步信号）；
- `action_group_a.yaml`（「抬-收」）与 `action_group_b.yaml`（「交替摆」）：两套动作，节奏/角度全不同，证明播放器认的是数据不是写死的动作；
- 播放器日志带时间戳，能看到每个 goal 的 result 时序。

播放链路已验证：播放器 → Action → 串口指令全部发出、result 全部回来（见调试记录 11-22）。单舵机空载可动作，**整机双舵机带负载不动作，卡在电池供电不足，来不及更换**（见文末已知问题 2）。

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

待补 B 站链接。已拍摄：正反转回中位、取消打断（goal accepted → 5 秒取消 → status CANCELED → 舵机当场停住并回中位）、改参数后行为变化（六步演示，含「不重启 param set 当场拒 goal」）。bag 回放段见第 7 节说明。

---

## 已知问题（重要）

**1. 整机总线负载过重，多数舵机掉线（11-22）**

整机 12 个舵机全挂一条总线时：PRAD 读回大量丢包（重试 3 次也拿不到）、部分舵机陆续掉线（扫描从 1~9 在线掉到只剩 2 和 4）、位置指令发出但舵机不执行。把总线上其他舵机物理断开、只留 2 和 4 串联后，PRAD 恢复正常回显（1505/1209）。

**2. 电池供电不足，整机动作未完成（11-22）**

用只剩 2、4 两个舵机的最小总线做了端到端验证：

- 单舵机空载：`send_goal id=4 pwm=1200` → 动作正常；
- 双舵机带腿上负载：`id=2 pwm=1750` → result 回来 `未到位，差 245（目标 1750，实际 1505）`，舵机纹丝不动；
- 手拧关节：**拧不动** —— 舵机有力（能锁位、能回话），但电机转动需要的大电流电池供不上，电压瞬间被拉垮。

结论：**电池带不动整机负载，来不及更换**，动作组「连跑三遍」的实物演示未能完成。播放器、两套 yaml、result 同步逻辑均已验证到指令层（服务端日志确认每条串口指令发出）。

**3. 硬件型号澄清（此前记录的「协议不匹配」已解决）**

之前误判为「PWM 板 + PWM 舵机，与考核要求的总线舵机协议不匹配」。11-22 经学长确认并实测：**这就是总线舵机**，`PID`/`PRAD`/`PULR`/`PULK` 全部有响应（如 `#004PRAD!` → `#004P1504!`），波特率 115200。此前的「无响应」实为接线和虚拟机 USB 转发问题（见调试记录 11-22 各条）。

**4. 受已知问题影响未完成的材料**

- 任务 3：动作组连跑三遍实物录屏、换 yaml 实物演示（代码/数据已提交）
- 任务 4：下电流程录屏

**已向带队学长反馈电池与供电问题。**
