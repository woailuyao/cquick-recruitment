# 电控组第 3、4 周考核 - lizhiqian

## 1. 架构图
两个节点：`servo_driver`（动作服务 + 串口）和 `sequence_player`（读 yaml 播动作组）。

```mermaid
flowchart LR
  PLAYER[sequence_player.py<br/>读 yaml] -->|Action goal| DRV[servo_driver.py]
  CLI[ros2 action send_goal] -->|goal/feedback/result| DRV
  DRV -->|直接写串口 ASCII| SERIAL[CH340 /dev/ttyUSB0]
  SERIAL -->|115200 半双工总线| BOARD[舵机控制板]
  BOARD -->|PULR/Pxxx/PRAD| SERVO[1~10 号总线舵机]
  DRV -->|ServoCommand| STATE[/servo/state]
  CMD[/servo/command] -->|仅 bag 回放时订阅| DRV
```

- Action 名：`/move_servo`，类型 `cquick_servo_interfaces/action/MoveServo`
- 话题：`/servo/command`（bag 回放）、`/servo/state`（状态发布），类型都是 `ServoCommand`

## 2. 参数表
| 参数 | 默认值 | 为什么是这个默认值 |
| --- | --- | --- |
| port | /dev/ttyUSB0 | 本机只有一个 CH340，枚举名固定为 ttyUSB0 |
| baudrate | 115200 | 总线舵机固定通信波特率，改了就收不到 |
| pwm_min | 900 | 实测低于 900 开始顶机械限位/抖动，留余量 |
| pwm_max | 2100 | 实测高于 2100 开始顶机械限位/抖动，留余量 |
| default_duration_ms | 800 | 太短会猛跳撞结构，太长动作拖沓，800 折中 |
| servo_ids | [1,2,3,4,5,6,7,8,9,10] | ID 扫描 001~010 全部在线，与实际一致 |
| startup_reset | true | 上电先统一回中立 1500，防止带电乱位 |

## 3. 接口契约
`MoveServo` 三段字段：

| 阶段 | 字段 | 含义 | 什么时候发 |
| --- | --- | --- | --- |
| Goal | id | 舵机号 | 客户端 send_goal 时填 |
| Goal | pwm | 目标位置（900~2100） | 客户端 send_goal 时填 |
| Goal | duration_ms | 运动时长，0 用 default | 客户端 send_goal 时填 |
| Feedback | current_pwm | 本次目标 PWM | 执行中每 200ms 发一次 |
| Feedback | progress | 0~1 执行进度 | 执行中每 200ms 发一次 |
| Result | success | 是否到位（误差≤30） | 舵机走完后返回 |
| Result | message | 结果描述/误差 | 舵机走完后返回 |
| Result | final_pwm | PRAD 读回的实际位置 | 舵机走完后返回 |

流程：goal 到达 → `goal_callback` 校验 id 在 servo_ids、pwm 在 900~2100 → ACCEPT/REJECT → `execute_callback` 直接发 `PULR` + `P<pwm>T<duration>` → 每 200ms 发 feedback → 等 duration 后发 `PRAD` 读回 → `succeed` 返回 result。

## 4. 一条启动命令
```bash
ros2 launch cquick_servo servo.launch.py port:=$(ls /dev/ttyUSB* | head -n1)
```
真实输出（关键几行）：
```text
[servo_driver] 串口已打开: /dev/ttyUSB0 @ 115200
[servo_driver] 发送: #001PULR!
[servo_driver] 发送: #001P1500T0500!
...
[servo_driver] 启动回中位完成
```

## 5. 安全区间怎么试出来的
从 1500 中立位向两端逐步试探：1500 → 1600 → 1700 → 1800 → 2100，再向下 1500 → 1400 → 1300 → 900。低于 900、高于 2100 时舵机开始顶机械限位/剧烈抖动，所以软件限制设为 900~2100。协议电气极限是 500~2500，不直接使用，防止撞坏结构。

## 6. 运行时改参数演示
```bash
ros2 param get /servo_driver pwm_max
ros2 param set /servo_driver pwm_max 1600
ros2 action send_goal /move_servo cquick_servo_interfaces/action/MoveServo "{id: 1, pwm: 1700, duration_ms: 1000}" --feedback
ros2 param set /servo_driver pwm_max 100000
ros2 param set /servo_driver pwm_max 2100
```
真实输出：
```text
Integer value is: 2100
Set parameter successful
Goal was rejected.
Setting parameter failed: pwm_max 必须在 500~2500
Set parameter successful
```

## 7. bag 说明
- 文件：`bag/run1`
- 话题：`/servo/command` + `/servo/state`
- 消息：16 条，时长 29.4 秒
- 回放：`ros2 bag play bag/run1`

`ros2 bag info` 真实输出：
```text
Bag size:          25.4 KiB
Duration:          29.416560566s
Messages:          16
Topic: /servo/command | Count: 8
Topic: /servo/state | Count: 8
```
回放后 `/servo/command` 重新下发到 `servo_driver`，动作按录制时复现。

## 8. 接线说明
- CH340 `TX` → 舵机串行总线信号线
- CH340 `GND` → 舵机 `GND`（必须共地）
- 舵机 `VCC`（红）→ 电池正极，不接 CH340
- 电池实测：初次 7.77V，充满后 8.21V

## 9. 动作 yaml
动作是数据，不是代码。播放器 `sequence_player.py` 读 `config/actions/*.yaml` 展开成一串 goal。

`all_up.yaml` 片段（10 舵机同时抬，分两段）：
```yaml
name: all_up
steps:
  - commands:              # step1：10 个舵机同时下发到 1700
      - {id: 1, pwm: 1700, duration_ms: 800}
      - {id: 2, pwm: 1700, duration_ms: 800}
      # ... id 3~10 同样
  - commands:              # step2：到位后再同时抬到 1800
      - {id: 1, pwm: 1800, duration_ms: 1200}
      # ... id 2~10 同样
```
- step 内：同一 step 的 command **同时下发**（并行）。
- step 间：上一个 step 所有 goal 都收到 result 才进下一步（串行），用 result 当同步信号，不用 sleep 猜时间。
- `duration_ms`：小步先给 800ms、大步给 1200ms，避免给太小猛跳撞结构。
- 配套 `all_down.yaml`：step1 全部回 1700，step2 全部回中立 1500。

## 10. 踩坑记录
见 `调试记录.md`（含任务三 10 舵机并发抢串口卡死、时间/位置四位补零等）。

## 11. 下电三步 + 原因
1. 停 ROS（`Ctrl+C` 停 launch）：避免还有 action 在下发、占着串口。
2. 发 `PULK` 全部卸力：让舵机卸掉力矩，防止断电瞬间带负载卡住、下次上电回弹。
3. 断电池：最后断主电源。

## 12. 录屏链接
- 正反转：https://b23.tv/uGIi3OL
- 取消打断：https://b23.tv/MnSxKXR
- 改参数生效：https://b23.tv/V6jm6fg
- bag 回放：https://b23.tv/FR7sdOR
- 任务 3 整机与动作组：https://b23.tv/th0vzOn
- 任务 4 下电：https://b23.tv/RywTi8o