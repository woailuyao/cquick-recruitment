# 7. bag 里录了什么、怎么回放

## 录的是什么

没有用 `-a` 全录，只录两个话题：

| 话题               | 内容                                  | 用途         |
| ---------------- | ----------------------------------- | ---------- |
| `/servo/command` | 驱动实际下发的每条指令（id / pwm / duration_ms） | 回放时靠它重放动作  |
| `/servo/state`   | 每条指令执行完后的最终状态                       | 看每步结果对不对得上 |

驱动在 `_send_command()` 里会往这两个话题各发一条，所以录下来的就是**真正发到串口的那批指令**，不是另外造的数据。

## bag info

```
$ ros2 bag info bag/run1
Files:             run1_0.db3
Bag size:          25.4 KiB
Storage id:        sqlite3
Duration:          18.068779574s
Start:             Nov 21 2026 00:16:06.893404105
End:               Nov 21 2026 00:16:24.962183679
Messages:          9
Topic information: Topic: /servo/state | Type: cquick_servo_interfaces/msg/ServoCommand | Count: 6
                   Topic: /servo/command | Type: cquick_servo_interfaces/msg/ServoCommand | Count: 3
```

两个话题都有条数，说明订阅是接上的。体积 25 KB、时长 18 秒，都在要求以内。

## 怎么录的

```bash
# 终端 A：先起驱动
ros2 launch cquick_servo servo.launch.py

# 终端 B：开录（只录两个话题）
ros2 bag record -o bag/run1 /servo/command /servo/state

# 终端 C：跑一组动作
ros2 action send_goal /move_servo cquick_servo_interfaces/action/MoveServo \
  '{command: {id: 1, pwm: 1560, duration_ms: 1200}}'
ros2 action send_goal /move_servo cquick_servo_interfaces/action/MoveServo \
  '{command: {id: 2, pwm: 1440, duration_ms: 1200}}'
ros2 action send_goal /move_servo cquick_servo_interfaces/action/MoveServo \
  '{command: {id: 1, pwm: 1500, duration_ms: 1200}}'

# 回终端 B 按 Ctrl+C 停止录制
```

## 怎么回放

```bash
# 驱动要开着（回放是驱动订阅 /servo/command 后重新下发）
ros2 launch cquick_servo servo.launch.py

# 另一个终端
ros2 bag play bag/run1
```

回放的原理：驱动订阅了 `/servo/command`，收到一条消息就按里面的 id / pwm / duration_ms 走同一个底层发送函数发一次。所以回放走的是和 Action 完全相同的发送路径，不是另写一套。

## 回放结果

回放时驱动日志会打印：

```
[servo_driver]: 收到 /servo/command 回放: id=1 pwm=1560 duration=1200
[servo_driver]: 收到 /servo/command 回放: id=2 pwm=1440 duration=1200
[servo_driver]: 收到 /servo/command 回放: id=1 pwm=1500 duration=1200
```



## 为什么 bag 不能省

录屏只能看，bag 能把当时的每一条指令原样重放。整机出问题时第一个被要看的就是 bag，因为它能证明"当时到底发了什么"，而录屏看不出发的数值。
