# cquick_servo · 第 3/4 周任务 2 说明

> 两个包：`cquick_servo_interfaces`（纯接口）+ `cquick_servo`（驱动 / 客户端 / launch / 配置）
> 目标：用 Action 让 ROS2 接管总线舵机，带反馈、可取消、参数全外置、能录 bag 回放。

---

## 一、数据流（一句话版）

```
ros2 action send_goal
        │  goal: id / pwm / duration_ms
        ▼
   goal_callback  ── 校验：ID 在不在 servo_ids？pwm 在不在 [pwm_min, pwm_max]？
        │  合法 → ACCEPT；不合法 → REJECT（日志写原因）
        ▼
   execute_callback
        │  ① #xxxPULR!      恢复扭力（不发这条舵机不动）
        │  ② #xxxP<pwm>T<ms>!  下发位置
        │  每 100ms 发一次 feedback（current_pwm / progress）
        │  中途收到 cancel → 回中位、result.message = "canceled"
        │  ③ #xxxPRAD!      读回实际位置
        ▼
   result: success / final_pwm / message（差 > ±30 判未到位）
```

话题 `/servo/command` 走**同一个底层发送函数**，所以 bag 回放能复现真实动作。

---

## 二、舵机协议（实测确认）

| 项目 | 值 |
|---|---|
| 波特率 | **115200**，8-N-1，指令结尾 `\r\n` |
| 舵机 ID | **000**（四个舵机出厂都是 0，一起上总线会冲突） |
| 帧格式 | `#` + **3 位 ID**（补零） + 功能码 + 参数 + `!` |

| 功能 | 指令 | 回包 |
|---|---|---|
| 读 ID | `#000PID!` | `#000P!` |
| 读位置 | `#000PRAD!` | `#000P1493!` |
| 恢复扭力 | `#000PULR!` | 无 |
| 卸力 | `#000PULK!` | 无 |
| 转角度 | `#000P2000T1000!` | 无（用 PRAD 读回确认） |

> 中位 1500，电气范围 500–2500；读回 1492–1508 都算正常。

---

## 三、参数（`config/servo_params.yaml`）

| 参数 | 默认 | 说明 | 校验 |
|---|---|---|---|
| `port` | `/dev/ttyUSB0` | 串口设备 | — |
| `baudrate` | `115200` | 波特率 | 必须在常见值里 |
| `pwm_min` | `900` | 安全下限 | 500–2500，且 < pwm_max |
| `pwm_max` | `2100` | 安全上限 | 500–2500，且 > pwm_min |
| `default_duration_ms` | `800` | goal 没给时长时用 | 1–30000 |
| `servo_ids` | `[1..6]` | 允许的 ID | 非空、1–254 |
| `startup_reset` | `true` | 启动回中位 | — |

> yaml 最外层 key 必须是 **节点名** `servo_driver`，否则参数一个都不加载。

---

## 四、怎么跑

```bash
cd ~/ros2_ws
colcon build --packages-select cquick_servo_interfaces cquick_servo
source install/setup.bash

# 一条命令起全套（换串口就加 port:=）
ros2 launch cquick_servo servo.launch.py
ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB1
ros2 launch cquick_servo servo.launch.py demo:=true demo_pwm:=1800
```

发动作：

```bash
ros2 action send_goal /move_servo cquick_servo_interfaces/action/MoveServo \
  "{id: 0, pwm: 1700, duration_ms: 1000}" --feedback

ros2 run cquick_servo move_client --id 0 --pwm 1700            # 自己写的客户端
ros2 run cquick_servo move_client --id 0 --sweep               # 正转→反转→回中位
```

参数当帧生效：

```bash
ros2 param get /servo_driver pwm_max
ros2 param set /servo_driver pwm_max 1600     # 之后 pwm=1700 的 goal 立刻被拒
ros2 param set /servo_driver pwm_max 100000   # 被拒：超出 500~2500
```

> 这个环境的 `ros2 daemon` 会挂（`RemoteDisconnected`）。碰到就先 `pkill -9 -f _ros2_daemon`；
> 参数调试也可以用 `ros2_ws/test_params.py`，它直接调服务，不依赖 daemon。

bag：

```bash
ros2 bag record -o bag/run1 /servo/command /servo/state   # 别加 -a，只录这两个
ros2 bag info bag/run1
ros2 bag play bag/run1                                    # 舵机会把动作重演一遍
```

---

## 五、文件清单

```
src/cquick_servo_interfaces/
├── msg/ServoCommand.msg          id / pwm / duration_ms
├── action/MoveServo.action       goal / result(success,final_pwm,message) / feedback(current_pwm,progress)
└── CMakeLists.txt                rosidl_generate_interfaces + 三件套

src/cquick_servo/
├── cquick_servo/servo_driver.py  Action Server + 串口 + 参数校验
├── cquick_servo/move_client.py   Action Client（--id/--pwm/--duration/--sweep）
├── launch/servo.launch.py        yaml 参数 + port 覆盖 + demo 开关
├── config/servo_params.yaml      七个参数
└── setup.cfg                     把 entry point 装进 lib/cquick_servo（少了会报 libexec 不存在）
```

---

## 六、今天验证到哪一步

| 项目 | 状态 |
|---|---|
| 两个包 `colcon build` | ✅ |
| `ros2 interface show` 两条 | ✅ |
| 一条 launch 起节点（参数从 yaml 加载） | ✅ |
| `port:=/dev/ttyUSB1` 覆盖 yaml | ✅ |
| 参数校验 5/5（越界 / min>=max / 合法生效） | ✅ |
| 越界 goal REJECT | ✅ |
| bag 录制 + `bag info` | ✅（90 条 / 7 秒） |
| feedback / 取消 / 真机动作 / bag 回放 | ⏳ 需要 Linux 能看到串口（实验室做） |

---

## 七、到实验室要补的

1. 把 USB-TTL 插到地瓜派（或任何 Linux 机），确认 `ls -l /dev/ttyUSB*`
2. 改 `config/servo_params.yaml` 里的 `port` 和 `servo_ids`，重新 `colcon build`
3. 跑 `demo_task2.sh` 那套，补三张图：`02_action_反馈`、`02_取消生效`、动作录屏
4. 重新录一次 bag（要包含真实动作），`ros2 bag play` 验证舵机能复现
5. 任务 3 之前：给每个舵机改 ID（现在全是 000），否则多舵机一起上总线会冲突
