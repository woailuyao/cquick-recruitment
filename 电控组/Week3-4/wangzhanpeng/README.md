# 电控组第 3、4 周考核 · 任务 1~4

姓名：王展鹏（wangzhanpeng）
仓库路径：`电控组/Week3-4/wangzhanpeng/`

## 1. 架构

1. 代码分成两个包：`cquick_servo_interfaces` 只放接口，里面有 `MoveServo.action`、`ServoCommand.msg`、`ServoState.msg`；`cquick_servo` 放驱动、客户端、动作组播放器、launch 和 config。
2. 驱动节点叫 `servo_driver`，对外只提供一个 Action，叫 `/move_servo`。
3. 客户端 `move_client` 和播放器 `sequence_player` 都是通过这个 Action 发 goal。
4. 驱动收到 goal 后，先检查 id、pwm、duration 合不合法，再通过 USB 转 TTL 和 24 路板，把 `#001P1700T1000!` 这样的字符串发给舵机。
5. 动作过程中驱动会读 PRAD，把实际位置放进 feedback；动作结束再读一次，放进 result 返回。
6. 驱动还发布 `/servo/command` 和 `/servo/state` 两个话题。bag 只录这两个；回放时驱动订阅 `/servo/command`，收到后走和 Action 一样的发送函数，所以舵机会把动作再做一遍。

## 2. 参数

| 参数 | 默认值 | 为什么 |
|---|---|---|
| `port` | `/dev/cq_servo` | 用 udev 固定成这个名字，拔插后不会变成 ttyUSB1 |
| `baudrate` | `115200` | 手册要求，8-N-1 |
| `pwm_min` | `900` | 结构安全下限，不是舵机本身的电气下限 |
| `pwm_max` | `2100` | 结构安全上限 |
| `default_duration_ms` | `1000` | 这块板子 T 小于 1000 不执行，所以默认给 1000 |
| `min_duration_ms` | `1000` | 低于它自动抬到 1000，并打一条 WARN |
| `servo_ids` | `[1, 2, 3, 4]` | 现在只控制 1~4 号，写别的 ID 会被拒 |
| `startup_reset` | `true` | 上电先让所有舵机回 1500，避免带错位启动 |
| `center_pwm` | `1500` | 中位 |
| `tolerance` | `30` | PRAD 和目标的允许误差，超过就算没到位 |
| `feedback_hz` | `10.0` | feedback 频率，要求至少 5Hz |
| `max_duration_ms` | `10000` | goal 里 duration 的上限 |

运行时改参数不用重启，比如：

```bash
ros2 param set /servo_driver pwm_max 1600
```

`goal_cb` 每次都会重新读参数，所以改完下一帧就生效。

## 3. 接口

`MoveServo.action` 分三段：

| 段 | 字段 | 什么意思 | 什么时候发 |
|---|---|---|---|
| Goal | `id` | 舵机 ID | 发 goal 的时候 |
| Goal | `pwm` | 目标位置，900~2100 | 发 goal 的时候 |
| Goal | `duration_ms` | 从当前位置走到目标用多久，至少 1000 | 发 goal 的时候 |
| Result | `success` | 有没有到位 | 动作结束、取消或者失败 |
| Result | `final_pwm` | 最后 PRAD 读到的位置 | 同上 |
| Result | `message` | `到位（误差 6）`、`未到位...`、`canceled`、`串口不可用` | 同上 |
| Feedback | `current_pwm` | 当前实际位置，读不到 PRAD 就用插值兜底 | 动作过程中 10Hz |
| Feedback | `progress` | 进度 0~1 | 动作过程中 10Hz |

## 4. 一条命令启动

```bash
source /opt/ros/humble/setup.bash
source ~/ros_real/install/setup.bash
ros2 launch cquick_servo servo.launch.py
```

起来之后能看到：

```text
[servo_driver-1] [INFO] 串口已打开: /dev/cq_servo @ 115200 8-N-1
[servo_driver-1] [INFO] startup_reset: [1, 2, 3, 4] -> pwm=1500 (T1000ms)
[servo_driver-1] [INFO] servo_driver ready: Action=/move_servo
```

换端口用 `port:=/dev/xxx`，想把客户端一起起来用 `demo:=true`。

## 5. 安全区间

我先用 1500 当中位，然后往两边每次加 50 试，试到结构开始顶住、舵机开始嗡嗡响，就往回退 50，最后定在 **900 到 2100**。

这个是我这台结构的安全范围，不是舵机本身的 500 到 2500。所以驱动只认这个区间，超了直接拒绝。

## 6. 实际反馈

正转 1700：

```text
Feedback:
    current_pwm: 1580    progress: 0.308
    current_pwm: 1614    progress: 0.462
    current_pwm: 1643    progress: 0.619
    current_pwm: 1668    progress: 0.925
Result:
    success: true
    final_pwm: 1694
    message: 到位（误差 6）
```

反转 1100 最后 `final_pwm=1107`，误差 7；回中 1500 最后 `final_pwm=1495`，误差 5。

运行时改参数：

```text
ros2 param set /servo_driver pwm_max 1600        → Set parameter successful
发 1700                                          → Goal was rejected.
ros2 param set /servo_driver pwm_max 100000      → Setting parameter failed: pwm_max=100000 超出允许范围 500~2500
ros2 param set /servo_driver pwm_min 2200        → Setting parameter failed: pwm_min(2200) 必须小于 pwm_max(1600)
ros2 param set /servo_driver pwm_max 2100        → Set parameter successful
再发 1700                                        → 到位，实际 1694，误差 6
```

取消：发 `1900 / 3000`，1 秒后 `Ctrl+C`，最后是：

```text
Goal canceled.
Result:
    success: false
    final_pwm: 1528
    message: canceled
Goal finished with status: CANCELED
```

## 7. bag

录的时候只录两个话题：

```bash
ros2 bag record -o bag/run1 /servo/command /servo/state
```

先等两三秒，再发 1700、1100、1500 三条 goal，等 result 回来再 `Ctrl+C`。

`ros2 bag info bag/run1` 的结果：

```text
Bag size:          25.4 KiB
Duration:          6.863867150s
Messages:          27
Topic: /servo/command   Count: 3
Topic: /servo/state     Count: 24
```

回放：

```bash
ros2 bag play bag/run1
```

回放的时候驱动要开着，因为 `/servo/command` 是驱动订阅的。播放后舵机会把 1700、1100、1500 再做一遍，和录屏里的动作一致。

如果第一条不动，就加 `--delay 3`，或者录的时候先多留几秒前导。

## 8. 接线

我是把上面两个舵机（1 号、2 号）的连接线插在 24 路控制板上，剩下的 8 个舵机，左边的首尾相接、右边的首尾相接，保证信号能传到每一个舵机就行。

上电前量过电池，实测 **8.15V**。

USB 转 TTL 用杜邦线接 24 路板的用户端：RX 接 TX、TX 接 RX、GND 接 GND，剩下的不接。

## 9. 动作 yaml

任务三用了两套动作，都是 yaml，换动作只改 yaml，不改 python：

- `one_leg_demo.yaml`：1/2 号成对外扩，3/4 号成对前后摆；
- `pair12_demo.yaml`：只动 1/2 号，做外扩和收回。

因为 1/2 是一对并联连杆，3/4 也是一对，所以同一个 step 里必须成对发、方向相反、`duration_ms` 一样，不然连杆会扭住。2 号还卡了 `pwm_max: 1500`，不能往里面顶。

```yaml
steps:
  - name: extend_out
    actions:
      - {id: 1, pwm: 1600, duration_ms: 1500}
      - {id: 2, pwm: 1400, duration_ms: 1500}

  - name: swing_forward
    actions:
      - {id: 3, pwm: 1400, duration_ms: 1200}
      - {id: 4, pwm: 1600, duration_ms: 1200}
```

跑：

```bash
ros2 run cquick_servo sequence_player --ros-args \
  -p action_file:=$HOME/ros_real/src/cquick_servo/config/actions/one_leg_demo.yaml

ros2 run cquick_servo sequence_player --ros-args \
  -p action_file:=$HOME/ros_real/src/cquick_servo/config/actions/pair12_demo.yaml
```

连跑三遍加 `-p repeat:=3`，改完 yaml 先 `-p dry_run:=true` 验一遍。

## 10. 踩坑记录

**坑 1：虚拟机认不出 CH340。**
现象是 `ls /dev/ttyUSB*` 什么都没有，但 Windows 设备管理器能看到。查了之后发现 Ubuntu 自带一个 `brltty` 的盲文服务，会把 CH340 当成盲文显示器强行占用。`sudo apt remove brltty` 卸掉，再重新拔插 USB 并连进虚拟机，就出现 `/dev/ttyUSB0` 了。

**坑 2：串口发指令没反应。**
在 CuteCom 发 `#001PID!` 毫无反应。用杜邦线把 TXD 和 RXD 短接做回环，能收到字符，说明模块没坏，是接板子的时候 TX/RX 接反了。断电对调后正常。

**坑 3：端口被占用。**
跑 Python 脚本报 `[Errno 16] Device busy`。是之前的 CuteCom 没关彻底，`sudo fuser -k /dev/ttyUSB0` 强杀进程后解决。

**坑 4：duration 给 800，舵机完全不动。**
一开始以为是舵机坏了，后来同一目标分别用 T800、T900、T1000、T1200 试，发现这块板子 T 小于 1000 的指令根本不执行。所以 `duration_ms` 全部改成 1000 以上，驱动里也加了 `min_duration_ms` 兜底。

**坑 5：关掉节点再启动，串口打不开，报 EIO。**
内核日志里是 `usb 1-2.1: failed to send control message: -110`。这是 CH340 在串口关闭的瞬间假死了，不是代码问题。拔插一次 USB 就好，所以演示的时候 launch 起来之后不要重启。

**坑 6：改完代码运行，还是旧的逻辑。**
把客户端改成键盘输入，结果一跑 launch 还是直接自动发指令，看不到提示符。打开文件确认代码确实改了，最后发现是 `build` 和 `install` 里残留了旧缓存。`rm -rf build install log` 清理干净，重新 `colcon build` 后恢复正常。

**坑 7：bag 录成 0 条。**
`ros2 bag record -o bag/run1` 的时候，如果 `bag/run1` 目录已经存在，它会直接报错退出，旧的空 bag 还留在那。必须先 `rm -rf bag/run1` 再录。判断标准是 `ros2 bag info` 里 `/servo/command` 的 Count 至少是 1。

## 11. 下电

我按三步走：

1. 先给所有舵机发 `PULK` 卸力，1~10 号一次发完；
2. 拔电池；
3. `sudo shutdown now`。

原因是：先卸力，怕舵机还顶着力矩直接断电，容易堵转发热；先断动力电，怕控制板在电压不对的时候乱发指令；最后关机，怕直接拔电把文件系统弄坏。

## 12. 录屏

- 任务二 正反转：https://b23.tv/LJ5c0vR
- 任务二 正反转（第二次）：https://b23.tv/l4OanTw
- 任务二 取消打断：https://b23.tv/L37pkio
- 任务二 改参数生效：https://b23.tv/sdtcGUr
- 任务二 bag 回放：https://b23.tv/k2jUP4A
- 任务三 整机与动作：https://b23.tv/mMYOIkH
- 任务四 安全下机：https://b23.tv/5h2vwQI

## 加分项

- udev 固定设备名 +2：`udev/99-cq-servo.rules`，把 `1a86:7523` 固定成 `/dev/cq_servo`，`port` 默认值也改成了它；
- pytest 单测 +2：`protocol.py` 把指令拼装抽成纯函数，`test/test_protocol.py` 不接硬件就能跑；
- 串口掉线自动重连 +3（部分）：写失败后把串口置空，下一次动作会自动重开，节点不会崩。

PRAD 自动重发、看门狗、动作组抢占这三项还没做。

## 材料说明

- `03_多舵机动作_关键帧.png` 没有单独截图，动作在任务三的录屏里；
- bag 里是 3 条 `/servo/command`、24 条 `/servo/state`，回放能复现 1700、1100、1500 三个动作。
