## 架构图
flowchart TB
    subgraph pkg_iface[cquick_servo_interfaces  ·  ament_cmake · 只放接口]
        MSG["msg/ServoCommand<br/>id / pwm / duration_ms"]
        ACT["action/MoveServo<br/>Goal / Result / Feedback"]
    end

    subgraph pkg_node[cquick_servo  ·  ament_python]
        DRV["servo_driver<br/>Action Server"]
        CLI["move_client<br/>Action Client"]
        CFG[("config/servo_params.yaml")]
        LAUNCH["launch/servo.launch.py<br/>一条命令启动"]
    end

    CLI -- "goal: {id, pwm, duration_ms}" --> DRV
    DRV -- "feedback 10Hz + result" --> CLI
    CFG -. "7 个参数" .-> DRV
    LAUNCH -. "启动 + 加载" .-> DRV
    ACT -. "类型定义" .-> DRV
    MSG -. "嵌套在 Goal 里" .-> DRV

    DRV -- "/servo/command（发布）" --> BAG["ros2 bag record<br/>/servo/command /servo/state"]
    DRV -- "/servo/state 5Hz" --> BAG
    BAG -- "ros2 bag play" --> DRV

    subgraph hw[硬件]
        TTL["USB 转 TTL"]
        BOARD["24 路控制板"]
        SERVO["舵机 001"]
    end
    DRV -- '"#001P1700T1000!"<br/>115200 8-N-1' --> TTL --> BOARD --> SERVO
    SERVO -. "PRAD 读回" .-> BOARD -.-> TTL -.-> DRV
## 参数表



| 参数 | 类型 | 默认值 | 为什么是这个值 |
| --- | --- | --- | --- |
| `port` | string | `/dev/ttyUSB0` | 插上 CH340 后内核认成 `ttyUSB0`，属主组 `dialout`，打开不需要 sudo。尚未做 udev 固定名，换 USB 口可能变成 `ttyUSB1`，用 `ros2 launch ... port:=/dev/ttyUSB1` 覆盖 |
| `baudrate` | int | `115200` | 控制板出厂默认 115200 8-N-1。实测 9600 / 38400 / 57600 下发字节全部为 0 回显，只有 115200 能收到 `#001P1503!` 这类应答，所以只能是它 |
| `pwm_min` | int | `900` | 安全下限。实测 1200~1800 区间内正反转全程无干涉、无异响；900 是在此基础上向左预留的余量，装配整机后按结构复测再收紧。 |
| `pwm_max` | int | `2100` | 安全上限。同上，实测通过 1200~1800；2100 为向右预留余量，超过时关节接近机械极限、堵转发热。
| `default_duration_ms` | int | `800` | 客户端不填 `duration_ms` 时使用。实测 `T1000` / `T1200` 走完肉眼平滑、不猛跳；800ms 用于短行程（如 1500→1600）足够快且不冲击结构，长行程由客户端显式给 duration |
| `servo_ids` | int[] | `[1, 2, 3, 4, 5, 6]` | 本机实际安装的 6 个舵机 ID，与标定记录 `calibration.csv` 一一对应。不在列表里的 ID 在 `goal_callback` 直接 REJECT，防止打错 ID 时对着空气发指令还显示成功。调试单舵机阶段临时为 `[1]` |
| `startup_reset` | bool | `true` | 启动时先对每个 ID 发 `PULR` 恢复扭矩、再全部回 1500 中位。任务 1 踩过“忘发 PULR 舵机纹丝不动”的坑，所以写进代码；开机回中位顺便当零点自检 |

## 接口契约

**Action**：`/move_servo`
**类型**：`cquick_servo_interfaces/action/MoveServo`

### Goal

| 字段 | 类型 | 含义 | 取值 |
| --- | --- | --- | --- |
| `command.id` | `uint8` | 目标舵机 ID | 1~254，且必须在驱动的 `servo_ids` 参数里，否则 goal 被 REJECT |
| `command.pwm` | `uint16` | 目标 PWM | 500~2500us 是电气极限；**实际必须落在 `pwm_min`~`pwm_max`（900~2100）内**，否则 REJECT |
| `command.duration_ms` | `uint16` | 从当前位置走到目标用多久 | 1~65535；**填 0 表示用 `default_duration_ms`（800ms）** |



### 一条启动命令


ros2 launch cquick_servo servo.launch.py


 [INFO] [launch]: All log files can be found below /home/gold/.ros/log/...
 [INFO] [launch]: Default logging verbosity is set to INFO
 [servo_driver-1] [INFO] [servo_driver]: 串口已打开 /dev/ttyUSB0 @ 115200 8-N-1
 [servo_driver-1] [INFO] [servo_driver]: 已对 [1, 2, 3, 4, 5, 6] 发送 PULR 恢复扭矩
 [servo_driver-1] [INFO] [servo_driver]: TX #001P1500T800!
 [servo_driver-1] [INFO] [servo_driver]: startup_reset: [1, 2, 3, 4, 5, 6] → 1500
 [servo_driver-1] [INFO] [servo_driver]: Action Server /move_servo 已就绪
### 踩坑记录
1一开始明明接线是对的，也共地了，但死活没有回显，舵机也不动，我一开始怀疑是硬件问题，也自己去买了杜邦线，换了试试，结果发现杜邦线应该没有问题。后来问了老大，才知道自己好像把舵机id和控制板上的编号弄混淆了，但是把指令中的id改成000后还是不行，后来给老大看了一下代码，老大说python代码有问题，
改了代码之后依然解决不了问题（只能说手机上的ds太拉了），后来用了ds harness，终于解决代码问题。

2 不知道那个ttl模块咋接线，一开始只是把线放在那里面，完全不紧，感觉有点奇怪，后来看到上面有螺丝，萌生出应该拧螺丝的想法，然后也看到群里有人遇到了和我一样的问题，就私信了一下他，他也非常善良地提示了我，
不过我在A区，无法去实验室用螺丝刀，所以只能自掏腰包了。

3 打开新的终端的时候经常会报文件不存在这些错误，然后发给ds，知道自己是没有重新source，现在终于真正理解为什么每次新开终端都要source，以前只是记得这句话
### 接线说明
ttl模块的tx接控制器的rx，ttl模块的rx接控制器的tx，然后还要接地线。电池实测电压7.96V
### 录屏链接
1 正反转 https://t.bilibili.com/1273547276065701911?share_source=pc_native
2 取消打断 https://t.bilibili.com/1273547447878025225?share_source=pc_native
3 改参数生效 https://t.bilibili.com/1273547920323379200?share_source=pc_native
4 bag回放 https://t.bilibili.com/1273548049139892226?share_source=pc_native
### 安全区间
900-2100 试的话是从1500出发，逐级往两极逼近，直到出现异常。
## 7. bag 里录了什么、怎么回放、回放结果

### 7.1 录了什么

只录**两个话题**，不用 `-a`：

| 话题 | 类型 | 作用 | 条数 |
| --- | --- | --- | --- |
| `/servo/command` | `cquick_servo_interfaces/msg/ServoCommand` | 驱动**真正发出**的每一条指令（id / pwm / duration_ms） | 3 |
| `/servo/state` | `cquick_servo_interfaces/msg/ServoCommand` | 5Hz 状态心跳，当前 PWM 估计值 | 114 |

不录全量（`-a`）：板卡上还有别的节点，全录会让 bag 变成几百 MB，而且和本次动作无关。

### 7.2 怎么录的

```bash
cd ~/cquick_ws
rm -rf bag/run1
ros2 bag record -o bag/run1 /servo/command /servo/state
# 另开终端发动作：1700 → 1300 → 1500
# 录完 Ctrl+C，等提示符回来（会写 metadata.yaml）
```

关键点：

- **先起驱动、再开录制、再发动作** —— 顺序反了录制会订阅不到话题；
- 录制必须**正常结束**（等 `Recording stopped` 和提示符），中途强杀会导致缺 `metadata.yaml`，`ros2 bag play` 会报 `No storage id specified`；
- 时长控制在 30 秒内，体积自然就小。

### 7.3 `ros2 bag info` 实测输出

```text
Files:             run1_0.db3
Bag size:          25.5 KiB
Storage id:        sqlite3
Duration:          22.599914041s
Start:             Nov 21 2026 21:12:21.830950882 (1795266741.830950882)
End:               Nov 21 2026 21:12:44.430864923 (1795266764.430864923)
Messages:          117
Topic information: Topic: /servo/state   | Type: cquick_servo_interfaces/msg/ServoCommand | Count: 114 | Serialization Format: cdr
                   Topic: /servo/command | Type: cquick_servo_interfaces/msg/ServoCommand | Count: 3   | Serialization Format: cdr
```

体积 25.5 KiB，远小于 10 MB 的限制；时长 22.6 s，小于 30 s。

### 7.4 怎么回放

```bash
ros2 bag play bag/run1 --topics /servo/command --disable-keyboard-controls
```

- `--topics /servo/command`：只灌指令，状态话题对回放没意义；
- `--disable-keyboard-controls`：去掉一堆 `Press SPACE...` 提示，画面干净；
- 可选 `-d 2` 延迟 2 秒开始，方便对准镜头。

回放前确认驱动在跑：

```bash
ros2 node list | grep servo_driver
ros2 topic info /servo/command      # Subscription count 必须是 1
```

### 7.5 回放结果

回放后舵机**按录制时的顺序重做了一遍**：1700 → 1300 → 1500，动作幅度和速度与录屏一致；驱动终端同时打出：

```text
回放指令: id=1 pwm=1700 duration=1000ms
回放指令: id=1 pwm=1300 duration=1000ms
回放指令: id=1 pwm=1500 duration=1000ms
```

最后 `Playback finished`。

### 7.6 为什么回放能复现（实现要点）

`/servo/command` 是**发布 + 订阅同一个话题**：

- 发布：`_send_move()` 每成功写一次串口，就把同一条指令发布到 `/servo/command`，所以录下来的是**指令流**；
- 订阅：`_on_command_topic()` 收到后调用**同一个** `send_move()`，不是另写一套发送逻辑 —— 否则回放动作和录屏一定对不上。

同话题自发自收会无限循环，所以驱动要吃掉自己的回显：每次发布登记一条 `(id, pwm, duration) + 时间戳`，收到完全相同的消息就丢一次（0.5 秒窗口）。这样：

| 消息来源 | 处理 |
| --- | --- |
| 自己刚发布的回显 | 匹配到登记项 → 丢弃，不执行 |
| bag 回放的消息 | 没有登记项 → 正常执行 → 执行时又发布一条 → 它的回显被丢弃 |

每条消息最多被吃一次，不会级联。


