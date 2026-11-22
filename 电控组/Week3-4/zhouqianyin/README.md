# CQuick 电控组第 3、4 周考核 · 整机拼装与舵机控制

姓名拼音：zhuqianyin　日期：2026-11　平台：ROS2 Humble（VMware Ubuntu 22.04 调试，目标板卡 RDK X5）

## 1. 系统结构

一共两个包：`cquick_servo` 是逻辑包（ament_python），`cquick_servo_interfaces` 是接口包（ament_cmake），接口单独拆包是按考核要求做的。

```
                     ┌──────────────────────────────────────────┐
 ros2 action         │            cquick_servo                  │
 send_goal ────────► │   servo_driver（Action Server）           │───/dev/cquick_servo──► 24路控制板 ──► 舵机 000/001
 /move_servo         │        │                                 │◄──PRAD 回显──────────┘
                     │        │ 0.5s 位置帧心跳 + 2.0s 看门狗      │
 sequence_player ──► │   /servo/command、/servo/state            │
 （读 yaml 发 goal）  │        （调试话题，bag 录的就是这两个）        │
                     └──────────────────────────────────────────┘

                     ┌──────────────────────────────────────────┐
                     │  cquick_servo_interfaces                 │
                     │  msg/ServoCommand.msg   action/MoveServo │
                     └──────────────────────────────────────────┘
```

节点只有一个 `servo_driver`，串口收发、限位、心跳、看门狗全在里面。`sequence_player` 是任务 3 的动作组播放器，它自己不做任何动作决策，只负责读 yaml 然后调 Action。

## 2. 参数

参数全部外置在 `config/servo_params.yaml`，启动时加载，运行时可改。几个值得解释的默认值：

- `port: /dev/cquick_servo` —— 用 udev 规则按 CH340 的 `1a86:7523` 绑定的固定名。之前用 ttyUSB0 时拔插一次编号就变，绑完就不再漂。
- `servo_ids: [0, 1]` —— PID 扫描实测的在线列表。这里有个容易踩的坑：ID 是烧在舵机里面的，跟插在哪个口无关，我之前按插口号去猜 ID 猜错了一次。
- `pwm_min: 900 / pwm_max: 2100` —— 来历见第 5 节。
- `position_tolerance: 30` —— PRAD 回读抖动实测 ±10 左右，取了三倍余量。
- `heartbeat_period: 0.5` —— 这个参数是调试出来的，不是拍的。控制板把一帧执行完就停发 PWM，舵机收不到信号会滴滴响、泄力；每 0.5s 重发一次位置帧之后这个现象完全消失。
- `watchdog_timeout: 2.0` —— 一开始设 1.0s，在虚拟机里因为线程饥饿误触发过（goal 并发执行时心跳定时器排不上队），调到 2.0s 之后零误触发，安全意义也没变。过程写在调试记录问题 4。
- 其余（`baudrate: 115200`、`default_duration_ms: 800`、`startup_reset: true`、`feedback_period: 0.2`）都是常规取值，实测工作正常就没再动。

## 3. Action 接口

`MoveServo.action`：

```
Goal:
  uint8  id            # 目标舵机 ID，必须在 servo_ids 里
  uint16 pwm           # 目标 PWM，落在 [pwm_min, pwm_max] 之外直接 rejected
  uint16 duration_ms   # 运动时间 ms，0 = 用 default_duration_ms

Feedback（每 0.2s 发一次）:
  uint16  current_pwm  # 当前位置（PRAD 回读）
  float32 progress     # 0.0 ~ 1.0

Result（duration 到点后发）:
  bool   success
  uint16 final_pwm     # PRAD 实测最终位置
  string message       # ok / rejected / canceled / 超限 / 串口错误 / 未到位(差 N) / ok(补救)
```

流程上：goal 一接受就发 PULR 加位置帧，移动期间按周期发 feedback，时间到读 PRAD 判定，然后回 result。步与步之间的同步只认 result，这一点在第 9 节展开说。

## 4. 启动

```bash
cd ~/cquick_ws && source install/setup.bash
ros2 launch cquick_servo servo.launch.py
```

正常起来的输出：

```
[servo_driver-1] 串口已打开: /dev/cquick_servo @ 115200
[servo_driver-1] 启动复位完成：[0, 1] -> 中位 1500
[servo_driver-1] 安全看门狗已启动 (timeout=2.0s)
```

launch 参数可以直接覆盖 yaml，不用改文件：

```bash
ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB1 pwm_max:=2000
```

## 5. 安全区间是怎么定的

协议本身允许 500-2500，但我没用这个范围。做法是：让舵机从 1500 出发往两边转，转到舵臂**贴上结构限位**时读 PRAD，大概是 800 和 2200。在这个基础上两边各留 100 的余量，定成 900-2100。这样任何通过校验的指令都不可能让舵机顶死结构——堵转状态三分钟就能把舵机烧穿。

限位保护是拿 2200/800 这些越界值验证的：把 `pwm_max` 改成 1600 再发 1700，goal 直接 rejected，对应截图 `02_限位保护.png`。

## 6. 运行时改参数（2.6 六步实测）

完整跑了一遍 2.6 的演示脚本，顺序是：`param list` → `param get pwm_max`（读到 2100）→ `param set pwm_max 1600`（successful）→ 再发 `{pwm: 1700}`（**被拒**，提示超出上限 1600）→ `param set 100000`（被拒）→ 恢复 2100。关键在第 ④ 步：改完参数**没有重启**，行为立刻就变了，说明判定走的是实时 `get_parameter`，不是构造时存的副本——代码里我是特意这么写的，就是为了配合这个演示。

## 7. bag

录制（15.36s，在 30s 和 10MB 限制内）：

```bash
ros2 bag record -o bag/run1 /servo/command /servo/state
```

`ros2 bag info bag/run1`：

```
Duration: 15.36 s
Topics:   /servo/command  Count: 6   (cquick_servo/msg/ServoCommand)
          /servo/state    Count: xx  (cquick_servo/msg/ServoCommand)
Size:     25.4 KiB
```

回放 `ros2 bag play bag/run1`，舵机把录制的动作原样重做了一遍（B 站视频见文末）。

有一个实现细节要说明：Action 指令不落话题，所以录 bag 之前我把同样的指令同时发了一份到 `/servo/command` 作为载体，回放时 driver 收到话题消息照常执行，底层走的是同一个发送函数。

## 8. 接线

- 上位机 USB → CH340 USB 转 TTL（`lsusb` 显示 `1a86:7523`）；
- CH340 TX/RX → 控制板串口，TX→RX、RX→TX 交叉；
- 2S 锂电池 → 控制板电源。电压量过三个节点：刚上手 7.63 V（3.82 V/节），连续调试几个小时后掉到 6.6 V，低压报警器开始响（3.3 V/节正好踩在它的默认阈值上）；充满后 8.40 V。全程没低于 5 V 下限，报警器我设在 3.4 V/节那一档；
- 控制板通道 → 舵机，3 针总线线，红棕是电源地、不能接反；
- 板卡、控制板、电池要**共地**——这个我实测过，只接信号不共地的话数据完全收不到。

## 9. 动作组

动作写在 `config/actions/` 下面，播放器只认数据。`stand_up.yaml` 全文：

```yaml
name: stand_up
description: "two joints: fold -> lift -> center one by one"

steps:
  - name: prepare
    moves:
      - {servo_id: 1, target_pwm: 1350, duration_ms: 1600}
      - {servo_id: 0, target_pwm: 1650, duration_ms: 1600}

  - name: lift
    moves:
      - {servo_id: 1, target_pwm: 1200, duration_ms: 1800}
      - {servo_id: 0, target_pwm: 1800, duration_ms: 1800}

  - name: joint1_center
    moves:
      - {servo_id: 1, target_pwm: 1500, duration_ms: 1500}

  - name: joint2_center
    moves:
      - {servo_id: 0, target_pwm: 1500, duration_ms: 1500}
```

几个设计上的取舍：

同步信号只有 result。step 内的所有 goal 先全部发出去再统一收 result（这是协同），上一个 step 的 result 全部回收后才进下一个 step。先后顺序用 step 拆开表达（比如两个舵机必须先后回中位，就分成 joint1_center / joint2_center 两个 step），而不是靠 duration 错时间硬凑。

duration 的数字是调出来的：一开始按行程给了 1000-1200，实测发现 PULR 使能窗口会吃掉首帧、而且斜坡时间不够（差 69 没走完），统一加到 1400-1800。代码里有两处 `sleep(0.3)` 和 `sleep(0.4)`，那个不是用来猜舵机走没走完的——是串口帧间隔，控制板处理不了零间隔的背靠背帧（详见调试记录问题 1），真正的同步一直是 result。

换动作零改码。跑第二套动作只换 `--file` 参数：

```bash
ros2 run cquick_servo sequence_player --file stand_up.yaml --repeat 3
ros2 run cquick_servo sequence_player --file wave.yaml    --repeat 3
```

## 10. 踩过的坑（完整版带时间戳和对照实验，见调试记录.md）

**改了代码不生效，查了半天是两个工程目录。** 最早踩的坑其实和硬件无关：`colcon build` 无报错、运行行为却完全没变，`param get` 一个源码里 declare 过的参数居然返回 not set。最后对比 `pwd` 和 VS Code 的工作区才发现 `~/ws` 和 `~/cquick_ws` 两个目录并存——编辑落在 `~/ws`，编译和 launch 都在 `~/cquick_ws`。教训：声明过的参数不可能返回 not set，出现这种事首先该怀疑跑的不是当前源码。

**电池低压报警。** 早上上电报警器一直响，静置电压 6.6 V。一算 6.6÷2=3.3 V/节，正好是报警器的出厂默认阈值，不是电池坏了。停止带载去充电，充满 8.40 V 后三遍连跑再没出现过欠压复位。电压链 7.63 → 6.6 → 8.40 V 说明供电设计没问题，纯粹是没电。

**背靠背丢帧。** 一个 step 里两条 goal 同时下发，两条 result 都返回 success，但只有一个舵机动，规律固定"后一条丢"。用同样两条指令做对照：零间隔只动一个，错峰 300 ms 两个都动——板子处理不了零间隔的两帧，这是固件行为。播发器里加了步内错峰 0.3 s、步间 0.4 s。

**回中位必挂。** stand_up 走到最后一步"回 1500"必失败，换舵机、换通道都复现。我先后怀疑过舵机坏了和 duration<1000 非法，两个都被实验推翻了（发 T0900 能成功；对调舵机后故障不跟着舵机走）。最后打印时序才发现：PULR 使能要约 1 秒，使能窗口内紧跟的位置帧被吞，而位置帧只发一次，duration 调多大都没用。解法是移动前重发 PULR，加一个"未到位补救分支"（等 0.6 s 重发位置帧再重读 PRAD）。

**看门狗启动误触发。** 换上带看门狗的新版 driver，launch 起来 0.1 秒就触发、全部卸力闩锁，可什么动作都没跑。原因是心跳时间戳在构造时就初始化了，而心跳定时器还没开始跳；VMware 打开 CH340 要 1.2 秒，被当成"失联"。把心跳时间戳的初始化推迟到第一次心跳之后就好了。

**看门狗运行途中误触发。** 修好启动竞态后，step 1 两条 goal 并发执行时看门狗又触发了。这次是 `MultiThreadedExecutor()` 没给线程数，默认取 CPU 核数（虚拟机 1-2 核），两条并发 goal 把线程占满，心跳定时器排不上队被饿死，"忙"被判成"死"。改成 `num_threads=8`、阈值放宽到 2.0 s，连跑多次零误触发。

**静止后滴滴响。** 舵机停稳几秒后持续鸣叫、泄力、PRAD 冻结，而且是"后来才响"，上电瞬间不响。刚经历过低压报警，我第一反应还是电池——满电复测照样响才排除。真因是板子把一帧执行完就停发 PWM，舵机收不到信号进了失联保护。加位置帧心跳（每 0.5 s 重发最后一条位置帧，移动中的跳过）解决，这个补丁还顺手兜住了上面"首帧被吞无人补发"的情况。

**PRAD 无回显。** 三遍连跑前最后试跑，result 返回 final_pwm=0、"PRAD 无回显"，而且 duration 到点瞬间就返回，完全没等回包。心跳刚发完位置帧，紧跟的读取帧又被背靠背丢掉了——和丢指令帧同源，这次丢的是读取帧。`read_position()` 改成最多重试 3 次、间隔 0.3 s。

**VMware 串口掉线（Errno 5 / Errno 110）。** 跑着跑着突然两个舵机同时秒失败，有时 `/dev/ttyUSB0` 直接消失。VMware 的 USB 透传在电流突变或重新枚举时会断 CH340。固化了一套排查顺序：插拔 USB → 脚本测 PRAD 有回显 → 起 launch → 看到"串口已打开"才跑播放器。顺序反了会出现"串口已打开"的假成功，看起来像代码问题其实是环境问题。迁到 X5 原生 USB 后这个问题从根上消失。

## 11. 下电

顺序是先卸力再断电最后关板卡：先给全部舵机发 PULK（停 launch 也会自动触发），撤掉力矩，避免断电瞬间舵机带力乱动撞结构；然后拔电池；最后 `sudo shutdown now` 关 RDK X5。上位机必须最后关，不然 bag 和日志没写完就断电会损坏文件。

## 12. 录屏链接（B 站）

任务二：

- 【舵机转动录屏】 <https://b23.tv/r14Nsz2>
- 【正反转回中位】 <https://b23.tv/Ktzx7y6>
- 【取消打断】 <https://b23.tv/4zYLQou>
- 【改参数后行为变化】 <https://b23.tv/DfHl7t4>
- 【ros2 bag play 回放复现】 <https://b23.tv/7wcrAaJ>

任务三：因为缺螺丝，机械组的腿部结构件拼不起来，整机实验做不了，改用两个单舵机把链路跑通，录屏如下：

- 【逐个动作】 <https://b23.tv/c1KGuau>
- 【动作组连跑三遍】 <https://b23.tv/Of6Ef5R>
- 【换 yaml 再跑一遍】 <https://b23.tv/95pTDHC>
