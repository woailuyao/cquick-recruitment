# CQuick 电控组第 3、4 周考核 · 整机拼装与舵机控制

姓名拼音：zhuqianyin　日期：2026-11　平台：ROS 2 Humble（VMware Ubuntu 22.04 调试）

---

## 1. 系统结构

两个包，按考核要求拆开：`cquick_servo` 是逻辑包（ament_python），`cquick_servo_interfaces` 是接口包（ament_cmake）。

```
           move_client / sequence_player / send_goal / bag 回放
                              │
                              ▼   Action /move_servo（MoveServo.action）
                 ┌────────────────────────────────────────┐
                 │  cquick_servo · servo_driver（唯一节点） │
                 │  三道校验：白名单 → 限位 → 看门狗闩锁      │
                 │  0.5s 位置帧心跳  ·  2.0s 看门狗         │
                 └───────┬───────────────────────┬────────┘
                         │ 串口帧 #005P1620T1200! │ /servo/command（回放入口）
                         ▼                       ▼
                 CH340 → 24 路控制板 → 舵机 1~10    /servo/state（位置广播）
```

只有 `servo_driver` 一个常驻节点：串口收发、校验、心跳、看门狗、PRAD 回读全在里面。串口只有一根，所有指令必须从这里收口，校验才绕不过去。

## 2. 参数（`config/servo_params.yaml`）

| 参数 | 值 | 来历 |
| --- | --- | --- |
| `port` | `/dev/cquick_servo`（现场覆盖为 `/dev/ttyUSB0`） | udev 按 CH340 的 `1a86:7523` 绑固定名，避免 ttyUSB 编号漂移 |
| `servo_ids` | 整机 `[1..10]` / 单舵机 `[0]` | PID 扫描实测。**启动时读取，改完必须重启 launch**，运行中改无效 |
| `baudrate` | 115200 | 舵机协议固定值，错了就是乱码或无回显 |
| `pwm_min / pwm_max` | 900 / 2100 | 结构限位实测 800/2200，两边各留 100 余量（见第 5 节） |
| `position_tolerance` | 30 | PRAD 抖动实测 ±10，取三倍余量 |
| `heartbeat_period` | 0.5 | 控制板执行完一帧就停发 PWM，舵机收不到会嗡鸣泄力；0.5s 重发后现象消失 |
| `watchdog_timeout` | 2.0 | 合法范围 0.2~10。设 1.0 在 VM 里曾线程饥饿误触发；且**必须 > 心跳周期 0.5**，否则正常心跳本身就算超时 |
| `default_duration_ms` | 800 | 未指定时长的默认值 |
| `feedback_period` | 0.2 | feedback 周期 |

参数判定走实时 `get_parameter`，不是构造时副本——所以 `pwm_max` 改完当帧生效（2.6 的演示就靠这个），而 `servo_ids` 刻意只在启动读一次（硬件拓扑变了才该变）。

## 3. Action 接口

```
Goal:
  uint8  id            # 目标舵机 ID，必须在 servo_ids 白名单里，否则 rejected
  uint16 pwm           # 目标位置，超出 [pwm_min, pwm_max] rejected
  uint16 duration_ms   # 运动时间 ms，0 = default_duration_ms

Feedback（0.2s 一次）:  uint16 current_pwm / float32 progress
Result:                bool success / uint16 final_pwm / string message
```

`message` 取值：`ok` / `watchdog_latched` / `超出上限/下限` / `未到位(差 N)` / 串口错误。**失败就如实报，不重试不掩盖**——重试会遮住硬件问题。

## 4. 启动

```bash
cd ~/cquick_ws && source install/setup.bash
ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB0
```

正常三行：

```
串口已打开: /dev/ttyUSB0 @ 115200
启动复位完成：[1..10] -> 中位 1500
安全看门狗已启动 (timeout=2.0s)
```

**铁律：开 launch 前先跑体检**（`lsusb | grep 1a86` → `ls -l /dev/ttyUSB*` → pyserial 试开打印 `串口OK`），三项全过再开。串口坏着开 launch 只会刷一屏 `PULK 失败：串口未打开`。

## 5. 安全区间怎么定

协议允许 500~2500，没用它。做法：从 1500 往两侧转，转到舵臂贴结构限位时读 PRAD（约 800 / 2200），各留 100 余量定为 **900~2100**。任何通过校验的指令都不可能顶死结构——堵转三分钟就能烧穿舵机。

验证：`param set pwm_max 1600` → 发 `pwm=1700` → 直接 rejected（截图 `02_限位保护.png`）。

## 6. 整机：10 个舵机怎么接、怎么验

**接线**：CH340 TX/RX 交叉接控制板串口；2S 锂电池给控制板；控制板到舵机走 3 针总线（一进一出菊花链，红棕是电源地，接反必坏）；**板卡、控制板、电池必须共地**——只接信号不共地，数据完全收不到。

**扫描**：`python3 -u ~/demo1.py /dev/ttyUSB0 0`，扫 0~10，实测**整机 10 个 ID（1~10）全部在线**。

**标定**：上电后逐个读 PRAD，**10 个全部落在 1500±25**，无需标定（机械装配到位）。

**逐关节验证**：`joint_check.py --delta 150 --duration 1200`，对每个 ID 走「回中位 → +delta → −delta → 回中位」，末尾判据 ±30。结果 **10/10 通过**，日志 `~/joint_log.md`。

**动作组**：`stand_up.yaml`（矢状面起立，膝踝 3/4/8/9 协同）、`wave.yaml`（只动 3/4/8/9）、`greet.yaml`（只动 1/6 髋部侧摆）。三遍连跑 **run 3/3 done in 8.139s，全 success**（`~/logs/standup_x3.log`）。

整机阶段的三个硬约束（都是实测踩出来的）：

1. **承重关节收行程**：ID 6 目标 1650 实测只到 1616（差 34 超容差），负载下力矩不足。把目标收到 1620 后稳定通过——**主动限制比硬顶强**；
2. **只做矢状面**：横向内收会让左右腿互撞（yaml 里我一开始假设左右镜像，实际是同向装配）。髋部只在小角度做演示；
3. **每个动作做完回中位**，避免累积偏移。

## 7. 动作组：同步只认 result

播放器读 yaml，`step` 是顺序，`moves` 是同一步内**同时**下发：

- **同一步**多条 goal 先全发出去再统一收 result → 多舵机协同；
- **上一步 result 全回收**才进下一步 → 先后顺序用 step 表达，不靠 sleep 猜时间；
- 代码里有两个很小的 `sleep`，那是**串口帧间隔**（控制板处理不了零间隔背靠背帧），不是用来等舵机走完的。

换动作零改码，只换 `--file`：

```bash
ros2 run cquick_servo sequence_player --file stand_up.yaml --repeat 3
ros2 run cquick_servo sequence_player --file wave.yaml    --repeat 3
ros2 run cquick_servo sequence_player --file greet.yaml
```

## 8. bag 录制与回放

```bash
# 终端1 launch  ·  终端2 开录  ·  终端3 发动作
ros2 bag record /servo/command /servo/state -o ~/bag/run1
ros2 bag info ~/bag/run1 ; du -sh ~/bag/run1
ros2 bag play ~/bag/run1
```

终版包：`/servo/command` 13 条、`/servo/state` 48 条、22.2 s、25.5 KiB（≤10 MB）。回放时 launch 日志逐条滚出 `TX #000P1600T1000!` → `TX #000P1400T1000!`，舵机 1:1 复现。

**一个必须说明的实现细节**：`/servo/command` 是**只收不发**的调试总线（Publisher count = 0），正常跑动作时它上面没有数据，所以直接 `ros2 bag record /servo/command` 只会录到 state。录之前要先用 `ros2 topic pub -r 2` 把它"养住"（撑到 recorder 订阅成功），再发动作。回放时 driver 收话题消息走**同一个发送函数**，所以能和 Action 1:1 对齐。

`ros2 bag record 话题名` 只订阅启动那一刻存在的话题——这是上面所有坑的根源。

## 9. 运行时改参数（2.6 六步实测）

`param list` → `param get pwm_max`（2100）→ `param set pwm_max 1600`（successful）→ 发 `{pwm: 1700}`（**被拒**，超上限 1600）→ `param set 100000`（被拒）→ 恢复 2100。

关键点：第 ④ 步没重启，行为立刻变——说明判定走实时参数。第 ⑤ 步被拒说明校验函数先查范围再比较，非法值进不了系统。

## 10. 看门狗（加分项）

闩锁机制：心跳超时 → 全部舵机 PULK 卸力 + 置闩锁 → 新 goal 一律拒绝 → **必须人工** `ros2 param set /servo_driver watchdog_reset true` 解锁。

为什么闩锁而不自动恢复：故障恢复瞬间可能积压一堆指令，机器人会突然乱动，有人靠近时很危险。把"恢复"从程序手里交给操作员是刻意的安全设计。

演示顺序（顺序错了会失败）：

```bash
ros2 param set /servo_driver watchdog_timeout 0.3   # 触发（< 心跳 0.5）
ros2 action send_goal ... {id: 0, pwm: 1700, ...}   # result: ABORTED + watchdog_latched
ros2 param set /servo_driver watchdog_timeout 2.0   # 先恢复阈值
ros2 param set /servo_driver watchdog_reset true    # 再解锁
ros2 action send_goal ... {id: 0, pwm: 1700, ...}   # SUCCEEDED
```

**拒绝发生在执行阶段**：`goal_callback` 只验参数合法性（id/pwm/duration），闩锁检查在 `send_move` 里。所以驱动日志永远显示"goal 接受 / 执行"，真拒绝要看客户端 `result`：`success: false` + `message: watchdog_latched` + `ABORTED`。

## 11. udev 固定设备名（加分项）

```bash
# /etc/udev/rules.d/99-cquick-ch340.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="cquick_servo", MODE="0666"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/cquick_servo     # -> ttyUSB1，插哪个口都是这个名字
```

默认 `ttyUSB0` 编号随插拔漂，launch 的 `port` 就得跟着改；绑固定名后参数不用动。截图 `加分文档/udev/udev设备名.png`。

## 12. 下电顺序（任务 4）

1. **卸力**：`#xxxPULK!`（或停 launch 自动触发），力矩归零；
2. **断动力电**：拔 2S 锂电池（舵机、控制板全断）；
3. **关板卡**：`sudo shutdown now`，**等指示灯灭/屏幕黑**再拔板卡电源。

理由：带力矩断电会冲击齿轮和电位器；先断电池再关板卡避免控制板在异常电压下乱发指令；X5 跑完整 Linux，运行中掉电会损坏文件系统。



## 13. 录屏

任务二（B 站）：

- 【舵机转动】<https://b23.tv/r14Nsz2>
- 【正反转回中位】<https://b23.tv/Ktzx7y6>
- 【取消打断】<https://b23.tv/4zYLQou>
- 【改参数后行为变化】<https://b23.tv/DfHl7t4>
- 【bag 回放复现】<https://b23.tv/7wcrAaJ>

任务三（整机）：待补

---【逐个动作-哔哩哔哩】 https://b23.tv/lfIKk4Y
【三遍连跑-哔哩哔哩】 https://b23.tv/4dK99Y8
【改yaml-哔哩哔哩】 https://b23.tv/pa4a7l5
安全下电：https://b23.tv/568B05b



完整踩坑过程与对照实验见 `调试记录.md`。
