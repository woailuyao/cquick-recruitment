# 电控组 Week3,4 · 徐靖轩

> 任务 1（点亮一个舵机）· 任务 2（ROS2 接管）· 任务 3（动作组，动作已数据化）
> 硬件：Zide 24 路总线舵机控制板 + 联迈纳 LES26（CH340）+ 2S 锂电 1500mAh 45C

---

## 1. 架构图

```
        ┌─────────────────────────────┐
        │        cquick_servo         │
        │                             │
  ros2 action send_goal             │   move_client / sequence_player
        │                           │        （Action Client）
        ▼                           │              │
  ┌───────────────┐                 │              │
  │ /move_servo   │◄────────────────┼──────────────┘
  │ Action Server │                 │
  │ servo_driver  │                 │
  └───────┬───────┘                 │
          │ 发布                     │
          ├──► /servo/state   （每帧 PRAD 实际读回 + progress）
          ├──► /servo/command （收到 Action 后镜像一份，专供 bag 录制/回放）
          │ 订阅
          └──◄ /servo/command （回放时由此驱动；带回声抑制，不会自己吃自己）
          │
          ▼ 串口 115200 8-N-1
   #000P1300T800!  ──►  控制板  ──►  舵机
   ◄── #000P1300!
```

| 东西 | 名字 |
| --- | --- |
| 包 1（接口） | `cquick_servo_interfaces`（ament_cmake） |
| 包 2（实现） | `cquick_servo`（ament_python） |
| Action | `/move_servo`（`MoveServo`） |
| 话题 | `/servo/state`、`/servo/command`（`ServoCommand`） |
| 节点 | `servo_driver`、`move_client`、`sequence_player` |

**为什么有 `/servo/command`**：`ros2 bag` 只能录话题、录不到 Action。
驱动把收到的 Action 指令镜像到这个话题，bag 录下来，`bag play` 时同一个节点订阅它就能复现动作。
节点自己也订阅这个话题，所以做了回声抑制（见第 10 节踩坑 2）。

---

## 2. 参数表

| 参数 | 默认值 | 为什么是这个值 |
| --- | --- | --- |
| `port` | `/dev/ttyUSB0` | CH340 在 Ubuntu 下固定是 ttyUSB0；属主组 dialout，不用 sudo |
| `baudrate` | `115200` | 控制板出厂默认，厂家文档与实测一致 |
| `pwm_min` | `900` | 结构安全下限：再小舵机开始顶到机械限位并持续堵转发热 |
| `pwm_max` | `2100` | 结构安全上限：同上。电气极限是 500~2500，但那是舵机能接受的，不是结构能承受的 |
| `default_duration_ms` | `800` | 没给 duration 时用它；800 ms 走 600 PWM 差不会猛跳，也不至于太慢 |
| `cancel_return_ms` | `500` | 取消后回中位专用。用原 goal 的 duration 会让"立刻停"变成慢挪（踩坑 3） |
| `servo_ids` | `[0]` | 本机舵机实测 ID = **000**（不是 001） |
| `startup_reset` | `true` | 上电统一回 1500 中位，避免带着上次位置进结构 |

代码里这 8 个名字只出现在「声明参数」和「读参数」两处，判定限位、拼指令、开串口全走读出来的值。
`02_参数自检.png` 是 `grep declare_parameter` + yaml 的实际输出。

---

## 3. 接口契约

```yaml
# MoveServo.action
goal:
  uint8  id            # 舵机 ID（本机 0）
  uint16 pwm           # 目标 PWM（电气范围 500~2500，安全范围见参数）
  uint16 duration_ms   # 从当前位置走到目标用多久
---
result:
  bool   success       # 成没成
  uint16 final_pwm     # 最终 PRAD 实际读回值
  string message       # 一句话说清原因：到位 / 超限 / 取消 / 串口错 / 没到位
---
feedback:
  uint16  current_pwm  # 当前 PRAD 读回值（不是插值）
  float32 progress     # 0.0~1.0
```

发送时机：
- **feedback**：goal 执行期间每 100 ms 一次，直到 progress 到 1.0；
- **result**：走完 `duration_ms` + 200 ms 稳定时间后读一次 PRAD，与目标比较（容差 ±30）再返回。

`ServoCommand.msg`（`id` / `pwm` / `duration_ms`）用在同一套指令的两个话题上：
`/servo/command` 是"要做什么"，`/servo/state` 是"现在到哪了"。

---

## 4. 一条启动命令

```bash
$ ros2 launch cquick_servo servo.launch.py port:=/dev/pts/0
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [servo_driver-1]: process started with pid [5877]
[servo_driver-1] [INFO] servo_driver 已启动：端口=/dev/pts/0 波特率=115200 安全范围=900~2100 舵机=[0]
[servo_driver-1] [INFO] 串口已打开：/dev/pts/0 @ 115200 8-N-1
[servo_driver-1] [INFO] ID 0 恢复扭力（PULR）
[servo_driver-1] [INFO] 启动复位完成：[0] 全部回 1500
```

launch 提供三个 arg：`port`（串口）、`baudrate`、`demo`（true 时顺带起客户端，
用 `IfCondition` 控制，默认 false —— 日常只需要驱动节点）。见 `02_launch一键启动.png`。

---

## 5. 安全区间怎么试出来的

不是拍脑袋定的，是"听到声音就退"试出来的：

1. 先按电气极限 500~2500 下发，从 1500 每次 ±100 往外走；
2. 走到 **2200** 时舵机接近结构限位，出现持续"嗡嗡"声（堵转）→ 立刻回 1500；
3. 走到 **860** 时同样出现顶死声 → 回 1500；
4. 取比这两个点各留 40~60 PWM 余量：**900 ~ 2100** 写进 `pwm_min` / `pwm_max`。

> 所以代码里"电气极限 500~2500"和"安全范围 900~2100"是两个概念，前者写进 msg 注释，后者进参数并用于拦截。

---

## 6. 运行时改参数（当帧生效）

```bash
$ ros2 param set /servo_driver pwm_max 1200
Set parameter successful

$ send_goal {id:0, pwm:1300, duration_ms:600}     # 改之前是合法的
Goal was rejected.
--- 驱动日志 ---
[INFO] 参数 pwm_max -> 1200，下一帧起生效
[WARN] goal 被拒绝：id=0 pwm=1300，原因：pwm 1300 超出安全范围 900~1200

$ ros2 param set /servo_driver pwm_max 2100
Set parameter successful
```

**没有重启节点**，下一条 goal 就按新上限判断 —— 因为 `goal_callback` 里每次都是
`self.get_parameter('pwm_max').value` 现读，不是在 `__init__` 里存副本。
完整截图见 `02_param_set现场.png`。

---

## 7. bag：录了什么、怎么回放

```bash
$ ros2 bag info bag/run1
Files: run1_0.db3        Bag size: 32 KiB       Storage id: sqlite3
Duration: 7.92s          Messages: 27
Topic: /servo/state   | ServoCommand | Count: 25
Topic: /servo/command | ServoCommand | Count: 2     ← 2 个动作，恰好 2 条
```

**真机录制**（2026-11-21 22:05，`/dev/ttyUSB0`，舵机 ID 000）：
两次动作 `1300`（PRAD 读回 1309）、`1700`（读回 1690），状态都 SUCCEEDED。

**回放验证**（真机，同一台设备）：

```
$ ros2 bag play bag/run1
[servo_driver] /servo/command -> ID 0 pwm 1300 1500ms
[servo_driver] /servo/command -> ID 0 pwm 1700 1500ms

$ 回放后直接读 PRAD：#000P1694!      ← 目标 1700，到位
```

所以回放不只是"重播日志"，而是**真的重新驱动了舵机**。

只录 `/servo/command` 和 `/servo/state`，**没有用 `-a`**（`-a` 会把 `/rosout`、`/parameter_events`
一起录进来，评审要看的是动作本身）。

---

## 8. 接线与供电

| 控制板引脚 | 接到 |
| --- | --- |
| `RX1`（"用户"座） | USB-TTL 的 **TXD** |
| `TX1` | USB-TTL 的 **RXD** |
| `GND` | USB-TTL 的 **GND**（必须共地） |
| `5V` | **不接**（板子自带电源，接上会和电池打架） |

- 舵机接控制板任一路输出口；
- 电池上电前万用表实测：**8.15 V**（2S 满电 8.4 V 的 97%，正常）；
- **黄色座 `TX4/RX4` 是蓝牙口**，插那里完全没有回显 —— 这是任务 1 花最久的一个坑。

---

## 9. 动作 yaml 长什么样

一套动作 = 有序的 `steps`；一步 = 若干条「同时下发」的 `moves`。

```yaml
name: stand_up
description: 双关节同时抬起，再同时收到中位（验证「同步下发」）
repeat: 1
steps:
  - name: 两个关节同时抬
    moves:
      - {id: 0, pwm: 1300, duration_ms: 1000}
      - {id: 1, pwm: 1700, duration_ms: 1000}

  - name: 两个关节同时回中位
    moves:
      - {id: 0, pwm: 1500, duration_ms: 1000}
      - {id: 1, pwm: 1500, duration_ms: 1000}
```

> ⚠️ **`duration_ms` 不能小于 1000** —— 真机实测：低于 1000 时控制板**静默不执行**
> （目标 1620 给 600/800/900ms，PRAD 一直停在 1500 附近；给 1000ms 就到 1592）。
> 驱动已加参数 `min_duration_ms: 1000` 统一钳制，看到 `duration xxx 低于控制板下限` 的告警属正常。

三条时序规则怎么表达：

| 要什么 | 怎么写 |
| --- | --- |
| 一步内多舵机协同 | 写在**同一个 step** 的 moves 里 → 播放器全部 `send_goal_async` 后一起等 |
| 步与步串行 | 天然串行：上一步**所有 result 回来**才进入下一步，没有 sleep |
| 一步之内有先后 | 单独开一个 step（`wave.yaml` 第 2、3 步就是单舵机独立 step） |

`duration_ms` 怎么定：结构能接受的最小值往上加 —— 两个关节同时动时取**较慢那个**的时长对齐，
避免一个先到位、另一个还在动导致结构互相别着。

播放器实测（`02` 截图之外，日志里）：

```
── 第 1/2 步：2 条指令同时下发（预计 0.80s）
   ok id=0 → 1300（PRAD 1300）
   ok id=1 → 1700（PRAD 1700）
   本步实测 1.10s / 预计 0.80s（若串行下发应为 1.60s）
── 第 2/2 步：2 条指令同时下发（预计 0.60s）
   本步实测 0.93s / 预计 0.60s（若串行下发应为 1.20s）
全部完成：1 遍 × 2 步，实测 2.03s（理论并行 1.40s，理论串行 2.80s）
```

实测 1.10 s ≈ 最长那条的 0.8 s（不是 1.6 s）→ 证明确实同时下发。

---

## 10. 踩坑记录（详见 `调试记录.md`）

| # | 现象 | 结论 |
| --- | --- | --- |
| 1 | 发 `#001PID!` 永远无回显 | 舵机 ID 是 **000**，扫 0~7 才确认 |
| 2 | 多舵机同一步播放时串口收到**重复指令**，播放器卡死 | 回声抑制只存"最后一条"，并发时互相覆盖 → 改待消费队列 |
| 3 | 取消 6 秒的 goal 后，舵机要 6 秒才挪回中心 | 回中位沿用了原 goal 的 duration → 新增 `cancel_return_ms` |
| 4 | `demo:=true` 一起启动客户端立刻被拒 | launch 里写死 `--id 1`，本机是 0 |
| 5 | **真机取消后舵机锁死**（查询正常、转动全无响应，只能断电） | 运动中被塞新目标，板子锁输出。**已修**：取消流程改为 `PULK` 卸力 → 等 150 ms → `PULR` 上扭力 → 再回中位；复测取消后仍可正常接收新指令（详见调试记录第 7、8 条） |
| 6 | 串口突然报 `No such file` / `Input/output error` | CH340 重枚举，`ttyUSB0` ↔ `ttyUSB1`；**用 udev 规则固定成 `/dev/ttySERVO`** |

---

## 11. 下电三步（按考核 4.1 的固定顺序）

```text
① 卸力  →  ② 断动力电（电池）  →  ③ 关板卡
```

| 步 | 做什么 | 为什么必须是这个顺序 |
| --- | --- | --- |
| **① 卸力** | 所有舵机发 `#xxxPULK!` | 舵机断电前还在顶力矩，直接断电极易堵转发热 |
| **② 断电池** | 拔动力电 | 先断动力，避免控制板在异常电压下乱发指令 |
| **③ 关板卡** | `sudo shutdown now` | 直接拔电会损坏文件系统，下次开不了机 |

实际命令：

```bash
# ① 卸力：先停驱动释放串口，再对所有舵机发 PULK
$ python3 ~/tools/cleanup_ros.py
$ python3 ~/tools/power_off.py            # 会打印 10 个舵机的 PULK 回包

# ② 拔电池（舵机此时已松，不会堵转）

# ③ 关板卡 / 上位机
$ sudo shutdown now
```

> ⚠️ `power_off.py` 一跑舵机就变软 —— 整机站着会塌，**先用手扶住或放倒再跑**。

### 四条红线

1. **禁止带电插拔舵机线** —— 浪涌会烧舵机驱动或控制板通道，这是最常见的人为损坏
2. **禁止反接电源** —— 上电前必须量电压、看极性
3. **电池鼓包立即停用** —— 交给队长，不要"再充一次看看"；充电必须有人看管
4. **禁止让舵机硬顶结构** —— 持续"嗡嗡"就是堵转，立刻回中位

**出现带电插拔 / 反接 → 整周不通过。**

---

## 12. 录屏链接（B 站）

| # | 内容 | 链接 |
| --- | --- | --- |
| 1 | 任务 2 · 正反转 + 反馈（7 s） | https://b23.tv/BV1jJyxzhEuG |
| 2 | 任务 2 · 改参数当场生效（111 s） | https://b23.tv/BV12oyVz2Efk |
| 3 | 任务 2 · 取消打断 Ctrl+C（55 s） | https://b23.tv/BV1Dhy5zWEH2 |
| 4 | 任务 3 · 逐个控制每个舵机（37 s） | https://b23.tv/BV18hy5zWEsk |
| 5 | 任务 3 · 整机与多舵机动作组（63 s） | https://b23.tv/BV1hby5zREfa |
| 6 | 任务 4 · 安全下电 PULK→断电池→关机（41 s） | https://b23.tv/BV1Yxy5zoEp2 |

> bag 回放那段录屏待补：提交时两条链接指向了同一个视频（`cw1oPmE` 与 `umY4wuq` 都是 BV1Dhy5zWEH2）。
> 仓库里只放截图与 bag（≤10 MB），视频一律走 B 站。
