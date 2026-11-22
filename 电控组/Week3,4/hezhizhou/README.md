# 第 3、4 周 · 整机拼装与舵机控制

姓名拼音：`hezhizhou`
提交日期：2026-11-22

---

## 1. 架构图

```
 ros2 action send_goal /move_servo
 （Action 客户端：命令行 或 move_client）
              │  Goal / Feedback / Result
              ▼
 ┌──────────────────────────────────────────────┐
 │ 包 cquick_servo  ·  节点 servo_driver         │
 │                                              │
 │  goal_callback    准入校验（不碰串口）        │
 │  cancel_callback  同意取消                    │
 │  execute_callback 150ms 循环：查取消/发反馈    │
 │                                              │
 │  move() ── 订阅话题 /servo/command（供 bag）  │
 │         ── 发布话题 /servo/state（实际下发）  │
 │         └─ 串口读写统一由 _io_lock 串行化      │
 └───────────────────┬──────────────────────────┘
                     │ 串口 /dev/cq_servo  115200 8-N-1
                     ▼
            24 路控制板（STM32）
                     │ 总线，按 ID 寻址
                     ▼
            舵机 ZP20D（通道 1~10）
```

| 项 | 名称 |
|---|---|
| 包 | `cquick_servo_interfaces`（接口）、`cquick_servo`（节点与配置） |
| 节点 | `servo_driver`（驱动）、`move_client`（交互客户端）、`sequence_player`（动作组播放器） |
| Action | `/move_servo`，类型 `cquick_servo_interfaces/action/MoveServo` |
| 话题（输入） | `/servo/command`，类型 `cquick_servo_interfaces/msg/ServoCommand` |
| 话题（输出） | `/servo/state`，同一类型 |

---

## 2. 参数表

| 参数 | 默认值 | 为什么是这个默认值 |
|---|---|---|
| `port` | `/dev/cq_servo` | udev 固定名。内核分配的 `ttyUSBx` 会随拔插和重启漂移，写死会导致换环境后打不开 |
| `baudrate` | `115200` | 控制板默认波特率；实测该值下 `PRAD` 回包稳定 |
| `pwm_min` | `1000` | 实测下限：向下试探到 1000 仍平稳无堵转，取此值并留余量 |
| `pwm_max` | `2000` | 实测到 2500 仍可正常转动（2500 为协议允许的最大值），但按"临界值回退"原则留出 500 的余量，防止结构受力变形或电池电压下降后堵转 |
| `default_duration_ms` | `1600` | 实测 800~900 时舵机不动：时间太短 → 瞬时电流过大 → 电池电压塌陷 → 舵机保护性停转。1600 以上可稳定到位 |
| `servo_ids` | `[1..10]` | 整机实际接入的通道；双足每条腿 5 个自由度。单舵机调试阶段设为 `[0]` |
| `startup_reset` | `false` | 10 个舵机同时回中位会拉出电流尖峰，先把功能验证稳定再改回 `true` |

---

## 3. 接口契约

`cquick_servo_interfaces/action/MoveServo`

### Goal

| 字段 | 类型 | 含义 |
|---|---|---|
| `id` | `uint8` | 舵机通道号，板卡为 0 基（0~23）；整机使用 1~10，单舵机调试使用 0 |
| `pwm` | `uint16` | 目标位置 |
| `duration_ms` | `uint16` | 期望运行时间（毫秒） |

发送时机：客户端每次下发新目标时，发送 1 次。

### Feedback

| 字段 | 类型 | 含义 |
|---|---|---|
| `current_pwm` | `uint16` | 当前所处位置 |
| `progress` | `float32` | 进度，0~1 |

发送时机：执行期间每 150 ms 发 1 次（约 6~7 Hz，满足"至少 5 Hz"的要求），直到动作结束或被取消。

### Result

| 字段 | 类型 | 含义 |
|---|---|---|
| `success` | `bool` | 是否成功到位 |
| `final_pwm` | `uint16` | 读 `PRAD` 得到的**实际**停止位置 |
| `message` | `string` | `ok` / `canceled` / `PRAD 读回失败` / `未到位：目标 X 实际 Y` / `执行异常：...` |

发送时机：每次 goal 结束时发送 1 次，且必然以 `succeed()` / `abort()` / `canceled()` 三者之一收尾。

> `Goal.pwm` 是"要求去哪"，`Result.final_pwm` 是"实际停在哪"，两者差值超过 30 判为未到位。

---

## 4. 一条启动命令

```bash
ros2 launch cquick_servo servo.launch.py
```

输出：

```
[INFO] [launch]: All log files can be found below /home/potato/.ros/log/2026-11-21-15-29-01-858385-potato-virtual-machine-50478
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [servo_driver-1]: process started with pid [50479]
[servo_driver-1] [INFO] [1795246142.611668724] [servo_driver]: 串口已打开：/dev/cq_servo
[servo_driver-1] [INFO] [1795246142.666719812] [servo_driver]: 已回中位：[0]
[servo_driver-1] [INFO] [1795246142.678573155] [servo_driver]: servo_driver 已就绪
```

参数覆盖：

```bash
ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB3
ros2 launch cquick_servo servo.launch.py demo:=true      # 额外启动 move_client
```

yaml 路径由 `get_package_share_directory('cquick_servo')` 拼出，不使用相对路径。

---

## 5. 安全区间是多少、怎么试出来的

### 方法

前提：舵机**空载**（不装结构或让关节离开地面）。

```
① 从 1500 出发，每次 +30，发 #00xP<值>T600!
② 发完听声音：
     平稳转动        → 继续
     持续"嗡——"     → 到达限位，记录前一个值
     "咯吱"声        → 立刻断电，齿轮打滑
③ 从临界值往回退一段余量（向上实测退 500，见下表）
④ 从 1500 向另一方向重复，得到下限
⑤ 装配进结构后重测一次（结构限位比舵机本身窄）
```

### 关节方向实测（双足，每腿 5 自由度）

| ID | 关节 | 1800 时方向 | 安装方式 |
|---|---|---|---|
| 5 | 左髋（左右） | 往外（左） | 同向：5、6 写相同值 |
| 6 | 右髋（左右） | 往内（也是左） | |
| 4 | 左大腿（上下） | 往后 | 镜像：7 = 3000 − 4 |
| 7 | 右大腿（上下） | 往前 | |
| 3 | 左小腿（上下） | 往后 | 镜像：8 = 3000 − 3 |
| 2 | 左踝（上下） | 往后 | 镜像：9 = 3000 − 2 |
| 8 | 右小腿（上下） | 往前 | |
| 9 | 右踝（上下） | 往前 | |
| 1 | 左脚掌（左右） | 往外（左） | 镜像：10 = 3000 − 1 |
| 10 | 右脚掌（左右） | 往外（右） | |

**镜像公式：`pwm_右 = 3000 − pwm_左`。** 髋部例外，两侧同值。

### 实测结果

| 方向 | 临界值 | 回退后取值 |
|---|---|---|
| 向上 | 2500（协议上限，未出现堵转异响） | **2000**（留 500 余量） |
| 向下 | 1000 | **1000** |

结论：`pwm_min = 1000`，`pwm_max = 2000`。

> 说明：向上测到 2500（协议最大值）仍未出现堵转，但实际把上限设为 2000。原因是舵机工作在接近量程末端时力矩下降、电流上升，结构受力或电池电压下降后容易堵转；留 500 余量比取满量程更安全。动作组里实际使用的幅度进一步收在 1250~1750。

---

## 6. 运行时改参数的演示记录

| 步 | 命令 | 实际输出 |
|---|---|---|
| ① | `ros2 param get /servo_driver pwm_max` | `Integer value is: 2000` |
| ② | `ros2 action send_goal /move_servo ... "{id: 0, pwm: 1700, duration_ms: 1000}"` | `Result: success: true, final_pwm: 1686, message: ok`（`Goal finished with status: SUCCEEDED`） |
| ③ | `ros2 param set /servo_driver pwm_max 1600` | `Set parameter successful` |
| ④ | 再次发送与 ② 相同的 goal | `Goal was rejected.`（节点终端：`拒绝：pwm 1700 超出安全区间 1000~1600`） |
| ⑤ | `ros2 param set /servo_driver pwm_max 100000` | `Setting parameter failed: pwm_max 必须在 500~2500，收到 100000` |
| ⑥ | `ros2 param set /servo_driver pwm_max 2000` + 再次发送 goal | `Set parameter successful`，随后 `Result: success: true, final_pwm: 1694, message: ok` |

> 本节演示使用单舵机（通道 0），便于聚焦参数行为；第 9 节的动作组使用整机的 1~10 号通道。

第 ④ 步是核心：不重启节点，改完参数后同一个 goal 立刻被拒，说明 `goal_callback` 每次实时读取 `get_parameter()`，而不是使用 `__init__` 中保存的副本。

完整的原始终端输出见 `02_param_set现场.png` 与 `02_限位保护.png`（含 `Goal was rejected` 和 `Setting parameter failed` 原文）。

---

## 7. bag 里录了什么、怎么回放、回放结果

### 录制内容

| 话题 | 方向 | 内容 |
|---|---|---|
| `/servo/command` | 输入 | 要执行的舵机指令 |
| `/servo/state` | 输出 | 实际下发到串口的指令 |

未使用 `-a`，只录上述两个话题。

> 注意：`rosbag2` 只订阅"当前存在发布者"的话题。`/servo/command` 平时只有订阅者，因此必须先发一条占位消息，看到 `Subscribed to topic '/servo/command'` 之后再发正式指令。

### 录制与回放

```bash
rm -rf bag/run1
ros2 bag record -o bag/run1 /servo/command /servo/state
# 另开终端，先发一条把订阅叫醒（这条同时也是"回中位"）
ros2 topic pub --once /servo/command cquick_servo_interfaces/msg/ServoCommand \
  "{id: 0, pwm: 1500, duration_ms: 2000}"
# 回到录制终端，等出现 Subscribed to topic '/servo/command' 后再发下一条
ros2 topic pub --once /servo/command cquick_servo_interfaces/msg/ServoCommand \
  "{id: 0, pwm: 1300, duration_ms: 2000}"
ros2 bag info bag/run1
ros2 bag play bag/run1
```

> 录制时节点的 `servo_ids` 为 `[0]`（单舵机阶段），所以 bag 里全部是 0 号通道；整机阶段改为 `[1..10]`。

### `ros2 bag info` 输出

```
Files:             run1_0.db3
Bag size:          25.4 KiB
Storage id:        sqlite3
Duration:          20.067461542s
Start:             Nov 22 2026 18:24:42.417645933
End:               Nov 22 2026 18:25:02.485107475
Messages:          5
Topic information: Topic: /servo/command | Type: cquick_servo_interfaces/msg/ServoCommand | Count: 2
                   Topic: /servo/state   | Type: cquick_servo_interfaces/msg/ServoCommand | Count: 3
```

目录体积：`32K`（要求 ≤10 MB，时长 20.1 s 要求 ≤30 s，均满足）。

### 回放结果

执行 `ros2 bag play bag/run1` 后，`/servo/command` 上记录的两条指令按原时间间隔重放：第 0.09 s 发 1500（回中位），第 19.8 s 发 1300，舵机随之先回中位、再转到 1300，与录制时的动作一致。

> 数据说明：`/servo/command` 只记到 2 条，是因为 bag 里只有"经话题发出"的指令——1500（占位兼回中位）和 1300；中间那次 1700 是用 Action `send_goal` 发的，不经过话题，因此只出现在 `/servo/state` 里。`/servo/state` 记到 3 条，对应三次实际下发（1500、1700、1300）——Action 与话题最终都调用同一个 `move()`。这也决定了回放能复现的只有话题上那两条。

> 说明：驱动节点真实订阅 `/servo/command`，且与 Action 走同一个 `move()`，因此回放动作与录制时一致。

---

## 8. 接线说明

### USB-TTL → 控制板

| USB-TTL | 控制板 |
|---|---|
| TXD | RXD |
| RXD | TXD |
| GND | GND |
| VCC | 不接 |

### 电池 → 控制板

```
电池正极（红）→ 控制板电源输入 +
电池负极（黑）→ 控制板电源输入 −
```

### 舵机 → 控制板

| 线色 | 接到 |
|---|---|
| 橙 / 白（信号） | 通道信号脚 |
| 红（正极） | 通道 + |
| 棕 / 黑（地） | 通道 − |

控制板通道编号为 **0~23**（本机实际使用 1~10），与指令中的 ID 一一对应。

### 电池实测电压

| 测点 | 测量方式 | 读数 |
|---|---|---|
| 电池输出（刚充满，空载） | 万用表直流档，红黑线两端 | **8.38 V**（见 `01_电池电压.png`） |
| 整机上电前 | 万用表直流档，整机线束两端 | **8.4 V**（见 `03_上电前电压.png`） |
| 电池输出（工作一段时间后，空载） | 万用表直流档，红黑线两端 | **8.0 V** |
| 舵机接口 +/− | 舵机自带 `#000PRTV!` 指令 | **7.9 V**（同时读到温度 11.6 ℃） |

> 本固件的 `PRTV` 返回格式为 `#000@T116V079!`，即 `T` 后三位为温度×10（116 → 11.6 ℃），`V` 后三位为电压×10（079 → 7.9 V）。
>
> 动作瞬间的压降未单独用万用表捕捉，但从"`duration_ms` 过短即停转、加长后恢复"的现象可以确认存在明显压降——这正是把 `duration_ms` 统一提到 1600 ms 以上的原因。

### udev 固定设备名（加分项）

规则文件 `99_cq_servo.rules`，内容：

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="cq_servo", MODE="0666"
```

安装：

```bash
sudo cp 99_cq_servo.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/cq_servo
```

验证：拔插 USB 与重启后，`/dev/cq_servo` 均指向当前 `ttyUSBx`，`port` 参数默认值即为该固定名。

---

## 9. 动作 yaml 长什么样

### 示例（`config/actions/squat.yaml`）

```yaml
name: squat
description: 双腿大幅下蹲再起立

steps:
  - name: 下蹲
    joints:
      - {id: 4, pwm: 1250, duration_ms: 2200}
      - {id: 3, pwm: 1750, duration_ms: 2200}
      - {id: 2, pwm: 1420, duration_ms: 2200}
      - {id: 7, pwm: 1750, duration_ms: 2200}
      - {id: 8, pwm: 1250, duration_ms: 2200}
      - {id: 9, pwm: 1580, duration_ms: 2200}

  - name: 蹲住
    joints:
      - {id: 4, pwm: 1250, duration_ms: 500}
      - {id: 3, pwm: 1750, duration_ms: 500}
      - {id: 7, pwm: 1750, duration_ms: 500}
      - {id: 8, pwm: 1250, duration_ms: 500}

  - name: 起立
    joints:
      - {id: 4, pwm: 1500, duration_ms: 2200}
      - {id: 3, pwm: 1500, duration_ms: 2200}
      - {id: 2, pwm: 1500, duration_ms: 2200}
      - {id: 7, pwm: 1500, duration_ms: 2200}
      - {id: 8, pwm: 1500, duration_ms: 2200}
      - {id: 9, pwm: 1500, duration_ms: 2200}
```

### step 怎么切

| 需求 | 写法 |
|---|---|
| 多个关节同时动 | 写在同一个 `joints` 列表里 |
| 先后动作 | 拆成多个 `steps` |
| 一步之内需要先后 | 单独再开一个 step，不靠调 `duration_ms` 硬凑 |

本动作"下蹲 → 蹲住 → 起立"是三个 step，中间"蹲住"只给 500 ms，作用是制造一个可见停顿，而不是靠拉长 `duration_ms` 制造迟滞。

### 谁先动

`steps` 从上到下依次执行；同一个 step 内的关节同时下发。

### `duration_ms` 怎么定

**由电流决定，不只是由行程决定。**

| 幅度 | `duration_ms` | 说明 |
|---|---|---|
| ±100 | 1600 | 最低可用值 |
| ±250 | 2200 | 本机实际使用 |
| ±400 | 2600 以上 | 需先确认电压不塌 |

实测 `duration_ms = 800/900` 时舵机**完全不动**（`final_pwm` 停在原值，手推有阻力），原因是动作太快导致瞬时电流过大、电池电压塌陷、舵机进入保护；把时间拉长后恢复正常，因此本项目统一用 ≥1600 ms。

### 怎么用 result 同步

播放器对每个 step 内的所有关节发送 goal，然后**等待每个 goal 的 result 返回**，全部返回后才进入下一个 step。全程不使用 `sleep`，所以连跑多遍动作一致。

```bash
ros2 run cquick_servo sequence_player --ros-args \
  -p action_file:=squat.yaml \
  -p actions_dir:=$HOME/cq_ws/src/cquick_servo/config/actions
```

换动作只改 `action_file`，不改 Python 代码。

---

## 10. 踩坑记录

| # | 现象 | 我的怀疑 | 怎么验证 | 结论 |
|---|---|---|---|---|
| 1 | `ls /dev/ttyUSB*` 为空，但 `lsusb` 能看到 CH340 | 驱动没装或线是充电线 | `sudo dmesg \| tail -20`，看到先 attach 到 ttyUSB0、0.6 秒后 `claimed by ch341 while 'brltty' sets config` 并 disconnected | Ubuntu 预装的 brltty 也匹配 `1a86:7523`，mask 掉 `brltty-udev.service` 与 `brltty.service` 后正常 |
| 2 | 发任何指令都无回包 | 波特率不对 | 做回环测试：短接 TX/RX，能收到自己发的字节 | 回环正常即模块与波特率无问题，故障在控制板侧接线 |
| 3 | `colcon build` 报 `list index: 1 out of range (-1, 0)` | 代码或 `CMakeLists.txt` 写错 | 报错路径是 `/Week3,4/...`，缺少前面的中文目录名 | 工作空间位于含中文和逗号的路径，rosidl 解析时截断；移到 `~/cq_ws` 后编译通过 |
| 4 | `ros2 bag record` 等 21 秒才订阅，只录到 1 条 | 消息发得太快 | 对比 `/servo/state` 立刻被订阅（它有常驻发布者） | `rosbag2` 只订阅存在发布者的话题；先发占位消息、看到 `Subscribed` 后再发正式指令 |
| 5 | 脚本调用 `flush()` 卡死 | 串口没打开 | 加打印定位，停在 `termios.tcdrain` | 该 CH340 在此环境下 `tcdrain` 不返回；改为轮询 `out_waiting` 并设 0.2 s 上限 |
| 6 | 重启虚拟机后 `ttyUSB0` 变成 `ttyUSB1`，节点打不开串口 | 设备损坏 | `lsusb` 仍能看到 `1a86:7523`，`dmesg` 显示重新枚举 | 序号由内核动态分配；用 udev 规则绑定固定名 `/dev/cq_servo` |
| 7 | 6 个关节同时下蹲，部分 `success=False`、`message` 为空、`final_pwm` 是别的舵机的值、出现 `PRAD 读回失败` | 串口被多个 `execute_callback` 并发访问 | 单舵机正常、多舵机出错，且读回值张冠李戴 | 串口是独占资源：`read_pwm` 的清缓冲→发指令→读回包必须整体加锁（`RLock`），executor 线程数提到 12；修后 16 个 goal 全部成功 |
| 8 | 目标 1400/1620 完全不动，`final_pwm` 停在 1490~1500，手推有阻力，`PRAD` 正常 | 舵机锁死或没供电 | 量舵机接口 8 V（有电）；把 `duration_ms` 由 900 改为 2000 后到达 1682 | 短时间大幅动作瞬时电流过大 → 电池电压塌陷 → 舵机保护性停转；`duration_ms` 需 ≥1600 |
| 9 | 16 个 goal 在 0.15 s 内全部失败，`message` 全空 | 串口句柄失效 | `ls /dev/cq_servo` 不存在，`dmesg` 显示 USB 断连 | USB-TTL 掉线，节点持有的 fd 失效；需重新插拔并重启节点。代码中加 `try/except` 兜底，把异常原因写进 `message`，避免 Result 全空无从判断 |

详细排查过程见 `调试记录.md`。

---

## 11. 下电三步 + 每一步的原因

```
① 所有舵机回 1500 中位，让结构不受力
② 拔电池（断动力电）
③ sudo shutdown now（关板卡）
```

| 步 | 原因 |
|---|---|
| ① 回中位 | 断电前若关节仍在顶力矩，直接断电极易堵转发热、损伤齿轮和线圈。把各关节回到 1500 使结构处于自然姿态，断电时不再受力 |
| ② 再断动力 | 先断动力可避免控制板在电压异常跌落时乱发指令 |
| ③ 最后关板卡 | 直接拔电可能损坏文件系统，导致下次无法启动 |

---

## 12. 录屏链接（B 站）

| # | 内容 | 链接 |
|---|---|---|
| 1 | 单舵机正转 / 反转 / 回中位 | https://www.bilibili.com/video/BV1M5yVzjEPG |
| 2 | 运行时改参数立刻生效（六步演示） | https://www.bilibili.com/video/BV1T5yVzjEqE |
| 3 | Action 取消打断 | https://www.bilibili.com/video/BV1T5yVzjEjq |
| 4 | bag 回放复现动作 | https://www.bilibili.com/video/BV1tGyVzmEdB |
| 5 | 整机逐个动作（1~10 号各单独动一次） | https://www.bilibili.com/video/BV1QgyLzoEAr |
| 6 | 动作组连跑三遍 + 换一套 yaml | https://www.bilibili.com/video/BV1auyLztEmn |
| 7 | 安全下电流程 | https://www.bilibili.com/video/BV1bLyLzWEtm |
