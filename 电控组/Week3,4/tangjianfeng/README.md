# 电控组 第 3、4 周：整机拼装与舵机控制（tangjianfeng）

包含 `cquick_servo_interfaces` 和 `cquick_servo` 两个 ROS 2 包，以及一套 10 舵机双足整机的联调与动作组实现。任务 1、2 先在单舵机上打通串口驱动与 Action 链路，任务 3、4 扩展到 10 舵机整机完成多关节协同动作组和安全下电。

---

## 一、硬件阶段、接线与 bag 回放

### 接线与供电

上位机运行 Ubuntu 22.04 + ROS 2 Humble，通过 USB-TTL（`/dev/ttyUSB0`，`115200` 8-N-1，当前用户已在 `dialout` 组，无需 `sudo`）接入 24 路舵机控制板，TX/RX 交叉连接并与控制板共地（GND）。舵机动力由 2S 锂电池提供（满电 8.4 V），上电前万用表实测电压为 `7.73 V`（见 `01_电池电压.jpg`）。整机 10 颗总线舵机串联挂在同一条总线上。

### 从单舵机到整机的 ID 变化

- **任务 1、2（单舵机阶段）**：新舵机出厂默认 ID 都是 `000`。多个同 ID 舵机一起挂总线会同时应答并撞包，因此单点点亮和 Action 联调时每次只接一颗舵机，`servo_ids` 为 `[0]`。
- **任务 3、4（整机阶段）**：10 颗舵机逐个单独接入并改写 ID 为 `1`~`10`，全部挂上总线后扫描在线为 `ONLINE IDS: [1, 2, ..., 10]`。仓库里的 `config/servo_params.yaml` 保存的是整机配置（`[1, 2, ..., 10]`）。

### `bag/run1/` 内容与回放

`bag/run1/`（`bag/run1/metadata.yaml` + `bag/run1/bagrun_0.db3`，共 66 条消息，时长约 29.5 s）录制于任务 2 单舵机阶段，内含 3 条 `/servo/command`（`id=0`，目标 PWM 依次为 `1200`、`1700`、`1500`，`duration_ms` 均为 `1500`）和 63 条 `/servo/state`。

在整机上直接执行 `ros2 bag play bag/run1` 时舵机不会动作。`_on_command` 回调只检查 500~2500 电气范围，不校验 ID 白名单，驱动会把 `#000P...T...!` 原样发上总线；因为整机舵机已改为 1~10，总线上已没有 0 号设备，所以没有舵机应答。

复现这段 bag 可以用下面任一方式（`/servo/command` 不检查 `pwm_min/pwm_max`，回放前需确认当前安全限位与录制时一致）：

```bash
# 方式 A：单独接回一颗出厂 ID 为 0 的舵机（servo_ids 参数校验已允许 0），将其加入列表后回放
ros2 param set /servo_driver servo_ids "[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]"
ros2 bag play bag/run1

# 方式 B：不改动整机舵机 ID，直接向 3 号舵机发送等价的三条指令
ros2 topic pub -1 /servo/command cquick_servo_interfaces/msg/ServoCommand \
  "{id: 3, pwm: 1200, duration_ms: 1500}"
ros2 topic pub -1 /servo/command cquick_servo_interfaces/msg/ServoCommand \
  "{id: 3, pwm: 1700, duration_ms: 1500}"
ros2 topic pub -1 /servo/command cquick_servo_interfaces/msg/ServoCommand \
  "{id: 3, pwm: 1500, duration_ms: 1500}"
```

---

## 二、包结构与编译运行

### 节点与通信拓扑

```text
cquick_servo_interfaces (ament_cmake)
  ├── msg/ServoCommand.msg
  └── action/MoveServo.action

cquick_servo (ament_python)
  ├── servo_driver      ──▶ Action Server: /move_servo (MoveServo)
  │                     ◀── Sub: /servo/command (ServoCommand, bag 回放/调试)
  │                     ──▶ Pub: /servo/state   (ServoCommand, 2 Hz)
  ├── move_client       ──▶ Action Client: /move_servo (单步 / --sweep 正反转)
  └── sequence_player   ──▶ Action Client: /move_servo (读 yaml 播动作组，别名 action_group)
```

### 编译与启动命令

```bash
cd ~/ros2_ws
colcon build --packages-select cquick_servo_interfaces cquick_servo
source install/setup.bash

# 启动驱动节点（参数来自 config/servo_params.yaml）
ros2 launch cquick_servo servo.launch.py

# 覆盖串口设备，或同时启动客户端演示正反转回中位
ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB1
ros2 launch cquick_servo servo.launch.py demo:=true demo_pwm:=1800

# 发送单条 Action 目标并查看 feedback
ros2 action send_goal /move_servo cquick_servo_interfaces/action/MoveServo \
  "{id: 3, pwm: 1700, duration_ms: 1200}" --feedback

# 播放动作组（两个命令等价）
ros2 run cquick_servo action_group --action stand_up
ros2 run cquick_servo sequence_player --action stand_up
```

`setup.py` 里将 `action_group` 和 `sequence_player` 两个入口名都指向 `cquick_servo.sequence_player:main`。调试时习惯用 `action_group`（日志前缀为 `[action_group]`），代码文件名为 `sequence_player.py`。

---

## 三、接口契约与话题

`msg/ServoCommand.msg`（用于 `/servo/command` 与 `/servo/state`）：

| 字段 | 类型 | 说明 |
|---|---|---|
| `id` | `uint8` | 舵机 ID，下发协议时格式化为 3 位数字（如 `#003`） |
| `pwm` | `uint16` | 目标脉宽，电气范围 500~2500；安全行程由 `pwm_min/pwm_max` 控制 |
| `duration_ms` | `uint16` | 运动耗时（毫秒），为 0 时使用 `default_duration_ms` |

`action/MoveServo.action`：

| 段 | 字段 | 说明 |
|---|---|---|
| Goal | `id` / `pwm` / `duration_ms` | 单条运动请求 |
| Result | `success` | 是否到达目标容差范围内 |
| | `final_pwm` | 动作结束后 `PRAD` 读回的实际位置（未成功时也会填入） |
| | `message` | `OK: target x, read y, diff z` / `canceled` / `NOT REACHED: ...` / `serial error: ...` |
| Feedback | `current_pwm` / `progress` | 运动过程中的插值位置与进度，发布频率 10 Hz |

话题与 Action 方向：

| 接口 | 方向 | 用途 |
|---|---|---|
| `/move_servo` | client → server | 动作组播放器、`move_client` 及手动控制入口 |
| `/servo/command` | 外部 → `servo_driver` | 话题调试与 bag 回放 |
| `/servo/state` | `servo_driver` → 外部（2 Hz） | 状态监看与 bag 录制 |

`/servo/command` 作为回放和调试通道，仅检查 500~2500 电气极值而不检查 `pwm_min/pwm_max` 安全限位，正式控制统一走 `/move_servo`。

---

## 四、参数与安全保护

### 参数表（`config/servo_params.yaml`）

| 参数 | 默认值 | 说明与取值依据 | 校验规则 |
|---|---|---|---|
| `port` | `/dev/ttyUSB0` | USB-TTL 串口设备路径 | — |
| `baudrate` | `115200` | 与舵机出厂串口波特率一致，不匹配会收乱码或超时无回包 | `9600/19200/38400/57600/115200/230400` |
| `pwm_min` | `900` | 安全下限，越界 goal 直接 REJECT | `500~2500` |
| `pwm_max` | `2100` | 安全上限，越界 goal 直接 REJECT | `500~2500`，且大于 `pwm_min` |
| `default_duration_ms` | `800` | goal 未填时长时的默认平滑时间 | `1~30000` |
| `servo_ids` | `[1..10]` | 整机在线舵机 ID 列表 | 非空，`0~254`（允许 `0` 以便回放单舵机阶段的 bag） |
| `startup_reset` | `true` | 启动后把列表内舵机统一回中位 `(pwm_min+pwm_max)//2` | — |

### 安全区间来历与三层防护

舵机协议允许的电气极限脉宽是 `500~2500`，写在代码常量 `ELECTRIC_MIN` / `ELECTRIC_MAX` 里，不放进运行时参数。把舵机装进 3D 打印双足结构后，接近 500 或 2500 的极限角度会让连杆顶到相邻关节壳体，因此把安全行程收窄在中心位 `1500` 两侧各 `600` 的区间，即 `[900, 2100]`。

节点内分三层做检查：
- 协议层：`500~2500` 电气边界由常量限制，任何参数设置都不能超出。
- 结构层：`/move_servo` 的 `goal_callback` 检查目标 ID 是否在 `servo_ids` 中、PWM 是否在 `[pwm_min, pwm_max]` 内，越界直接拒绝。
- 参数回调层：`_on_param_set` 拦截非法单值（如波特率不在允许列表、`pwm_max` 超出 500~2500）以及组合冲突（如 `pwm_min >= pwm_max`）。

### 运行时修改参数与保护测试

`_pwm_min()` 和 `_pwm_max()` 每次判定都会重新调用 `get_parameter()` 读取参数，不在成员里缓存旧值，所以 `ros2 param set` 修改后不用重启节点，下一条 goal 就会按新限位执行。非法值则在参数服务回调里直接拒掉。

实测修改命令（对应 `02_限位_参数_自检.png`）：

```bash
ros2 param get /servo_driver pwm_max                 # Integer value is: 2100
ros2 param set /servo_driver pwm_max 1600            # Set parameter successful，之后发 pwm=1700 的 goal 会被拒
ros2 param set /servo_driver pwm_max 100000          # 被拒：out of electrical range 500~2500
ros2 param set /servo_driver pwm_min 3000            # 被拒：out of electrical range 500~2500
ros2 param set /servo_driver pwm_max 2100            # Set parameter successful，改回后发 1700 重新通过
```

越界、非法 ID 与非法参数验证结果（对应 `02_保护机制_越界_ID_参数.jpg`）：

```text
1) position out of range (pwm 3000 > 2100)        → Goal was rejected.
2) id not in servo_ids (id 99)                    → Goal was rejected.
3) param out of range (pwm_max 100000)            → Setting parameter failed:
                                                    pwm_max=100000 out of electrical range 500~2500
4) param out of range (pwm_min 3000)              → Setting parameter failed:
                                                    pwm_min=3000 out of electrical range 500~2500
5) restore legal value (pwm_max 2100)             → Set parameter successful
```

其中客户端打印 `Goal was rejected.` 时，驱动节点侧输出的对应日志为 `拒绝 goal：pwm=3000 超出安全范围 [900,2100]`。

---

## 五、动作组（YAML 与 result 同步）

动作组全部用 yaml 数据描述，播放器 `sequence_player.py` 负责读取 yaml、展开成 goal 并按顺序发送，切换或新增动作只需修改 yaml 文件，不需要改动 Python 代码。

时序按下面的规则组织：
- 步内并发：同一个 `step` 下的多个关节会同时调用 `send_goal_async` 下发，各自带目标 `pwm` 和 `duration_ms`。
- 步间串行：等上一步所有关节的 `result` 都返回后，才发送下一个 `step`。
- 步内先后：如果某个关节必须先动，直接拆成前一个独立 `step`，不靠修改 `duration_ms` 或固定 `sleep` 来硬凑延时。

以 `config/actions/wave.yaml` 为例，动作定义的格式如下：

```yaml
name: wave
description: 1 号关节 中位 → 正转 1700 → 反转 1300 → 回中位 1500
loop: false

steps:
  - name: 回中位起手
    joints:
      - {id: 1, pwm: 1500, duration_ms: 800}
  - name: 正转出去
    joints:
      - {id: 1, pwm: 1700, duration_ms: 900}
  - name: 反转回来
    joints:
      - {id: 1, pwm: 1300, duration_ms: 1200}
  - name: 回中位收尾
    joints:
      - {id: 1, pwm: 1500, duration_ms: 900}
```

仓库 `config/actions/` 下共放了三套可直接复现的动作：

| 文件 | 步数 | 内容 |
|---|---|---|
| `wave.yaml` | 4 | 1 号单关节测试（中位 → 正转 → 反转 → 回中） |
| `stand_up.yaml` | 3 | 四腿同时抬起 → 保持 → 落回中位 |
| `wave_hi.yaml` | 8 | 抬起一条腿 → 左右摆两次 → 放下 |

现场演示使用的是 `stand`（4 步）、`march`（8 步）和 `reset_all`（1 步），清单与播放日志见 `03_动作组清单与日志.jpg`。这三套动作的目标脉宽和实机装配零位绑定，没有直接放在仓库里，运行时用 `--action /绝对路径/xxx.yaml` 传入就能直接替换。

舵机协议里的 `T` 只是预期过渡时长，实际运动耗时受负载和齿轮回差影响，用固定 `sleep` 等待会在连续播放时产生累积偏移，因此播放器统一把 `result` 当成同步点。下面这段日志可以看到同一步三个关节的 `result` 在 490 ms 内陆续返回，全部返回后才打印播放完成：

```text
[INFO] [1795361282.265197726] [action_group]: step[脚掌放平] id=3 完成 success=True final_pwm=1506 message=OK: target 1500, read 1506, diff 6
[INFO] [1795361282.506914316] [action_group]: step[脚掌放平] id=1 完成 success=True final_pwm=1511 message=OK: target 1500, read 1511, diff 11
[INFO] [1795361282.756365085] [action_group]: step[脚掌放平] id=4 完成 success=True final_pwm=1501 message=OK: target 1500, read 1501, diff 1
[INFO] [1795361282.758617714] [action_group]: 动作组 stand 播放完成
```

---

## 六、Action 执行流程与线程处理

一条 `/move_servo` goal 在 `servo_driver` 内部经过以下几步：

```text
ros2 action send_goal /move_servo
        │  goal{id, pwm, duration_ms}
        ▼
[1] goal_callback ── 检查 id 是否在 servo_ids、pwm 是否在 [pwm_min, pwm_max]、
        │              duration 是否合法以及串口是否打开
        │              通过 → ACCEPT；不通过 → REJECT（日志打印具体原因）
        ▼
[2] handle_accepted ── 立即调用 goal_handle.execute() 交由执行器线程池处理
        ▼
[3] execute_callback
        │  ① #xxxPULR!            恢复扭力（未发此指令前舵机不响应位置命令）
        │  ② #xxxP<pwm>T<ms>!     下发目标位置和时长
        │  ③ 每 100 ms 发布 feedback 并同步发 /servo/state
        │  ④ 若收到 cancel 请求，发回中位并将 result.message 设为 "canceled"
        │  ⑤ 周期结束先等 350 ms，再发 #xxxPRAD! 读回实际位置（未进容差时最多读 3 次）
        │  |读回 - 目标| > 30 → result.message = "NOT REACHED: ..."，success=False
        ▼
   result{success, final_pwm, message}
```

驱动主节点使用 `MultiThreadedExecutor` 并为回调挂了 `ReentrantCallbackGroup`。`handle_accepted_callback` 只触发 `goal_handle.execute()`，不阻塞在回调里，也不在执行函数内自己调 `spin`。如果把执行循环堵在单线程回调里，同一执行器内的取消服务和 `/servo/command` 订阅就无法调度，会导致取消命令和 `Ctrl+C` 失去响应。目前取消标记在每 100 ms 一次的 feedback 循环中轮询检查。

并发控制上，动作组播放器本身按 step 串行等待 `result`，不会重叠下发。如果手动同时发两条 goal，串口收发由内部互斥锁保护，但目标位置会互相覆盖，当前版本没有实现 goal 抢占。

---

## 七、常见踩坑摘要

调试中主要碰到的几类问题如下，完整 9 条记录见 `调试记录.md`：

1. **多颗舵机出厂 ID 同为 `000` 引起总线冲突**：几颗新舵机直接并接上总线后，发 `#000PRAD!` 会收到粘连的多份回包，位置在 1491~1495 之间跳动，发运动指令也只有一个会动。对比接 1、2、4 颗时的回包数量，确认数量与在线舵机数一致，属于同 ID 抢应答。之后改为单颗接入，全部逐个改为 `1`~`10` 再并入整机。
2. **指令发出后立刻读 `PRAD` 读到旧位置**：发完 `#000P1500T1000!` 马上读会拿到上一次的值（1510）。按 100 ms 间隔连续读取后看到位置随时间单调爬升、总耗时与 `T` 基本一致，确认是指令异步执行。代码里改为跑完 feedback 周期再延时 `0.35 s` 读第一次 `PRAD`，超出容差时最多尝试 3 次。
3. **`PULK` 卸力后无法靠 `PULR` 恢复**：尝试在下电前自动发 `#000PULK!` 卸力，但卸力后再发 `#000PULR!` 或纯查询 `#000PID!` 都无回包，测电池电压 7.73 V 正常，只有断电重启才恢复。节点退出改为只关串口、不发 `PULK`，把卸力放在手动断电前一步。
4. **整机联调时 9 号舵机损坏**：动作组运行中 9 号关节反复不到位或不动，断电手动转动顺畅无刮线。核对 `servo_ids` 包含 9 且 `#009PID!` 能正常回包，将线束换到相邻接口复测后故障仍然跟着 9 号舵机走，单发动作 `PRAD` 读回偏差远超容差 30。按安全流程断电更换新舵机、单独改号为 9 并先发 `1500` 对准结构基准重装，重新扫 ID 并连跑三遍动作组后恢复。

---

## 八、安全下电（任务 4）

下电按固定顺序执行：

```text
① 全部舵机回中位（或播放 reset_all）
② 对每个舵机发送 #xxxPULK! 卸力
③ 拔掉电池（先断开动力电源）
④ 最后拔掉 USB-TTL 并关闭板卡
```

按这个顺序操作的原因：
- 先卸力再断电：避免舵机在顶住结构力矩的状态下突然断电，或带电强行掰动关节导致堵转和齿箱过热。
- 先断动力再拔信号线：反过来会让串口电平参考悬空，容易损坏控制板或舵机串口芯片。
- 不带电插拔舵机线：热插拔瞬间的电流浪涌很容易烧毁控制板通道或舵机驱动。

实测这批舵机收到 `PULK` 后会直接进入锁定状态，无法再通过 `PULR` 唤醒，必须重新断上电，因此第 ② 步卸力完成后不要再发动作指令，直接拔电池断电即可。

---

## 九、材料清单

### 任务 1 · 点亮一个舵机

| 文件 | 内容 |
|---|---|
| `01_串口设备.png` | `ls -l /dev/ttyUSB*`，属主组 `dialout`，无 root |
| `01_PID回显.png` | `#000PID!` → `#000P!`，`#000PRAD!` → `#000P1510!` |
| `01_报文记录_正转与回中位.png` | `PULR` + `P1700T1500` + `PRAD` 往返记录 |
| `01_中立位标定.png` | 舵机中位读回（1493 / 1495 / 1492 / 1491） |
| `01_电池电压.jpg` | 万用表测得电池电压 7.73 V |

### 任务 2 · ROS2 接管

| 文件 | 内容 |
|---|---|
| `02_接口编译与节点动作.png` | 接口编译及节点、Action 列表 |
| `02_保护机制_越界_ID_参数.jpg` | 越界、非法 ID、非法参数拒绝及恢复 |
| `02_限位_参数_自检.png` | 限位保护、`param set` 现场生效与硬编码 grep 自检 |
| `02_action_反馈_1.png`、`02_action_反馈_2.png` | 连续三条动作的 feedback 滚动与 result |
| `02_取消生效.png` | `cancel_return_code: 0`，`message=canceled` |
| `02_bag_info.png` | `ros2 bag info` 统计与回放输出 |
| `bag/run1/` | `bag/run1/metadata.yaml` + `bag/run1/bagrun_0.db3`（66 条消息 / 29.5 s） |

### 任务 3 · 整机 + 动作组

| 文件 | 内容 |
|---|---|
| `03_ID扫描.jpg` | 10 颗舵机全在线：`ONLINE IDS: [1..10]` |
| `03_动作组清单与日志.jpg` | `stand`（4 步）/ `march`（8 步）/ `reset_all`（1 步）及 result 时间戳 |
| `03_整机照片_正面.jpg` | 整机正面视图 |
| `03_整机照片_侧面.jpg` | 整机侧面视图 |
| `任务3.mp4`（录屏附件） | 45 s，横屏，终端日志与整机动作同框 |

### 任务 4 · 安全下电

| 文件 | 内容 |
|---|---|
| `任务4.mp4`（录屏附件） | 29.5 s，回中 → 卸力 → 拔电池 → 拔 USB |

### 录屏链接

视频文件不进 git 仓库（`.gitignore` 已忽略 `*.mp4`），其中任务 3、4 随交付附件包提供，全部四段演示已上传 B 站多 P 稿件（`BV1isyjzKEQU`）：

- 稿件总入口：<https://www.bilibili.com/video/BV1isyjzKEQU/>
- 任务 1（舵机点亮，p1 / 22 s）：<https://www.bilibili.com/video/BV1isyjzKEQU/?p=1>
- 任务 2（正反转回中位 / 取消打断 / 改参数生效 / `ros2 bag play` 复现，p2 / 66 s）：<https://www.bilibili.com/video/BV1isyjzKEQU/?p=2>
- 任务 3（动作组，p3 / 45 s）：<https://www.bilibili.com/video/BV1isyjzKEQU/?p=3>
- 任务 4（下电，p4 / 30 s）：<https://www.bilibili.com/video/BV1isyjzKEQU/?p=4>

---

## 十、已知问题与取舍

1. **读回偏差 ±13 与容差 30**：齿轮存在机械回差，正向走到 1700 读回 1691，反向走到 1200 读回 1207，10 颗舵机实测最大偏差为 ±13。因此把到位容差设为 30，若收窄到 5 左右会让正常运动频繁被判为 `NOT REACHED`。
2. **`bag/run1/` 的单舵机 ID 差异**：该 bag 录制于任务 2 单舵机阶段（`id=0`），整机配置默认 `servo_ids` 为 `1~10`，直接回放不会驱动整机舵机，可按第一节的两种方法复现。
3. **不支持 goal 抢占**：手动并发发送多个 goal 时，串口写通过内部锁串行化，但不会主动打断前一个未完成的 goal，目标位置会互相覆盖；动作组播放器靠串行等待 `result` 避免这一情况。
4. **`/servo/command` 仅检查电气范围**：该话题面向 bag 回放与底层调试，只拦截 `500~2500` 之外的值，不做 `pwm_min/pwm_max` 检查，回放前需确认限位设置与录制时一致。
5. **节点退出不自动发 `PULK`**：实测舵机收到 `PULK` 后会锁死直到断电重启，因此 `destroy_node()` 只负责关闭串口，卸力留到手动下电流程中执行。
