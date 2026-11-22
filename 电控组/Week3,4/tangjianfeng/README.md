# 电控组 第 3、4 周 · 整机拼装与舵机控制 - tangjianfeng

> 两个包 + 一台 10 舵机整机。
> 任务 1、2 在**单舵机**上打通链路，任务 3、4 复制到**整机**并做动作组与安全下电。

---

## 一、先说清楚硬件是怎么演进的

这一周跨了两个硬件阶段，`servo_ids` 的口径不一样，不是配置写错：

| 阶段 | 总线上有几个舵机 | ID | `servo_ids` | 为什么 |
|---|---|---|---|---|
| 任务 1 · 点亮 | 1 个 | `000`（出厂默认） | `[0]` | 新舵机到手先单点点亮，多个一起上总线会因同 ID 抢应答 |
| 任务 2 · ROS2 接管 | 1 个 | `000` | `[0]` | 沿用任务 1 的单舵机链路，先把 Action 跑通 |
| 任务 3 · 整机 | **10 个** | **`1`~`10`** | `[1, 2, ..., 10]` | 逐个改号后全部上总线，扫描确认在线 |

仓库里的 `config/servo_params.yaml` 是**最终的整机配置**（`[1..10]`）。

**所以 `bag/run1/` 里录的是 `id=0` 的指令**（任务 2 阶段、单舵机）。

这里要把原因说准确：`/servo/command` 是回放通道，**只做 500~2500 电气范围检查，不做 ID 白名单校验**
（`servo_driver.py` 的 `_on_command`）。回放时驱动把 `#000P...T...!` 原样发上总线，
而整机阶段的十个舵机已经改成 1~10，**总线上不再有 0 号设备**，没有任何舵机应答 ——
表现就是"bag play 了但舵机不动"。不是代码把消息丢了，是总线上没有这个 ID。

要复现那份 bag，两条路：

```bash
# 办法 A：把一颗舵机单独接回总线、改回出厂 ID 0（servo_ids 允许 0），再回放
ros2 param set /servo_driver servo_ids "[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]"
ros2 bag play bag/run1

# 办法 B：不接旧舵机，把等价的三条指令按整机 ID 重发（效果一样）
ros2 topic pub -1 /servo/command cquick_servo_interfaces/msg/ServoCommand \
  "{id: 3, pwm: 1200, duration_ms: 1500}"
ros2 topic pub -1 /servo/command cquick_servo_interfaces/msg/ServoCommand \
  "{id: 3, pwm: 1700, duration_ms: 1500}"
ros2 topic pub -1 /servo/command cquick_servo_interfaces/msg/ServoCommand \
  "{id: 3, pwm: 1500, duration_ms: 1500}"
```

> 回放通道不做 `pwm_min/pwm_max` 安全限位、也不查 ID —— 它是调试口，不是动作入口。
> 带限位和 ID 校验的是 `/move_servo` 这个 Action。回放前请确认当前限位与录制时一致。

---

## 二、怎么编、怎么跑

```bash
cd ~/ros2_ws
colcon build --packages-select cquick_servo_interfaces cquick_servo
source install/setup.bash

# 起驱动节点（参数从 config/servo_params.yaml 读）
ros2 launch cquick_servo servo.launch.py

# 发一条动作（带 feedback）
ros2 action send_goal /move_servo cquick_servo_interfaces/action/MoveServo \
  "{id: 3, pwm: 1700, duration_ms: 1200}" --feedback

# 播一套动作组（两个入口名等价，见下）
ros2 run cquick_servo action_group --action stand_up
ros2 run cquick_servo sequence_player --action stand_up
```

`action_group` 和 `sequence_player` 是**同一个节点**的两个入口名（`setup.py` 里指向同一个
`cquick_servo.sequence_player:main`）。现场调试时习惯用 `action_group`，日志前缀就是
`[action_group]`；交付代码的文件名按细则结构叫 `sequence_player.py`。

换串口不用改文件：

```bash
ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB1
```

---

## 三、接口：每个字段干什么

`msg/ServoCommand.msg`（只用于 `/servo/command` 和 `/servo/state`）

| 字段 | 类型 | 干什么 |
|---|---|---|
| `id` | `uint8` | 舵机 ID，协议里补成 3 位（`#003`） |
| `pwm` | `uint16` | 目标位置 500~2500，**这是电气范围**；安全范围由 `pwm_min/pwm_max` 决定 |
| `duration_ms` | `uint16` | 用多少毫秒转过去，0 表示用默认时长 |

`action/MoveServo.action`

| 段 | 字段 | 干什么 |
|---|---|---|
| Goal | `id` / `pwm` / `duration_ms` | 一条完整动作指令 |
| Result | `success` | 成没成 |
| | `final_pwm` | PRAD 读回的实际位置（失败时也填，能看出停在哪） |
| | `message` | `OK: target x, read y, diff z` / `canceled` / `NOT REACHED` / `serial error` |
| Feedback | `current_pwm` / `progress` | 过程中的位置与进度（10 Hz） |

**话题方向**

| 话题 | 方向 | 谁在用 |
|---|---|---|
| `/move_servo` | client → server | 动作组、客户端、手动调试 |
| `/servo/command` | 外部 → 驱动（订阅） | bag 回放 |
| `/servo/state` | 驱动 → 外部（发布，2 Hz） | bag 录制、监看 |

> ⚠️ `/servo/command` 只做**电气范围**检查（500~2500），不做 `pwm_min/pwm_max` 安全限位检查 ——
> 它是调试与回放通道。回放 bag 前请确认当前的安全限位与录制时一致。

---

## 四、参数（`config/servo_params.yaml`）

| 参数 | 值 | 说明 | 校验 |
|---|---|---|---|
| `port` | `/dev/ttyUSB0` | USB-TTL 设备 | — |
| `baudrate` | `115200` | 必须和舵机一致，错了回包是乱码或没有回包 | 只能是 9600/19200/38400/57600/115200/230400 |
| `pwm_min` | `900` | 安全下限，goal 低于它直接 REJECT | 500~2500 |
| `pwm_max` | `2100` | 安全上限，goal 高于它直接 REJECT | 500~2500，且 > `pwm_min` |
| `default_duration_ms` | `800` | goal 没写时长时用它 | 1~30000 |
| `servo_ids` | `[1..10]` | 整机在线的十个舵机 | 非空，0~254（`0` 是出厂默认 ID，留作回放旧 bag） |
| `startup_reset` | `true` | 启动全部回中位（中位 = `(pwm_min+pwm_max)/2`） | — |

电气范围 500~2500 写在协议常量里（`ELECTRIC_MIN` / `ELECTRIC_MAX`），不进参数 ——
那是硬件决定的，不是"可调的安全范围"。

参数当帧生效，改完不用重启：

```bash
ros2 param get /servo_driver pwm_max                 # 2100
ros2 param set /servo_driver pwm_max 1600            # 成功 → 之后 pwm=1700 的 goal 立刻被拒
ros2 param set /servo_driver pwm_max 100000          # 被拒：out of electrical range 500~2500
ros2 param set /servo_driver pwm_min 3000            # 被拒：同上
ros2 param set /servo_driver pwm_max 2100            # 改回来，再发 1700 又能过
```

之所以能当帧生效：`pwm_min()` / `pwm_max()` 每次都是 `get_parameter` 现取，没有任何缓存。
`_on_param_set` 回调负责拒掉非法值，包括 `min >= max` 这种"单个值合法、组合非法"的设置。

---

## 五、动作组：动作是数据，不是代码

**换一套动作只改 yaml，不动一行 python。** 现场给一份没见过的动作，改 yaml 就能跑。

yaml 要能表达三件事：

| 要表达 | 怎么表达 |
|---|---|
| 一步之内多舵机协同 | 同一个 step 里放多个关节，各自带目标值和用时，**同时下发** |
| 步与步之间串行 | 上一个 step 的 result 全回来了才发下一个 |
| 一步之内有先后 | 给那个关节单独开一个 step，**不靠调 duration 硬凑** |

仓库里 `config/actions/` 放了三套：

| 文件 | 步数 | 干什么 |
|---|---|---|
| `wave.yaml` | 4 | 单关节（1 号）冒烟测试：中位 → 正转 → 反转 → 回中 |
| `stand_up.yaml` | 3 | 四腿同时抬起 → 保持 → 落回 |
| `wave_hi.yaml` | 8 | 抬一条腿 + 左右摆两次 + 放下 |

> 现场演示用的是 `stand` / `march` / `reset_all` 三套（4 步 / 8 步 / 1 步），
> 它们的动作清单见 `03_动作组清单与日志.jpg`。**这三套的 pwm 值与整机结构强相关**
> （每个关节的安全行程不同），没有直接放进仓库；仓库里的三套是同样结构的可复现示例，
> 改个 yaml 就能替换。

**时序不用 sleep 猜**。舵机 `T` 毫秒走完的实际时间不等于 `duration_ms`，累积误差会让三遍动作越跑越偏。
所以播放器等的是 **result**：

```
[INFO] [1795361282.265197726] [action_group]: step[脚掌放平] id=3 完成 success=True final_pwm=1506 message=OK: target 1500, read 1506, diff 6
[INFO] [1795361282.506914316] [action_group]: step[脚掌放平] id=1 完成 success=True final_pwm=1511 message=OK: target 1500, read 1511, diff 11
[INFO] [1795361282.756365085] [action_group]: step[脚掌放平] id=4 完成 success=True final_pwm=1501 message=OK: target 1500, read 1501, diff 1
[INFO] [1795361282.758617714] [action_group]: 动作组 stand 播放完成
```

同一个 step 的三个关节 result 在 490 ms 内陆续回来，**全部回来之后**才打"播放完成"。
下一步的"下发"一定晚于上一步所有 result —— 这一步评审会看日志时间戳。

---

## 六、三层保护

| 层 | 管什么 | 在哪 |
|---|---|---|
| 协议层 | 500~2500 电气极限 | `ELECTRIC_MIN` / `ELECTRIC_MAX` 常量，写死不可配 |
| 结构层 | `pwm_min` / `pwm_max` 安全行程 | 参数，装结构后按实测改；goal 超范围直接 REJECT |
| 组合层 | `min >= max` 这类"单个值合法、组合非法"的设置 | `_on_param_set` 参数回调，直接拒 |

现场验证（`02_保护机制_越界_ID_参数.jpg`）：

```
1) position out of range (pwm 3000 > 2100)        → Goal was rejected.
2) id not in servo_ids (id 99)                    → Goal was rejected.
3) param out of range (pwm_max 100000)            → Setting parameter failed:
                                                    pwm_max=100000 out of electrical range 500~2500
4) param out of range (pwm_min 3000)              → Setting parameter failed:
                                                    pwm_min=3000 out of electrical range 500~2500
5) restore legal value (pwm_max 2100)             → Set parameter successful
```

`Goal was rejected.` 是 `ros2 action send_goal` 客户端打印的，节点侧对应日志是
`拒绝 goal：pwm=3000 超出安全范围 [900,2100]`。

关键是第 3、4 条：**参数服务自己就拒了**，不用等发指令时才发现。

---

## 七、一个 goal 到底走了哪几步

```
ros2 action send_goal /move_servo
        │  goal{id, pwm, duration_ms}
        ▼
[1] goal_callback ── ID 在不在 servo_ids？pwm 在不在 [pwm_min, pwm_max]？
        │              duration 合法？串口开着？
        │              合法 → ACCEPT；不合法 → REJECT（日志写清哪条不过）
        ▼
[2] handle_accepted ── 立刻 execute() 交给线程，**不在这里阻塞**
        │              （阻塞的话 cancel 回调和 Ctrl+C 都会失灵）
        ▼
[3] execute_callback
        │  ① #xxxPULR!            恢复扭力（不发这条舵机根本不动）
        │  ② #xxxP<pwm>T<ms>!     下发位置
        │  ③ 每 100 ms publish_feedback + /servo/state
        │  ④ 收到 cancel → 回中位，result.message = "canceled"
        │  ⑤ 走完 #xxxPRAD!       读回实际位置（读不到会重试 3 次）
        │  |读回 - 目标| > 30 → message = "NOT REACHED: ..."，success=False
        ▼
   result{success, final_pwm, message}
```

**为什么 `execute_callback` 不能阻塞**：它跑在 executor 的线程池里。如果在这里同步等串口回包、
或者自己调 `spin`，这个线程就被占住，同一执行器里的 cancel 服务和 `/servo/command` 订阅都进不来 ——
表现就是"按了取消没反应、Ctrl+C 也停不下来"。
所以 `handle_accepted_callback` 里只调 `goal_handle.execute()`，真正的活儿交给线程池；
取消检查放在 feedback 循环内部，每 100 ms 看一次。

**关于并发**：同一时刻应该只有一个 goal 在执行。动作组播放器是"下发 → 等 result → 再下发"，
不会重叠；手动并发发送两条 goal 时，两条会各自跑完自己的 feedback 循环，串口写由内部锁串行化，
但位置会互相覆盖 —— 这是这套实现没有做抢占的取舍。

---

## 八、安全下电（任务 4）

顺序固定，不能反：

```
① 全部回中位（或直接播 reset_all）
② 卸力：对每个舵机发 #xxxPULK!
③ 拔电池（先断动力）
④ 最后拔 USB-TTL
```

为什么是这个顺序：

- **先卸力再断电**：带电掰结构 = 堵转，电流拉满，齿箱发热、电机烧。
- **先断动力后断信号**：反过来会让串口基准电压悬空，有概率打坏舵机板的串口芯片。
- **带电插拔舵机线是红线**：热插拔浪涌会打坏板子，细则里这一条是整周不通过。

> 实测记录：这只舵机收到 `PULK` 后会进入锁死态，`PULR` 叫不回来，只能断电重启。
> 所以卸力之后**不要**再做任何动作，直接进第 ③ 步。

---

## 九、材料清单

### 任务 1 · 点亮一个舵机

| 文件 | 内容 |
|---|---|
| `01_串口设备.png` | `ls -l /dev/ttyUSB*`，属主组 dialout，会话非 root |
| `01_PID回显.png` | `#000PID!` → `#000P!`；`#000PRAD!` → `#000P1510!` |
| `01_报文记录_正转与回中位.png` | PULR + P1700T1500 + PRAD 往返 |
| `01_中立位标定.png` | 舵机中位读回（1493 / 1495 / 1492 / 1491） |
| `01_电池电压.jpg` | 万用表 7.73 V |

### 任务 2 · ROS2 接管

| 文件 | 内容 |
|---|---|
| `02_接口编译与节点动作.png` | 接口编译 + 节点与 action 列表 |
| `02_保护机制_越界_ID_参数.jpg` | 越界 / 非法 ID / 非法参数三重拒绝 + 恢复 |
| `02_限位_参数_自检.png` | 限位保护 + param set + 硬编码 grep 自检 |
| `02_action_反馈_1.png` / `_2.png` | feedback 滚动 + result（三条动作） |
| `02_取消生效.png` | `cancel_return_code: 0`，`message=canceled` |
| `02_bag_info.png` | `bag info` 消息条数 + 回放 |
| `bag/run1/` | metadata.yaml + bagrun_0.db3（66 条 / 29.5 s） |

### 任务 3 · 整机 + 动作组

| 文件 | 内容 |
|---|---|
| `03_ID扫描.jpg` | 十个舵机全在线：`ONLINE IDS: [1..10]` |
| `03_动作组清单与日志.jpg` | `stand` 4 步 / `march` 8 步 / `reset_all` 1 步 + result 时间戳 |
| `03_整机照片_正面.jpg` | 整机正面：双足脚掌、白色结构件、舵机编号标签、电池与红白线束 |
| `03_整机照片_侧面.jpg` | 整机侧面：关节叠层、走线路径、脚掌与电池仓侧视 |
| 录屏 · 任务 3 | `任务3.mp4`（45 s，横屏）：动作组播放全程，**终端日志与整机动作同框** |

> 任务 3 录过两遍，跑的是同一套脚本流程。另一版是竖屏机器人特写，终端日志基本被裁掉，
> 证明不了 result 同步，所以只保留取证更完整的这一版；原始两段 mp4 都在本地留存，没有删除。
> 「换一套新动作不用改 python」由播放器的 `--action /绝对路径/xxx.yaml` 和
> `03_动作组清单与日志.jpg` 里的三套动作共同支撑，录屏本身只覆盖一遍流程。

**整机照片说明**：两张为原样归档的实物照片，画面内未叠加姓名 + 时间戳，
文件 EXIF 亦无拍摄时间（传输时被剥掉）；未做任何后期补加。

细则第 501 行还要求**接线特写**：现在两张都是整机全景，只能看出线束走向，
还需要一张能看清端子、共地与插头位置的局部特写。

### 任务 4 · 安全下电

| 文件 | 内容 |
|---|---|
| 录屏 · 任务 4 | `任务4.mp4`（29.5 s），回中 → 卸力 → 拔电池 → 拔 USB |

**录屏链接**

四段做成一个 **B 站多 P 稿件**（`BV1isyjzKEQU`），每项用 `?p=N` 直达对应分 P。
细则按内容逐项验收，这样评审不用自己翻分集。

- 稿件总入口：<https://www.bilibili.com/video/BV1isyjzKEQU/>
- 任务 1（舵机点亮，p1 / 22 s）：<https://www.bilibili.com/video/BV1isyjzKEQU/?p=1>
- 任务 2（正反转回中位 / 取消打断 / 改参数生效 / `ros2 bag play` 复现，p2 / 66 s）：<https://www.bilibili.com/video/BV1isyjzKEQU/?p=2>
- 任务 3（动作组，p3 / 45 s）：<https://www.bilibili.com/video/BV1isyjzKEQU/?p=3>
- 任务 4（下电，p4 / 30 s）：<https://www.bilibili.com/video/BV1isyjzKEQU/?p=4>

> 稿件标题沿用上传时第一分 P 的名字「录屏1_任务1_舵机点亮」，
> 实际四个分 P 依次是任务 1 / 2 / 3 / 4。
> 原始 mp4 不进 git 仓库（`.gitignore` 忽略 `*.mp4`），随交付包单独提供；
> 任务 1、2 两段原片合计约 807 MB，仅本地留存。

---

## 十、已知问题与取舍

1. **中位读回偏差 ±13**：机械回差。正向走到 1700 读回 1691，反向走到 1200 读回 1207。
   所以判定容差给 30，往小调会让每次动作都判 `NOT REACHED`。
2. **`bag/run1/` 是单舵机阶段录的**（`id=0`）。整机总线上已无 0 号设备，直接回放不会动；复现办法见第一节。
3. **没有实现动作抢占**：新 goal 会等前一个跑完。动作组本身是串行下发的，正常流程不会重叠。
4. **`/servo/command` 不做安全限位检查**（只做电气范围），回放前需确认限位一致。
5. **卸力靠手动发 `PULK`**：节点退出不自动卸力，因为实测 `PULK` 后舵机会锁死、要断电重启。
