# 第 3–4 周考核 · 整机拼装与舵机控制 · xuyuhan

- **姓名**：徐宇涵（xuyuhan）
- **完成日期**：2026-11-20
- **系统**：Ubuntu 22.04.5 LTS · ROS2 Humble · Python 3.10
- **串口**：USB 转 TTL（CH340，`1a86:7523`）→ `/dev/ttyUSB0`，**115200 8-N-1**，属组 `dialout`（无需 sudo）
- **被控对象**：任务 1/2 桌面阶段使用旧舵机 4 只（ID 001~004）；任务 3 使用**全新双腿整机 10 只**（ID 001~010）
- **电池**：任务 1/2 桌面电池实测 **7.74 V**；任务 3 整机电池约 **8.3 V**（见 `03_电池电压.jpg`）

> ⚠️ **桌面旧舵机与整机新舵机是两套硬件。** 桌面 001 号在调试过程中损坏，
> 后期桌面演示使用 002 号（见 `调试记录.md` 第 6 条）；任务 3 整机已换成 10 颗新舵机，
> 左右腿各 5 颗，映射见 [`03_整机关节映射.md`](03_整机关节映射.md)。

---

## 一、架构

```
        键盘 / ros2 action send_goal / 动作组 yaml
                        │
                        ▼
            ┌───────────────────────┐
            │  /move_servo (Action) │
            │  Action Server        │
            │  goal → 校验 → 执行    │
            │  feedback → result    │
            └───────────┬───────────┘
                        │  同一个底层发送函数 _issue_move()
                        ▼
        ┌───────────────┴───────────────┐
        │  ServoCommand msg             │
        │  /servo/command  ← bag 回放   │
        │  /servo/state    → 心跳/状态   │
        └───────────────┬───────────────┘
                        ▼
              "#002P1700T1000!"  → serial.write + flush
                        ▼
              串口 115200 8-N-1（共地）
                        ▼
              USB 转 TTL → 24 路舵机控制板
                        ▼
              半双工总线（同一根信号线，靠 ID 区分）
                        ▼
                     总线舵机
```

**两个包**：

| 包名 | 构建类型 | 内容 |
| --- | --- | --- |
| `cquick_servo_interfaces` | `ament_cmake` | `msg/ServoCommand.msg`、`action/MoveServo.action` |
| `cquick_servo` | `ament_python` | 驱动节点、客户端、取消演示、动作组播放器、launch、config |

**节点与接口**：

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `/servo_driver` | Node | 驱动节点（Action Server） |
| `/move_servo` | Action | `cquick_servo_interfaces/action/MoveServo` |
| `/servo/command` | Topic | `ServoCommand`，驱动发出 + 订阅（bag 回放通道） |
| `/servo/state` | Topic | `ServoCommand`，周期心跳（1 Hz）+ 动作过程中实时位置 |

---

## 二、参数表

配置文件：`src/cquick_servo/config/servo_params.yaml`
**最外层 key 必须等于节点名 `servo_driver`**（写错不会报错，只会静默不加载）。

| 参数 | 类型 | 默认值 | 为什么是这个值 |
| --- | --- | --- | --- |
| `port` | string | `/dev/ttyUSB0` | CH340 直通进虚拟机后的固定节点名；插拔后可能变 `ttyUSB1`，所以做成参数 |
| `baudrate` | int | `115200` | 众灵总线舵机协议固定 115200 8-N-1，不是可调项，但必须外置以便换型号 |
| `pwm_min` | int | **`1050`** | 实测下止点约 1007，留 43 余量（见第五节） |
| `pwm_max` | int | **`2050`** | 电气上限 2100，留 50 余量（见第五节） |
| `default_duration_ms` | int | `800` | goal 里 `duration_ms=0` 时使用；800 ms 对多数关节是"不猛跳"的下限 |
| `servo_ids` | int[] | `[1, 2, 3, 4, 5, 6, 7, 8, 9, 10]` | 任务 3 双腿整机的 10 只舵机；上线时必须与 ID 扫描结果一致 |
| `startup_reset` | bool | **`true`（正式配置）** | 启动后全部回到逻辑 1500 叠加各关节 offset 后的校准标准位 |
| `home_pwm` | int | `1500` | **逻辑中位**；启动/取消回中会叠加对应关节 offset |
| `joint_offsets` | int[] | `[9,-119,-22,44,119,-51,-68,135,-20,10]` | 同一台机械在另一台电脑上实测；实际 PWM = 逻辑目标 + offset |
| `prad_retries` | int | `1` | 正式运行允许额外重读 1 次；分级逼近测试时设为 0 |
| `torque_enable` | bool | `true` | 设为 `false` 立即全部卸力并拒绝新动作 |
| `watchdog_timeout_s` | float | `3.0` | goal 执行停滞超时后全部卸力；0 为关闭，整机测试保持开启 |

**硬编码自检**：`grep -n "ttyUSB\|115200" servo_driver.py` 只命中 `declare_parameter` 两行；
`grep -n "1050\|2050" servo_driver.py` **无任何输出**（安全区间只在 yaml 里）。
见 `02_参数自检.png`。

---

## 三、接口契约：`MoveServo`

```text
# Goal
uint8  id            舵机 ID（必须在 servo_ids 里）
uint16 pwm           目标位置（必须在 [pwm_min, pwm_max] 里）
uint16 duration_ms   从当前位置走到目标用多久；0 = 用 default_duration_ms

# Feedback（执行过程中 ≥5 Hz 滚动上报）
uint16 current_pwm   当前读回的 PRAD
float32 progress     0.0 ~ 1.0

# Result
bool   success       是否完成且到位
string message       ok / out_of_range / unknown_id / canceled / serial_error / not_reached
uint16 final_pwm     结束时读回的 PRAD
```

**什么时候发什么**：

| 阶段 | 动作 |
| --- | --- |
| 收到 goal | `goal_callback` 先校验（pwm 区间 / ID 是否存在 / duration 合法），不合法直接 `REJECT`，**根本不会碰串口** |
| 开始执行 | 发 `PULR`（恢复扭力）→ 发 `P<目标>T<ms>!` |
| 执行中 | 每 100 ms 发一条 feedback；每 250 ms 读一次 `PRAD` 更新 `current_pwm` |
| 取消 | `cancel_callback` 返回 `ACCEPT`；`execute_callback` 检测到 `is_cancel_requested` → `PULK` 卸力急停 → 回 `home_pwm` → `goal_handle.canceled()` |
| 结束 | 停稳 0.4 s 后读 `PRAD`，与**逻辑目标 + offset 后的实际目标**比较，差 **>30** 判 `not_reached` |

**为什么 `execute_callback` 不能阻塞**：
`execute_callback` 跑在 executor 的线程里。如果它里面用阻塞 sleep 占死线程，ActionServer 的
`cancel_callback`、goal 接收回调就进不来，`Ctrl+C` 取消会失效。
所以本实现用 **`MultiThreadedExecutor` + `ReentrantCallbackGroup`**，让取消请求能在执行过程中被处理。

---

## 四、一条启动命令

```bash
ros2 launch cquick_servo servo.launch.py
```

实际输出：

```text
[INFO] [launch]: All log files can be found below /home/xmx/.ros/log/...
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [servo_driver-1]: process started with pid [28566]
[servo_driver-1] [INFO] [1794892581.399950648] [servo_driver]: 串口已打开 /dev/ttyUSB0 @ 115200
[servo_driver-1] [INFO] [1794892581.403130578] [servo_driver]: servo_driver ready  port=/dev/ttyUSB0 baud=115200 safe=[1050, 2050] ids=[1, 2, 3, 4] default_dur=800
[servo_driver-1] [INFO] [1794892581.506135254] [servo_driver]: startup_reset -> id 1 1500
[servo_driver-1] [INFO] [1794892581.608304462] [servo_driver]: startup_reset -> id 2 1500
[servo_driver-1] [INFO] [1794892581.710123544] [servo_driver]: startup_reset -> id 3 1500
[servo_driver-1] [INFO] [1794892581.812365292] [servo_driver]: startup_reset -> id 4 1500
```

`launch` 的三个要求：

| 要求 | 实现 |
| --- | --- |
| 参数从 yaml 读 | `os.path.join(get_package_share_directory('cquick_servo'), 'config', 'servo_params.yaml')`，**不用相对路径** |
| launch 参数能覆盖 yaml | `port:=` 非空时才追加 `{'port': ParameterValue(port, value_type=str)}` |
| 有条件启动 | `demo:=true` 才起 `move_client`，默认不起 |

```bash
ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB1      # 换端口
ros2 launch cquick_servo servo.launch.py demo:=true              # 连客户端一起起
```

> `launch/` 和 `config/` 必须在 `setup.py` 的 `data_files` 里显式声明才会进 `share/`，改完要重新 build。

---

## 五、安全区间是多少、怎么试出来的

**方法：单向逼近法 + 读回判定（不靠耳朵）**

1. 从 1500 出发，每次向外走 100 个单位；
2. 每步给 `T1000`，**等 3 秒**（含 2 秒 settle）后连读两次 `PRAD`，两次一致才采用；
3. 读回与目标偏差 **> 30** 即判定「顶到机械限位」。

**实测数据（001 号舵机）**：

| 目标 | 读回 | 偏差 | 判定 |
| --- | --- | --- | --- |
| 1400 | 1405 | +5 | ok |
| 1300 | 1308 | +8 | ok |
| 1200 | 1207 | +7 | ok |
| 1100 | 1105 | +5 | ok |
| 1000 | 1007 | +7 | ok |
| **900** | **1007** | **+107** | **❌ 读回卡死在 1007 不再跟随** |

向上方向 1600~2100 全程偏差 ≤7，未探到机械边界，按厂家电气上限 2100 封顶。

**结论**：

```yaml
pwm_min: 1050     # 物理下止点约 1007，留 43 余量
pwm_max: 2050     # 电气上限 2100，留 50 余量
```

> 注意区分三个区间：**电气范围 500~2500**（厂家协议，用了会顶死）→
> **机械可达范围**（本机实测 1000~2100）→ **结构安全范围**（装进结构后还要按关节再收窄）。

---

## 六、运行时改参数（当帧生效）

```bash
ros2 param get /servo_driver pwm_max              # → Integer value is: 2050
ros2 action send_goal /move_servo ... "{id: 2, pwm: 1500, duration_ms: 800}"
#   → success: true, final_pwm: 1494
ros2 param set /servo_driver pwm_max 1600         # → Set parameter successful
ros2 action send_goal /move_servo ... "{id: 2, pwm: 1700, duration_ms: 800}"
#   → Goal was rejected          ← 关键：没重启节点，行为立刻变了
ros2 param set /servo_driver pwm_max 100000
#   → Setting parameter failed: pwm_max 必须是 500~2500 的整数
ros2 param set /servo_driver pwm_max 2050         # → Set parameter successful
ros2 param get /servo_driver pwm_max              # → Integer value is: 2050
```

**两个实现要点**：

1. **绝不缓存参数**。判断时每次实时 `self.get_parameter('pwm_max').value`，
   所以 `param set` 之后下一个 goal 就按新值校验。
2. **必须注册 `add_on_set_parameters_callback`** 才能拒绝非法值。
   回调返回 `successful=True` 才会真正写入；返回 `False` 整个 set 被拒，参数保持原样。
   校验内容：`pwm_min/pwm_max/home_pwm` 必须在 500~2500，`baudrate` 必须在合法范围，
   且 `pwm_min < pwm_max`。

---

## 七、bag：录了什么、怎么回放、回放结果

**录制**（动作开始前开录，跑完立刻停）：

```bash
ros2 bag record -o ~/ros2_ws/bag_run1 /servo/command /servo/state
```

| 项 | 实际值 |
| --- | --- |
| 话题 | 只录 `/servo/command` 和 `/servo/state`（**没有用 `-a`**） |
| Duration | **22.0 s**（要求 ≤30 s） |
| Messages | 59 条（`/servo/command` 3 条，`/servo/state` 56 条） |
| 体积 | 25.5 KiB（要求 ≤10 MB） |
| 文件 | `bag/run1/bag_run1_0.db3` + `metadata.yaml` |

**回放**：

```bash
ros2 bag play bag/run1
```

**回放结果**：驱动端收到 `/servo/command` 后走**同一个底层发送函数**，舵机把
1200 → 1800 → 1500 这套动作原样重演，驱动日志出现：

```text
[servo_driver]: [topic] replay id=2 pwm=1200 dur=1500
[servo_driver]: [topic] replay id=2 pwm=1800 dur=1500
[servo_driver]: [topic] replay id=2 pwm=1500 dur=1500
```

> 关键设计：Action 和 `/servo/command` **共用 `_issue_move()`**。
> 如果两条路径各写一套发送逻辑，回放出来的动作和录屏里的一定对不上。
> 另外驱动自己发的消息会被自己订阅到，代码里用「待消化签名列表（3 秒窗口）」把它吃掉，
> 所以正常运行时不会出现 `[topic] replay` 行 —— 它**只在真正回放时出现**。

---

## 八、接线说明

```
CH340 ──USB──> 电脑（VMware 直通给 Ubuntu，/dev/ttyUSB0）
  │
  ├─ TXD ──> 控制板 RX
  ├─ RXD <── 控制板 TX
  └─ GND ──> 控制板 GND        ← 必须共地，只接信号不接地一定收不到数据

            电池 V+ / GND ──> 控制板 舵机电源输入
                                 │
                          舵机通道（橙/白=信号，红=V+，棕/黑=GND）
```

| 项 | 值 |
| --- | --- |
| 电池实测电压 | 桌面阶段 **7.74 V**；整机阶段约 **8.3 V**（均在允许范围内） |
| 上电前检查 | 万用表量电压、看极性、看电池有无鼓包 |
| 串口参数 | 115200 8-N-1 |
| 舵机 ID | 任务3整机 001～010，扫描确认10颗全部在线（见 `03_ID扫描.png`） |

---

## 九、动作组 yaml

动作定义放 `src/cquick_servo/config/actions/`，播放器 `sequence_player.py` 读 yaml、展开 goal 序列。
**换动作只换 yaml，不改 Python。**任务 3 使用双腿 10 舵机，关节映射与逐关节实测行程见
[`03_整机关节映射.md`](03_整机关节映射.md) 和 `task3_joint_ranges.json`。

### 两套验收动作

| 文件 | 内容 | 主要参数 |
| --- | --- | --- |
| `wave.yaml` | 双侧同步屈伸 | 髋 250、膝 300、踝 200，1200 ms 分级 |
| `march_in_place_final.yaml` | 原地踏步 | 支撑侧重心 50→100、提跟 75、左髋 320、右髋 500、双膝 400，1500 ms 分级；不做骨盆附加动作 |

所有 YAML 数值都是**逻辑目标**，驱动只叠加一次 `joint_offsets[ID-1]` 得到实际 PWM。
限位与最终 `PRAD` 均按实际值校验。左髋逻辑 1180（实际 1061）和双膝边界点不得继续外扩。

### 数据结构与同步

| 要求 | 实现 |
| --- | --- |
| 一步之内多舵机协同 | 同一 step 的 `servos` 同时 `send_goal_async`，禁止重复 ID |
| 步与步之间串行 | 上一 step 的所有 result 回收后才发下一 step |
| 一步之内有先后 | 拆成独立 step，不靠 `duration_ms` 或 `sleep` 硬凑 |
| 失败处理 | 拒绝、未到位、超时或 Ctrl+C 立即取消已知 goal，并请求 `torque_enable=false` |
| 并发与总线负载 | 16 线程执行器；同 ID 互斥；动作中 PRAD 全总线统一 250 ms 限频 |

### 验收命令

```bash
# 双侧同步屈伸，连续三遍
ros2 run cquick_servo sequence_player --ros-args \
  -p action_file:=$(ros2 pkg prefix cquick_servo)/share/cquick_servo/config/actions/wave.yaml \
  -p repeat:=3

# 换一套 yaml，原地踏步一遍，不改代码
ros2 run cquick_servo sequence_player --ros-args \
  -p action_file:=$(ros2 pkg prefix cquick_servo)/share/cquick_servo/config/actions/march_in_place_final.yaml \
  -p repeat:=1
```

**实机验收结果**：

- `03_屈伸三遍.log`：三遍共 24 个 step 全部 `ok=True`，结束于 01:13:29，显示 `共 3 遍`；
- `03_原地踏步.log`：29 个 step 全部 `ok=True`，结束于 01:14:52，显示 `共 1 遍`；
- `03_逐个关节自检.log`：10 颗舵机均完成中位→+50→中位，全部成功；
- 三份日志都有 send/result 时间戳，无 `success=False`。

`neutral.yaml` 用于全部回校准位，`joint_selfcheck` 用于逐关节自检，
`joint_range_check` 用于装配态行程测试。异常卸力后必须人工检查，再显式执行
`ros2 param set /servo_driver torque_enable true` 恢复。

---

## 十、踩坑记录

完整版见 [`调试记录.md`](调试记录.md)，这里摘要 4 条：

1. **`colcon build --symlink-install` 被 Week2 的包卡住**
   `ball_pos_pkg` 的 `build/` 里是旧的真实目录，无法替换成符号链接 →
   `failed to create symbolic link ... Is a directory`，还把 `cquick_servo_interfaces` 连累成 `Aborted`。
   **解决**：把 `ball_pos_pkg` 移出 `src/`，清掉它的 `build/ install/` 残留再编。

2. **`ros2 action send_goal` 按 `Ctrl+C` 不会取消 goal**
   客户端只是打断自己的等待，**根本不向服务端发 cancel 请求**，结果照样 `SUCCEEDED`。
   **解决**：自写 `cancel_demo.py`，由客户端显式调用 `handle.cancel_goal_async()`。

3. **动作期间高频轮询 `PRAD` 把舵机动作挤死**
   原实现每 160 ms 读一次位置，导致目标 2050 的 1.4 秒动作只走了 13 个单位，
   读数还错乱（目标 1500 读到 1620）。
   **解决**：动作期间改为 **250 ms 读一次**，动作结束**再等 0.4 秒**才判定到位。
   改完之后 1800→1793 / 1200→1205 / 1500→1494，全部准确。

4. **烧了一颗舵机**
   连续高频动作约 1 小时后，001 号出现焦味、通信正常但完全不出力。
   **解决**：换 002 号继续；后续收窄 pwm 区间、动作之间留散热间隔、每轮测温。
   详见 `调试记录.md` 第 6 条。

---

## 十一、安全下电三步

```text
先卸力  →  再断动力电（电池）  →  最后关板卡
```

| 步骤 | 做什么 | 为什么是这个顺序 |
| --- | --- | --- |
| ① 卸力 | 执行 `ros2 param set /servo_driver torque_enable false`，驱动向 001～010 全部发送 `PULK` | 先禁止新运动再卸力，避免断电前持续顶力矩；详见 `04_安全下电.md` |
| ② 断动力电 | 拔电池 | 先断动力，避免控制板在异常电压下继续乱发指令 |
| ③ 关板卡 | `sudo shutdown now`，**不要直接拔电** | 直接断电会损坏文件系统，下次开不了机 |

**四条红线**：

1. **禁止带电插拔舵机线** —— 浪涌会击穿驱动级或控制板通道；
2. **禁止反接电源** —— 上电前必须量电压、看极性；
3. **电池鼓包立即停用**，充电必须有人看管；
4. **禁止让舵机硬顶结构** —— 持续「嗡嗡」就是堵转，立刻回中位。

---

## 十二、B 站演示录屏

**命名规范**（与同批同学一致）：`CQuick电控 第X周任务Y <内容> xuyuhan`

| 序号 | 任务 | 内容 | 标题 | 链接 |
| --- | --- | --- | --- | --- |
| 1 | 一 | 舵机转动 | `CQuick电控 第3周任务1 舵机转动 xuyuhan` | [视频](https://t.bilibili.com/1271563855862956040?share_source=pc_native) |
| 2 | 二 | 正转 → 反转 → 回中位 | `CQuick电控 第3周任务2 正反转回中位 xuyuhan` | [视频](https://t.bilibili.com/1271564019104219139?share_source=pc_native) |
| 3 | 二 | 取消打断（运行中取消） | `CQuick电控 第3周任务2 取消打断 xuyuhan` | [视频](https://t.bilibili.com/1271564062040260628?share_source=pc_native) |
| 4 | 二 | 运行时改参数，行为当帧变化 | `CQuick电控 第3周任务2 参数当帧生效 xuyuhan` | [视频](https://t.bilibili.com/1271564165091164161?share_source=pc_native) |
| 5 | 二 | `ros2 bag play` 复现动作 | `CQuick电控 第3周任务2 bag回放复现 xuyuhan` | [视频](https://t.bilibili.com/1271700465904517124?share_source=pc_native) |
| 6 | 三 | 整机逐个舵机动作 | `CQuick电控 第4周任务3 整机逐个舵机动作 xuyuhan` | [视频](https://t.bilibili.com/1272756117282226181?share_source=pc_native) |
| 7 | 三 | 双侧同步屈伸连跑三遍 | `CQuick电控 第4周任务3 屈伸动作连跑三遍 xuyuhan` | [视频](https://t.bilibili.com/1272756117282226181?share_source=pc_native) |
| 8 | 三 | 换 yaml 原地踏步一遍，不改代码 | `CQuick电控 第4周任务3 原地踏步 xuyuhan` | [视频](https://t.bilibili.com/1272756117282226181?share_source=pc_native) |
| 9 | 四 | 安全下电（PULK → 断电池 → 关机） | `CQuick电控 第4周任务4 安全下电 xuyuhan` | [视频](https://t.bilibili.com/1272757457368645640?share_source=pc_native) |

任务 3 的第 6～8 项录在同一段视频中，因此共用第 6 个链接。

> 每段开头都有 `echo "姓名: xuyuhan"; date "+%Y-%m-%d %H:%M:%S"`，未剪辑。
