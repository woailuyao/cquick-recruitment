# CQuick 电控组 · 第 3、4 周：整机拼装与舵机控制

> 姓名拼音：`xibailing`
> 周期：2026-11-17 ～ 2026-11-22
> 环境：Ubuntu 22.04.5 + ROS 2 Humble + Python 3.10

---

## 0. 一句话说明

用 **ROS 2 Action** 驱动一条半双工总线上的舵机：客户端发 goal，服务端校验限位、拼 ASCII 指令经串口发出，再用 `PRAD` 读回位置做闭环校验。所有硬件相关数值（串口、波特率、限位、ID 列表）全部来自参数，运行时可改。

```
键盘 / 命令行 / yaml
      ↓ goal
Action Client ──────────────> Action Server (cquick_servo: servo_driver)
      ↑                            │ ① 校验 pwm ∈ [pwm_min, pwm_max]、id ∈ servo_ids
      │ feedback / result          │ ② 拼 ASCII： "#001P1700T1200!"
      └────────────────────────────┤ ③ write + flush → /dev/ttyUSB0 @115200 8-N-1
                                   ↓
                        USB-TTL → 24 路控制板 → 总线舵机
                                   ↓
                             #001PRAD! 读回 → 与目标比对（±30）
```

---

## 1. 目录结构

```
Week3,4/xibailing/
├── 01_串口设备.png
├── 01_PID回显.png
├── 01_中立位标定.png
├── 01_电池电压.png
├── 02_接口编译.png
├── 02_launch一键启动.png
├── 02_action_反馈.png
├── 02_取消生效.png
├── 02_限位保护.png
├── 02_param_set现场.png
├── 02_bag_info.png
├── 02_参数自检.png
├── 03_整机照片.png
├── 03_接线特写.png
├── 03_ID扫描.png
├── 03_多舵机动作_关键帧.png
├── 调试记录.md
├── README.md              ← 本文件
├── bag/run1/              ← metadata.yaml + bag_run1_0.db3（≤10 MB）
└── src/
    ├── cquick_servo_interfaces/   (ament_cmake)
    │   ├── msg/ServoCommand.msg
    │   ├── action/MoveServo.action
    │   ├── CMakeLists.txt
    │   └── package.xml
    └── cquick_servo/              (ament_python)
        ├── cquick_servo/
        │   ├── servo_driver.py     Action Server（串口驱动）
        │   ├── move_client.py      Action Client
        │   └── sequence_player.py  动作组播放器（读 yaml）
        ├── launch/servo.launch.py
        ├── config/
        │   ├── servo_params.yaml
        │   └── actions/
        │       ├── stand_up.yaml
        │       └── wave.yaml
        ├── resource/cquick_servo
        ├── package.xml
        └── setup.py
```

---

## 2. 架构与接口一览

### 2.1 节点

| 节点 | 所属包 | 作用 |
|---|---|---|
| `servo_driver` | `cquick_servo` | **Action Server**，唯一对外控制口；同时订阅 `/servo/command`、发布 `/servo/state` |
| `move_client` | `cquick_servo` | 命令行客户端，发单个 goal，打印 feedback / result，支持 Ctrl+C 与 `--cancel-after` |
| `sequence_player` | `cquick_servo` | 读 yaml → 按 step 展开成 goal 序列 → **等 result 回来再发下一步** |

### 2.2 接口

| 类型 | 名称 | 说明 |
|---|---|---|
| Action | `/move_servo` | 类型 `cquick_servo_interfaces/action/MoveServo` |
| Topic（调试总线） | `/servo/command` | 类型 `cquick_servo_interfaces/msg/ServoCommand`，**供 bag 录制与回放** |
| Topic（状态心跳） | `/servo/state` | 同上类型，1 Hz 发布最近一次到位位置 |

启动后自查：

```bash
ros2 node list            # /servo_driver
ros2 action list -t       # /move_servo [cquick_servo_interfaces/action/MoveServo]
ros2 topic list | grep servo   # /servo/command  /servo/state
```

---

## 3. 一条命令启动

```bash
cd ~/cq_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch cquick_servo servo.launch.py
```

实测输出（节选）：

```text
[INFO] [launch]: All log files can be found below /home/xbl/.ros/log/...
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [servo_driver-1]: process started with pid [6311]
[servo_driver-1] [INFO] [servo_driver]: servo_driver up: port=/dev/ttyUSB0 baud=115200 pwm=[900,2100] ids=[1, 2, 3, 4] center=1500
[servo_driver-1] [INFO] [servo_driver]: 启动回中位: [1, 2, 3, 4] -> 1500
[servo_driver-1] [INFO] [servo_driver]: 串口打开 /dev/ttyUSB0 @ 115200 8-N-1
```

launch 参数（可覆盖 yaml）：

```bash
ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB1     # 换串口
ros2 launch cquick_servo servo.launch.py demo:=true            # 额外起 move_client
ros2 launch cquick_servo servo.launch.py startup_reset:=false  # 启动时不回中位
```

---

## 4. 参数表

`config/servo_params.yaml`（yaml 顶层 key 必须等于节点名 `servo_driver`，写错不报错、参数静默不加载）：

| 参数 | 类型 | 默认值 | 为什么是这个默认值 |
|---|---|---|---|
| `port` | string | `/dev/ttyUSB0` | 本机 USB-TTL（CH340，1a86:7523）枚举为 ttyUSB0；插拔顺序变化时会变成 ttyUSB1，故 launch 支持 `port:=` 覆盖 |
| `baudrate` | int | `115200` | 舵机控制板固定 115200，8-N-1。串口助手默认 9600，是最常见的乱码原因 |
| `pwm_min` | int | `900` | 安全下限。电气绝对极限是 500~2500，但结构不允许跑满行程，故收窄 |
| `pwm_max` | int | `2100` | 安全上限，同上 |
| `default_duration_ms` | int | `1200` | **实测**: `duration < 1000` 时位置指令不执行（见调试验证），故上调到 1200 |
| `servo_ids` | int[] | `[1, 2, 3, 4]` | 本机 4 个舵机；用于在 goal 被接受前拒掉不存在的 ID |
| `startup_reset` | bool | `true` | 上电后自动全部回中位，避免上次残留姿态 |
| `center_pwm` | int | `1500` | 中立位。取消 goal 时回这个位置 |
| `allowed_baudrates` | int[] | `[9600, 19200, 38400, 57600, 128000, 256000]` | 参数校验白名单，用于拒绝"不认识的波特率" |

> 代码自检：`grep -nE "ttyUSB|115200|(^|[^0-9])900([^0-9]|$)|2100" cquick_servo/servo_driver.py`
> 命中行应**全部落在 `declare_parameter`**，业务逻辑中不出现这些字面量。

---

## 5. 接口契约

### 5.1 `ServoCommand.msg`（调试总线）

```
uint8  id           # 舵机 ID，1~254
uint16 pwm          # 目标 PWM（电气范围 500~2500，安全范围见参数）
uint16 duration_ms  # 从当前位置走到目标用时（ms），0 表示用 default_duration_ms
```

### 5.2 `MoveServo.action`

```
# ---- Goal ----
uint8  id
uint16 pwm
uint16 duration_ms      # 0 表示用 default_duration_ms

---
# ---- Result ----
bool   success          # 是否真正到位（以 PRAD 读回为准，不是插值）
uint16 final_pwm        # 最后一次 PRAD 读回值；取消/串口失败时为 0
string message          # 到位/未到位/取消/串口错误，一句话说清

---
# ---- Feedback ----
uint16  current_pwm     # 起点到目标的线性插值（不是实测值，见下方说明）
float32 progress        # 0.0 ~ 1.0
```

**什么时候发**：

| 阶段 | 内容 |
|---|---|
| Goal 校验（`goal_callback`） | `id` 不在 `servo_ids` → REJECT；`pwm` 不在 `[pwm_min, pwm_max]` → REJECT；`duration_ms > 10000` → REJECT |
| 执行中（`execute_callback`） | 10 Hz 发 feedback（≥5 Hz 要求） |
| 结束 | 读 `PRAD`；偏差 > ±30 判未到位，自动重发一次；仍不到则 `success=False` 并写清差值 |

> ⚠️ **feedback 的 `current_pwm` 是插值，不是实测**。因为半双工总线上每个 feedback 周期都去读 PRAD 会显著增加丢包，所以只在开始和结束各读一次。判断是否真到位，请看 `result.final_pwm`。

---

## 6. 安全区间 900 / 2100 的来历

- 舵机**电气**可用范围是 **500–2500**，这是厂家给的 PWM 极限，不是结构允许的范围，**直接跑满会打齿、顶裂打印件**。
- 本机以 **1500 为中位**，向外只取 ±600，即 **900 ~ 2100**，作为 `pwm_min / pwm_max`。
- 装配阶段所有测试动作控制在 **1200 ~ 1750**（±250），先小幅度验证方向与时序，再逐步放开。
- **未完成**：整机结构限位尚未做全程实测（见调试记录第 3 节），当前区间按文档建议值 + 小幅度实测确定，后续应逐关节逼近限位后回退 50 再定稿。

改区间不需要动代码：

```bash
ros2 param set /servo_driver pwm_max 1600     # 下一帧生效
```

---

## 7. 运行时改参数（当帧生效）实测记录

关键实现：驱动里的取值一律走 `self.p('pwm_max')` → `self.get_parameter(...)`，**从不在 `__init__` 里缓存副本**；同时在 `add_on_set_parameters_callback` 里做校验。

实测（节选，同一进程内完成，中途未重启）：

| 步骤 | 命令 | 实测结果 |
|---|---|---|
| ① | `ros2 param get /servo_driver pwm_max` | `Integer value is: 2100` |
| ② | `send_goal {id:1, pwm:1700, duration_ms:1200}` | `SUCCEEDED`，`final_pwm=1687` |
| ③ | `ros2 param set /servo_driver pwm_max 1600` | `Set parameters successful` |
| ④ | 再发同一个 `pwm=1700` | **`Goal rejected`**；服务端：`拒绝: pwm=1700 超出安全区间 [900,1600]` |
| ⑤ | `ros2 param set /servo_driver pwm_max 100000` | `Setting parameter failed: pwm_max=100000 超出电气范围 500~2500` |
| ⑥ | `ros2 param set /servo_driver pwm_min 2200` | `Setting parameter failed: pwm_min(2200) 必须小于 pwm_max(1600)` |
| ⑦ | `ros2 param set /servo_driver pwm_max 2100` + 再发 1700 | `2100` / `SUCCEEDED`，`final_pwm=1691` |

④ 是关键：**同一个 1700，改参数前能过、改完立刻被拒、改回来又能过**，证明判断逻辑读的是实时参数。

---

## 8. bag（交付数据）

- 录制内容：**只录** `/servo/command` 与 `/servo/state`，**不使用 `-a`**
- 位置：`bag/run1/`（`metadata.yaml` + `.db3`）
- 时长 ≤ 30 s，体积 ≤ 10 MB

录制：

```bash
ros2 bag record -o ~/cq_ws/bag_run1 /servo/command /servo/state
# 另一个窗口发 3 条 goal（15 秒内完成）
# 回到录制窗口 Ctrl+C
```

`ros2 bag info` 实测（数值以实际提交的 bag 为准）：

```text
Files:             bag_run1_0.db3
Bag size:          ~33 KiB
Storage id:        sqlite3
Duration:          12~20 s
Messages:          ~20
Topic information: Topic: /servo/command | Type: cquick_servo_interfaces/msg/ServoCommand | Count: 3~4
                   Topic: /servo/state   | Type: cquick_servo_interfaces/msg/ServoCommand | Count: ~15
```

回放：

```bash
ros2 bag play ~/cq_ws/bag_run1
```

**回放结果**：舵机按 1700 → 1200 → 1500 重演，与录屏动作一致。

> 为什么能重演：Action 执行时会把 goal **镜像发布**到 `/servo/command`，驱动本身订阅该话题并调用**同一个 `_send()`**，所以回放和实时走的是同一条发送路径。为避免自己收到自己发的消息导致重复执行，订阅回调里有一个时间窗抑制（`_ignore_topic_until`）。

---

## 9. 接线说明

```
电脑/板卡 ──USB──> USB-TTL(CH340) ──TX/GND──> 24 路控制板 ──> 舵机
                                        ↑
                                     电池 7.93V（实测）
```

| 线 | 接到 | 注意 |
|---|---|---|
| USB-TTL `TX` | 控制板 `RX`（或单线 `S`） | 半双工板部分只需 TX + GND |
| USB-TTL `GND` | 控制板 `GND` | **必须共地**，漏接则只发不收 |
| 电池 红 | 控制板 `+`（VCC） | 5–8.4 V，**绝对不能接 12 V** |
| 电池 黑 | 控制板 `-`（GND） | 与 USB-TTL 的 GND 同一电位 |

舵机三线（对着通道丝印 `S / + / -` 插）：

| 颜色 | 含义 |
|---|---|
| 橙 / 白 | 信号 S |
| 红 | 正极 +（5–8.4 V） |
| 棕 / 黑 | 负极 GND |

**实测电池电压：7.93 V**（2S 锂电池，满电 8.4 V）。

> 接电池瞬间会有一次小火花，属控制板电容充电浪涌（见调试记录 2.8），一次插到底可减小；长期建议换 XT30 防打火插头或加开关。

---

## 10. 动作组：动作是数据，不是代码

动作定义放在 `config/actions/*.yaml`，播放器 `sequence_player` 负责读文件、展开成 goal、按 result 同步。**换一套动作只新增一个 yaml，不改一行 python。**

### 10.1 yaml 结构

```yaml
name: stand_up
steps:
  - name: 同时抬前两个关节        # 同一 step 内 = 并行下发
    joints:
      - {id: 1, pwm: 1650, duration_ms: 1200}
      - {id: 2, pwm: 1350, duration_ms: 1200}
  - name: 再抬后两个关节
    joints:
      - {id: 3, pwm: 1600, duration_ms: 1200}
      - {id: 4, pwm: 1400, duration_ms: 1200}
  - name: 全部回中位
    joints:
      - {id: 1, pwm: 1500, duration_ms: 1200}
      - {id: 2, pwm: 1500, duration_ms: 1200}
      - {id: 3, pwm: 1500, duration_ms: 1200}
      - {id: 4, pwm: 1500, duration_ms: 1200}
```

三种表达能力的对应关系：

| 要表达 | 怎么写 |
|---|---|
| 一步之内多舵机协同 | 同一个 `step.joints` 里放多条，**同时下发** |
| 步与步之间串行 | 不同 `step`，播放器等上一步**全部 result 回收**才发下一步 |
| 一步之内有先后 | 拆成两个独立 `step`。**不要靠调 `duration_ms` 硬凑先后** |

### 10.2 播放

```bash
SHARE=$(ros2 pkg prefix --share cquick_servo)
ros2 run cquick_servo sequence_player --ros-args \
  -p action_file:=$SHARE/config/actions/stand_up.yaml -p repeat:=3
```

- `repeat:=3` 用于"连跑三遍"一致性验证
- 日志每行带 `HH:MM:SS.mmm`，可看到 `[result] ...` 时间戳严格串行 → 证明是**用 result 同步**，不是 `sleep`

### 10.3 为什么不用 sleep

舵机是"按 `duration_ms` 平滑走位"，实际到位时刻 ≠ `duration_ms`（受负载、死区影响）。用 `sleep(1.0)` 硬等会累积误差，第三遍和第一遍动作必然对不上。播放器统一用 `result` 作为同步信号。

---

## 11. 踩坑记录（摘要，详见 `调试记录.md`）

| # | 坑 | 处理 |
|---|---|---|
| 1 | `--symlink-install` 与旧 `build/` 目录冲突，报 "existing path cannot be removed: Is a directory" | 删 `build/ install/ log/` 后统一使用 `--symlink-install` |
| 2 | `PULR` 后 20 ms 就发位置指令，指令被丢弃，舵机纹丝不动 | 改为 **300 ms**；用手动脚本验证"手动能转、节点不能转"定位到间隔问题 |
| 3 | `duration_ms < 1000` 时位置指令不执行 | 默认值上调到 **1200**，演示统一用 1200 |
| 4 | `Ctrl+C` 取消 goal 无效（goal 跑满时长） | rclpy 吞 SIGINT → `signal.signal(SIGINT, signal.default_int_handler)`，并加 `--cancel-after` |
| 5 | bag 里 `/servo/command` 为 0 条，回放不动 | Action 执行时镜像发布到该话题 + 回环抑制 |
| 6 | bag 时长 143.9 s 超标 | 录制与发 goal 控制在 15 s 内 |
| 7 | 虚拟机里 `/dev/ttyUSB0` 存在但报 `Input/output error` | 先查 `lsusb`，重新做 USB 直通 |
| 8 | feedback 的 `current_pwm` 是插值，看着"到 1700"其实没动 | 判断到位一律看 `PRAD` / `result.final_pwm` |

---

## 12. 安全下电（三步，顺序不能反）

```text
① 卸力 → ② 断动力电（电池）→ ③ 关板卡
```

| 步骤 | 做什么 | 为什么 |
|---|---|---|
| ① 卸力 | 对所有 ID 发 `#xxxPULK!` | 舵机断电前还在顶力矩，直接断电极易堵转发热，可能几分钟就烧 |
| ② 断动力电 | 拔电池 | 先断动力，避免控制板在异常电压下乱发指令 |
| ③ 关板卡 | `sudo shutdown now` | 直接拔电会损坏文件系统，下次开不了机 |

卸力命令：

```bash
python3 - << 'PY'
import serial, time
s = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.3)
for i in (1, 2, 3, 4):
    s.write(("#%03dPULK!\r\n" % i).encode()); s.flush(); time.sleep(0.2)
    print("已卸力 ID", i)
s.close()
PY
```

四条红线：**禁止带电插拔舵机线**、**禁止反接电源**、**电池鼓包立即停用**、**禁止让舵机硬顶结构**。

---

## 13. 录屏（B 站）

| 段落 | 内容 | 链接 |
|---|---|---|
| ① | 正反转 + 回中位（feedback 滚动） | [https://b23.tv/GDMvOiG](https://b23.tv/GDMvOiG) |
| ② | 取消打断（`message=canceled`） | 待补 |
| ③ | 改参数当帧生效（六步） | 待补 |
| ④ | `ros2 bag play` 复现动作 | 待补 |
| ⑤（任务 3） | 整机逐个舵机动作 | 待补 |
| ⑥（任务 3） | 动作组连跑三遍 + 换 yaml | 待补 |
| ⑦（任务 4） | 安全下电流程 | 待补 |

> 仓库不放视频，只放截图和 bag。
>
> ①～④ 若合成为一个多 P 视频，一个链接即可，注明 P1/P2/P3/P4。

---

## 14. 已知问题与后续计划

| 项目 | 现状 | 影响 | 下一步 |
|---|---|---|---|
| 控制板无回显 | 11-22 起单舵机最小系统 `--scan` 返回 `[]`。已逐项排除：串口设备与权限、115200 8-N-1、TX/RX 直连与交叉两种接法、只接 TX+GND（半双工单线）、GND 共地、更换 USB 口 | 任务 3 的"整机逐个控制"与"动作组"无法完成实物验证 | ① 测控制板通道 `+`/`-` 电压，区分"供电异常"与"通信异常"；② 借同型号板子交叉验证（同一 TTL、同一舵机、同一线序）；③ 更换后按"单舵机点亮 → 标定 → 装配"重走一遍 |
| 舵机 ID 重号 | 接入第二个舵机后 1 号失联，单独接第二个时读回的仍是 ID 1 | 无法单独控制某一个舵机，多舵机动作组受限 | 取得舵机型号后按厂家协议修改 ID（未确认型号前不操作，避免写坏 ID） |
| 结构限位未全程实测 | `pwm_min/pwm_max` 取 900/2100，按文档建议值 + 小幅度（1200~1750）验证确定 | 极限姿态下的安全余量未经实测 | 逐关节逼近限位后回退 50 再定稿，写入参数表 |
| udev 固定设备名未启用（加分项） | 规则内容已备好（附录 A），未写入 `/etc/udev/rules.d/` | 插拔顺序变化时 `ttyUSB0` 可能变成 `ttyUSB1` | 写入规则并验证 `/dev/cq_servo` 软链接稳定，再把 `port` 默认值改过去 |

> 说明：动作组播放器（`sequence_player.py`）与两套 yaml 已完成并通过编译，目前仅因控制板问题未做实物联调；软件侧逻辑（step 内并行、step 间 result 同步）已按接口契约实现。

---

## 附录 A：udev 固定设备名规则（待启用）

```bash
sudo tee /etc/udev/rules.d/99-cq-servo.rules > /dev/null << 'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0660", GROUP="dialout", SYMLINK+="cq_servo"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
# 拔插一次后验证
ls -l /dev/cq_servo
```

启用后把 `servo_params.yaml` 的 `port` 改为 `/dev/cq_servo`，代码无需改动。
