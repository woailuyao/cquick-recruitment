# 第 3、4 周 · 整机拼装与舵机控制

> 提交人：朱文轩
> 仓库路径：`电控组/Week3,4/zhuwenxuan/`

---

## 1. 架构图

```
ros2 launch cquick_servo servo.launch.py
        │
        ├── servo_driver（Action Server）
        │     goal_callback ── 校验：ID 在 servo_ids？pwm 在 [pwm_min, pwm_max]？
        │     execute_callback
        │        ① #xxxPULR!      恢复扭力
        │        ② #xxxP<pwm>T<ms>!  下发位置
        │        ③ 每 100ms 发 feedback（current_pwm / progress）
        │        ④ #xxxPRAD!     读回位置，差 > ±30 判未到位
        │        中途 cancel → 回中位，result.message = "canceled"
        │
        ├── /servo/command（订阅）→ 与 Action 走同一个发送函数（给 bag 回放用）
        └── /servo/state（发布，2 Hz）→ 当前每个舵机的目标位置

sequence_player（动作组播放器）
        yaml → steps → 每一步并发下发 goal → 等所有 result → 下一步
```

数据流：`goal → 校验 → PULR → P<pos>T<ms> → feedback → PRAD 读回 → result`

---

## 2. 参数表

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `port` | string | `/dev/ttyUSB0` | 串口设备 |
| `baudrate` | int | `115200` | 只允许 9600/19200/38400/57600/115200/230400 |
| `pwm_min` | int | `900` | 安全下限，越界 goal 被拒 |
| `pwm_max` | int | `2100` | 安全上限 |
| `default_duration_ms` | int | `800` | goal 没给 duration 时用 |
| `servo_ids` | int[] | `[1..10]` | 允许控制的 ID，不在表里的直接被拒 |
| `startup_reset` | bool | `false` | 启动是否把所有舵机拉回中位 |

全部从 `config/servo_params.yaml` 加载，代码里没有硬编码（见 `02_参数自检.png`）。

---

## 3. 接口契约

**msg/ServoCommand.msg**
```
uint8  id
uint16 pwm            # 电气范围 500~2500，安全范围由参数决定
uint16 duration_ms
```

**action/MoveServo.action**
```
# Goal
uint8  id
uint16 pwm
uint16 duration_ms
---
# Result
bool   success
uint16 final_pwm      # PRAD 读回值
string message        # OK / NOT REACHED / canceled / serial error
---
# Feedback
uint16 current_pwm
float32 progress      # 0.0 ~ 1.0
```

---

## 4. 启动命令

```bash
colcon build --packages-select cquick_servo_interfaces cquick_servo
source install/setup.bash
ros2 launch cquick_servo servo.launch.py          # 一条命令起全套
ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB1   # 换串口
```

---

## 5. 安全区间的来历

- 电气极限 **500–2500**：手册给的，超出会打到结构或堵转
- 实际安全区间 **900–2100**：逐个舵机点动测出来的——往两端走，听到结构顶住或电流声就回退 50，取能自由活动的区间
- 出厂中位是 1500，实测读回 1492–1510 都算正常（齿轮间隙）

---

## 6. 参数六步的输出

```
1) ros2 param get /servo_driver pwm_max      → 2100
2) ros2 param set /servo_driver pwm_max 1600 → successful
3) 发 goal pwm=1700                          → Goal was rejected   ← 当帧生效
4) ros2 param set /servo_driver pwm_max 100000 → failed: out of range 500~2500
5) ros2 param set /servo_driver pwm_max 2100 → successful
```

---

## 7. bag 说明

```bash
ros2 bag record -o bag/run1 /servo/command /servo/state
ros2 bag info bag/run1        # /servo/command 3 条 + /servo/state 63 条
ros2 bag play bag/run1        # 舵机自己复现动作
```

只录这两个话题（不加 `-a`），文件 32 KB。回放时 `/servo/command` 走的是和 Action 相同的下发路径，所以能复现。

---

## 8. 接线

```
电池 +  → 控制板 V+        舵机 棕/黑 → GND
电池 −  → 控制板 GND       舵机 红    → V+
USB-TTL TX → 控制板 RX1    舵机 橙/白 → 信号
USB-TTL RX → 控制板 TX1
USB-TTL GND → 控制板 GND   ← 漏了就百分百没回包
```

TTL 的 5V / 3V3 **不接**（会跟电池对打）。

---

## 9. 动作组怎么加

`config/actions/` 下每个 yaml 就是一套动作，**换动作只改 yaml，不动代码**：

```yaml
name: stand_up
steps:
  - name: 屈膝进入站姿
    moves:
      - {id: 5, pwm: 1200, duration_ms: 1400}
      - {id: 6, pwm: 1200, duration_ms: 1400}
```

- 同一个 step 里的 move **并发**下发
- step 之间**串行**：上一步所有 result 回来才发下一步
- 要让某个关节先动，就单独开一个 step —— 不要靠调 `duration_ms` 硬凑先后

```bash
ros2 run cquick_servo sequence_player --list
ros2 run cquick_servo sequence_player --action stand_up
ros2 run cquick_servo sequence_player --file 新动作.yaml   # 现场给的新动作
```

---

## 10. 踩过的坑

1. **舵机 ID 不是 0–9 而是别人改过的 000/002/004/006**：一开始按 `#000` 发指令，一半舵机没反应。扫一遍 ID（`#XXXPID!`）才发现编号是乱的，最后统一改成 001–010。

2. **`PULK` 卸力之后舵机锁死**：这个型号收到 `PULK` 后不再响应位置指令，`PULR` 也叫不回来，只能断电重启。所以退出时不发 `PULK`，只在“确定要断电”的下电流程最后一步才用。

3. **PRAD 读太早导致误判“没到位”**：舵机是「按 T 毫秒平滑转过去」的，到点立刻读会读到上一个位置。加了 0.35 s 等待 + 最多重试 3 次。

4. **低电量时快转带不动**：7.7 V 时 1.2 s 能到位，往下到 6.8 V 时同样指令会卡住不动，看起来像舵机坏了。先量电压再怀疑代码。

---

## 11. 下电三步

1. `reset_all` 全部回中位 —— 避免带着姿态断电
2. `PULK` 卸力 —— 齿轮和电机不受力
3. 断电池 → 关板卡

反过来（先断电）会让舵机带着力突然掉电，齿轮受冲击，也容易憋着电流发热。

---

## 12. 录屏

| 内容 | 链接 |
|---|---|
| 任务 1 舵机点亮 / 正反转 / 回中位 | https://www.bilibili.com/video/BV1fCyJzSEiU |
| 任务 2 反馈 + 取消 + bag 回放 | https://www.bilibili.com/video/BV1fCyJzSExZ |
| 任务 3 整机站立 + 踏步动作组 | https://www.bilibili.com/video/BV1JCyJzDEAC |
| 任务 4 安全下电（回中位 → 卸力 → 断电池） | https://www.bilibili.com/video/BV1fCyJzSEsM |

> 任务 2 的「改参数当帧生效」这一项，证据见 `02_param_set现场.png`：设 `pwm_max=1600` 之后不重启，
> 再发 `pwm=1700` 的 goal 立刻被拒，最后恢复 2100。这一段没有单独录像，以截图 + 日志为准。
