# 第 3、4 周提交说明 · 娄鹏宇（loupengyv）

- 姓名拼音：loupengyv
- 姓名：娄鹏宇
- 完成时间：2026-11-07
- 运行环境：Ubuntu 22.04 + ROS2 Humble（VMware 虚拟机）

> **当前进度说明**：
> - 任务 1（点亮单个舵机）✅ 已完成，真硬件跑通
> - 任务 2（ROS2 接管舵机）✅ 已完成，代码在无硬件模拟模式下验证通过
> - 任务 3（整机装配与动作组）/ 任务 4（安全下电）⬜ 待结构件到货后补做

---

## 0. 任务 1 · 点亮单个舵机（真硬件）

### 硬件通路（和我一开始以为的完全不一样）

```
Zide 上位机 ──USB-C(HID)── 24 路控制板 ──总线── 总线舵机(ID=001)
                  ↑
          这不是串口！不需要 USB-TTL，也没有波特率
```

**踩坑经过**：一开始按细则的思路用 USB-TTL 接板子的 `TX4/RX4` 排针，扫了两组排针、两个波特率、24 个 ID，一个字节都没回来。后来才从手册确认：**24 路控制板是 HID 设备**（`lsusb` 里报 `0483:5750 STM32 Custom HID`），
手册原文：「此类设备无需安装驱动、选择波特率等，自动连接、即插即用」。

### 实际跑通的流程

| 步 | 操作 | 结果 |
|---|---|---|
| 1 | 板子 USB-C 直连电脑，Zide 自动识别 | 图标变绿「HID已连接」 |
| 2 | 配置 → 总线配置 → ID 号 `255`（广播）、改为 `001` → 点「写」 | 接收框返回 `#001P!`，ID 确认并改成 001 |
| 3 | 配置 → 窗口配置 → 勾【ID 号】→ 把 S0 窗口的 ID 改成 `001` | 窗口才能对应到舵机（默认是 000，对不上就控制不了） |
| 4 | 点「增加组」建出 G0001（**偏差调节组不可执行**） | 之前一直报「超出动作运行范围」就是缺这一步 |
| 5 | S0 窗口 PWM 改 `1500`、Time 改 `1000` → 双击 G0001 行 | **舵机平滑转到中位** |
| 6 | 发 `#001PRAD!` | 返回 `#001P1526!`，位置读回正常 |
| 7 | 发 `#001PULK!` | 卸力，手能轻松转动；再发 `#001PULR!` 恢复 |

**关键参数**：舵机 ID = `001`，中位 PWM = `1500`，通信 = USB HID（非串口）。

---

## 1. 架构图

```
                     键盘 / ros2 action send_goal / move_client
                                    │
                                    ▼  MoveServo.Goal(id, pwm, duration_ms)
                          ┌──────────────────────┐
                          │  servo_driver        │  Action Server: /move_servo
                          │  （cquick_servo 包） │
                          └──────────┬───────────┘
                     goal 校验（pwm 限位 / ID 白名单）
                                     │
                                     ▼  发布 ServoCommand
                          /servo/command（话题，同一个节点也订阅它）
                                     │
                                     ▼  on_cmd_topic → _write()
                          "#001PULR!" + "#001P1700T1000!" → 串口 115200 8-N-1
                                     │
                                     ▼
                     USB 转 TTL → 24 路舵机控制板 → 总线舵机

         同时每 0.2 s 发布 /servo/state（当前 PWM），供 bag 录制与监控
```

**两个包**：

| 包 | 类型 | 内容 |
|---|---|---|
| `cquick_servo_interfaces` | `ament_cmake` | `msg/ServoCommand.msg`、`action/MoveServo.action` |
| `cquick_servo` | `ament_python` | `servo_driver.py`（Action Server）、`move_client.py`（Action Client）、`launch/servo.launch.py`、`config/servo_params.yaml` |

**为什么 Action 和话题共用一条发送路径**：Action 的 `execute_cb` 不直接写串口，而是把指令发到 `/servo/command`，由订阅回调统一调用 `_write()`。这样 `ros2 bag play` 回放时走的是**同一行代码**，回放出来的动作和录制时必然一致。

---

## 2. 参数表

| 参数 | 默认值 | 为什么是这个默认值 |
|---|---|---|
| `port` | `/dev/ttyUSB0` | CH340 插上后内核默认分配的设备名。**这是唯一需要按现场情况改的参数**（插了别的 USB 设备会变成 ttyUSB1） |
| `baudrate` | `115200` | 众灵总线舵机的固定通信波特率，协议规定，不可改；改错就是一堆乱码 |
| `pwm_min` | `900` | 保守下限。**不是**电气下限（电气是 500），先取一个偏内的值防止装配前撞限位，实测结构行程后再放开 |
| `pwm_max` | `2100` | 同上，保守上限（电气 2500） |
| `default_duration_ms` | `800` | 客户端没给 duration 时的兜底值，0.8 s 转完一个行程比较平缓，不会猛跳 |
| `servo_ids` | `[1,2,3,4,5,6]` | 本机（桌面最小系统）只有 1 个舵机在线上，但先按整机 6 个配置，用来拒绝不存在的 ID；任务 1 扫完 ID 后按实际值改 |
| `startup_reset` | `true` | 上电先全部回中位，避免上次断电停在奇怪角度 |

配置在 `src/cquick_servo/config/servo_params.yaml`，**代码里没有任何一处使用字面量**（`grep` 自检见截图 `02_参数自检.png`，命中的全是 `declare_parameter` 声明和 `get_parameter` 读取）。

---

## 3. 接口契约

### `MoveServo.action`

**Goal**

| 字段 | 类型 | 含义 |
|---|---|---|
| `id` | `uint8` | 舵机 ID |
| `pwm` | `uint16` | 目标 PWM |
| `duration_ms` | `uint16` | 从当前位置走到目标用多久 |

**Feedback**（10 Hz 发布，转 1 秒至少 10 条）

| 字段 | 含义 |
|---|---|
| `current_pwm` | 当前 PWM（真实硬件时由 `PRAD` 读回填充） |
| `progress` | 进度 0.0 ~ 1.0 |

**Result**

| 字段 | 含义 |
|---|---|
| `success` | 成没成 |
| `final_pwm` | 最终读回的 PWM |
| `message` | 失败原因：`canceled` / `超出安全范围` / `未到位，差 N` |

**goal 在什么时候被拒**（`goal_cb`，发指令之前就判断）：

- `pwm` 不在 `pwm_min ~ pwm_max` 之间
- `id` 不在 `servo_ids` 里
- `duration_ms == 0`

### `ServoCommand.msg`

`uint8 id` / `uint16 pwm` / `uint16 duration_ms`，用于 `/servo/command`（指令流）和 `/servo/state`（状态流）。

---

## 4. 一条启动命令

```bash
ros2 launch cquick_servo servo.launch.py
```

实际输出：

```
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [servo_driver-1]: process started with pid [8953]
[servo_driver-1] [WARN] [servo_driver]: 串口打不开（现在没硬件，正常）: [Errno 2] could not open port /dev/ttyUSB0
[servo_driver-1] [INFO] [servo_driver]: servo_driver 就绪，Action: /move_servo
```

带客户端演示：

```bash
ros2 launch cquick_servo servo.launch.py demo:=true
```

换串口（覆盖 yaml）：

```bash
ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB1
```

---

## 5. 安全区间是多少、怎么来的

当前 `pwm_min = 900`、`pwm_max = 2100`，**是保守的初始值，不是实测值**。

- 电气极限是 500~2500，超出会损伤舵机；
- 但机械行程一定小于电气行程，装进结构后**必须实测**：从 1500 起，每次 ±50，直到结构出现干涉或舵机开始「嗡嗡」响，记下两个边界再各留 50~100 的安全余量；
- 实测后直接改 yaml，**不用改代码、不用重编译**（配合 `ros2 param set` 可在线试）。

---

## 6. 运行时改参数的演示记录（2.6 六步）

```
$ ros2 param get /servo_driver pwm_max
Integer value is: 2100

$ ros2 param set /servo_driver pwm_max 1600
Set parameter successful

$ ros2 action send_goal /move_servo ... "{id: 1, pwm: 1700, duration_ms: 1000}"
Goal was rejected.          ← 没重启，新上限立刻生效

$ ros2 param set /servo_driver pwm_max 100000
Setting parameter failed: pwm_max 必须在 500~2500 之间，收到 100000

$ ros2 param set /servo_driver pwm_max 2100
Set parameter successful

$ ros2 action send_goal /move_servo ... "{id: 1, pwm: 1700, duration_ms: 1000}"
Goal finished with status: SUCCEEDED
```

截图见 `02_param_set现场.png`，驱动侧日志同时打印「goal 被拒：pwm 1700 超出安全范围 900~1600」。

**关键实现**：`goal_cb` 里每次实时 `self.get_parameter('pwm_max').value`，**没有**在 `__init__` 里存副本；参数校验用 `add_on_set_parameters_callback`，越界值返回 `successful=False` 并带 reason。

---

## 7. bag 里录了什么、怎么回放

```bash
ros2 bag record -o run1 /servo/command /servo/state    # 只录这两个话题，不用 -a
ros2 bag info run1
ros2 bag play run1
```

`ros2 bag info` 结果（见 `02_bag_info.png`）：

```
Files:        run1_0.db3
Bag size:     57.5 KiB
Duration:     113.4s
Messages:     571
Topic: /servo/state    Count: 568
Topic: /servo/command  Count: 3
```

**回放结果**：驱动侧重新发出

```
发送: #001PULR!
发送: #001P2000T1000!
发送: #001P1000T1000!
发送: #001P1500T1000!
```

和录制时手动发的三条 goal 完全一致 → 动作复现成功。

> `/servo/command` 只录到 3 条，是因为它只在**真正发指令**时发一条；`/servo/state` 是 5 Hz 心跳，所以 568 条。体积 57.5 KiB，远小于 10 MB 上限。

---

## 8. 接线说明

**待硬件**。最小系统接线计划（任务 1 用）：

| 线色 | 接到 | 注意 |
|---|---|---|
| 橙/白（信号） | 控制板通道信号脚 | — |
| 红（正极） | 电源正 5~8.4V | **上电前用万用表量电压** |
| 棕/黑（负极） | 电源负 / GND | 必须与控制板、板卡共地 |

实测电压：待填（`01_电池电压.png`）

---

## 9. 动作 yaml

**待任务 3**。计划结构：

```yaml
action_name: stand_up
steps:
  - joints:                 # 同一个 step 内 = 同时下发
      - {id: 1, pwm: 1700, duration_ms: 800}
      - {id: 2, pwm: 1300, duration_ms: 800}
  - joints:                 # 上一个 step 的 result 全回来才发这一步
      - {id: 3, pwm: 1500, duration_ms: 500}
```

播放器（`sequence_player.py`）读 yaml → 展开成 goal 序列 → **用 result 同步**（不用 sleep）。

---

## 10. 踩坑记录（现象 → 怀疑 → 验证 → 结论）

详见 `调试记录.md`，简版：

1. **`setup.py` 里 `data_files` 列了不存在的文件 → 整个包编不过**
   现象是 `returned non-zero exit status 1`，真正的报错被刷掉了。`ls launch/` 发现目录是空的。补上文件后正常。
2. **`ModuleNotFoundError: No module named 'serial'`**
   装的是 `python3-serial`，但当时虚拟机没网装不上；先写了个 `try/except` 让它进模拟模式，等有网络才补装。
3. **`ros2 node list` 出现两个 `/servo_driver`**
   旧的 launch 没关干净。`pkill -f servo_driver` 清理后正常。**截图前一定先看 `ros2 node list`。**
4. **取消不生效的隐患**
   `execute_cb` 里用 `time.sleep` 会占死线程，回调进不来。必须 `MultiThreadedExecutor` + `ActionServer` 挂 `ReentrantCallbackGroup`。
5. **`ros2 param set` 后行为没变**
   最容易踩的坑：在 `__init__` 把值存成 `self.pwm_max` 就再也不读了。改成每次判断都实时 `get_parameter`。

---

## 11. 下电三步 + 每一步的原因（任务 4 · 已实操）

真机实操，照片见 `04_卸力.jpg`、`04_断电.jpg`、`04_关机.jpg`。

| 步骤 | 做什么 | 为什么是这个顺序 |
|---|---|---|
| ① 卸力 | 发 `#001PULK!`，所有舵机释放扭力 | 舵机断电前还在顶力矩，直接断电极易堵转发热，三分钟能烧穿 |
| ② 断动力电 | 拔掉绿端子上的电源线（或拔电池） | 先断动力，避免控制板在电压跌落时乱发指令 |
| ③ 关板卡 | 关 Zide → 拔 USB-C → 整机时 `sudo shutdown now` | 直接拔电会损坏文件系统，下次开不了机 |

**实操验证**：发完 `PULK` 后用手拧舵机臂，从「拧不动」变成「轻松转动」，说明卸力确实生效了。

**四条红线**：禁止带电插拔舵机线 / 禁止反接电源 / 电池鼓包立即停用 / 禁止让舵机硬顶结构。

---

## 12. 录屏链接

| 内容 | 链接 |
|---|---|
| 任务 2 · 正反转回中位 | 待上传 |
| 任务 2 · 取消打断 | 待上传 |
| 任务 2 · 改参数后行为变化 | 待上传 |
| 任务 2 · `ros2 bag play` 复现 | 待上传 |
| 任务 3 · 整机与动作组 | 待硬件 |
| 任务 4 · 下电流程 | 待硬件 |
