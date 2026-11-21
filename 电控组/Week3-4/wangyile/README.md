# 电控组 Week3,4 考核提交 - wangyile

## 1. 系统整体架构

整个控制链路分为自定义接口包 `cquick_servo_interfaces` 和功能驱动包 `cquick_servo`：

```text
[命令行 / YAML动作组] ──(MoveServo.action)──► [servo_driver 节点] ──(115200 串口)──► [24路控制板] ──► [1~10号舵机]
                                                     ▲
[ros2 bag play] ─────────(/servo/command)────────────┘
```

* `cquick_servo_interfaces`：定义 `ServoCommand.msg` 话题消息和 `MoveServo.action` 动作接口。
* `cquick_servo`：包含底层驱动 `servo_driver`、测试客户端 `move_client` 和多舵机动作播放器 `sequence_player`。

---

## 2. 核心参数表 (`config/servo_params.yaml`)

| 参数                    | 默认值            | 说明                           |
|:--------------------- |:-------------- |:---------------------------- |
| `port`                | `/dev/ttyUSB0` | CH340 串口设备名（可在 launch 时传参修改） |
| `baudrate`            | `115200`       | 舵机控制板固定通信波特率                 |
| `pwm_min` / `pwm_max` | `900` / `2100` | 结合整机结构实测的安全脉宽上下限             |
| `default_duration_ms` | `800`          | 默认单次运动时间（ms）                 |
| `servo_ids`           | `[1..10]`      | 本机 10 个下肢总线舵机的合法 ID 列表       |
| `startup_reset`       | `true`         | 启动恢复扭力后自动回 1500 中立位          |

---

## 3. 接口说明 (`MoveServo.action`)

* **Goal**：目标舵机 `id`、目标脉宽 `pwm`、运动时间 `duration_ms`。在 `goal_callback` 中校验 ID 是否在列表内及 PWM 是否越界，越界直接拒绝。
* **Feedback**：实时估算的当前脉宽 `current_pwm` 和运动进度 `progress`（频率 ≥ 5Hz）。
* **Result**：是否成功 `success`、动作结束后发送 `#xxxPRAD!` 读回的真实脉宽 `final_pwm` 以及结果说明 `message`。

---

## 4. 一键启动与运行输出

启动命令：

```bash
ros2 launch cquick_servo servo.launch.py
```

终端输出：

```text
[INFO] [servo_driver-1]: process started
[servo_driver-1] [INFO] [servo_driver]: 成功打开串口: /dev/ttyUSB0 @ 115200
[servo_driver-1] [INFO] [servo_driver]: Action Server /move_servo 已就绪
```

---

## 5. 安全区间 (`900 ~ 2100`) 确定过程

总线舵机原始范围是 `500 ~ 2500`。装上 3D 打印件后，先断电手掰确认大致活动余量，再以 50 PWM 为步长逐点测试，发现在小于 `900` 或大于 `2100` 时关节连杆会顶到打印件产生堵转声，因此要将安全限位锁定在 `900 ~ 2100`。

---

## 6. 运行时动态改参数验证

1. 先发送 `pwm=1700`（在默认 `2100` 范围内），动作正常执行并返回 `success=True`。
2. 执行 `ros2 param set /servo_driver pwm_max 1600` 在线收紧上限。
3. 再次发送 `pwm=1700`，被 `goal_callback` 实时拦截并 `REJECT`（提示超出 `[900, 1600]`）。
4. 尝试设置非法值 `pwm_max=100000`，参数回调拒绝修改并报错；最后恢复 `pwm_max=2100`，重发 `1700` 恢复正常。

---

## 7. ROS 2 Bag 录制与回放

录制时只指定了 `/servo/command` 和 `/servo/state` 两个话题（未使用 `-a`），包体积在 10MB 以内。回放 `ros2 bag play bag/run1` 时，`servo_driver` 订阅 `/servo/command` 接收历史指令并下发串口，舵机完整复现了录制时的动作。

---

## 8. 硬件接线与实测电池电压

* **接线**：锂电池正负极接控制板电源端；CH340 与控制板串口交叉连接（`TXD-RXD`、`RXD-TXD`、`GND-GND` 共地，`5V` 悬空）；1~10 号总线舵机串联接入总线口。
* **实测电压**：上电前万用表实测锂电池空载电压为  7.62V（在 5–8.4V 的正常范围内）。

---

## 9. 多舵机动作组与同步方式

动作组存放在 `src/cquick_servo/config/actions/` 下。`sequence_player` 读取 YAML 后，同一个 `step` 内的多个关节目标会同时异步发送 Goal 实现协同；程序会等待当前 `step` 所有舵机的 `Action Result` 全部返回后，再执行下一个 `step`，避免纯靠固定 `sleep` 导致时间累积偏差。

---

## 10. 遇到的问题与解决（详见 `调试记录.md`）

1. **识别不到 `ttyUSB0`**：查 `dmesg` 发现被系统的 `brltty` 服务抢占，卸载 `brltty` 后恢复正常。（后续任务三后换了一个转换头，不知道为啥出现了USB1，再次重新插拔后又变回USB0）
2. **节点启动报 `AttributeError`**：`__init__` 中上电复位发消息写在了 `create_publisher` 之前，调换初始化顺序后解决。
3. **新终端报错 `action type is invalid`**：未加载当前工作空间环境变量，执行 `source ~/cquick_ws/install/setup.bash` 后识别。

---

## 11. 安全下电流程

1. **先卸力**：向 1~10 号舵机发送 `#xxxPULK!` 释放电机保持力矩，防止断电瞬间重力下落或堵转烧坏驱动。
2. **再断动力电**：关闭控制板电源开关，拔掉锂电池插头，避免电池过放及低电压乱码。
3. **最后关上位机**：Ubuntu 安全关机后再拔除 USB 转 TTL 模块，保护文件系统与串口芯片。

---

## 12. B 站演示视频链接

* **任务 2（单关节控制 / 取消 / 在线改参数 / Bag 回放）**：https://t.bilibili.com/1273056842343252000?share_source=pc_native
* **任务 3（整机逐关节测试与多舵机动作组连跑）**：https://t.bilibili.com/1273057903177105408?share_source=pc_native
* **任务 4（完整安全下电流程）**：https://t.bilibili.com/1273057959037894673?share_source=pc_native
