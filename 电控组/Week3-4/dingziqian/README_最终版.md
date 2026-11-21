# Week 3–4 电控组招新考核

**姓名：** dingziqian  
**项目：** ROS 2 总线舵机控制、整机装配与动作组  
**提交目录：** `电控组/Week3-4/dingziqian/`

本目录按任务编号保存考核材料、ROS 2 工程、动作数据和 rosbag。录屏放在 B 站，仓库仅保存链接、截图和 bag。

## 任务视频

以下按发送顺序对应任务 1–4：

| 任务 | 内容 | 视频 |
| --- | --- | --- |
| 1 | 串口通信、舵机 ID、中立位标定 | [B 站视频](https://www.bilibili.com/video/BV16Ns86mEQw/) |
| 2 | ROS 2 Action 接管舵机、参数与 launch | [B 站视频](https://www.bilibili.com/video/BV1kcs86nEox/) |
| 3 | 整机装配与多舵机动作组 | [B 站视频](https://www.bilibili.com/video/BV1S7y4zDEKg/) |
| 4 | 安全下电流程 | [B 站视频](https://www.bilibili.com/video/BV1mny4zUEYT/) |

## 目录说明

- `01_*`：任务 1 的串口设备、PID 回显、中立位标定、电池电压证据。
- `02_*`：任务 2 的接口编译、Action 反馈/取消、参数保护与运行时修改、launch 和 bag 信息。
- `03_*`：任务 3 的整机正侧面照片、接线特写、舵机 ID 扫描和动作关键帧。
- `src/cquick_servo_interfaces/`：`ServoCommand.msg` 与 `MoveServo.action` 自定义接口。
- `src/cquick_servo/`：驱动、Action 客户端、动作组播放器、launch 和 YAML 配置。
- `bag/run1/`：任务 2 的 rosbag（`metadata.yaml` 与 `.db3`）。
- `加分项/`：看门狗、掉线重连、抢占、PRAD 重发、pytest 和 udev 的材料与规则。
- `调试记录.md`：根据已有聊天和文件证据整理的调试过程；未证实的项目标为待验证。

## 系统结构

```text
ros2 launch cquick_servo servo.launch.py
                  │
                  ├── cquick_servo/servo_driver
                  │      ├── Action: /move_servo（MoveServo）
                  │      ├── Topic:  /servo/command（ServoCommand，bag 回放入口）
                  │      └── 串口总线 → 舵机
                  │
                  ├── cquick_servo/move_client（演示客户端）
                  └── cquick_servo/sequence_player（读取动作 YAML，按 result 同步步骤）
```

主要实现位于 `src/cquick_servo/cquick_servo/servo_driver.py`、`move_client.py` 和 `sequence_player.py`。接口定义位于 `src/cquick_servo_interfaces/`。

## 接口与控制方式

- Action 名称：`/move_servo`，类型：`cquick_servo_interfaces/action/MoveServo`。
- Goal 提供舵机 ID、目标 PWM 和动作时长；服务端校验 ID、PWM 安全限位及动作时长后执行。
- Feedback 报告当前 PWM 和动作进度。
- Result 报告成功状态、最终位置及失败原因；动作组播放器等当前步骤全部收到成功 result 后，才开始下一步。
- `/servo/command` 用于记录/回放舵机命令。动作 YAML 位于 `src/cquick_servo/config/actions/`，当前有 `stand_up.yaml` 与 `wave.yaml`。

字段的精确定义以 `src/cquick_servo_interfaces/action/MoveServo.action` 和 `msg/ServoCommand.msg` 为准。

## 参数与安全范围

`src/cquick_servo/config/servo_params.yaml` 当前记录的值：

| 参数 | 当前值 | 用途 / 选择依据 |
| --- | --- | --- |
| `port` | `/dev/cq_servo` | udev 固定别名，避免 ttyUSB 编号变化；规则见 `加分项/串口设备证据代码/99-cq-servo.rules`。 |
| `baudrate` | `115200` | 串口通信波特率，需与舵机总线设置一致。 |
| `pwm_min` | `900` | 当前配置的动作下限；应以装配后不干涉、不顶结构的实测安全边界为准。 |
| `pwm_max` | `2100` | 当前配置的动作上限；应以装配后不干涉、不顶结构的实测安全边界为准。 |
| `default_duration_ms` | `800` | 未指定时的默认动作时长，作为平滑动作的起始值。 |
| `servo_ids` | `[1]` | 当前参数文件配置的在线舵机 ID；应按本机扫描结果维护。 |
| `startup_reset` | `false` | 关闭启动自动回中，避免启动时未确认结构状态就驱动舵机。 |
| `feedback_hz` | `10.0` | Action 运行反馈频率。 |
| `simulate` | `false` | 使用真实串口舵机。 |

PWM 范围当前配置为 900–2100。实际使用前应结合整机无干涉、舵机无硬顶的检查确认安全范围；本说明不把配置值冒充结构极限实测结果。

## 启动与 bag 回放

在 ROS 2 Humble 工作区构建并 source 后：

```bash
ros2 launch cquick_servo servo.launch.py
```

bag 位于 `bag/run1/`，可检查并回放：

```bash
ros2 bag info bag/run1
ros2 bag play bag/run1
```

回放前先确认舵机供电、ID 和 PWM 限位。bag 元数据截图见 `02_bag_info.png`；回放动作与现场表现以任务 2 视频为准。

## 接线与安全下电

接线照片和电池读数见 `03_接线特写.jpg`、`01_电池电压.jpg`。上电前核对电源极性、电压及控制器和舵机总线的共地；禁止带电插拔舵机线。

下电顺序：

1. 对每个舵机发送对应 ID 的 `#xxxPULK!` 卸力（或使用上位机的卸力/释放功能），避免舵机继续顶力矩。
2. 拔除舵机动力电池，先断动力电。
3. 在板卡 Ubuntu 终端执行 `sudo shutdown now`，待系统关机后再断板卡电源，避免直接拔电损坏文件系统。

## 加分项说明

`加分项/` 存放加分实现材料。PRAD 闭环重发此前标记为待实机验证：需要实际观察“未到位 → 重发 → 到位”日志后，才能认定该场景验证完成。udev 规则按当前 VMware USB 路径匹配，更换虚拟 USB 端口后需重新核对 `ID_PATH`。
