# 第 3，4 周 —— 整机拼装与舵机控制

姓名：郑睿阳
姓名拼音：zhengruiyang
年级：大一
组别：第五组，电控

---
## 0.特别声明/事先声明/致评委
声明：
1.本人是本小组第一个开始做任务的，因一开始没看到要给舵机编号，所以视频中所出现的舵机没有编号，但是我第一个做不存在借他人材料做任务的情况，我也将情况如实告诉电控组组长，希望评委理解（我做完之后材料给下一个人了不方便重新做一遍）

2.本人所在小组因 缺少零件 缺少自攻螺丝等问题 导致整机迟迟无法拼装完成。我们组目前只拼了一条腿，故任务三 整机照片 动作组播放等都是在一只腿上完成的。情况已和电控组组长和队长说明，他们说可以调一只腿做动作，特此说明。

---

## 1. 架构图


    包 cquick_servo_interfaces
        MSG[msg: ServoCommand      uint8 id / uint16 pwm / uint16 duration_ms]
        ACT[action: MoveServo      Goal / Feedback / Result]
    

    包 cquick_servo
        SD[ servo_driver    节点 ActionServer「move_servo」]
        SP[ sequence_player 节点 ActionClient，批量发 goal]
        MC[ move_client     节点 ActionClient，单发 goal]
        WD[ watchdog        节点 订阅 /servo/state 心跳]

    （这几个字母是简写，我打起来方便点）
    SP -->|goal（按 yaml 顺序）| -->SD
    MC -->|goal（单发）| -->SD
    SD -->|串口 #xxxPxxxxTxxxx!| -->24路控制板
    24路控制板 -->|总线| -->总线舵机
    SD -->|心跳 /servo/state (Header, 0.5s)| -->WD
    SD <-->|/servo/command (ServoCommand)| SP
```

- **节点**：`servo_driver`（Action 服务器）、`sequence_player`（动作组播放器）、`move_client`（单发客户端）、`watchdog`（安全看门狗）。
- **Action**：`/move_servo`
- **话题**：`/servo/command`（bag 回放）、`/servo/state`（心跳）。

## 2. 参数表

| 参数 | 默认值 | 为什么是这个默认值 |
|---|---|---|
| `port` | `/dev/cq_servo` | 写了 udev 规则把 CH340 固定成这个名字，拔插/USB 编号漂移都不影响 |
| `baudrate` | `115200` | 手册规定的通信波特率 |
| `pwm_min` | `900` | 安全下限，沿用考核文档默认值，实测发现没毛病 |
| `pwm_max` | `2100` | 安全上限，同上 |
| `default_duration_ms` | `800` | 中等速度，兼顾平稳和响应 |
| `servo_ids` | `[1..10]` | 整条腿 10 个舵机 |
| `startup_reset` | `true` | 上电自动回中位，防止舵机在上电瞬间憋力烧毁 |
| `timeout_sec`(watchdog) | `1.0` | 考核要求"超 1 秒没心跳就卸力" |

## 3. 接口契约（MoveServo）

- **Goal**：`command`（ServoCommand）。 `id` 目标舵机、`pwm` 目标位置、`duration_ms` 转动时长；客户端发一次 goal 就是一次"转到哪"。
- **Feedback**：执行过程中每 200ms（5Hz）发一次 。`id` 哪个舵机、`current_pwm` 按进度估算的当前 PWM、`progress` 0~1 进度。
- **Result**：执行结束发一次 。`success` 是否到位、`message` 原因（ok / 没到位 / 重发后到位 / canceled）、`final_pwm` PRAD 读回的实际位置。

## 4. 一条启动命令

```bash
ros2 launch cquick_servo servo.launch.py &
```

输出：见02_launch一键启动.png （注意我后来加了watchdog节点，之后launch的输出可能多了个看门狗的东西）


## 5. 安全区间

- `pwm_min = 900`、`pwm_max = 2100`：沿用考核文档给的默认安全值；
我用2100之外的发现有一个舵机竟然堵转冒烟了，感觉900-2100的手册参考值确实很有道理，实测后发现这个范围舵机都能稳定转到。逐步+-20往外尝试发现都不太稳定，就900-2100吧

## 6. 运行时改参数演示（六步）

真实终端输出：见02_param_set现场.png


## 7. bag 说明

- **录了什么**：正转，反转，回中位。
- **怎么回放**：`ros2 bag play bag/run1`
- 回放后舵机自动复现刚才那套动作（回放前先回中位才看得出）。
- 回放结果：见02_bag_info.png 和 视频链接。

## 8. 接线说明

- **ZL-IS2 24 路控制板**：
  - 绿色螺丝端子 = 电源（5~8.4V，丝印 VOLT）；
  - 黄色端子 = 总线通信接口（接 CH340 转接板，CH340 的 USB 端插地瓜派 USB 口）；
  - 白色端子 = 总线舵机接口（8 组，可串联，靠 ID 区分）。
- **电池实测电压**：见01_电池电压.png

## 9. 动作 yaml

`config/actions/stand_up.yaml`：

steps:
  - joints:
      - {id: 1, pwm: 1700, duration_ms: 1300}
      - {id: 3, pwm: 1300, duration_ms: 1300}
  - joints:
      - {id: 1, pwm: 1300, duration_ms: 2000}
      - {id: 3, pwm: 1700, duration_ms: 2000}
     
  - joints:
      - {id: 1, pwm: 1500, duration_ms: 1200}
      - {id: 3, pwm: 1500, duration_ms: 1200}


- **step 怎么切**：一个 step 里放多个舵机就是同时动；分多个 step就是先后动。
- **谁先动**：上面就是"先 1 3号一起动，再 1 3号一起动，最后1 3一起回中位"。
- **duration_ms 怎么定**：、用800发现关节声音有点响，这里用 1200-1300ms保证平稳吧。
- **result 同步**：sequence_player 等一个 step 里所有 goal 的 result 都回来，才发下一个 step。

## 10. 踩坑记录（至少 3 个）

| 现象 | 怀疑 | 怎么验证 | 结论 |
|---|---|---|---|
| `ls /dev/ttyUSB*` 找不到 | USB 没透传/驱动 | `dmesg` 看到 brltty 抢占 | 卸载 brltty |
| 总线舵机发指令无回显 | 接错口 | 换白色端子后通了 | 总线舵机接白色端子，不是编号排针 |
| 舵机发位置指令不动 | 卸力/PULR 没生效 | 手动 PULR 后能动 | PULR 和位置指令间加 sleep |
| 发 goal 一直 waiting | action 名错 | `ros2 action list` 显示 `/move_servo` | action 名相对命名空间 |
| 每条指令发两遍导致"未到位" | 双发 | 去掉直接发、只 publish | 订阅和 Action 走同一发送函数 |
| 看门狗启动时误判超时抢串口 | 宽限期放错位置 | 把 +3.0 挪到 `__init__` | 启动 3 秒内不判超时 |

## 11. 下电三步 + 原因

1. **PULK 卸力**：`python3 pulk_all.py` —— 用脚本给所有舵机松劲，防止断电瞬间舵机憋力；
2. **断动力电**：关控制板电源 —— 先卸力再断，舵机不会卡死；
3. **关板卡**：`sudo shutdown now` —— 优雅关机，直接拔电会损坏文件系统，这是必要操作。

## 12. 录屏链接

- 舵机转动：【Cquick电控组week3-4作业提交：用串口助手操作舵机转动】 https://www.bilibili.com/video/BV1aUsH6jEXE/?share_source=copy_web&vd_source=3a22558c03be4f58b55c8013662e489c
- 正反转回中位：【Cquick电控组作业Week3-4:用ros2 action实现舵机正反转、复位以及cancel中断】 https://www.bilibili.com/video/BV1ymy5zkEwr/?share_source=copy_web&vd_source=3a22558c03be4f58b55c8013662e489c
- 取消打断：【Cquick电控组作业Week3-4:用ros2 action实现舵机正反转、复位以及cancel中断】 https://www.bilibili.com/video/BV1ymy5zkEwr/?share_source=copy_web&vd_source=3a22558c03be4f58b55c8013662e489c
- 改参数生效6步演示：【Cquick电控组作业Week3-4:改参数后行为变化的六步演示】 https://www.bilibili.com/video/BV1bqy5zJEGc/?share_source=copy_web&vd_source=3a22558c03be4f58b55c8013662e489c
- bag 复现：【Cquick电控组考核作业Week3-4:bag复现与运行日志】 https://www.bilibili.com/video/BV1UBy5zUE75/?share_source=copy_web&vd_source=3a22558c03be4f58b55c8013662e489c
- 动作组连跑三次，换动作，动作组日志：【Cquick电控组作业Week3-4:整机动作组连跑三遍、换动作组、动作组日志】 https://www.bilibili.com/video/BV1iTy7zqEiN/?share_source=copy_web&vd_source=3a22558c03be4f58b55c8013662e489c
- 安全下电流程：【Cquick电控组作业Week3-4:安全下电流程程程程】 https://www.bilibili.com/video/BV1cGy5zqE8M/?share_source=copy_web&vd_source=3a22558c03be4f58b55c8013662e489c
