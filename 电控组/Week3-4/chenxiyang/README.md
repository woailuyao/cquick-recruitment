# CQuick 战队电控组第 3、4 周考核作业

- **姓名拼音**：chenxiyang
- **日期**：2026-11-22

## 一、 任务概述与演示视频

本周期完成了从桌面单舵机点亮到整机多舵机动作组控制的完整链路。

1. **任务 1：串口点亮总线舵机**
   - 内容：手动通过 Python 验证串口通信、ID 读取、PULR 恢复扭力、PWM 平滑转动及 PULK 卸力。
   - **视频链接**：[点击观看 (B站)](https://www.bilibili.com/video/BV14By5zUEaB/)
2. **任务 2：ROS2 Action 接管与保护机制**
   - 内容：通过 ROS2 Action Client 发送 Goal，演示正反转、Ctrl+C 取消打断、修改参数当帧生效、越界拒绝以及 Bag 回放。
   - **视频链接**：[点击观看 (B站)](https://www.bilibili.com/video/BV12By5zUEUp/)
3. **任务 3：整机装配与多舵机动作组**
   - 内容：完成 4 个舵机的 ID 分配与整机装配。通过 YAML 定义动作组，播放器读取 YAML 执行连贯动作。
   - **视频链接**：*(如果你有单独发，请贴在这里)*
4. **任务 4：安全下电演示**
   - 内容：按标准流程执行“发 PULK 卸力 -> 拔动力电池 -> shutdown 关机”。
   - **视频链接**：[点击观看 (B站)](https://www.bilibili.com/video/BV1cuy5zzE82/)

---

## 二、 架构图（数据流说明）

```mermaid
graph TD
    A[sequence_player.py<br>Action Client] -->|1. 发送 Goal (id, pwm, duration)| B[servo_driver.py<br>Action Server]
    B -->|2. 校验限位/ID| B
    B -->|3. 拼装 ASCII 指令| C[串口驱动 (pyserial)]
    C -->|4. write + flush| D[USB转TTL / 24路控制板]
    D -->|5. 总线协议| E[总线舵机]
    E -->|6. 物理转动| F((机械结构))
    E -.->|7. 回传 PRAD 位置| C
    C -.->|8. 读回数据| B
    B -.->|9. 持续发送 Feedback| A
    B -.->|10. 动作结束返回 Result| A
    
三、 参数配置表（servo_params.yaml）
参数名	                   类型	  默认值	                                      为什么是这个值？/ 说明
port	        string   /dev/cq_servo(固定后）	 串口设备名。Linux 下 USB 端口号会变，通过 udev 规则绑定成固定名字，保证插拔不掉线。
baudrate 	int	 115200	                波特率:必须一致，否则乱码。
pwm_min	 	int	 900	                安全下限。500 是电气极限会撞坏结构.
pwm_max	 	int      2100	                安全上限。2500 也会撞结构.
default_duration_ms	int	800	        默认动作时间。如果客户端发指令时没填时间，给个 800ms 的缓冲，避免舵机猛跳。
servo_ids	int[]	[1, 2, 3, 4]	        本机有哪些舵机。声明这个可以拒绝不存在的错误 ID 指令。
startup_reset	bool	true	                启动时是否全部回中位。上电自动发 PULR 恢复扭力，保证初始状态受控。

四、 接口契约（MoveServo）
Goal 段：包含 id（舵机编号）、pwm（目标 PWM 位置）、duration_ms（用多少毫秒转到目标）。

Feedback 段：包含 current_pwm（当前实际位置）和 progress（进度 0.0 到 1.0），每 200ms 反馈一次，至少 5Hz。

Result 段：包含 success（是否成功）、final_pwm（最终读回的位置）和 message（失败原因，比如超限、取消或串口错）。

五、 动作组数据驱动（任务 3 核心）
动作没有写死在 Python 代码里，全部写在 config/actions/ 下的 YAML 文件里（例如 stand_up.yaml）。

执行逻辑：播放器 sequence_player.py 会读取这个 YAML 文件，按顺序将 YAML 里的步骤展开成一个个 Goal 发送给 Action Server。

同步方式：坚决不使用 sleep()。上一个动作的 result 没有返回之前，绝对不会发下一个指令。这保证了无论重复跑多少遍，动作时序都不会累积误差。

六、 安全下电流程
第一步：让所以舵机回中
第二步：给所有舵机发 PULK 卸力。因为如果不卸力直接断电，舵机内部还在顶着力矩，很容易堵转发热甚至烧毁。
第三步：拔掉电池动力电。切断大电流。
第四步：在系统里输入 sudo shutdown now 关闭主控板。绝对不能直接拔电源，那样会损坏文件系统。（不过没用地瓜派，对于控制板就直接下电后拔了）

七、 踩坑记录与心得
(详细踩坑记录见同目录下的 调试记录.md，这里简述最核心的三个)

串口被 brltty 抢占：插上 USB 发现找不到 /dev/ttyUSB*。查内核日志发现是 Ubuntu 自带的盲文服务 brltty 抢占了设备。停用该服务并写了 udev 规则固定设备名后解决。

ros2 bag 录制为空（Messages: 0）：运行 ros2 bag info 发现没有任何数据。排查发现录制终端没有 source 自定义消息包的工作空间，导致录制工具不认识话题类型。每次开新终端先 source 后解决。

YAML 解析类型错误：跑动作组时 Python 报错 'list' object has no attribute 'items'。原因是 YAML 文件缩进格式写成了列表，而代码用字典去遍历。统一 YAML 格式和代码解析逻辑后解决。
