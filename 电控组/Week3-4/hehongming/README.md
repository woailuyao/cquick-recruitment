# 第 3、4 周考核提交说明 - hehongming

## 一、任务完成情况
本机由 10 个总线舵机组成双足下半身（1-2胯部，3-4大腿，5-6膝盖，7-8小腿，9-10脚踝）。通过 Action 通信与 YAML 数据驱动，实现了整机多舵机并发动作控制，并完成了串口掉线重连等加分项。

## 二、架构设计
- **接口包**：`cquick_servo_interfaces` (定义 `MoveServo.action` 和 `ServoCommand.msg`)。
- **控制节点**：`servo_driver` (服务端) + `sequence_player` (播放器客户端)。
- **通信流**：客户端发起 Goal -> 服务端校验 -> 拼接指令 -> 发布 `/servo/command` 话题 -> 写入串口硬件。

## 三、关键参数说明
- `port`: "/dev/ttyUSB0" (串口设备)
- `pwm_min`: 500 (放宽下限)
- `pwm_max`: 2500 (极其重要！因10号舵机机械偏差，拼装基准为2295，必须放宽上限至2500才能放行)
- `servo_ids`: [1~10] (10个舵机全在线)
- `default_duration_ms`: 1000

## 四、接口契约 (MoveServo)
- **Goal**: id (舵机ID), pwm (目标值), duration_ms (耗时)。
- **Feedback**: current_pwm (当前值), progress (进度0~1)。
- **Result**: success (是否成功), final_pwm (最终值), message (信息)。

## 五、一条启动命令
```bash
ros2 run cquick_servo servo_driver --ros-args -p servo_ids:="[1, 2, 3, 4, 5, 6, 7, 8, 9, 10]" -p pwm_max:=2500
```
**输出**：
✅ 成功打开串口: /dev/ttyUSB0
Action Server 已启动！

## 六、安全区间来历
常规安全区间为 900~2100。由于 10 号舵机装配偏差，姿态达到了 2295。故启动时将 pwm_max 放宽至 2500。实际测试中通过 `ros2 param set /servo_driver pwm_max 1600` 可实时将上限改回并拦截超限目标。

## 七、Bag 录制与回放
录制的 bag 包含 `/servo/command` 指令序列。使用 `ros2 bag play bag/run1` 可以完美复现物理动作。

## 八、接线与电池
TX/RX 交叉接线，GND 共地。上电前万用表实测电池电压为 7.4V。

## 九、动作 YAML 说明
动作定义在 `config/actions/`。为消除“接续转动”导致的不连贯，服务端在发完指令后立刻返回 Result，客户端通过异步回调确认全部下发后，再根据最大 duration_ms 延迟进入下一个 step，保证多关节并发同步。

## 十、踩坑记录 (详见调试记录.md)
1. 初始 1500 导致变形，重写了 2295 基准。
2. 2295 超出 2100 被拦截，通过启动参数放宽至 2500。
3. 串口双重发送导致死机，重构代码统一由 command_callback 写入。
4. 关节接续转动，改为并发下发。

## 十一、下电三步与原因
1. 发送 `#000PULK!` 广播卸力（防堵转烧毁）。
2. 拔除电池动力线（切断大电流）。
3. 关闭板卡/虚拟机。

## 十二、B站演示录屏
链接：[BV1xAs36TEhh](https://www.bilibili.com/video/BV1xAs36TEhh)
