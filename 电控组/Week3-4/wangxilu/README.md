# CQuick 舵机驱动 — Week 3/4

## 1. 架构

- **包1**：`cquick_servo_interfaces`（CMake，放 msg + action）
- **包2**：`cquick_servo`（Python，放节点 + launch + config）
- **节点**：`servo_driver`（Action Server）、`move_client`（Action Client）
- **Action**：`/move_servo`
- **串口**：USB-TTL → 24路板 → 众灵总线舵机，115200 8-N-1

## 2. 参数表

| 参数 | 类型 | 默认值 | 原因 |
|------|------|--------|------|
| port | string | /dev/ttyUSB0 | CH340 默认设备名 |
| baudrate | int | 115200 | 众灵协议固定值 |
| pwm_min | int | 900 | 结构实测下限，再小打到限位 |
| pwm_max | int | 2100 | 结构实测上限，防止顶打印件 |
| default_duration_ms | int | 800 | 800ms 走完，平滑不猛跳 |
| servo_ids | int[] | [1,2,3,4,5,6] | 整机 6 舵机 |
| startup_reset | bool | true | 启动发 PULR 恢复扭力 |

## 3. 接口契约：MoveServo.action

| 段 | 字段 | 含义 |
|----|------|------|
| Goal | command (ServoCommand) | id + pwm + duration_ms |
| Feedback | current_pwm, progress, status | 每 100ms 发，进度 0~1 |
| Result | error_code, message, final_pwm, success | 到位/失败/取消后一次性发 |

error_code：0=OK, 1=超限, 2=取消, 3=串口错, 4=没到位

## 4. 启动

```bash
cd ~/cquick_ws
colcon build --packages-select cquick_servo cquick_servo_interfaces
source install/setup.bash
ros2 launch cquick_servo servo.launch.py
```

输出：
```
[INFO] [servo_driver]: 串口已打开: /dev/ttyUSB0 @ 115200
[INFO] [servo_driver]: ServoDriver 初始化完成，ActionServer 就绪
```

## 5. 安全区间

- **pwm_min = 900**，**pwm_max = 2100**
- 试法：舵机单独接出来，从 500 往上扫，900 附近听到"嗡嗡"就停；从 2500 往下扫，2100 附近停。

## 6. 参数校验演示（2.6）

```bash
ros2 param get /servo_driver pwm_max
# Integer value is: 2100

ros2 param set /servo_driver pwm_max 1600
# Set parameter successful

ros2 param set /servo_driver pwm_max 10000
# Setting parameter failed: pwm_max=100000 超出 [500,2500]

ros2 action send_goal /move_servo cquick_servo_interfaces/action/MoveServo "{command: {id: 3, pwm: 1700, duration_ms: 1000}}"
# Goal was rejected

ros2 param set /servo_driver pwm_max 2100
# Set parameter successful

ros2 action send_goal /move_servo cquick_servo_interfaces/action/MoveServo "{command: {id: 1, pwm: 1800, duration_ms: 1000}}" --feedback
# Goal accepted → feedback 滚动 → success: true
```

## 7. 接线

```
电脑 ──USB──> TTL(CH340) ──3根杜邦线──> 24路板 ──> 舵机
                            │
                            ↑
                       电池（5~8.4V）
```

| 线色 | 接哪里 |
|------|--------|
| 白 | 控制板信号脚 |
| 红 | 电池正极 |
| 棕/黑 | 电池负极 + 控制板GND |

## 8. 踩坑

**① BRLTTY 抢 CH340**
- 现象：`ls /dev/ttyUSB*` 时有时无
- 验证：`dmesg | tail` 看到 `claimed by brltty` → `disconnected`
- 结论：`sudo apt remove brltty`

**② param set 改了不生效**
- 现象：set 1600 成功，但发 1700 的 goal 舵机仍转
- 验证：在 goal_callback 里打印 `get_parameter().value`，发现值是对的，但判断用的是 `__init__` 存的副本
- 结论：参数必须用 `self.get_parameter('xxx').value` 实时读

**③ 多个 servo_driver 实例抢响应**

- 现象：发 goal 后报 `Ignoring unexpected goal response`，result 收不到，ros2 node list 里有重复 servo_driver
- 验证：之前用 `Ctrl+C` 关终端，节点进程没死透；`ps aux | grep servo_driver` 看到两个进程
- 结论：每次重开节点前先 `pkill -f servo_driver`，再 `ros2 daemon stop && start` 清图

## 9. 下电三步

| 顺序 | 操作 | 原因 |
|------|------|------|
| 1 | 发 PULK 卸力 | 切断扭力，防止断电瞬间烧电机 |
| 2 | 关电池电源 | 彻底断电 |
| 3 | 拔 USB | 防止电脑端意外发指令 |

红线：严禁带电插拔舵机线、严禁反接电源、听到"嗡嗡"立刻回中位。

## 10. 总结
因时间有限，其余暂未完成，若可以继续，我会继续完成

## 11. 录屏链接
任务一： https://b23.tv/0R9nbt2
任务二： https://b23.tv/7QuqrCf