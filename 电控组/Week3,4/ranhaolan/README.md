# 第 3、4 周考核 · ranhaolan

## 1. 架构图（就这样连线嘛就是）

```
move_client / sequence_player / ros2 action send_goal
        │  Action: goal → feedback → result
        ▼
servo_driver（Action Server，串口驱动节点，参数全外置）
        │  串口 115200 8-N-1
        ▼
USB 转 TTL ──> 24 路舵机控制板 ──> 总线舵机（4 个，ID 1~4）

内部调试话题：/servo/command（ServoCommand，供 ros2 bag 回放）
```

- 两个包：`cquick_servo_interfaces`（CMake，只放 msg/action）+ `cquick_servo`（Python，节点 + launch + config）；
- 控制口只用 **Action**（`move_servo`），不用话题/服务当控制口；
- 动作组播放器 `sequence_player` 读 yaml → 发 goal → 等 result。

## 2. 参数表

| 参数                    | 默认值        | 为什么                                                                                                                                                   |
| ----------------------- | ------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `port`                | `auto`      | 自动检测 `/dev/ttyUSB*`，避免设备编号变动（后面看了加分项直接改成固定了，但是这个自动识别也给我省了非常多调试时间）                                    |
| `baudrate`            | `115200`    | 总线舵机协议固定                                                                                                                                         |
| `pwm_min`             | `900`       | 结构实测安全下限                                                                                                                                         |
| `pwm_max`             | `2100`      | 结构实测安全上限                                                                                                                                         |
| `default_duration_ms` | `800`       | 客户端没给 duration 时的默认（后面大多数时候会人工改为1000调试，最后发现是电池的问题，但是不知道为啥800不行1000有时行，难道是机魂不悦？）                |
| `servo_ids`           | `[1,2,3,4]` | 腿上 4 个舵机的 ID（没用10个是因为腿上还有个舵机的有问题，但是cos机械实在力竭了，于是用4个写了简单的小动作，自己调试一下能保证安全稳定，能够拓展到10个） |
| `startup_reset`       | `true`      | 启动时全部回中位（这个很重要，但是遇到卡住的舵机就hhh，后面把12舵机换了这个很好用！）                                                                    |

## 3. 接口契约（MoveServo）

- **Goal**：`id`(uint8 舵机号)、`pwm`(uint16 目标位置)、`duration_ms`(uint16 用时)；
- **Feedback**：`current_pwm`(uint16 当前)、`progress`(float32 0~1)，≥5Hz；
- **Result**：`success`(bool)、`final_pwm`(uint16 读回值)、`message`(string 失败原因)。

## 4. 启动命令

```bash
ros2 launch cquick_servo servo.launch.py
```

一条命令起驱动节点，参数从 yaml 加载，客户端默认不起（`demo:=true` 才起）。

## 5. 安全区间说明

`pwm_min=900`、`pwm_max=2100`。实测过程：逐个关节从中立 1500 往两个方向逐步加，找到「开始挤压/卡住」的边界，取保守值。动作 yaml 里的值都落在这个区间内。（这些是ai的话，其实无论动作还是这些边界，一是保守估计，然后就是自己开着脚本一点一点测试，说的很NB，其实很狼狈好笑）

## 6. 运行时改参数（2.6 六步）

```
① ros2 param get /servo_driver pwm_max          → 2100
② action send_goal pwm=2000                     → 成功
③ ros2 param set /servo_driver pwm_max 1600     → successful
④ action send_goal pwm=2000                     → 被拒（2000 > 1600）
⑤ ros2 param set /servo_driver pwm_max 100000   → 被拒（超 500~2500）
⑥ ros2 param set /servo_driver pwm_max 2100     → 恢复
```

第 ④ 步改完不重启立刻生效（参数实时 `get_parameter` 读，参数校验回调拒绝非法值）。

## 7. bag

- 录 `/servo/command` 话题（动作镜像），run6，约 24KB；
- 回放：`ros2 bag play bag/run1`，舵机自动重做动作；
- `ros2 bag info` 显示消息条数 > 0。

## 8. 接线

- USB 转 TTL `TXD` → 控制板 `RX`、`RXD` → 控制板 `TX`、`GND` → 控制板 `GND`（共地）；
- 舵机三线：信号/正极/负极，正极 5~8.4V；
- 电池实测电压 7.86V（hhh告诉我为什么到我手上就过放了，looking in my eyes！不过感谢愿意借给我电池的兄弟们，没有他们我做不完，再过放我是dog）

## 9. 动作 yaml

`config/actions/` 下两套动作，换动作只换 yaml：

```yaml
steps:
  - name: "双腿往前迈步"
    joints:
      - {id: 3, pwm: 1400, duration_ms: 1000}
      - {id: 4, pwm: 1600, duration_ms: 1000}
  - name: "全部回中"
    joints:
      - {id: 1, pwm: 1500, duration_ms: 1000}
      ...
```

- 一个 step 内多个 joints **并行**（同时发 goal）；step 之间**串行**（等 result 全回来才进下一个）；
- 用 result 当同步信号，不用 sleep 猜时间。

## 10. 踩坑记录

1. **BRLTTY 抢占串口**：ttyUSB0 一出现就被断。现象是 `lsusb` 有 CH340、`/dev/ttyUSB*` 时有时无。查 dmesg 看到 `BRLTTY` 抢设备，`sudo apt remove brltty` 解决。
2. **USB 转 TTL 半通**：`Input/output error` / `TimeoutError` 反复出现，串口时好时坏。现在反思应该是电池的原因，用别人的电池坐下来连贯的很，仿佛吃了泻药一泻千里。
3. **yaml 改了不生效**：`port: auto` 改了但 launch 还是读旧的 ttyUSB0。原因是 ament_python 的 yaml 要重新 `colcon build` 才装进 install，且 launch 文件里 `DeclareLaunchArgument('port', default_value='/dev/ttyUSB0')` 覆盖了 yaml。这波真的蠢了，自己改的代码，改了外层，结果自己内层的没改，还以为硬件坏了去拷问ai。
4. **并发 goal 串口竞争**：动作组回中 step 里 4 个 goal 并发写串口，指令交叉导致回中失效。加 `threading.Lock` 串行化串口写解决。
5. **舵机上电往极限转**：一个舵机上电就转到极限堵转、发指令回不来，其实人家只是卡住了，拆下来重新调试再装回去好得很

## 11. 下电三步

```
① 卸力（所有舵机发 PULK）→ ② 断动力电（拔电池）→ ③ 关板卡（shutdown）
```

- 先卸力：舵机顶着力直接断电会堵转发热烧；
- 再断电池：控制板电压不稳会乱发指令；
- shutdown 关机：直接拔电损坏文件系统。

## 12. 录屏链接（B站）

- 任务1 舵机转动：https://b23.tv/gEtr1Z3
- 任务2 正反转回中位：https://b23.tv/0MIOWPr
- 任务2 取消打断：https://b23.tv/wlLdqT5
- 任务2 改参数生效：https://b23.tv/2hKaUPI
- 任务2 bag 录制：https://b23.tv/RMtPdbA
- 任务3 整机逐个动作 + 动作组（连跑三遍 + 换 yaml，同一视频）：https://b23.tv/ngd1x5u
- 任务4 安全下电：https://b23.tv/mA0ixRT

## 加分项说明

- **串口掉线自动重连（+3）**：`servo_driver.py` 的 `send_raw()` 中，串口写入失败时关闭旧串口并重新 `open_serial()`（auto 检测新编号），日志打「串口写入失败，尝试自动重连」。
- **PRAD 闭环校验（+3）**：`execute_callback()` 末尾发 `PRAD` 读回真实位置（`_parse_prad` 正则解析），与目标差 >30 时自动重发一次，日志打「未到位 → 自动重发一次」。
- **udev 固定设备名（+2）**：`/etc/udev/rules.d/99-cq-servo.rules` 把 CH340（1a86:7523）绑定成固定名 `/dev/cq_servo`，拔插不变号。
- **pytest 单测（+2）**：`test/test_build_cmd.py` 测试 `build_cmd` 拼 ASCII 指令，`python3 -m pytest test/test_build_cmd.py -v` 3 passed

加分项代码与截图见 `加分项/` 文件夹。

最后想给上本周的一些感想吧，平时很忙，社团，班长，加上大创还有这周数模校赛也截止，这周完全是超人执行力在干，收获还是很多的，虽然还是有很多代码的地方ai写的比我好太多了，可能现在对于我们而言找到问题的能力更为重要，从开始一点看不懂到慢慢能让电机动起来，很舒服，很有成就感，还有一些其他队问我一些问题时我能直接回答，我才发现原来不知不觉已经学到了很多东西，希望后面能更加上手吧！
