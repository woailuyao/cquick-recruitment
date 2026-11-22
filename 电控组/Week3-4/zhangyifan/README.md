# 电控组第 3、4 周考核

姓名：zhangyifan
日期：2026-11-22
环境：Ubuntu 22.04 + ROS 2 Humble（VMware 虚拟机）

## 一、架构

两个包：

- `cquick_servo_interfaces`（ament_cmake）：放 `ServoCommand.msg` 和 `MoveServo.action`
- `cquick_servo`（ament_python）：放驱动节点、客户端、动作组播放器、launch、config

数据流：


ros2 action send_goal / sequence_player │ goal ▼ servo_driver（Action Server，节点名 /servo_driver） │ 校验 id 在 servo_ids 里、pwm 在 pwm_min~pwm_max 里 │ 拼 "#001P1700T1000!" 写串口 ▼ /dev/ttyUSB0 @ 115200 8-N-1 → USB 转 TTL → 24 路控制板 → 10 个总线舵机 │ ▼ 读 PRAD 回包 → 发 feedback → 收尾时读一次 PRAD 比对 → 发 result

代码块

两个话题：

- `/servo/command`（ServoCommand）：内部调试总线。执行 action 时也往这里发一份，给 bag 录
- `/servo/state`（String）：0.5 秒一次心跳，进 bag 用

Action 名：`/move_servo`

## 二、参数表

| 参数 | 默认值 | 为什么是这个值 |
|---|---|---|
| port | /dev/ttyUSB0 | CH340 插上就是这个 |
| baudrate | 115200 | 众灵总线舵机手册写的固定波特率 |
| pwm_min | 900 | 结构上能到的最小位置，再往外顶限位会嗡嗡响 |
| pwm_max | 2100 | 同上，上限 |
| default_duration_ms | 800 | 舵机从中间到端点大约这个时间，给小了会猛跳撞结构 |
| servo_ids | [1, 2, 3, 4, 5, 6, 7, 8, 9, 10] | 整机 10 个舵机的实际 ID |
| startup_reset | true | 启动先全部回 1500，避免停在上次的位置 |
| replay_mode | false | bag 回放开关，默认关 |

## 三、接口契约

`ServoCommand.msg`：

- `uint8 id`：舵机 ID
- `uint16 pwm`：目标位置
- `uint16 duration_ms`：用多久走到

`MoveServo.action`：

goal：
- `uint8 id`
- `uint16 pwm`
- `uint16 duration_ms`

feedback（执行过程中约 5Hz）：
- `uint16 current_pwm`：这一轮 PRAD 读回的当前位置
- `float32 progress`：0 到 1

result（结束时一次）：
- `bool success`：成功 true；被取消、PRAD 读失败、重发后仍未到位都是 false
- `uint16 final_pwm`：最后一次 PRAD 读回值
- `string message`：ok / canceled / 发送失败 / PRAD 读回失败 / 未到位，差 X

## 四、启动命令


$ ros2 launch cquick_servo servo.launch.py [INFO] [launch]: All log files can be found below /home/future/.ros/log/2026-11-20-...-future-virtual-machine-3906 [INFO] [launch]: Default logging verbosity is set to INFO [INFO] [servo_driver-1]: process started with pid [3907] [servo_driver-1] [INFO] [...]: 串口已打开: /dev/ttyUSB0 @ 115200 [servo_driver-1] [INFO] [...]: 全部舵机已回中位

代码块

launch 带两个参数：

- `port:=` 覆盖串口设备。试过 `ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB1`，日志报「串口打开失败 ttyUSB1」，说明覆盖生效
- `demo:=true` 额外起一个 `move_client` 节点，默认不起

## 五、安全区间怎么来的

`pwm_min=900`、`pwm_max=2100`。

从 1500 开始，每次往外走 200，走到 900 和 2100 时能听到轻微嗡嗡声，说明已经顶到结构限位，再往外就是堵转。所以定在 900~2100。

另外代码里对「到位」的判定阈值是 50，不是 30。原因是 PRAD 读回本身有 ±30 左右的抖动（舵机死区 + 腿悬空时的重力负载），阈值定 30 会把正常抖动判成失败。

## 六、运行时改参数（六步实测输出）


$ ros2 param get /servo_driver pwm_max Integer value is: 2100

$ ros2 action send_goal /move_servo cquick_servo_interfaces/action/MoveServo "{id: 1, pwm: 1700, duration_ms: 1000}" Result: success: true final_pwm: 1694 message: ok Goal finished with status: SUCCEEDED

$ ros2 param set /servo_driver pwm_max 1600 Set parameter successful

$ ros2 action send_goal /move_servo cquick_servo_interfaces/action/MoveServo "{id: 1, pwm: 1700, duration_ms: 1000}" Goal was rejected.

$ ros2 param set /servo_driver pwm_max 100000 Setting parameter failed: pwm_max=100000 超出 500~2500

$ ros2 param set /servo_driver pwm_max 2100 Set parameter successful

代码块

第 3 步到第 4 步之间没有重启节点。参数回调用 `add_on_set_parameters_callback`，`goal_callback` 里每次实时 `get_parameter`，所以改完下一个 goal 就按新上限判断。

## 七、bag

录的是 `/servo/command` 和 `/servo/state`。


$ ros2 bag info ~/week34/bag/run1 Files: run1_0.db3 Bag size: 25.4 KiB Storage id: sqlite3 Duration: 24.506632228s Messages: 50 Topic: /servo/command | Count: 1 Topic: /servo/state | Count: 49

代码块

回放：


$ros2 param set /servo_driver replay_mode true Set parameter successful$ ros2 bag play bag/run1 [INFO] [rosbag2_storage]: Opened database 'bag/run1/run1_0.db3' for READ_ONLY.

代码块

回放时驱动节点日志打出「回放: id=1 pwm=1800 dur=1500」，舵机把录下来的动作又做了一遍。

为什么要有 `replay_mode`：正常执行 action 时也会往 `/servo/command` 发一份（为了被 bag 录到），如果订阅端无条件执行就会发两遍。加开关后，正常跑不响应，回放时才响应。

第一次录是失败的，`Count: 0`，因为动作在开始录制之前就做完了。第二次按「先开录、再做动作、马上停」才对。

## 八、接线

- USB 转 TTL 的 TXD 接控制板 RX1
- RXD 接控制板 TX1
- GND 接控制板 GND（共地，不接什么都别想通）
- 电池接控制板绿色端子，正负别反
- 舵机插在白色三针壳子接口上，不是裸针排

电池实测 7.55V，在 5~8.4V 区间内。

## 九、任务 3：整机与动作组

### 舵机编号

10 个舵机，编号 1~10，纸片贴在舵机旁边。编号 = ID，不用做映射。

| 编号 | 位置 |
|---|---|
| 1~5 | 左腿，从下往上（1 踝 → 5 髋） |
| 6~10 | 右腿，从上往下（6 髋 → 10 踝） |

整机扫描结果，10 个全部在线：


1 -> b'#001P1489!' 2 -> b'#002P1485!' 3 -> b'#003P1494!' 4 -> b'#004P1494!' 5 -> b'#005P1495!' 6 -> b'#006P1493!' 7 -> b'#007P1497!' 8 -> b'#008P1480!' 9 -> b'#009P1494!' 10 -> b'#010P1490!' 在线: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

代码块

### 动作组怎么组织

动作写在 yaml 里，播放器读 yaml、展开成 goal、按顺序发。

一套动作 = 若干 step。**同一个 step 里的 goal 同时下发**；**step 之间等上一步所有 result 都回来才发下一步**。

`config/actions/action1.yaml`（左右摆髋）的真实内容：

```yaml
name: 左右摆髋
steps:
  - name: 髋部左偏
    goals:
      - {id: 5, pwm: 1560, duration_ms: 900}
      - {id: 6, pwm: 1560, duration_ms: 900}
      - {id: 1, pwm: 1540, duration_ms: 900}
      - {id: 10, pwm: 1540, duration_ms: 900}
  - name: 回正
    goals:
      - {id: 5, pwm: 1500, duration_ms: 900}
      - {id: 6, pwm: 1500, duration_ms: 900}
      - {id: 1, pwm: 1500, duration_ms: 900}
      - {id: 10, pwm: 1500, duration_ms: 900}

config/actions/action2.yaml 是第二套（蹲起），换动作只换文件，不碰 python。

为什么不用 sleep：舵机走完的时间不等于 duration_ms，用 sleep 会累积误差，跑三遍一定对不上。播放器里用 spin_until_future_complete 等 result：

python
for gh in handles:
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(self, rf)
    res = rf.result().result
    self.get_logger().info(
        f'  result success={res.success} final={res.final_pwm} msg={res.message}')

duration_ms 怎么定：髋部 1560 这种小幅度给 900，屈膝这种带负载的给 1200。给小了舵机会猛跳，撞击结构。

要表达「某个关节先动」，就把它单独拆成一个 step，不靠调 duration_ms 去凑先后。

PRAD 闭环校验

转完读一次 PRAD 和目标值比，误差超过 50 就自动重发一次：

代码块
[id=6] 目标 1500 实际 1440 差 60，重发一次
[id=6] 重发后实际 1490，差 10
result success=True final=1490 msg=ok
装配中发现的两个具体问题

1. 关节到不了目标位。 跑动作组时反复出现 未到位，差 31。

排查：单发一个 goal 就成功，两个以上一起发就有一个失败；失败的那个总是差 30 上下，而 PRAD 读回值本身在 1514/1474 之间跳。

结论：不是通信问题，是 PRAD 读回本身有 ±30 的抖动（舵机死区 + 腿悬空时的重力负载）。阈值从 30 放宽到 50，并加失败自动重发一次，之后全绿。

2. 走线会刮关节。 第一次走完线手动转关节，发现有一根线跨过会转的关节，转 30 度就绷紧。重新走线，把线贴到结构内侧，用扎带固定，再手动转一遍确认不刮。

十、下电三步
先卸力：给所有舵机发 #xxxPULK!。舵机断电前还顶着力矩，直接拔电池会堵转发热。
再断动力电：拔电池。先断动力，避免控制板在电压异常的时候乱发指令。
最后关板卡：sudo shutdown now，别直接拔电，直接拔会损坏文件系统。
十一、踩过的坑

1. ttyUSB0 插上就消失

现象：ls /dev/ttyUSB* 能看到设备，一两秒后就没了。 怀疑：驱动被别的进程抢了。 验证：dmesg 里看到 interface 0 claimed by ch341 while 'brltty' sets config #1。 结论：brltty 抢走了 CH340，sudo apt remove -y brltty 之后重插就稳定了。

2. 舵机插在裸针排上没反应

现象：发 #001PID! 返回空，PRAD 也空。 怀疑：波特率错了或者舵机没电。 验证：换到白色三针壳子接口上，立刻收到 #001P!。 结论：舵机要插白色壳子接口，裸针排那排是给别的东西用的。

3. 两个 goal 并发抢串口

现象：动作组每一步只有一个成功，另一个 final_pwm=0。 怀疑：两个 execute 同时读串口，互相抢回包。 验证：单独发一个 goal 就成功，两个一起发就只有一个成功。 结论：加 threading.RLock 串行化串口读写。用 RLock 是因为 read_prad 内部会调 send_cmd，同线程要能重入。

4. PID 回读的 ID 字段是空的

现象：发 #001PID! 收到 #001P!，中间空的。 怀疑：指令格式不对。 验证：#001PRAD! 能正常回 #001P1495!，说明通信是好的。 结论：ID 判断改用「发运动指令看哪个舵机动」，不依赖 PID 回读。

5. bag 录成空文件

现象：ros2 bag info 里 /servo/command 是 Count: 0，但 /servo/state 有 8551 条，时长 71 分钟。 怀疑：录制时没产生命令消息，而且录制忘了停。 验证：Start: 21:33:47，而动作是 21:31 做的，命令发生在录制之前。 结论：顺序必须是「先开录 → 再做动作 → 马上停」。

6. USB 反复掉线

现象：跑动作组跑到一半报 串口打开失败: could not open port /dev/ttyUSB0，之后所有 goal 都返回「发送失败」。 怀疑：线接触不良或虚拟机把设备释放了。 验证：ls /dev/ttyUSB* 为空，lsusb 有时还能看到 CH340。 结论：换到主机后面的 USB 口，重插后重新连到虚拟机就恢复。这个问题一晚上出现 5 次，最后靠固定线材、不碰插头解决。

十二、录屏链接

任务 2：

正转 / 反转 / 回中位：https://www.bilibili.com/video/BV1VEyYzLErM/?spm_id_from=333.1387.homepage.video_card.click&vd_source=73232109fdc7ac91c3783ba6d72e9c58
取消打断：https://www.bilibili.com/video/BV15jyYz2E7c/?spm_id_from=333.1387.homepage.video_card.click&vd_source=73232109fdc7ac91c3783ba6d72e9c58
改参数后行为变化：https://www.bilibili.com/video/BV1ZuyYznEqk/?spm_id_from=333.1387.homepage.video_card.click&vd_source=73232109fdc7ac91c3783ba6d72e9c58
bag 回放：https://www.bilibili.com/video/BV1XEyYz5EQB/?spm_id_from=333.1387.homepage.video_card.click&vd_source=73232109fdc7ac91c3783ba6d72e9c58

任务 3：

整机逐个动作：https://www.bilibili.com/video/BV1ZuyYznEjc/?spm_id_from=333.1387.homepage.video_card.click&vd_source=73232109fdc7ac91c3783ba6d72e9c58
动作组连跑三遍：https://www.bilibili.com/video/BV1fTyYzQE7E/?spm_id_from=333.1387.homepage.video_card.click&vd_source=73232109fdc7ac91c3783ba6d72e9c58
换 yaml 复现：https://www.bilibili.com/video/BV1fTyYzQE4F/?spm_id_from=333.1387.homepage.video_card.click&vd_source=73232109fdc7ac91c3783ba6d72e9c58
任务 4：

安全下电：https://www.bilibili.com/video/BV1ZTyYzQEvT/?spm_id_from=333.1387.homepage.video_card.click&vd_source=73232109fdc7ac91c3783ba6d72e9c58
