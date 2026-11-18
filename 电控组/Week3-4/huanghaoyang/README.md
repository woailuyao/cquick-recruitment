# README 

- 姓名拼音：huanghaoyang
- 姓名：黄昊阳
- 完成时间：<2026-11-17>
- 运行环境：Ubuntu 22.04 + ROS2 Humble + RDK X5

## 0. 写在前面
完成了所有任务以及加分项

我的包的播放器叫作action_player.py，不是sequence_player.py 

所有视频在B站建了一个合集，可以直接连播

提交部分会多一部分截图，在文件夹加分项\中，那些是加分项截图（图号 加_），任务三动作组抢占这个单独发了B站并贴了链接

提交部分相比要求的多了一些文件，经过审查，那些事包运行必须具备的文件，没有那些文件包无法运行
\src\cquick_servo\cquick_servo\__init__.py    
大肥鱼：
① __init__.py（0 字节，空白）
作用：把一个目录标记成 Python 包。

有了它，cquick_servo/ 才能被 import cquick_servo.servo_driver 找到。没有它，setup.py 的 find_packages() 会直接跳过这个目录，colcon build 后 ros2 run cquick_servo servo_driver 会报 No module named cquick_servo。

空白是正常的，甚至是最常见的写法。它不需要任何代码，只要存在就行。

有些项目会在这里写包级说明或用它导出接口，但 ROS2 的 ament_python 包默认就是空的

src\cquick_servo\setup.cfg
大肥鱼：
② setup.cfg
不是编译产物，是 ros2 pkg create 生成的配置文件，属于源码。

内容只有 4 行：

ini


[develop]
script_dir=$base/lib/cquick_servo
[install]
install_scripts=$base/lib/cquick_servo
作用：告诉 setuptools 把 console_scripts（也就是 servo_driver、action_player 这些命令）安装到 lib/cquick_servo/ 下，而不是默认的 bin/。

这是 ament_python 的目录约定：ROS2 的可执行文件放在 lib/<包名>/ 里，ros2 run 从这个路径找。

如果没有它：

colcon build 不报错
但 ros2 run cquick_servo servo_driver 会报 No executable found
或者可执行文件被装到 install/cquick_servo/bin/，ros2 run 找不到

src\cquick_servo\test\test_command.py
任务二加分项的代码



## 1. 架构图

<!-- 直接使用下方文字图，按实际节点名核对一遍 -->

```text
                        ┌──────────────────────────┐
                        │     action_player        │
                        │  读 yaml → 逐个 step 发   │
                        │  step 内批量下发          │
                        └────────────┬─────────────┘
                                     │ Action goal
                                     │ 等 result 再发下一个
                                     ▼
  ┌───────────────┐        ┌──────────────────────┐        ┌────────────────┐
  │  move_client  │───────▶│     /move_servo      │◀───────│  ros2 action   │
  │ 单点调试客户端 │        │  MoveServo (Action)  │        │  send_goal     │
  └───────────────┘        └──────────┬───────────┘        └────────────────┘
                                      │
                                      ▼
                           ┌────────────────────────┐
                           │      servo_driver      │
                           │    Action 服务端        │
                           │  7 个参数 / 串口独占    │
                           │  看门狗 / 重连 / 抢占    │
                           └───────────┬────────────┘
                                       │ /dev/cq_servo  115200 8-N-1
                                       ▼
                           ┌────────────────────────┐
                           │   USB 转 TTL (CH340)   │
                           └───────────┬────────────┘
                                       │ 单线总线
                                       ▼
                           ┌────────────────────────┐
                           │  24 路舵机控制板 ZL-IS2 │
                           └───────────┬────────────┘
                                       │ 白色总线口（两条串联线）
                                       ▼
                           ┌────────────────────────┐
                           │  舵机 101~110（10 个）  │
                           └────────────────────────┘
```

**包与节点**

| 包 | 类型 | 内容 |
|---|---|---|
| `cquick_servo_interfaces` | ament_cmake | `msg/ServoCommand.msg`、`action/MoveServo.action` |
| `cquick_servo` | ament_python | `servo_driver.py`、`action_player.py`、`move_client.py` |

**话题 / Action 一览**

| 名称 | 类型 | 方向 | 作用 |
|---|---|---|---|
| `/move_servo` | `MoveServo` Action | 客户端 → 服务端 | 唯一的控制入口 |
| `/servo/command` | `ServoCommand` 话题 | 外部 → 服务端 | **仅用于 bag 录制/回放** |
| `/servo/state` | `ServoCommand` 话题 | 服务端 → 外部 | 当前位置状态，供 bag 录制 |
| `/servo/heartbeat` | `std_msgs/Empty` | 外部 → 服务端 | 看门狗心跳，1 秒收不到就卸力 |
| `/servo/preempt` | `std_msgs/String` | 外部 → 播放器 | 抢占：传目标动作名 |

---

## 2. 参数表

| 参数 | 默认值 | 为什么是这个默认值 |
|---|---|---|
| `port` | `/dev/cq_servo` | udev 固定名；直接写 `/dev/ttyUSB0` 拔插后会变 |
| `baudrate` | `115200` | 控制板默认波特率，手册标注 |
| `pwm_min` | 900 | 这个值是大肥鱼根据经验直接给出来的“pwm_min=900 / pwm_max=2100 是初始保守值，按电气范围 500~2500 内缩得到，用于保证首次上电不撞结构。”装机完后测试过，各关节基本没有问题，大多数会因为结构问题（牵引的重量过大、3D打印机没有留余量导致结构夹的非常非常紧，进而引发了不少问题），自出发位至目标位少大约20pwm左右（比如1500->2000,会到1978位，2000->1000,回到1025位）。但硬上下限归硬上下限，实际执行的上下限是1000-2000（我称为软上下限），一般也只有调试的时候才会使用超过软上下限的pwm值|
| `pwm_max` | 2100 | 同上|
| `default_duration_ms` | `800` | 防止舵机运动过快，也不至于太慢 |
| `startup_reset` | `true` | 上电回中位，避免姿态随机影响后续动作 |
| `watchdog_timeout_s` | `1.0` | 原文要求“超过 1 秒没心跳就卸力” |
| `armed` | `true` | 看门狗触发后为 False，`param set armed true` 之后才恢复 |

## 3. 接口契约（MoveServo）

**Goal**

| 字段 | 类型 | 含义 |
|---|---|---|
| `id` | uint8 | 单舵机模式：目标舵机 ID |
| `pwm` | uint16 | 单舵机模式：目标 PWM |
| `duration_ms` | uint16 | 从当前位置走到目标用多久，0 表示用默认值 |
| `ids[]` | uint8[] | 批量模式：舵机 ID 列表 |
| `pwms[]` | uint16[] | 批量模式：与 ids 一一对应的目标 |

**Feedback**（每 200ms 一条）

| 字段 | 含义 | 什么时候发 |
|---|---|---|
| `current_pwm` | 当前 PWM（起点到目标插值） | 动作执行期间 |
| `progress` | 0.0 ~ 1.0 | 同上 |

**Result**

| 字段 | 含义 |
|---|---|
| `success` | 所有舵机都在 ±30 内才 True |
| `final_pwm` | 第一个舵机的读回值（批量时） |
| `message` | `ok` / `canceled` / `preempted` / `未到位：id=... 目标X 实际Y` |

## 4. 启动命令

seacloud@seacloud-virtual-machine:~/cquick_ws$ cd ~/cquick_ws
seacloud@seacloud-virtual-machine:~/cquick_ws$ source install/setup.bash
seacloud@seacloud-virtual-machine:~/cquick_ws$ ros2 launch cquick_servo servo.launch.py
[INFO] [launch]: All log files can be found below /home/seacloud/.ros/log/2026-11-14-22-04-08-800615-seacloud-virtual-machine-8206
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [servo_driver-1]: process started with pid [8207]
[servo_driver-1] [INFO] [1794665049.015749667] [servo_driver]: 串口已打开: /dev/cq_servo @ 115200
[servo_driver-1] [INFO] [1794665049.037051943] [servo_driver]: startup_reset: 广播 #255 回中位 1500
[servo_driver-1] [INFO] [1794665049.060341124] [servo_driver]: servo_driver 已启动

## 4.5 udev 固定设备名

/etc/udev/rules.d/99-cq-servo.rules：
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="cq_servo"

生效：

sudo udevadm control --reload && sudo udevadm trigger


## 5. 安全区间怎么来的

和参数表里写的一样

大肥鱼根据经验直接给出来的“pwm_min=900 / pwm_max=2100 是初始保守值，按电气范围 500~2500 内缩得到，用于保证首次上电不撞结构。”装机完后测试过，各关节基本没有问题，大多数会因为结构问题（牵引的重量过大、3D打印机没有留余量导致结构夹的非常非常紧，进而引发了不少问题），自出发位至目标位少大约20pwm左右（比如1500->2000,会到1978位，2000->1000,回到1025位）。但硬上下限归硬上下限，实际执行的上下限是1000-2000（我称为软上下限），一般也只有调试的时候才会使用超过软上下限的pwm值

## 6. 运行时改参数的演示
文字为识图结果，会有一些词句断行，详情查看图片param_set现场
终端B：
seacloud@seacloud-virtual-machine:~/cquick_ws$~/param_demo
.sh
=查看当前上限———
$ ros2 param get /servo_driver pwm_max
Integer value is: 2100
=发1700，应成功
$ ros2 action send_goal /move_servo cquick_servo_interfaces
/action/MoveServo {id:1,pwm: 1700,duration_ms: 800}
Waiting for an action server to become available..
Sending goal:
id:1
pwm:1700
duration_ms: 800
ids:[]
pwms:[]
Goal accepted with ID: 3e99df41a0274e01b7d106c4b8505d08
Result:
success: true
final_pwm: 1687
message: ok
Goal finished with status: SUCCEEDED
=上限压到1600==
$ ros2 param set /servo_driver pwm_max 1600
Set parameter successful
=再发1700，应被拒
$ ros2 action send_goal /move_servo cquick_servo_interfaces
/action/MoveServo {id:1，pwm:17e0,duration_ms:800}
Waiting for an action server to become available...
Sending goal:
id:1
pwm:1700
duration_ms: 800
ids:[]
pwms:[]
Goal was rejected.
设100000，应被拒=
$ros2 param set /servo_driver pwm_max 100000
Setting parameter failed:pwm_max必须在5o0~2500
=恢复2100
$ ros2 param set /servo_driver pwm_max 2100
Set parameter successful
$ ros2 action send_goal /move_servo cquick_servo_interfaces
/action/MoveServo {id: 1,pwm:1700,duration_ms:800}
waiting for an action server to become available...
Sending goal:
id:1
pwm:1700
duration_ms: 800
ids:[]
pwms:[]
Goal accepted with ID: e1561162642b41d1bcd37223fae6992b
Result:
success: true
final_pwm: 1689
message: ok
Goal finished with status: SUCCEEDED
==组合校验pwm_min=2200
$ ros2 param set /servo_driver pwm_min2200
Setting parameter failed:pwm_min必须小于pwm_max
seacloud@seacloud-virtual-machine:~/cquick_ws$

终端A（部分）：
r]:goal:ids=[1] pwms=[17ee] dur=1eeoms batch=False
[servo_driver-1][INFo][1794230869.552374077][servo_drive
r]:goal: ids=[1] pwms=[2e0e] dur=5000ms batch=False
[servo_driver-1][INFo] [1794230871.960859049][servo_drive
r]：收到取消请求，己接受
[servo_driver-1] [WARN][1794230872.160441795] [servo_drive
r]：goal被取消，全部回中位
[servo_driver-1][INFo][1794230967.592619826][servo_drive
r]: goal:ids=[1] pwms=[200e] dur=500oms batch=False
[servo_driver-1] [INFo] [1794230970.202106611] [servo_drive
r]：收到取消请求，己接受
[servo_driver-1][WARN] [1794230970.400978171] [servo_drive
r]：goal被取消，全部回中位
[servo_driver-1][wARN][1794231045.88612500e] [servo_drive
r]：拒绝：pwm2400超出900~2100
[servo_driver-1][wARN][1794231059.652925748][servo_drive
r]：拒绝：ID9不在servo_ids[1，2，3，4]
[servo_driver-1] [INFo] [1794231137.329131181] [servo_drive
r]: goal:ids=[1] pwms=[170e] dur=80oms batch=False
[servo_driver-1][wARN][1794231141.338348183][servo_drive
r]：拒绝：pwm1700超出900~1600
[servo_driver-1][INFo][1794231146.925521326][servo_drive
r]: goal:ids=[1] pwms=[170e] dur=80oms batch=False
[servo_driver-1][INFo][1794231596.531579245][servo_drive
r]: goal:ids=[1] pwms=[15ee] dur=80oms batch=False
[servo_driver-1][INFo][1794231775.992853563][servo_drive
r]: goal: ids=[1] pwms=[150e] dur=80oms batch=False
[servo_driver-1][INFo][1794231810.884749447][servo_drive
r]: goal:ids=[1] pwms=[17ee] dur=10ooms batch=False
[servo_driver-1][INFo][1794231813.550426960][servo_drive
r]: goal: ids=[1]pwms=[130e] dur=10ooms batch=False
[servo_driver-1][INFo][1794231819.456385878][servo_drive
r]: goal: ids=[1] pwms=[15oe] dur=10ooms batch=False
[servo_driver-1][INFo][1794231977.917810992][servo_drive
r]: goal: ids=[1] pwms=[17ee] dur=800ms batch=False
[servo_driver-1][WARN] [1794231980.671411853] [servo_drive
r]：拒绝：pwm1700超出900~1600
[servo_driver-1][INFo] [1794231984.129893661] [servo_drive
r]: goal:ids=[1] pwms=[170e] dur=80Oms batch=False
0

## 7. bag
seacloud@seacloud-virtual-machine:~/cquick_ws/src$ cd ~/cquick_ws
source /opt/ros/humble/setup.bash
ros2 bag info bag/run1

Files:             run1_0.db3
Bag size:          25.4 KiB
Storage id:        sqlite3
Duration:          3.941227093s
Start:             Nov  9 2026 21:34:49.384469057 (1794231289.384469057)
End:               Nov  9 2026 21:34:53.325696150 (1794231293.325696150)
Messages:          6
Topic information: Topic: /servo/command | Type: cquick_servo_interfaces/msg/ServoCommand | Count: 3 | Serialization Format: cdr
                   Topic: /servo/state | Type: cquick_servo_interfaces/msg/ServoCommand | Count: 3 | Serialization Format: cdr


回放：（路径要自己适配）
cd ~/cquick_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 bag play bag/run1

舵机按 1200 → 1500 → 1800 的顺序复现了录制时的动作。 原因是 /servo/command 和 /move_servo Action 共用同一个 write_command()， 回放走的是与现场控制完全相同的下发路径。


## 8. 接线

| 线 | 接到 |
|---|---|
| USB-TTL TXD | 控制板 RXD |
| USB-TTL RXD | 控制板 TXD |
| USB-TTL GND | 控制板 GND |
| USB-TTL VCC | **不接** |

共地

舵机：总线串联

电池实测：7.87 V（万用表 20V档）。


## 9. 动作 yaml 示例

name: stand_squat
description: 站立 -> 蹲下 -> 站起（实测姿态）

steps:
  - name: 站立
    duration_ms: 900
    servos:
      - {id: 103, pwm: 1500}
      - {id: 104, pwm: 1500}
      - {id: 105, pwm: 1500}
      - {id: 106, pwm: 1500}
      - {id: 107, pwm: 1500}
      - {id: 108, pwm: 1500}

  - name: 蹲下
    duration_ms: 1600
    servos:
      - {id: 107, pwm: 2000}
      - {id: 108, pwm: 1000}
      - {id: 105, pwm: 1700}
      - {id: 106, pwm: 1300}
      - {id: 103, pwm: 1685}
      - {id: 104, pwm: 1315}

  - name: 站起
    duration_ms: 1600
    servos:
      - {id: 103, pwm: 1500}
      - {id: 104, pwm: 1500}
      - {id: 105, pwm: 1500}
      - {id: 106, pwm: 1500}
      - {id: 107, pwm: 1500}
      - {id: 108, pwm: 1500}


- **step 怎么切**：
根据姿态、重心进行切分，必要时会有重心左/右移在先的步骤，以防重心不稳倒下
- **谁先动**：
这个动作是多个舵机同时动的，并通过了测试。（能同时动为什么要分开，难道蹲下前要先下腰吗）（还有这个不应是在step里就决定好了的吗，step直接决定哪些舵机动）
- **duration_ms 怎么定**：
根据动作的幅度确定。大幅度动作时间就长。整个动作测试无误后再适度减少时间，达到一个我主观上的期望值
- **怎么用 result 同步**：
  `play_step()` 里发完 goal 后调 `get_result_async()`，再用`spin_until_future_complete()` 阻塞到 result 返回才 return；主循环 `for step in steps: play_step(step)` 靠这个返回值串行推进，


## 10. 踩坑记录（至少 3 个）

（诶不是，这个和调试记录功能重复有点太多了吧）
（算了，这里主要就写我一些映像非常非常深刻的，折磨大肥鱼的就不往这里写了）
（更多内容见调试记录）

### 坑 1：

- 现象：装配腿部时，转动105号舵机的舵盘，转不动
- 怀疑：舵机废了
- 验证：单独上电测试，转不动卡死。从结构中拆下来（留着舵盘）上电，转不动。拆了舵盘，机魂大悦，舵机正常运行了
- 结论：哪个大天才设计的打印机没有预留一点误差空间，真以为3D打印机和大学牲是大国工匠。装上去整个那个外面的白色连接件两边如星穹铁道般膨胀开，把舵机舵盘夹的死死的。那个舵盘也是粗制滥造，拆下来的时候还有拧螺丝拧下来的屑，我说这么卡这么死，合着这破连接件和舵盘把螺丝干成膨胀螺栓了。也难怪机魂不悦，卡这么死谁能机魂大悦

### 坑 2：
（此在调试记录中亦有记载）
- 现象：尝试4舵机联动的时候，只有一个舵机能动
- 怀疑：多个goal一起发的时候，程序是一个一个发的，发生了堵塞
- 验证：让大肥鱼去修改，加延迟不成，直接让大肥鱼大刀阔斧的去大改程序，直接根据官方通讯协议增加了指令批量发送模式
- 结论：通讯堵塞

### 坑 3：

- 现象：粘贴进去的py脚本有语法错误
- 怀疑：看终端里似乎粘贴的文本不全
- 验证：让大肥鱼把代码分成多段发送，或者自己直接去编辑对应的文件。改完再验证语法
- 结论：终端在复制超长文本的时候会丢文本，之后每次进行此类操作时都要进行语法检查

### 坑 4：

- 现象：起launch等程序时报错
- 怀疑：没sourch
- 验证：补了sourch
- 结论：还真是，之后每次运行这类程序之前都先sourch

### 坑 5：

- 现象：舵机都已经事先标定中位，但装机玩通电却不在预期位置
- 怀疑：装完后不能动舵臂
- 验证：再次通电回中位，歪了的部件重新装
- 结论：通电装或者装的时候不要动舵臂

（另：已完成地瓜板的使用学习，这方面手册一个字都不写😠）

（好像剩下印象深刻的都给大肥鱼写进调试记录了）

## 11. 下电三步 + 原因

| 步骤 | 操作 | 为什么 |
|---|---|---|
| ① | 舵机卸力|   不卸力舵机可能会烧 |
| ② |舵机主板断电，下电池 |   防止主板发癫乱发指令 |
| ③ | 地瓜板关机 | 先关机等着地瓜板的tf数据写入，地瓜板那个数据写入可比电脑慢多了 ，停launch之后那日志什么的数据都要写半天|
| ④ | 地瓜板拔电| 最后一步也要解释吗，也只有这个电没下了吧 |
---

## 12. 录屏链接

## 所有视频都已经在一个合集里了，可以从第一个链接开始连播

| 内容 | B 站链接 |
|四步验证|(https://www.bilibili.com/video/BV1Tus66dE7h/?vd_source=a6e58f1cde76598653842b24204ba968)和https://www.bilibili.com/video/BV1mus66REAw?vd_source=a6e58f1cde76598653842b24204ba968&spm_id_from=333.788.videopod.sections|
| 正反转回中位 |https://www.bilibili.com/video/BV1mus66REgj?vd_source=a6e58f1cde76598653842b24204ba968&spm_id_from=333.788.videopod.sections|
| 取消打断 |https://www.bilibili.com/video/BV1Tus66dEoc?vd_source=a6e58f1cde76598653842b24204ba968&spm_id_from=333.788.videopod.sections |
| 改参数后行为变化 |https://www.bilibili.com/video/BV1mus66REMA?vd_source=a6e58f1cde76598653842b24204ba968&spm_id_from=333.788.videopod.sections |
| bag play 复现动作 |https://www.bilibili.com/video/BV1mus66REH9?vd_source=a6e58f1cde76598653842b24204ba968&spm_id_from=333.788.videopod.sections |
| 整机逐个动作 | https://www.bilibili.com/video/BV1MMs664E59?vd_source=a6e58f1cde76598653842b24204ba968&spm_id_from=333.788.videopod.sections|
| 动作组连跑三遍+换一套 yaml|https://www.bilibili.com/video/BV1MMs664EuT?vd_source=a6e58f1cde76598653842b24204ba968&spm_id_from=333.788.videopod.sections |
| 动作组抢占 |https://www.bilibili.com/video/BV1UMs66xETN?vd_source=a6e58f1cde76598653842b24204ba968&spm_id_from=333.788.videopod.sections|
| 安全下电 |https://www.bilibili.com/video/BV1mMs66xEwQ?vd_source=a6e58f1cde76598653842b24204ba968&spm_id_from=333.788.videopod.sections |