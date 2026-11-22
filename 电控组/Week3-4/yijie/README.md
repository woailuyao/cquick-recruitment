#1：架构图
由 ros2 launch cquick_servo servo.launch.py 一条命令驱动整个 cquick_servo 包，消息/动作类型由 cquick_servo_interfaces 包在编译期生成，通过 servo_driver 节点把 Action goal 按 config/servo_params.yaml 校验后，经 ServoCore 拼成 #001P1600T1200! 走串口驱动控制板上的舵机、以 10 Hz 回传 feedback 并用 PRAD 读回给出 result，通过 sequence_player 把 config/actions/*.yaml 的动作组展开成按 result 同步的 goal 序列、通过 move_client 提供手动调试入口，并把整条链路的指令与状态发布到 /servo/command、/servo/state 供 ros2 bag 录制回放。
#2：参数表
port : 串口设备，写的udev老是跑不通，只好固定成ttyUSB0了，之后我会再尝试一下写的
baudrate ： 波特率 默认值115200 控制板的默认波特率是115200，如果不默认115200会收到一堆乱码
pwm_min 最小pwm值，默认900，为结构实测
pwm_max 最大pwm值，默认2100，为结构实测
default_duration_ms 默认动作800，在客户端没给时间是默认为1000ms
servo_ids 实际装进结构的舵机 测试时只采用了1, 2, 3, 4, 5, 6六个舵机实际腿部结构中有十个，后续进行更改
startup_reset True  上电回中位，避免带着上一次姿态上电顶结构
position_tolerance 50 偏差在这个范围内算到位，否则自动重发一次
watchdog_timeout_s 1 超过这么多秒没有心跳就全部 PULK 卸力
require_prad false读不到 PRAD 回显时，用最后下发的 PWM 作为结果（message 标注 open-loop）；设 `true` 则严格模式，读不到判失败
#3：串口契约
goal : id是舵机编号，pwm是目标pwm值，duration_ms是运行时长，就是从当前位置走到目标的时长，在客户端发一条动作请求时
Result ：message：okdiff=0说明走完且 PRAD 在 ±50 内
ok diff=-xx (retried)说明第一次没到位，自动重发一次后到位
canceled表示执行中收到取消，已下发回中位
preempted表示同一舵机来了新 goal，旧 goal 让位且多级不回中位
watchdog是看门狗触发安全模式，动作中止
serial unavailable表示串口打不开 / 写失败
not in position说明重发后仍未到位，附 target/actual/diff/tol
success说明动作成功运行， final_pwm返回最终pwm值，一条动作结束（到位、失败、取消、抢占）时发送Resuit，只发一次
Feedback  :  current_pwm 实时返回当前pwm值 progress表示时间进程，范围为0-1，表示动作进行了多少进程
#4：启动命令
ros2 launch cquick_servo servo.launch.py
[INFO] [servo_driver]: 启动回中位：ids=[1, 2, 3, 4, 5, 6] pwm=1500 duration=800ms
[INFO] [servo_driver]: 串口已打开：/dev/ttyUSB0 @ 115200 8-N-1
[INFO] [servo_driver]: 看门狗已启动：1.0 秒没有心跳就全部卸力
[INFO] [servo_driver]: servo_driver 就绪：port=/dev/ttyUSB0 baudrate=115200 ids=[1, 2, 3, 4, 5, 6] 安全区间=[900, 2100] 看门狗=1.0s
#5：安全区间
用串口助手一点点试，快要把舵机的线撤下来了就取做上限或者下限值
#6：动态改参数
见README_1(做的时候忘记保留日志了。。)
#7：bag
我还是忘记保存了，因为我习惯最后写README所以最后来看这部分确实会少交一部分日志，我很抱歉，下次作业我会改得，然后是有复现的，视频里会有
#8：接线结构
这周进行动作三我并没有接入板卡，只是舵机在腿部结构内固定，用线连接起来，再接入控制板，连接串口的，但根据我的学习，这个板卡应该是需要用最左端的typec接口通电，然后中间的接口用来导入数据进去编译自己的环境，好吧这些是在和队友研究的时候队友教我的，哈哈。实测电压是8.21V，图片里有
#9: 动作组
# 动作组 1：从趴下姿态撑起
# 结构：steps 是有序列表（步与步串行），每个 step 里的 joints 同时下发（步内并行）。
# 想表达「谁先动」就单独开一个 step，不要靠 duration_ms 硬凑。
name: stand_up
description: 双腿同时发力撑起 -> 双臂抬起 -> 小臂收拢
repeat: 1

steps:
  - name: 01_legs_push
    joints:
      - {id: 1, pwm: 1620, duration_ms: 2000}   # 左髋
      - {id: 2, pwm: 1380, duration_ms: 2000}   # 右髋

  - name: 02_arms_lift
    joints:
      - {id: 3, pwm: 1700, duration_ms: 2000}   # 左肩
      - {id: 4, pwm: 1300, duration_ms: 2000}   # 右肩

  - name: 03_forearms_close
    joints:
      - {id: 5, pwm: 1400, duration_ms: 2000}   # 左小臂
      - {id: 6, pwm: 1600, duration_ms: 2000}   # 右小臂
以上是我的stand up动作组，怎么切的话，一个joint为一个动作，在一个joint里面的动作同时进行，然后只有等上一个动作做完，收到了result才能进行下一个动作，运动时长怎么定在我看来取决于舵机的能力和需要的动作吧，舵机的能力主要是看这个舵机最快转速，要求转速太高可能无法在规定时间到达指定位置，然后有些动作需要快速完成就只能要求短时长
#10：踩坑记录
我的踩坑记录其实在调试记录里面都写得差不多了，主要就是时长设定过短导致pwm无法快速到达位置最后报错中断动作组，这个问题任务二任务三都有出现，还有就是存包的问题，然后还有就是我们组至今舵机二是坏的，得找个时间去修，所以在任务三演示中均把舵机一二摘除了，这个在线下演示中应该会补上的嗯对
#11 ：下电
pkill -f servo_driver; sleep 1
python3 ~/pulk_all.py -p /dev/ttyUSB0 --ids 1-6
以上命令让舵机卸力，然后再拔掉电池电源，使用指令sudo shutdown now之后再给板卡断电否则会烧坏板卡，先卸力否则舵机堵转会烧坏舵机，再断力电，也就是电源电池，避免控制板在电压跌落时电压不稳定，乱发指令驱动舵机或者其他，使用指令后再断电停止服务，将文件系统缓存写回磁盘，防止文件系统被损坏下次开不了机
#12：视频链接
1：【【把落叶谱成诗】第一支视频求三连！-哔哩哔哩】 https://b23.tv/s7YUtJa 任务一
任务三：【【把落叶谱成诗】最新视频来袭，快来看看吧！-哔哩哔哩】 https://b23.tv/ub81dLy
任务二：【【把落叶谱成诗】最新视频已上线，快来围观！-哔哩哔哩】 https://b23.tv/ziSgWeO
任务四：把落叶谱成诗刚刚发布的视频https://b23.tv/g4tKhz9
#13：补充说明
我缺了应该是好几张日志的照片，但是我视频里面都有，如果对审核作业造成不便深感抱歉各位老师！

