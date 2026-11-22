参数名 默认值 为什么是这个默认值
port /dev/ttyUSB0 默认串口设备名，可通过 udev 规则修改
baudrate 115200 舵机控制板的标准波特率
pwm_min 900 下限
pwm_max 2100 上限
default_duration_ms 800 默认转动时间，避免时间过短舵机瞬间猛跳
servo_ids [1,2,3,4,5,6] 当前整机实际接入的舵机 ID
startup_reset true 启动时自动全部回中位(1500)

Goal:
	• id (uint8): 目标舵机 ID。
	• pwm (uint16): 目标 PWM 位置。
	• duration_ms (uint16): 转动耗时。
• Feedback:
	• current_pwm (uint16): 当前 PWM
	• progress (float32): 进度
• Result:
	• success (bool): 成功为 True，失败为 False。
	• final_pwm (uint16): PRAD 读回的实际位置。
	• message (string): 失败原因

4启动命令
ros2 launch cquick_servo servo.launch.py demo:=true

5 pwmmin=900 pwmmax=2100
给单一舵机发指令。当 PWM 小于/大于某个值时，结构件会顶住底座或发出“嗡嗡”的堵转声

6# 步骤③：改参数不重启
$ ros2 param set /servo_driver pwm_max 1600
Set parameters successful

# 步骤④：立刻发超限 goal，被拒
$ ros2 action send_goal /move_servo cquick_servo_interfaces/action/MoveServo "{id: 1, pwm: 1700, duration_ms: 1000}"
Goal rejected: pwm 1700 exceeds max limit 1600

# 步骤⑤：参数校验拒掉非法值
$ ros2 param set /servo_driver pwm_max 100000
Set parameters failed: 超出 500~2500 范围
7录了什么：/servo/command 和 /servo/state 两个话题，时长约 31.5 秒。
• 怎么回放：ros2 bag play bag/run1
• 回放结果：执行后，舵机自动复现了录屏中的正反转回中位动作。
rose@rose-virtual-machine:~/ros2_ws$ ros2 bag info bag/run1
Files:            run1_0.db
Bag size:         25.5 KiB
Storage id:       sqlite3
Duration:         31.500726s
Start:            Nov 18 2026 20:08:28.475990816 (1795003708.475990816)
End:              Nov 18 2026 20:08:59.976717013 (1795003739.976717013)
Messages:         67
Topic information: Topic: /servo/state | Type: cquick_servo_interfaces/msg/ServoCommand | Count: 64 | Serialization Format: cdr
Topic: /servo/command | Type: cquick_servo_interfaces/msg/ServoCommand | Count: 3 | Serialization Format: cdr
rose@rose-virtual-machine:~/ros2_ws$ rm -rf bag/run1 
8
 TTL模块的 TX/RX/GND 接 24路控制板 的对应引脚。
控制板 供电接电池正负极，电池实测电压：7.72
9
action_name: "stand_up"
steps:
  - step: 1
    joints:
      - id: 1
        pwm: 1500
        duration_ms: 1000
      - id: 2
        pwm: 1600
        duration_ms: 1000
  - step: 2
    joints:
      - id: 3
        pwm: 1400
        duration_ms: 800

当一个step返回result后进行下一步
看动作协调性来确定
播放器内部用 Action Client，发送 goal 后阻塞等待 result 回调，收到成功后再解析下一个 step 的指令下发


10
11月17日 ros2 启动launch节点明明usb检测没问题却报错 怀疑接线错了，写的程序有bug，ubuntu没读到usb 终端输入 ls-l/dev/ttyusb发现没问题后求助大肥鱼发现是之前启动的cutecom占用
11 17 启动节点初始化回中位没有执行·  怀疑程序不对 deep seek建议在initi函数中加sleep终断几秒
11-17 16:15 用 ros2 param set 测试在线改参数 终端返回 Set parameters successful，但发超限的 goal 仍然被执行了，没有拦截 代码里在 __init__ 里把值存进了 self.pwm_max，改参数没触发更新 在 goal_callback 里加了 self.get_logger().info(f"当前限位:{self.pwm_max}") 打印，发现改参数后打印的还是旧值。 删掉 __init__ 里的缓存，改为在回调里实时 self.get_parameter('pwm_max').value。
11-17 17:30 测试中途按 Ctrl+C 取消动作 按了 Ctrl+C 没反应，舵机继续转完才停 execute_callback 里的 time.sleep() 阻塞了线程，导致取消回调进不来 查了官方文档，确认单线程 executor 会被阻塞。 引入 MultiThreadedExecutor 和 ReentrantCallbackGroup，将 time.sleep 改为 self.get_clock().sleep_for，并在循环里检查 is_cancel_requested。
11
1. 先卸力：所有舵机发 #xxxPULK!。原因：防止舵机带着满力矩断电，导致内部齿轮应力集中或堵转发热烧毁。
2. 再断动力电：拔掉电池。原因：避免控制板在电压跌落时发出不可控的乱码指令。
3. 最后关板卡：执行 sudo shutdown now。原因：直接拔电会损坏树莓派/地瓜派的文件系统，导致下次无法开机。
12
1. 任务2：正反转回中位BV1USy3zCEqD
2. 任务2：取消打断	BV1bSy3zyEJK
3. 任务2：改参数后行为变化BV1MSy3zyE7t
4. 任务2：bag回放复现动作BV1mSy3zCEDm
