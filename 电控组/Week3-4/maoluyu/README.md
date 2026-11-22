# # CQuick 电控招新考核 README
姓名：毛鲁渝
提交日期：2026-11-23

参数名	参数含义	默认值	默认值来源
pwm_min	舵机 PWM 最小值，低于该值拒绝执行	500	config/servo_params.yaml
pwm_max	舵机 PWM 最大值，高于该值拒绝执行	2100	config/servo_params.yaml
serial_port	串口设备路径	/dev/ttyUSB0	config/servo_params.yaml
baudrate	串口波特率	115200	config/servo_params.yaml

MoveServo action 接口契约说明
# Goal（请求）
int32 id          # 舵机ID
int32 pwm         # 目标PWM值
int32 duration_ms # 运动时间(ms)
---
# Result（返回结果）
bool success      # 是否成功执行
int32 final_pwm   # 最终舵机PWM
string message    # 提示信息
---
# Feedback（实时反馈）
float progress    # 运动进度 0~1
int32 current_pwm # 当前舵机PWM

4. 启动命令复制输出
# 编译
cd ~/cquick_ws
colcon build --packages-select cquick_servo cquick_servo_interfaces
source install/setup.bash

# 启动舵机驱动节点
ros2 launch cquick_servo servo.launch.py

# 任务2：发送Action测试
ros2 action send_goal /move_servo cquick_servo_interfaces/action/MoveServo "{id: 1, pwm:1600, duration_ms:1000}" --feedback

# 任务2：bag录制
ros2 bag record /servo/command -o servo_run
# bag查看信息
ros2 bag info servo_run
# bag回放
ros2 bag play servo_run

# 任务3：YAML动作序列播放器
ros2 run cquick_servo sequence_player

安全区间
出厂默认：pwm_min=500，pwm_max=2100
实测：PWM <500 或者> 2100 时，舵机容易卡死、堵转、发热，存在机械限位风险
测试： 
1.发送 pwm=400，goal 被服务端拒绝，不执行
2.发送 pwm=2200，goal 被服务端拒绝，不执行
3.在区间 [500,2100] 内的 PWM 指令，舵机可以平稳运动，无卡顿异响
结论：安全区间为 500 ≤ pwm ≤ 2100，动态参数修改后会实时更新这个安全判断边界
Integer value is: 2100
[INFO] 已发布 /servo/command Feedback: progress: 0.3, current_pwm:1200 Feedback: progress: 0.7, current_pwm:1500 Result: success: True, final_pwm:1600, message: "move finish"
Set parameter successful
Goal rejected: pwm 1700 exceeds pwm_max 1600
Set parameter failed: value 500-2500
Set parameter successful

bag 信息、回放说明
Files: servo_run_0.db3 Bag size: 8.4 KiB Storage id: sqlite3 Duration: 1.2s Messages: 1 Topic information: Topic: /servo/command | Type: cquick_servo_interfaces/msg/ServoCommand | Count: 1
三条问题：
1.舵机总是断连，不知道什么原因，时灵时不灵，后来发现是tx/rx接反了
2.bag info 消息数量为 0，回放舵机不动 | 录制时没有发布 /servo/command 话题 | 用 ros2 topic echo 监听，发 action 无输出；在 execute_callback 增加 publish 代码，重新编译，echo 正常输出 | Action 本身不能录制，必须在 Action 回调里主动发布 topic 供 bag 录制 |
3.任务三舵机有些时候无法回中
录屏链接：【任务二视频（舵机在下面，应该有声音）-哔哩哔哩】 https://b23.tv/NLmyrDv
