# 电控组 Week3,4 · 整机拼装与舵机控制

姓名：周慧（zhouhui）　日期：2026-11-22

## 1. 架构图
ros2 launch cquick_servo servo.launch.py
  │
  ├─ servo_driver（Action Server /move_servo）── 串口 /dev/ttyUSB0 115200 8-N-1
  │      │                                        USB 转 TTL(CH340) → 24 路控制板
  │      │                                        → 半双工总线 → 总线舵机
  │      ├─ 发布 /servo/state（alive / locked / online_count / detail，心跳）
  │      ├─ 订阅 /servo/command（内部调试总线，bag 回放走同一条发送路径）
  │      └─ 服务 /servo_driver/watchdog_trigger、/servo_driver/watchdog_reset
  ├─ move_client（Action Client）
  └─ sequence_player（动作组播放器，读 config/actions/*.yaml 展开成 goal 序列）
包：cquick_servo_interfaces（msg/ServoCommand、ServoState、action/MoveServo） / cquick_servo（节点+launch+config）
Action：/move_servo（对外唯一控制口，不用话题 / 服务当控制口）

## 2. 参数表
| 参数 | 默认值 | 为什么是这个默认值 |
|---|---|---|
| port | /dev/ttyUSB0 | CH340 插上后 ls -l /dev/ttyUSB* 实际枚举出来的名字；外置到 yaml 就是为了换机器只改这一行 |
| baudrate | 115200 | 控制板固件固定 115200，波特率不一致的表现是回显乱码或干脆没回显 |
| pwm_min | 900 | 结构实测的安全下限，再往下会顶到限位 |
| pwm_max | 2100 | 同上，安全上限 |
| default_duration_ms | 800 | 客户端不写 duration 时的兜底；800 ms 走小角度平滑、不猛跳撞结构 |
| servo_ids | [1,2,3,4,5,6]​ | 本机在线的舵机 ID（单点实测到的是 006）；不在列表里的 ID 一律拒绝 |
| startup_reset | true | 启动先全部回 1500，避免带着上一次的位置上电顶着结构 |

这些值只写在 config/servo_params.yaml 里，代码里只出现 "声明参数" 和 "读参数" 两处，其余全部走 self.get_parameter(...).value。允许保留字面量的只有协议常量：#、!、PID/PRAD/PULR/PULK、P/T、ID 补 3 位。

## 3. 接口契约 MoveServo
- Goal：command.id (uint8) ; 舵机 ID，1~254；不在 servo_ids 里 → 直接 REJECT
- Goal：command.pwm (uint16) ; 目标 PWM。500~2500 是电气极限，本节点只接受 pwm_min~pwm_max
- Goal：command.duration_ms (uint16) ;  从当前位置走到目标的用时；给 0 或超范围时用 default_duration_ms
- Feedback：current_pwm / progress ; 当前 PWM 与进度 0~1，每 200 ms 一帧（≥5 Hz)
- Result：success、final_pwm（PRAD 读回值）、message（超限/取消/串口错/没到位） ; 成没成、动作末尾 PRAD 读回的位置、失败原因（超限 / 取消 / 串口错 / 没到位）

时机：goal_callback 在发串口之前判超限和 ID；execute_callback 先发 PULR 恢复扭力再拼运动指令；结束时读 PRAD；收到取消立刻停发、回中位、返回 message='canceled'。

## 4. 一条启动命令
​```bash
ros2 launch cquick_servo servo.launch.py
​```
起来后驱动节点从 config/servo_params.yaml 加载七个参数，并发布 /servo/state。真机不在位时 detail 会是 link=down，节点不崩、仍可做命令行验证（下文 "无硬件模式" 指的就是这个状态）。​
换端口：ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB1​

## 5. 安全区间怎么试出来的
pwm_min = 900，pwm_max = 2100，以 1500 为中位、两侧各约 ±600。​
试法（装结构之前、单点桌面系统上做）：​
发 #006PULR! 恢复扭力，再发 #006P1500T1000! 停在 1500，确认平滑到位、#006PRAD! 读回接近 1500（实测从 0961 读回到 1500 附近）；​
以 1500 为基准向两侧以 100 为一档往外试，每档等它走完再听声音、看结构；​
出现持续 "嗡嗡" 或结构快顶住，就退回上一档，两端各留约 100 余量，取 900 / 2100；​
两个数写进 yaml，代码里不再出现字面量。​
不直接用 500~2500：那是舵机的电气范围，结构顶死后电流全变成热，三分钟能烧穿。

## 6. 运行时改参数演示（2.6 六步真实终端输出）
① ros2 param get /servo_driver pwm_max            → 2100
② send_goal pwm=1700                              → 接受、舵机转过去
③ ros2 param set /servo_driver pwm_max 1600       → Set parameters successful
④ send_goal pwm=1700                              → Goal was rejected，日志打出超出限位 [900, 1600]
⑤ ros2 param set /servo_driver pwm_max 100000     → Set parameters failed：pwm_max 超出 500~2500
⑥ ros2 param set /servo_driver pwm_max 2100       → 恢复；再发 1700 → 接受

第 ④ 步是关键：没有重启节点，改完 pwm_max 下一条 goal 立刻被拒。能做到，是因为限位判断每次都 self.get_parameter('pwm_max').value 实时读，而不是在 __init__ 里存了一份副本；参数校验走 add_on_set_parameters_callback，界外值和 pwm_min >= pwm_max 这种打架组合都返回 SetParametersResult(successful=False, reason=...)。对应截图：02_param_set现场.png、参数校验与当帧生效（任务二 参数部分）.png。

## 7. bag
录制命令：ros2 bag record /servo/command /servo/state -o bag/run1
ros2 bag info bag/run1 输出：见 02_bag_info.png，消息条数与话题如下：
Topic information: Topic: /servo/state   | Type: ... | Count: 
                   Topic: /servo/command | Type: ... | Count: 
                   
回放：ros2 bag play bag/run1。回放能复现的原因是 Action 进来的 goal 和 /servo/command 进来的指令最后调用同一个底层发送函数，只有一份发送逻辑，不会出现 "回放和录屏对不上"。 （这次录制时手上没有舵机，/servo/state 里 detail 为 link=down，所以回放看到的是指令与状态序列本身，舵机不会真的动。）

## 8. 接线说明 + 实测电压
舵机三线:白 = 信号，红 = 正极（5~8.4 V）， 黑 = GND
USB 转 TTL（CH340）:TX → 控制板 RX，RX → 控制板 TX，GND → 控制板 GND
电池（2S 1500 mAh 7.4 V）:控制板电源输入，红接正、黑接负，上电前先量电压
共地:板卡 / 控制板 / 电池负极必须连在一起；只接信号不共地，数据一定收不到
上电前万用表实测:7.8V,7.6V,7.2V

## 9. 动作 yaml 结构
# config/actions/stand_up.yaml
## 9. 动作 yaml（任务 3）

一套动作 = 若干 step 组成的有序列表；每个 step 有 `label` 和一个 `moves` 列表，`moves` 里的多条**同时下发**，step 之间**串行**（上一个 step 的 result 全回来才发下一个）。

```yaml
name: stand_up
description: 屈膝下蹲 → 起立 → 重心前压 → 全机归位
default_duration_ms: 3000

steps:
  - label: 屈膝下蹲（双腿同时，step 内并行）
    moves:
      - {id: 2, pwm: 1360, duration_ms: 2500}
      - {id: 4, pwm: 1360, duration_ms: 2500}
      - {id: 1, pwm: 1420, duration_ms: 2500}
      - {id: 3, pwm: 1420, duration_ms: 2500}

  - label: 起立回中位
    moves:
      - {id: 2, pwm: 1500, duration_ms: 2500}
      - {id: 4, pwm: 1500, duration_ms: 2500}
      - {id: 1, pwm: 1500, duration_ms: 2500}
      - {id: 3, pwm: 1500, duration_ms: 2500}
```

- **谁先动**：会互相干涉的关节拆成两个 step，靠"上一个 step 的 result 回来"保证先后，不靠调 `duration_ms` 硬凑；要同时到位的放同一个 `moves` 里。
- **duration 怎么定**：按时长 ∕ 舵机自身速度给足（实测约 30~50 PWM/s），幅度取 150~170 PWM（约 13~15°）——大于驱动 ±30 的到位容差，又离结构限位有余量；给小了会报 `Not in place`。
- **result 同步**：`sequence_player.py` 每一步把 `moves` 展开成若干 goal 并发打出去，等本 step 全部 result 回来才进下一步，不用固定 `sleep`（舵机走完的真实时间不等于 `duration_ms`，用 sleep 会累积误差）。

播放器在 `cquick_servo` 里，动作文件在 `config/actions/`，和 `servo_params.yaml` 同层。现有四套：`home.yaml`、`stand_up.yaml`、`desk_single.yaml`、`wave.yaml`。换动作只新增/替换一个 yaml，不改一行 Python。


(整机由组内成员装配，目前尚未拼装完成，因此任务 3 只交付代码与动作数据（sequence_player.py + config/actions/ 四套 yaml），没有整机照片、接线特写与多舵机动作录像。动作组 "yaml → goal 序列 → result 同步" 这条链在桌面单点上验证过。）

## 10. 踩坑记录（≥3 个）
（1）colcon build 报 error: can't copy ... resource/cquick_servo → 包目录下根本没有 resource/，setup.py 的 data_files 引用了不存在的文件 → mkdir -p resource && touch resource/cquick_servo 后通过；
（2）虚拟机里 ls /dev/ttyUSB* 什么都没有 → brltty 抢占了 CH340 → systemctl stop/enable --now brltty 后重新插拔出现 ttyUSB0（顺带：usermod -aG dialout 后必须重新登录才生效）；
（3）发 #001PID! 无回显、舵机不动 → 这个舵机出厂 ID 不是 1 → 1~24 逐个扫，扫到 #006PID! 有回显，改成 #006 后 PULR + P1500T1000 平滑到位。

## 11. 下电三步与原因
① 所有舵机发 #xxxPULK! 卸力（断电前还顶着力矩会堵转发热）
② 拔电池断动力电（避免控制板在异常电压下乱发指令）
③ sudo shutdown now 关板卡（直接断电会损坏文件系统）

## 12. 录屏链接（B 站）
任务一：https://b23.tv/fHgN3QM
任务二：https://b23.tv/6pimstY
