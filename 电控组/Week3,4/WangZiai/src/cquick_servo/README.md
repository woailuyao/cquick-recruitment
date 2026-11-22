# cquick_servo

总线舵机（ZP 系列）的 ROS 2 驱动包：用 Action 控制单个舵机，用 YAML 编排多舵机动作组。

配套接口包：`cquick_servo_interfaces`（`msg/ServoCommand`、`action/MoveServo`），两个包需要一起编译。

## 1. 环境依赖

```bash
# ROS 2 Humble 已安装的前提下
sudo apt install python3-serial python3-yaml
# 或者在 workspace 根目录
rosdep install --from-paths src --ignore-src -y
```

## 2. 编译

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select cquick_servo_interfaces cquick_servo
source install/setup.bash
```

> 注意：本包 `--symlink-install` **不会**把 Python 文件软链到 install 目录，
> 改完 `src/` 必须重新 `colcon build`，否则 `ros2 run` 跑的还是旧代码。

## 3. 运行

```bash
# 无硬件自测（虚拟串口，全部指令模拟）
ros2 launch cquick_servo servo.launch.py port:=mock

# 真机（已装 udev 规则，默认就是 /dev/cq_servo）
ros2 launch cquick_servo servo.launch.py

# 真机（没装 udev 规则）
ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB0
```

发一个目标（另开终端，记得 `source install/setup.bash`）：

```bash
ros2 action send_goal /move_servo cquick_servo_interfaces/action/MoveServo \
  "{id: 1, target_pwm: 1800, duration_ms: 1200}" --feedback
```

跑动作组：

```bash
ros2 run cquick_servo sequence_player --ros-args -p action:=stand_up
ros2 run cquick_servo sequence_player --ros-args -p action:=squat_down -p duration_scale:=0.5
ros2 run cquick_servo sequence_player --ros-args -p dry_run:=true -p action:=up_and_down   # 只预检不发指令
```

## 4. 参数（config/servo_params.yaml）

| 参数 | 默认 | 说明 |
|---|---|---|
| `port` | `/dev/cq_servo` | 串口设备；无规则时用 `/dev/ttyUSB0`，无硬件用 `mock` |
| `baudrate` | 115200 | 8-N-1 |
| `pwm_min` / `pwm_max` | 900 / 2100 | 安全行程，超出直接 REJECT |
| `default_duration_ms` | 800 | 未指定时长时的默认值 |
| `servo_ids` | [1..10] | 在线舵机 ID 列表，不在列表里的目标直接拒绝 |
| `startup_reset` | true | 启动时全部回 1500 中位 |
| `watchdog_timeout` | 3.0 | 秒；超时无指令全部 PULK，0=关闭 |
| `resend_threshold` | 30 | PRAD 偏差超此值自动补发一次；0=关闭 |
| `success_tolerance` | 30 | 判定"到位"的误差容差 |
| `accept_wait_timeout` | 3.0 | 秒；goal 到达时舵机正忙，最多等这么久再 REJECT（0=立即拒） |
| `send_wait_timeout` | 1.0 | 秒；插值每一段抢舵机锁的等待上限，抢不到才算"舵机忙" |
| `stop_on_finish` | true | 动作结束后用 T=0 钉住当前位置，立刻停掉残余插值 |
| `release_on_finish` | false | 每个动作结束自动对全部舵机 PULK 卸力（整机负载重时慎用） |

## 5. 动作组 YAML（config/actions/*.yaml）

```yaml
name: stand_up
description: 膝关节起立
steps:
  - name: 起立
    servos:
      - {id: 5, pwm: 1500, duration_ms: 900}
      - {id: 6, pwm: 1500, duration_ms: 900}
```

- 同一个 step 内的舵机**并行**执行；
- step 之间等所有 result 回来才发下一步（不是 sleep）；
- 换动作只加 YAML，**不用改 Python**。

## 6. udev 固定设备名（可选，推荐）

```bash
sudo cp udev/99-cq-servo.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
ls -l /dev/cq_servo        # 应指向 ttyUSB*
```

规则内容：CH340（`1a86:7523`）生效，CP2102（`10c4:ea60`）注释备用，改成自己模块的 VID:PID 即可。

没装规则时，用 `port:=/dev/ttyUSB0` 覆盖即可，功能不受影响。

## 7. 测试

```bash
cd src/cquick_servo
python3 -m pytest test -q          # 需要先 source /opt/ros/humble/setup.bash
python3 -m flake8 --max-line-length=99 .
```

当前：`8 passed, 1 skipped`，flake8 干净。

## 8. 协议速查

| 指令 | 含义 | 回显 |
|---|---|---|
| `#001PID!` | 查询 ID | `#001P!` |
| `#001PULR!` | 上电使能 | 静默 |
| `#001PULK!` | 卸力 | 静默 |
| `#001P1500T1000!` | 1 秒转到 1500 | 静默 |
| `#001PRAD!` | 读位置 | `#001P1500!` |

格式：`#` + 3 位 ID + `P` + 4 位 PWM + `T` + 4 位时间 + `!`。

## 9. 常见问题

| 现象 | 原因 | 处理 |
|---|---|---|
| `打不开串口 /dev/cq_servo` | 没装 udev 规则 / 没插设备 | 用 `port:=/dev/ttyUSB0`，或装规则 |
| `Goal was rejected` | pwm 超 `pwm_min~pwm_max`、id 不在 `servo_ids` | 看驱动日志的拒绝原因 |
| `读不到位置（PRAD 无应答）` | 舵机没供电 / 半双工线没并 | 量电池电压、检查 TXD+RXD 是否并到信号线 |
| `ros2 run` 改了代码没生效 | install 里是副本 | 重新 `colcon build` |
| 动作组跑到一半 `✗ 动作组失败`、日志有"舵机 X 正忙" | 同一舵机被别的 goal 占着（并发 goal / 上一步还没释放） | 调大 `accept_wait_timeout`（驱动）与 `goal_retry`（播放器），见下节 |
| `Goal was rejected` 但 id/pwm 都合理 | 该舵机正在执行另一个 goal | 同上；或 `-p goal_retry:=10` |

## 安全下电（`/servo_shutdown` 服务）

除了 Ctrl+C 退出（`destroy_node` 里同样会卸力），还提供一条“主动下电”的服务：

```bash
ros2 service call /servo_shutdown std_srvs/srv/Trigger
# response: success=True, message='已全部卸力，可断电池'
```

行为：

1. 对 `servo_ids` 里每个舵机发 `#xxxPULK!`，逐个打印 ✓ / ✗
2. 关闭串口
3. 之后新 goal 一律 `REJECT`（日志：`✗ 已进入下电流程，拒绝新目标`）
4. 看门狗与 `/servo/command` 订阅同时停下，不会刷错误日志

考核任务 4 的标准顺序：

```bash
ros2 service call /servo_shutdown std_srvs/srv/Trigger   # 1) 卸力
# 2) 看到回执“已全部卸力，可断电池”
# 3) 拔电池
# 4) sudo shutdown now
```

> 这个服务只负责“让舵机松劲”，断电和关机仍需人工确认。

## 舵机忙 / 并发调参

多个 goal 打到同一个舵机时，驱动不会立刻拒绝，而是先等一会儿：

```yaml
# config/servo_params.yaml
accept_wait_timeout: 3.0   # goal 接受阶段最多等 3 秒
send_wait_timeout: 1.0     # 插值时抢舵机锁最多等 1 秒
```

播放器端还有一层重试（`ros2 run cquick_servo sequence_player ...`）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `goal_retry` | 6 | 被拒后重试次数，0=不重试 |
| `retry_interval` | 0.3 | 每次重试间隔（秒） |

推荐值：

```bash
# 舵机多、动作密集（默认即可）
ros2 launch cquick_servo servo.launch.py

# 现场演示，宁可多等也不失败
-p accept_wait_timeout:=6.0 -p goal_retry:=12

# 调试，要求立刻失败
-p accept_wait_timeout:=0.0 -p goal_retry:=0
```

> 等待是在回调里 `sleep` 实现的，`accept_wait_timeout` 别超过 10 秒，否则 Ctrl+C 响应会变慢。

## 动作播完还在动？

三种常见原因，分别对号入座：

| 现象 | 原因 | 处理 |
|---|---|---|
| 结果已经返回，舵机还在慢慢挪几度 | 最后一段插值还在总线里跑（T 时长没走完） | 默认已开 `stop_on_finish: true`，动作一结束就发 T=0 钉住 |
| 动作结束后 3 秒左右，关节突然"软下来" | 看门狗超时 PULK 卸力 | 表演期间设 `watchdog_timeout: 0.0` 关掉，或加大到 10 |
| 停在目标位但持续轻微抖动/嗡嗡 | 负载重、PWM 卡在临界点 | 换更大扭矩舵机，或把目标 PWM 稍微调离临界值 |

想手动立刻全部停住：

```bash
python3 ~/WangZiai/ros2_ws/tools/servo_send.py --ids 1-10 --template '#{id:03d}PULK!'
```

### 动作结束要不要自动卸力？

```yaml
release_on_finish: false   # 默认：保持上力（关节能撑住）
release_on_finish: true    # 每个动作结束全部 PULK（关节会松，重载会塌）
```

| 场景 | 建议 |
|---|---|
| 整机站立/承重 | `false`，否则动作一停关节就软 |
| 台架测试、单舵机 | `true`，避免长时间堵转发热 |

打开后，下一条 goal 到达时驱动会自动先 `PULR` 再动，日志会写：

```text
▸ 动作结束自动卸力：已 PULK 10 台（共 10 台）
▸ [舵机 5] 已卸力 → 自动 PULR
```
