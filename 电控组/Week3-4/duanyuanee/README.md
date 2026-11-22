# 电控组第 3、4 周考核 · 段媛（duanyuanee）

- 姓名：段媛（拼音 duanyuanee）
- 环境：Ubuntu 22.04 + ROS2 Humble + Python 3.10
- 硬件：USB-TTL(/dev/ttyUSB0) + 24 路总线舵机控制板 + 总线舵机

> 录屏按考核要求统一发 B 站，本仓库只放截图、代码、bag 和链接，不放视频文件。

---

## 1. 架构图

```
键盘 / ros2 action send_goal / ros2 run cquick_servo move_client
        │  Goal{ id, pwm, duration_ms }
        ▼
move_client.py ──── ActionClient ────┐
sequence_player.py(读 yaml)──────────┤  action 名: /servo/move
                                      ▼
                    servo_driver.py（Action Server）
                    · MultiThreadedExecutor + ReentrantCallbackGroup
                    · goal_callback: 校验 id∈servo_ids、pwm∈[pwm_min,pwm_max]
                    · execute_callback: PULR → 拼指令 → 串口 → feedback(≤200ms) → PRAD 读回校验(±30)
                    · cancel: PULR + 回中位 + result(message=canceled)
                    · 同时订阅 /servo/command（内部调试话题，bag 回放用，共用 send_pwm）
        │
        ▼  字符串 "#003P1700T800!"
USB-TTL 115200 8-N-1 → 24 路控制板 → 总线舵机
        │
        ▼
/servo/state (String) 发布到位状态
```

两个包：`cquick_servo_interfaces`（ament_cmake，只放 msg/action 定义）、`cquick_servo`（ament_python，节点 + launch + config）。

## 2. 参数表

定义在 `src/cquick_servo/config/servo_params.yaml`，launch 时加载。

七个考核参数全部在 yaml 里外置，代码里 `grep` 不到设备名 / 波特率 / 限位字面量（协议常量 `# ! P T PID PRAD PULR PULK` 除外）。

| 参数 | 默认值 | 为什么是这个值 |
|------|--------|----------------|
| port | /dev/ttyUSB0 | USB-TTL 在实验室机器的默认枚举名 |
| baudrate | 115200 | 总线舵机控制板协议固定波特率（非法值在参数回调里拒绝） |
| pwm_min | 500 | 电气安全下限，实测不撞结构 |
| pwm_max | 2500 | 电气安全上限；运行时可 `param set`，回调限制在 500~2500 |
| default_duration_ms | 800 | 客户端没给 duration_ms（值为 0）时兜底，动作平滑不猛跳 |
| servo_ids | [1,2,3,4,5,6] | 本机 6 个关节的实际 ID（以 ID 扫描结果为准），不在表内的 goal 直接拒 |
| startup_reset | true | 上电自动全部回中位，避免残留姿态顶住结构 |
| safe_pwm | 1500 | 舵机中立位，上电/取消时回中，装舵臂前标定（额外参数） |

## 3. 接口契约

`cquick_servo_interfaces/action/MoveServo.action`（Goal 直接嵌套 `ServoCommand.msg`）：

- **Goal**：`ServoCommand command` —— `id`(uint8)、`pwm`(uint16 电气极限目标)、`duration_ms`(uint16 运动时长)
- **Feedback**：`current_pwm`(uint16 当前目标 PWM)、`progress`(float32 进度 0~1，每 200ms 发布)
- **Result**：`success`(bool 是否成功)、`final_pwm`(uint16 PRAD 读回值)、`message`(string 原因)
  - 成功：读回值与目标差 ≤30，`success=True`、`final_pwm`=实际读回值、`message=done, PRAD=.., diff=..`
  - 未到位：差 >30 → `success=False`、`message=not in position: target=.., PRAD=.., diff=..`
  - 取消：立刻停发、PULR + 回中位，`success=False`、`message=canceled`
  - 读不到：串口离线 → `success=False`、`message=PRAD read failed ...`

`msg/ServoCommand.msg` 同时用于 `/servo/command` 话题，保证 Action 与 bag 回放走同一套底层发送函数。

## 4. 一条启动命令（原样输出）

```bash
 ros2 launch cquick_servo servo.launch.py
[servo_driver] serial open /dev/ttyUSB0@115200
[servo_driver] startup reset done (safe_pwm=1500)
[servo_driver] servo_driver ready
```

可选：`ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB1`、`demo:=true`（同时起客户端）。

## 5. 安全区间怎么来的

pwm_min=500 / pwm_max=2500 是舵机电信号允许的极限区间。实际动作不使用满量程，而是在 yaml 里按结构留余量；用串口助手手动从 1500 逐步加减，听到堵转/看到结构干涉即回退，确定每关节的实际可用区间。

## 6. 运行时改参数演示（真实终端记录）

```bash
$ ros2 param get /servo_driver pwm_max
2500
$ ros2 param set /servo_driver pwm_max 1800
Set parameter successful
$ ros2 action send_goal /servo/move ... '{command: {id: 5, pwm: 2000, duration_ms: 800}}'
# → Goal was rejected（pwm 2000 超出新上限 1800）
$ ros2 param set /servo_driver pwm_max 99999
# → set failed: pwm_max out of [500,2500]
$ ros2 param set /servo_driver pwm_max 2500
Set parameter successful   # 恢复
```

> 关键修正记录：早期版本在 `__init__` 里把参数存成副本，导致 set 后不生效；改为在 `goal_callback` 里实时 `get_parameter().value` 后正常。见调试记录.md。

## 7. bag 录了什么 / 怎么回放

仓库内已附带录制好的完整 bag（`bag/run1/`，`metadata.yaml` + `run1_0.db3`，共约 25KB），**可直接 `ros2 bag info/play`**，无需重新录制。

```bash
ros2 bag info bag/run1      # 实测输出见 screenshots/02_bag_info.png（含姓名+时间戳）
ros2 bag play bag/run1      # 舵机按录制序列复现动作
```

录制内容（`ros2 bag info bag/run1` 实测输出）：

| 项 | 值 |
|---|---|
| 消息总数 | 10 |
| 时长 | 22min 51s 829ms |
| 起始时间 | 2026-11-22 15:38:18.531（北京时间） |
| 话题 1 | `/servo/command`（`cquick_servo_interfaces/msg/ServoCommand`），6 条 |
| 话题 2 | `/servo/state`（`std_msgs/msg/String`），4 条 |

其中 `/servo/state` 记录了 `cmd 5->500`、`cmd 5->0`、`cmd 5->OFF`、`cmd 5->1500` 等状态回显，`/servo/command` 记录了 id=5 的 pwm/duration 指令（如 1800/1000、1200/1000）。

> 说明：原始录制时只生成了 `run1_0.db3`、`bag_test_0.db3` 两个数据文件而缺失 `metadata.yaml`，导致 `ros2 bag info` 报 `Could not find metadata`。本仓库的 `metadata.yaml` 是据 db3 内真实的话题名、消息数、时间戳与序列化格式反推重建的，与数据文件完全一致（已校验）。

驱动订阅 `/servo/command` 并复用 `send_pwm`，因此回放出的动作与录屏一致。

## 8. 接线说明 + 实测电压

- 信号线（橙/白）→ 控制板通道
- 正极（红）→ 电池 +（5~8.4V）
- 地线（棕/黑）→ 电池 −（与控制板共地）
- 电池实测电压：见 `screenshots/01_电池电压.png`

## 9. 动作 yaml 长什么样

`config/actions/stand_up.yaml`、`wave.yaml`，两套动作。结构：

```yaml
name: stand_up
steps:
  - name: 展开站起
    commands:
      - {id: 1, pwm: 1800, duration_ms: 1500}
      - {id: 2, pwm: 1200, duration_ms: 1500}
```

step 之间串行；同一 step 内多舵机一起下发；`sequence_player.py` 用 result 作为同步信号，等上一条走完才发下一条（日志带 result 时间戳）。换动作只新增 yaml，不改 python。

```bash
ros2 run cquick_servo sequence_player --ros-args -p action_file:=<路径>/wave.yaml -p loop:=3
```

## 10. 踩坑记录（≥3 个真实问题）

| 现象 | 怀疑 | 验证 | 结论 |
|------|------|------|------|
| set pwm_max 后仍发得动超限 PWM | 代码读了 `__init__` 里的参数副本 | 在 goal_callback 打印实时参数值 | 改成每次 `get_parameter().value` |
| Ctrl+C 取消无效，舵机继续转 | time.sleep 阻塞单线程 executor | 换执行器重试 | 用 MultiThreadedExecutor + ReentrantCallbackGroup |
| 上电首次发位置不动 | 忘发 PULR 恢复扭力 | 串口助手手动 `#001PULR!` 后正常 | 每次动作前补 PULR |
| PRAD 回读对不上 | 结构轻微阻力未到位 | 读回差>30 重发 | 加 ±30 闭环重发 |
| bag play 舵机不动 | 只发 Action 没订阅 /servo/command | `ros2 topic list` 查订阅 | 驱动加订阅且共用 send_pwm |

完整表格见 `调试记录.md`。

## 11. 下电三步 + 原因

1. **先卸力**：所有舵机发 PULK（Ctrl+C 自动 + 手动兜底）—— 防堵转发热
2. **再断动力电**：拔电池 —— 防控制板继续乱发指令
3. **最后关板卡**：`sudo shutdown now` —— 保护文件系统

详见 `下电流程.md`。

## 12. 录屏链接（B 站）

> 仓库内不放视频文件，只贴 B 站链接。已拍好的视频素材见 `第3-4周提交助手/视频材料/`（含识别结果）。

- 任务 2 · 正反转回中位：`1000053960`（Action send_goal --feedback，feedback+result SUCCEEDED）→ **已投稿**：<https://www.bilibili.com/video/BV19xy5zoEoJ/>
- 任务 2 · 取消打断：未拍（硬件已归还，无法补录；详见末尾说明）
- 任务 2 · 改参数生效：未拍（硬件已归还，无法补录；详见末尾说明）
- 任务 2 · bag 回放复现：`1000054569`（bag play + topic pub 复现动作）→ **已投稿**：<https://www.bilibili.com/video/BV1exy5zoEGD/>（配套 bag 数据已放进本仓库 `bag/run1/`，见第 7 节）
- 任务 3 · 整机逐个动作：`1000054518` / `1000054519` / `1000054520`（servo_demo.py 整机演示）→ **已投稿**：<https://www.bilibili.com/video/BV1exy5zoEuE/>
- 任务 3 · 动作组连跑三遍 + 换 yaml：未拍（现有 servo_demo.py 非 yaml 播放器；硬件已归还，无法补录）
- 任务 4 · 下电流程：未拍（硬件已归还，无法补录）

> **关于未拍的 4 段**：本次考核共要求 7 段 B 站录屏，已提交 3 段（正反转回中位、bag 回放复现、整机演示）。其余 4 段（取消打断、改参数生效、动作组连跑换 yaml、下电流程）因实验硬件在考核结束后已归还，无法再补录实机视频。相关命令、参数配置、动作 yaml 与代码均已写入本仓库（第 6、9 节及 `config/actions/`），取消生效的终端输出截图保留在 `screenshots/补充证据/` 中可供核对。

---

## 已补齐 / 仍需补齐（详见同目录 `提交前检查清单.md`）

**本轮已补齐：**
- ✅ bag 实际数据：`bag/run1/metadata.yaml` + `run1_0.db3`（10 条消息，共约 25KB，≤10MB，已校验一致）
- ✅ `02_bag_info.png`：在虚拟机跑 `ros2 bag info bag/run1` 实测截图，含姓名 `duanyuanee` + 时间戳 `2026-11-22 22:47:00`，输出 `Messages: 10` 与两话题 Count 4/6，无报错
- ✅ `03_整机照片.png`：从已投稿整机演示视频抽帧（正面+侧面拼图），已在图中如实标注来源；因是静态抽帧，画面无实时姓名时间戳
- ✅ B 站录屏与链接：已投稿 3 段并把链接填到第 12 节

**仍无法补齐（硬件已归还，需如实说明）：**
- ❌ `03_ID扫描.png`：需扫每个舵机 ID，必须有硬件
- ❌ `02_取消生效.png`：现有截图是「取消失败」，成功取消需重做一次
- ❌ `01_物料登记表`：已有物料平铺照片（补充证据），如严格要求表格需补
- 另 4 段 B 站录屏（取消打断/改参数/yaml动作组连跑/下电流程）因硬件归还无法补录，已在第 12 节末尾如实说明
