# 加分项说明 · youliangsiyuan（第 3-4 周 电控组）

> 本文件夹单独提交加分项材料。
> 作者：游梁思远 / `youliangsiyuan`
> 整理日期：2026-11-19（真机补测）

---

## 一、六项加分项的实际状态

| # | 加分项 | 分值 | 状态 | 证据文件 |
|---|---|---:|---|---|
| 1 | pytest 单测 | +2 | **已完成（真机无关，可复现）** | `01_pytest单测_真实输出.log`（27 passed） |
| 2 | PRAD 闭环校验 | +3 | **已完成（真机实测）** | `03_Prad闭环_未到位重发到位.png` |
| 3 | 安全看门狗 | +3 | **已完成（真机实测）** | `04_看门狗_触发卸力与外部恢复.png` |
| 4 | 串口掉线自动重连 | +3 | **已完成（真机实测）** | `05_串口重连_拔插恢复.png` |
| 5 | udev 固定设备名 | +2 | **已完成（真机实测）** | `06_udev_固定设备名.png`、`07_udev_默认端口生效.png` |
| 6 | 动作组抢占 | +4 | **未实现** | `sequence_player` 无打断机制 |

**已完成项目对应分值合计：+13**（pytest 2 + 闭环 3 + 看门狗 3 + 重连 3 + udev 2），最终加分按考核上限 **+10** 计。第 6 项未做，不申报。

---

## 二、本文件夹内容

```text
加分项/
├── README.md                        # 本文件：状态总表 + 每项实测证据
├── 01_pytest单测_真实输出.log        # pytest -v 完整真实输出（27 passed）
├── 02_代码证据与机制说明.md          # 闭环 / 看门狗 / 重连的代码位置与触发条件
├── 03_Prad闭环_未到位重发到位.png    # 未到位 → 重发 → 到位
├── 04_看门狗_触发卸力与外部恢复.png  # 触发 → 10 台 PULK → 锁死 → 外部确认恢复
├── 05_串口重连_拔插恢复.png          # 写入失败 → 每秒重试 → 插回自动恢复
├── 06_udev_固定设备名.png            # 规则文件 + /dev/cq_servo 符号链接
└── 07_udev_默认端口生效.png          # 不带 port:= 启动，日志显示打开 /dev/cq_servo
```

被测代码在主目录：

- `src/cquick_servo/cquick_servo/servo_driver.py`（闭环、看门狗、重连）
- `src/cquick_servo/cquick_servo/servo_protocol.py`（纯函数：拼指令、解析回包）
- `src/cquick_servo/launch/servo.launch.py`（端口默认值 `/dev/cq_servo`）
- `src/cquick_servo/config/servo_params.yaml`（`port: /dev/cq_servo`）
- `src/cquick_servo/test/test_servo_protocol.py`（27 条用例）

---

## 三、PRAD 闭环校验（+3）实测证据

**做法**：`prad_tolerance` 临时设为 25、`prad_retries` 设为 1，用 `duration_ms=300` 的短行程动作，让第一次读回落在阈值外。

**真机日志**（`03_Prad闭环_未到位重发到位.png`）：

```text
[WARN] [PRAD] 未到位：目标 1181，实测 1211，差 30 > 25 -> 第 1 次重发
[INFO] [PRAD] 到位：目标 1181，PRAD 读回 1202，误差 21 <= 25（重发 1 次）
```

完整链路“未到位 → 重发 → 到位”有日志时间戳可查（`success=true`、`retry_count=1`）。

> 说明：常规参数下不会触发（重发同一目标不改变停稳后的位置）；上面的证据是把动作时长压短、让第一次读回发生在行进中产生的，属于人为构造的真实触发。

---

## 四、安全看门狗（+3）实测证据

**做法**：`watchdog_timeout_s` 临时压到 0.05 s，用 `duration_ms=50` 的短动作让推进循环跟不上阈值。

**真机日志**（`04_看门狗_触发卸力与外部恢复.png`）：

```text
[ERROR] 看门狗触发：动作推进超过 0.05s 没有进展，全部舵机卸力！恢复需要外部确认：ros2 param set /servo_driver torque_enable true
[ERROR] 看门狗触发：动作推进停滞，已全部卸力
[WARN]  PULK：servo_ids=[1, 2, 3, 4, 5, 6, 7, 8, 9, 10] 全部卸力
[ERROR] 看门狗已触发，全部舵机已卸力；确认安全后 ros2 param set /servo_driver torque_enable true 恢复
[WARN]  看门狗已由外部确认恢复（torque_enable=true），舵机重新上力
```

验证了四件事：**触发 → 全部 10 台 PULK → 之后 goal 被拒（锁死）→ 只有外部 `torque_enable=true` 才恢复**，恢复后能正常动作。

> 诚实说明：本实现判据是“**动作推进是否停滞**”，不是直接监听 `/servo/state` 心跳；`/servo/state` 的 1 Hz 心跳是独立发布的。覆盖的风险一样：上位机卡死、播放器挂在半路时舵机不会一直顶着满力矩。

---

## 五、串口掉线自动重连（+3）实测证据

**做法**：跑 `sync_check.yaml` 期间拔掉 USB-TTL，等 3～5 秒插回。

**真机日志**（`05_串口重连_拔插恢复.png`）：

```text
[ERROR] 串口写入失败：write failed: [Errno 5] Input/output error（USB 可能掉了）
[WARN]  串口打不开：/dev/ttyUSB0 -> [Errno 2] ... 之后会每 1.0s 自动重试，不再重复刷屏
[INFO]  串口已打开：/dev/ttyUSB0 @ 115200 8-N-1      ← 第一次插回，自动恢复
[ERROR] 串口读取失败：device reports readiness to read but returned no data ...
[WARN]  串口打不开：/dev/ttyUSB0 -> [Errno 2] ...
[INFO]  串口已打开：/dev/ttyUSB0 @ 115200 8-N-1      ← 第二次插回，再次自动恢复
```

拔线全程 `ros2 node list` 仍能看到 `/servo_driver`、进程 pid 未变（**节点没有崩**），插回后发 goal 返回 `success: true`。

> 附带现象：拔线期间动作推进停滞，看门狗被真实触发并全部卸力，之后必须 `torque_enable true` 恢复。说明看门狗在真实故障下也生效。

---

## 六、udev 固定设备名（+2）实测证据

**规则文件** `/etc/udev/rules.d/99-cq-servo.rules`：

```text
# CQuick 总线舵机 USB-TTL（CH340）固定设备名
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="cq_servo", MODE="0660", GROUP="dialout"
```

**符号链接生效**（`06_udev_固定设备名.png`）：

```text
$ ls -l /dev/cq_servo
lrwxrwxrwx 1 root root 7 11月 19 23:11 /dev/cq_servo -> ttyUSB0
```

**默认端口生效**（`07_udev_默认端口生效.png`）：把 `servo_params.yaml` 的 `port` 与 `servo.launch.py` 的 launch 默认值都改成 `/dev/cq_servo` 后，**不带 `port:=`** 直接启动：

```text
13:    port: /dev/cq_servo
47:            'port', default_value='/dev/cq_servo',
[INFO] [servo_driver]: 串口已打开：/dev/cq_servo @ 115200 8-N-1
[INFO] [servo_driver]: startup_reset 完成：所有舵机 -> 1500
[INFO] [servo_driver]: MultiThreadedExecutor 线程数=12（servo_ids=10 + 2）
```

**踩坑记录（重要）**：一开始只改了 `servo_params.yaml`，启动日志仍显示打开 `/dev/ttyUSB0`。原因是 `servo.launch.py` 里的 `DeclareLaunchArgument('port', default_value='/dev/ttyUSB0')` 会把端口作为参数覆盖传给节点——**launch 参数优先级高于 yaml**。两处都改后才生效。

这反过来也证明“launch 参数覆盖 yaml”机制有效：`ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB0` 仍能覆盖回 `ttyUSB0`。

**兼容性说明**：没有这条 udev 规则的机器上默认值 `/dev/cq_servo` 不存在，需要显式覆盖：

```bash
ros2 launch cquick_servo servo.launch.py port:=/dev/ttyUSB0
```

---

## 七、未实现的一项

- **动作组抢占（+4）**：`sequence_player` 是“发完整轮”的模型，没有实现“新 goal 打断当前动作、从当前位置平滑接上”。

---

## 八、参数回默认

所有演示用的临时参数已恢复：

```text
prad_tolerance      30
prad_retries         1
watchdog_timeout_s  1.0
torque_enable       true
```

端口保持 udev 固定名 `/dev/cq_servo`。