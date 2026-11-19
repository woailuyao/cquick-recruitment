# 加分项（+10 / 上限）

本周六项加分全部实现，代码在主仓库里，本文件夹放**规则文件、验证脚本、证据截图**。

## 六项清单

| 项 | 分值 | 代码位置 | 证据截图（放本文件夹） |
|---|---|---|---|
| udev 固定设备名 | +2 | `99-cquick-servo.rules` + `servo_params.yaml` 的 `port: /dev/cq_servo` | `04_udev固定设备名.png` |
| PRAD 闭环校验重发 | +3 | `servo_driver.execute_cb` 末段 + `prad_retry` 参数 | `04_PRAD重发.png` |
| 安全看门狗 | +3 | `servo_driver.watchdog_cb` / `trip_watchdog` / 三个 Trigger 服务 | `04_看门狗卸力.png` |
| 动作组抢占 | +4 | `goal_cb` 抢占分支 + `preempted` 结果 + `smooth_duration_ms` | `04_动作组抢占.png` |
| 串口掉线自动重连 | +3 | `link_check_cb` / `send_raw` 异常捕获 | `04_串口重连.png` |
| pytest 单测 | +2 | `src/cquick_servo/test/test_servo_protocol.py`（23 条） | `04_pytest.png` |

> 分值合计 +17，按细则加分上限 **+10** 计算，全部拿到即封顶。

## 本文件夹只放这四类

| 放进来 | 不放进来（以及它们该在哪） |
|---|---|
| 规则文件 `99-cquick-servo.rules` | 单测代码 → `src/cquick_servo/test/`（挪走 `colcon test` 就扫不到） |
| 验证脚本 `verify_bonus.sh` | `build/` `install/` `log/` → 不提交 |
| 说明 `README.md`（本文件） | `__pycache__/`、`.pytest_cache/` → 跑测试的缓存，已在 `.gitignore` 里，直接删 |
| 六张证据截图 `04_*.png` | 舵机驱动源码 → `src/cquick_servo/cquick_servo/`（混进来会破坏包结构） |

截图命名固定：`04_udev固定设备名.png`、`04_PRAD重发.png`、`04_看门狗卸力.png`、
`04_动作组抢占.png`、`04_串口重连.png`、`04_pytest.png`，与本文件夹平级存放（文件名保留 `04_` 前缀，方便和 01/02/03 组对齐）。

## 逐项验证（一条一屏，便于单拍截图）

```bash
bash 加分项/verify_bonus.sh        # 6 项顺序跑一遍
bash 加分项/verify_bonus.sh 3      # 只跑第 3 项，拍完再跑下一项
```

`1`=udev　`2`=pytest　`3`=PRAD 重发　`4`=看门狗　`5`=抢占　`6`=串口重连

每项开头都打印 `姓名: wangxuanhe 时间: YYYY-MM-DD HH:MM:SS`，所以**每一项单独截一张图**就自带时间戳，不用一次拍全部。

| 跑哪项 | 截图存成 |
|---|---|
| `verify_bonus.sh 1` | `04_udev固定设备名.png` |
| `verify_bonus.sh 2` | `04_pytest.png`（已有，可复用） |
| `verify_bonus.sh 3` | `04_PRAD重发.png` |
| `verify_bonus.sh 4` | `04_看门狗卸力.png` |
| `verify_bonus.sh 5` | `04_动作组抢占.png` |
| `verify_bonus.sh 6` | `04_串口重连.png` |

截图建议开两个终端：**左窗驱动日志**（要有 `TX:`、`未到位→重发`、`看门狗触发`、`串口掉线/恢复`），**右窗跑脚本**。第 3 项如果手按不住舵臂，就用脚本里的 B 方案（100 ms 跨大距离），物理上必然来不及，同样会触发重发。

## udev 规则怎么装

```bash
sudo cp 加分项/99-cquick-servo.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/cq_servo        # 应指向 ttyUSB0/ttyACM0，组为 dialout
# 拔掉再插回，名字必须不变
```

规则里同时写了 CH340（`1a86:7523`）、CP2102（`10c4:ea60`）、FT232（`0403:6001`），
按 `lsusb` 查到的实际芯片保留对应一行即可。

## 关键设计（被追问时怎么答）

- **看门狗**：节点每 0.2 s 发 `/servo/state`，看门狗同时检查"心跳是否还在跑"和"执行循环是否停摆"，任一超过 `watchdog_timeout_s`（1 s）就全体 `PULK` 并把 `servo_lock` 置位，此后 goal / 话题指令一律拒绝；**只有 `watchdog_reset` 服务能解锁**，节点不会自己恢复。
- **抢占**：新 goal 到来时先 `cancel` 旧 goal，并在 `_preempt_flags` 打标记，旧 goal 检测到取消时**不 PULK**（否则舵机会先掉力再被抬起，就是"先卸力再切"）；新 goal 用 `smooth_duration_ms` 按 `speed_pwm_per_s` 限速，从读回的当前位置接上，所以不跳变、不回中位。
- **PRAD 重发**：只在动作走完后读一次，差 >30 才重发一次（`prad_retry=1`），避免反复撞结构。
- **重连**：2 s 巡检 + 写异常捕获，掉线只 `WARN`；重连后**不自动重发旧动作、不自动回中位**，防止拔插瞬间乱动。

## 证据收集进度

| 图 | 状态 |
|---|---|
| `04_pytest.png` | ✅ 已收：`colcon test` 绿 + 姓名 / 日期同屏 |
| `04_udev固定设备名.png` | ✅ 已收 |
| `04_PRAD重发.png` | ⬜ 待截 |
| `04_看门狗卸力.png` | ⬜ 待截 |
| `04_动作组抢占.png` | ⬜ 待截 |
| `04_串口重连.png` | ⬜ 待截 |

## 待补

- 剩余 5 张 `04_*.png` 需在硬件上电后按 `verify_bonus.sh` 的顺序逐一截取（要带姓名 + 时间戳）。
- 看门狗"自然触发"（心跳真断）现场难复现，用 `watchdog_trip` 手动触发演示保护链路，并在讲解时说明"这是手动注入故障验证保护逻辑"。
