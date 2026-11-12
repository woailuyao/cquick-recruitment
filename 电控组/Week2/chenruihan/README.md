# 第 2 周提交说明 · 陈睿涵

## 一、基本信息

- 姓名拼音：chenruihan
- 姓名：陈睿涵
- 完成时间：2026-10-31
- 运行环境：Ubuntu 22.04 + ROS2 Humble（VMware 虚拟机）
- 工作空间 / 包名：`~/ros2_ws` / `ball_comm`

## 二、三项任务完成情况

### 任务 1 · `BallPos` + 发布/订阅

- 包名：`ball_comm`
- 话题名：`ball_pos`
- 发布频率：2 Hz（后改成 5 Hz）
- 消息字段：`int32 id` / `float32 x` / `float32 y`
- 做了什么：用 `ament_cmake` 建包，`rosidl_generate_interfaces()` 编译 `msg/BallPos.msg`；定时器每 0.5 s 依次发 id=1(2.0, 3.0)、2(5.5, 8.0)、3(9.0, 4.0) 三条并打印，订阅端收到后打印 id 和坐标。
- 验证结果：`colcon build --packages-select ball_comm` 输出 `Summary: 1 package finished [2.66s]`；`ros2 interface show` 结果为id / x / y；`ros2 topic info /ball_pos` 显示 Publisher 1 / Subscription 1；订阅端持续接受；改 5 Hz 后日志间隔也随之收缩。

### 任务 2 · 三个球的目标点导航

- 实现路线：闭环（订阅 `/turtle1/pose`）
- 到达情况：

| 球编号 | 目标坐标 | 实际到达坐标 | 误差 |
|---|---|---|---|
| 1 | (2.0, 3.0) | (2.09, 3.05) | 0.100 |
| 2 | (5.5, 8.0) | (5.44, 7.92) | 0.097 |
| 3 | (9.0, 4.0) | (8.94, 4.07) | 0.096 |

- 关键参数：线速度比例系数 `0.5`（速度上限 1.0），角速度比例系数 `1.0`，到点距离阈值 `0.1`，角度对准阈值 `0.2`。
- 为什么这样调：速度太大会超过预定节点，所以代将速度结合剩余距离 `min(0.5 * d, 1.0)`，使得距离越近速度越慢；如果角度差大于 0.2，就优先进行原地转圈（`linear.x=0`），对准行进路线，防止轨迹弧线过大。角度使用`atan2(sin,cos)` 归一化，否则会原地打转。


### 任务 3 · 串口与总线舵机（学习笔记）

- 详细笔记见：`03_串口与总线舵机笔记.md`
- 回环测试：未做（计划下周拿到舵机后补做）。预期 TX 和 RX 短接后发出即之后收到。

## 三、三种通信方式对比

| 通信方式 | 模型 | 特点 | 足球机器人场景 |
|---|---|---|---|
| 话题 Topic | 发布/订阅，多对多 | 单向、持续流、不需同时在线 | 视觉持续广播球位置；战术持续广播速度指令 |
| 服务 Service | 请求/响应，一对一 | 同步、有返回值、调一次答一次 | 「现在比分多少」「切到防守模式」 |
| 动作 Action | 目标/反馈/结果 | 长任务、报进度、可取消 | 「走到点 (1,1)」→ 过程报进度，可中途取消 |

我实际用到的服务调用：

```bash
ros2 service call /clear std_srvs/srv/Empty
```

因为它是一次性的请求（请求清屏），服务端执行完立刻返回响应，不需要像话题那样持续广播数据流，所以使用的是服务。

## 四、差速三公式

- `v = (vL + vR) / 2` → 底盘的线速度等于左右轮速度的平均值。
- `ω = (vR − vL) / L` → 底盘的角速度与左右轮速度差成正比，轮距 L 越小转得越快。
- `r = v / ω` → 转弯半径，角速度越小（直行时 ω=0），转弯半径越大。


## 五、踩过的坑 & 怎么解决的

1. `colcon build`时报错 `file INSTALL cannot find .../turtle_nav.py` → 因为 `CMakeLists.txt` 的 `install(PROGRAMS ...)` 里提前写了还没创建的脚本 → 先建好空的 `turtle_nav.py` 占位，重新编译通过。
2. `ros2 interface show` 报错 `Could not find the resource ...` → 排查发现 `install/` 下只有脚本，没有生成 rosidl 资源 → 发现 `rosidl_generate_interfaces` 必须放在 `ament_package()` 之前，修正 `CMakeLists.txt` 后，清理 `build/` 和 `install/`，重新编译最终解决。
3. 修改多次代码后，Sub与pub代码发生了混淆，`ros2 run ball_comm ball_pub.py` 时 `[ball_sub]: 订阅者启动`*→ 因为复制代码时把 `ball_pub.py` 的内容误写成了订阅者版本 → 重写发布者代码，用 `grep -n "super().__init__"` 确认节点名，重新编译解决。

## 六、给下周自己的提醒

跑节点前先确认 `source`，改了 Python 代码要重新 `colcon build`。下周拿到舵机后写角度前要 `PULR` 恢复扭力，下电前需要 `PULK` 卸力。

## 七、截图索引

| 文件 | 内容 | 图内时间戳 |
|---|---|---|
| `01_colcon_build.png` | `colcon build` 无报错，双终端 pub/sub 同时运行 | 2026-11-01 14:12:52 |
| `01_interface_show.png` | `ros2 interface show`，看到 id / x / y 三个字段 | 2026-10-31 14:27:46 |
| `01_发布订阅分屏.png` | 双终端分屏，`ball_pub` 连发 1/2/3，`ball_sub` 持续收到（2 Hz） | 2026-10-31 15:22:00 |
| `01_话题信息.png` | `ros2 topic info /ball_pos`，Publisher 1 / Subscription 1 | 2026-10-31 15:23:10 |
| `01_改频率_前_2Hz.png` | 改频率前，日志间隔约 0.5 s | 2026-10-31 15:22:06 |
| `01_改频率_后_5Hz.png` | 改频率后，`Set parameter successful`，间隔约 0.2 s | 2026-10-31 15:45:24 |
| `02_三个球轨迹.png` | 乌龟按 1→2→3 走完，轨迹清楚可见 | 2026-10-31 19:14:49 |
| `02_运行日志.png` | 到达球 1/2/3，实际坐标与误差 | 2026-10-31 19:18:31 |
| `02_误差汇总.png` | 三点误差汇总，与运行日志同内容、时间戳不同 | 2026-10-31 19:19:57 |
| `02_节点列表.png` | `ros2 node list`，看到 `/ball_pub`、`/turtle_nav`、`/turtlesim` | 2026-10-31 19:19:27 |
