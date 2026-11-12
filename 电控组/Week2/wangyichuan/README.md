# 第 2 周提交说明 · 王一川

## 一、基本信息

- 姓名拼音：wangyichuan
- 姓名：王一川
- 完成时间：2026-11-01
- 运行环境：Ubuntu 22.04 + ROS2 Humble

## 二、三项任务完成情况

### 任务 1 · `BallPos` + 发布/订阅

- 包名：ball_comm
- 话题名：/ball_pos
- 发布频率：2 Hz（做成了 ROS 参数 `publish_rate`，可用 `ros2 param set` 在线改，不用改代码）
- 消息字段：int32 id / float32 x / float32 y
- 做了什么：每周期按 id 1→2→3 连发三条（坐标用细则给的值：2.0/3.0、5.5/8.0、9.0/4.0），发布端打一条、订阅端打一条。频率没有用固定周期的定时器，而是 100 Hz 小定时器 + 时间累加判断「该发了没」，所以 `ros2 param set /ball_publisher publish_rate 5.0` 立即生效，不用重启。
- 验证结果：`ros2 interface show ball_comm/msg/BallPos` 输出 id / x / y 三个字段；`ros2 topic info /ball_pos` 显示 Publisher count: 1 / Subscription count: 1；订阅端持续收到，改频率后打印间隔从约 0.5 s 变成约 0.2 s。

### 任务 2 · 三个球的目标点导航

- 实现路线：闭环（路线 A，订阅 `/turtle1/pose` 拿实时位姿，20 Hz 控制循环里每步重新算距离和方位角）
- 到达情况（一次运行的实际日志）：

| 球编号 | 目标坐标 | 实际到达坐标 | 误差 |
|---|---|---|---|
| 1 | (2.0, 3.0) | (2.08, 3.05) | 0.094 |
| 2 | (5.5, 8.0) | (5.44, 7.92) | 0.097 |
| 3 | (9.0, 4.0) | (8.94, 4.07) | 0.095 |

- 关键参数：`max_linear = 0.8` m/s、`max_angular = 2.0` rad/s、`goal_tolerance = 0.10`、`rotate_threshold = 0.30` rad
- 控制逻辑：先算 `atan2(目标−当前)` 与当前朝向的夹角；`|夹角| > 0.30` 时原地转（`linear.x = 0`，先对准）；对准后 `linear.x = min(0.8, 1.5 × 距离)`，越近越慢；距离 ≤ 0.10 就停车、打日志、切下一个点。
- 为什么这样调：一开始到点阈值给 0.15，误差一直贴着 0.14 停；收到 0.10 后误差降到 0.095 左右。线速度不能太大，超过 1.0 时控制周期只有 20 Hz，一步就走 5 cm，会在目标点附近来回冲过头。

### 任务 3 · 串口与总线舵机（学习笔记）

- 详细笔记见：`03_串口与总线舵机笔记.md`
- 回环测试：未做（计划下周拿到舵机后补做）

## 三、三种通信方式对比

| 通信方式 | 模型 | 特点 | 足球机器人场景 |
|---|---|---|---|
| 话题 Topic | 发布/订阅，多对多 | 单向、持续流、发的人不管有没有人听 | 视觉持续广播球位置（本任务 1 的 `/ball_pos`）；战术持续广播速度指令 |
| 服务 Service | 请求/响应，一对一 | 同步、有返回值、调一次答一次就结束 | 「现在比分多少」「切到防守模式」 |
| 动作 Action | 目标/反馈/结果 | 长任务、过程中报进度、可中途取消 | 「走到点 (1,1)」→ 一路报进度，可随时取消 |

**我实际用到的服务调用**：任务 2 开始前的 `ros2 service call /clear std_srvs/srv/Empty`。它必须有回应、只调一次就结束，这就是服务和话题最直观的差别；而 `/ball_pos` 是「一直发、不管有没有人听」，所以用话题。任务 2 同时订阅 `/ball_pos` 和 `/turtle1/pose`，也说明话题可以一对多、一个节点能同时听多个话题。

## 四、差速三公式

- `v = (vL + vR) / 2` → 底盘整体前进速度是左右轮速的平均值，两轮同速就是直行。
- `ω = (vR − vL) / L` → 左右轮速差除以轮距，差得越多转得越快；差为 0 就不转。为照应一般的规定，逆时针转角速度为正，顺时针转角速度为负。
- `r = v / ω` → 转弯半径 = 线速度 / 角速度；ω 越小半径越大（弯越大），想原地转就让 `v = 0`、`ω ≠ 0`。

## 五、踩过的坑 & 怎么解决的

1. **`colcon build` 输出 `Summary: 0 packages finished`，不报错但什么也没编** → 先 `ls ../电控组` 看目录到底在不在，发现仓库根上面还套了一层 `cquick-recruitment-main`，`../电控组` 指到了不存在的地方 → `cd` 到 `ros2_ws` 再执行 `colcon build --base-paths ../电控组/Week2/wangyichuan/src`，或者干脆用绝对路径。

2. **`ros2 run ball_comm ball_publisher` 报 `No executable found`** → 明明 `install(PROGRAMS ...)` 写了，就用 `ros2 pkg executables ball_comm` 查，发现装进去的名字是 `ball_publisher.py`（带后缀）→ 改成 `install(PROGRAMS scripts/ball_publisher.py DESTINATION lib/${PROJECT_NAME} RENAME ball_publisher)`，重新 `colcon build` 后 `ros2 run ball_comm ball_publisher` 才通。

3. **订阅节点一启动就 `ModuleNotFoundError: No module named 'ball_comm'`** → `ros2 pkg prefix ball_comm` 同样报找不到，说明这个终端没加载工作空间 → `source install/setup.bash` 后在**同一个终端**里重新启动节点。

4. **`ros2 topic info /ball_pos` 显示 `Subscription count: 2`** → `ros2 node list` 看到两个 `/ball_subscriber`，是之前测试没关的进程还活着 → 关掉多余终端（或 `pkill -f ball_subscriber`）重跑，恢复成 Publisher 1 / Subscription 1。截图前一定要确认这个数字。

5. **乌龟能到点，但误差一直卡在 0.14 下不去** → 对照日志发现每次的误差都刚好略小于 `goal_tolerance=0.15`，是「贴着阈值停」而不是走不到 → 把阈值收到 0.10，误差降到 0.095；同时把 `max_linear` 从 1.0 降到 0.8，因为控制周期 20 Hz 时 1.0 m/s 一步就走 5 cm，容易在目标点附近来回冲。

6. **Ctrl-C 退出节点时报 `failed to shutdown: rcl_shutdown already called`** → 查文档发现 rclpy 自己的信号处理已经关过一次 context，`finally` 里再 `rclpy.shutdown()` 就成了关第二次 → 改成先捕获 `ExternalShutdownException`，再 `if rclpy.ok(): rclpy.shutdown()`，退出就干净了。

7. **VScode Remote ssh 插件出问题，尝试远程连接虚拟机时总是卡在“正在使用scp将vs code服务器复制到主机”进程而无法建立连接** → 通过浏览器搜索引擎直接检索该语段 → 最终得到解决方案如下：
使用power shell手动连接到虚拟机：
```cmd
ssh <username>@<ip_address>
```
进入 .vscode-server/bin 目录：
```cmd
cd ~/.vscode-server/bin
```
删除所有文件夹：
```cmd
rm -rf *
```
重启 VSCode 并重新连接，等待 VSCode 自动重新下载所需文件，此后即可正常连接。


## 六、给下周自己的提醒

对ROS2相关内容进行操作之前终端一定要切换到对应的目录（工作空间），防止命令运行时出现“无法找到已有的包”等情况。

跑节点之前先 `source ~/ros2_ws/install/setup.bash`，防止“明明编译过了却 import 不到”。
