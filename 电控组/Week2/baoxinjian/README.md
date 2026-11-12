# 第 2 周考核 · 鲍鑫剑（baoxinjian）

## 一、任务 1：自定义消息 + 发布 3 个球坐标

### 做了什么

在 `~/ros2_ws/src/ball_pos_pkg` 下用 `ament_cmake` 建包，`msg/BallPos.msg` 定义三个字段（`int32 id` / `float32 x` / `float32 y`），`package.xml` 补 `rosidl_default_generators`（build_depend）、`rosidl_default_runtime`（exec_depend）和 `member_of_group`（rosidl_interface_packages），`CMakeLists.txt` 里调 `rosidl_generate_interfaces` 生成消息，并用 `install(PROGRAMS ...)` 把三个脚本装进 `lib/ball_pos_pkg`。

发布者每 0.5 s 触发一次（2 Hz），一个周期内把 id 1/2/3 三条依次发出，坐标用考核给定的 (2.0, 3.0)、(5.5, 8.0)、(9.0, 4.0)；订阅者订阅同名话题 `ball_pos`，回调里打印 id 和坐标。频率用参数 `period_sec` 控制，可以用 `--ros-args -p period_sec:=0.2` 改成 5 Hz，不用改代码重新编译。

### 遇到的问题

1. **`package.xml` 少 `member_of_group` 导致编译报一堆 rosidl 错误**。一开始只补了 generators 和 runtime，编译一直挂在消息生成阶段，折腾了很久才定位到是少了"本包属于接口包"这条声明。
2. **网页复制代码带进零宽字符**，`if __name__ == '__main__':` 那一行反复报 SyntaxError，肉眼看不出问题。后来改用 `python3 -m py_compile` 检测，并把入口改成顶格的 `main()` 调用才彻底绕开。
3. **改完代码不重新 `colcon build` 就重启节点**，跑的还是旧行为，一度怀疑自己逻辑写错。
4. **参数回调里写 `p.value.double_value`**，`ros2 param set` 时节点直接崩（AttributeError）。rclpy 传进来的已经是 float，改成 `p.value` 就好了。
5. **`ros2 topic info` 报 Unknown topic**：DDS 节点发现需要几秒，等 3~5 秒重跑就正常，不是代码问题。

## 二、任务 2：订阅 3 个球坐标，依次走过去

### 做了什么

写 `nav.py` 订阅 `ball_pos`，用字典按 id 存坐标，收齐 3 个才动。采用闭环路线（路线 A）：同时订阅 `/turtle1/pose` 拿当前位姿，每个周期算到目标的距离和方位角差，把角度差归一化到 [-π, π]；角度差大于 0.15 rad 就原地转（`linear.x = 0`，`angular.z = 2.0 * aerr`），对准之后直线走，速度取 `min(1.0, 0.4 * dist)`，越近越慢；距离小于 0.15 判定到达，打印实际坐标和误差，切下一个点。每 20 个周期打印一行调试信息（目标、当前位姿、距离、角度差）。

### 结果

三点全部按 1 → 2 → 3 顺序到达，误差分别是 0.14、0.14、0.15，都远小于 0.5 的要求，全程无人工按键。

### 遇到的问题

1. **`tid, tx, ty = self.targets[self.current]` 解包错误**——字典里存的是 `(id, (x, y))`，应该是 `tid, (tx, ty) = ...`，否则直接 ValueError。
2. **乌龟撞墙、原地乱转**：以为是自己代码的锅，其实是之前调试时终端只折叠不关闭，留下了两个 `nav.py` 进程，两个节点同时往 `/turtle1/cmd_vel` 发速度互相抢。用 `pkill -f "ball_pos_pkg/"` 清干净后单实例重跑就正常了。这件事的教训比代码本身大：**折叠终端不等于结束进程**。
3. **角度误差不归一化时，乌龟会绕远路**：`atan2` 结果和自己算的朝向差值可能超过 π，用 while 循环把误差压回 [-π, π] 才对。
4. **`ros2 node list` 偶尔返回空**：节点明明在跑，列表却什么都没有。是 daemon 缓存卡住了，`ros2 daemon stop` 之后重新 `ros2 node list` 就能列出来，不是节点挂了。

## 三、任务 3：串口与总线舵机笔记

见 `03_串口与总线舵机笔记.md`。本周未做回环测试（没有 CH340 和舵机），计划下周拿到舵机后补做。

## 四、三种通信方式对比

| 通信方式 | 模型 | 特点 | 足球机器人典型场景 |
|---|---|---|---|
| 话题 Topic | 发布/订阅，多对多 | 单向、持续流、不管对方在不在 | 视觉持续广播球位置；战术持续广播速度指令 |
| 服务 Service | 请求/响应，一对一 | 同步、有返回值、调一次答一次 | "现在比分多少""切到防守模式" |
| 动作 Action | 目标/反馈/结果 | 长任务、报进度、可取消 | "走到点 (1,1)"，过程报进度、可中途取消 |

任务 2 里实际用到的服务调用（开始导航前清屏，让轨迹看得清）：

```bash
ros2 service call /clear std_srvs/srv/Empty
```

它发一次请求，服务端清空画布后回一个响应就结束，不会再持续发数据——这就是服务和话题最直观的差别：话题是一直在流，服务是问一句答一句。

## 五、差速底盘三个公式

设左右轮速 `vL`、`vR`，轮距 `L`：

- 线速度：`v = (vL + vR) / 2`
- 角速度：`ω = (vR − vL) / L`
- 转弯半径：`r = v / ω`

`ω = 0` 时走直线，`v = 0` 且 `ω ≠ 0` 时原地转。所以控制乌龟本质就是：想清楚这一步要"走多快、转多急"，再翻译成 `linear.x` 和 `angular.z` 两个数。

## 六、给下周自己的提醒

先确认终端是真关了还是只折叠了，再怀疑代码——这次一半的时间都花在跟残留进程和自己较劲上。
