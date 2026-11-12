# 电控组 Week2 提交说明

姓名拼音：zhangchengcheng　　完成日期：2026-10-29　　环境：Ubuntu 22.04 + ROS 2 Humble（VirtualBox 虚拟机）

## 一、三项任务做了什么

### 任务 1：自定义消息 BallPos + 发布/订阅（40 分）

- 建了 `ament_cmake` 包 `ball_tracker`，`msg/BallPos.msg` 按细则定义三个字段：`int32 id` / `float32 x` / `float32 y`；
- 发布者 `ball_publisher.py` 以 2 Hz 定时发布，每个周期把 3 个球（(2.0,3.0) / (5.5,8.0) / (9.0,4.0)）依次发一遍并发一条打一条；
- 订阅者 `ball_subscriber.py` 收到后打印 `id / x / y`，话题名 `/ball_pos` 与发布端一字不差。

**遇到的坑**：

1. 编译报一堆 `rosidl` 相关错误 —— `package.xml` 里漏了 `<member_of_group>rosidl_interface_packages</member_of_group>`，补上就过了。这个坑 FAQ 里说过，但自己踩一遍才知道它必须和消息生成工具的依赖（`rosidl_default_generators`）同时存在；
2. `ros2 run` 报 No executable found —— CMakeLists.txt 里忘了 `install(PROGRAMS ...)`，脚本装进 `lib/<包名>` 才能被 `ros2 run` 找到；
3. 脚本没 `chmod +x` 时安装直接被跳过，编译"成功"但啥也没装上。

**加分项**：发布频率做成了 ROS 参数，不改代码就可以调：`ros2 run ball_tracker ball_publisher.py --ros-args -p publish_frequency:=5.0`（见 `01_改频率.png`，`ros2 topic hz` 实测 15 条/秒 = 5 Hz × 每周期 3 条；改前 2 Hz 时约 6 条/秒）。

### 任务 2：订阅 3 个球坐标，依次走过去（35 分）

- 节点 `nav_to_balls.py` 走**路线 A 闭环**：订阅 `ball_pos` 收齐 3 个球，再订阅 `/turtle1/pose` 拿当前位姿，10 Hz 控制循环实时算 `Twist`；
- 控制逻辑：方位角误差大 → `linear.x = 0` 原地转（先对准再走）；对准后边走边微调，线速度 `v = 0.9 × dist` 且限幅 `[0.18, 1.6]`，**越近越慢**防止冲过头；距离 ≤ 0.15 判定到达，打印实际坐标和误差，切下一个点；
- 收齐坐标后自动调用 `/clear` 服务清屏，轨迹从头画；
- 实测结果（见 `02_运行日志.png` / `02_误差汇总.png`）：

| 球 | 目标 | 实际 | 误差 |
| ---- | ---- | ---- | ---- |
| 1 | (2.0, 3.0) | (2.12, 3.08) | 0.146 |
| 2 | (5.5, 8.0) | (5.42, 7.89) | 0.134 |
| 3 | (9.0, 4.0) | (8.91, 4.11) | 0.147 |

顺序 1→2→3，全程无按键一次跑完，全部满足 ≤ 0.5。

**加分项**：闭环实现（订阅 `/turtle1/pose`）且日志打印了每个点的实际误差。

**踩的坑**：第一版"到达"判定阈值给太松，乌龟在目标附近小幅震荡；把线速度和距离变成相关+ 抬高最低速度下限之后收敛了。角度归一化也踩过坑：-179° 和 179° 差 2° 而不是 358°，要先把角度差归一化到 [-π, π]。

### 任务 3：串口与总线舵机笔记（25 分）

见 `03_串口与总线舵机笔记.md`。回环测试未做（本周无设备），笔记里写了下周补做的计划和预期现象。

## 二、三种通信方式对比

| 通信方式 | 模型 | 特点 | 足球机器人典型场景 |
| ---- | ---- | ---- | ---- |
| **话题 Topic** | 发布/订阅，多对多 | 单向、持续流、不管对方在不在 | 视觉持续广播球位置；战术持续广播速度指令 |
| **服务 Service** | 请求/响应，一对一 | 同步、有返回值、调一次答一次 | 「现在比分多少」「切到防守模式」 |
| **动作 Action** | 目标/反馈/结果 | 长任务、报进度、可取消 | 「走到点 (1,1)」→ 过程报进度，可中途取消 |

**任务 2 里实际用到的服务调用**：导航节点收齐 3 个球坐标后，调用 `std_srvs/srv/Empty` 类型的 `/clear` 服务清掉 turtlesim 画布旧轨迹（`nav_to_balls.py` 的 `clear_screen()`，日志里能看到"已调用服务 /clear"）。它必须是请求-响应模式——清屏动作只做一次、要确认成功，这正是服务和话题的差别：话题是"发了就发，有没有人听不管"，服务是"调一次必须答一次"。

## 三、差速底盘三公式

两轮差速底盘，左右轮速 `vL`、`vR`，轮距 `L`：

- **线速度** `v = (vL + vR) / 2` —— 两轮同速时就是车身前进速度
- **角速度** `ω = (vR − vL) / L` —— 两轮速差决定转弯快慢，`vR > vL` 左转
- **转弯半径** `r = v / ω` —— ω 越小弯越大，ω = 0 走直线

代码里的对应关系：直行 `linear.x = v, angular.z = 0`；原地转 `linear.x = 0, angular.z ≠ 0`（先对准再走就是这个状态）；边走边转两者都非 0（接近目标时的微调）。

## 四、给下周自己的一句提醒

**先 `ros2 daemon stop` 再折腾环境** —— `topic hz`/`topic list` 报消息类型无效，十有八九是 daemon 缓存了旧环境的类型信息，重启 daemon 比重启虚拟机快一百倍。（另外：舵机动之前先 PULR 恢复扭力，断电前记得 PULK 卸力。）

## 五、文件清单

```
zhangchengcheng/
├── 01_interface_show.png      消息定义 ros2 interface show
├── 01_发布订阅分屏.png        colcon build 无报错 + 发布/订阅同时运行
├── 01_话题信息.png            ros2 topic info（Publisher 1 / Subscription 1）
├── 01_改频率.png              2Hz→5Hz，ros2 topic hz 实测 15 条/秒
├── 02_三个球轨迹.png          乌龟 1→2→3 轨迹
├── 02_运行日志.png            到达球 1/2/3 的完整日志
├── 02_误差汇总.png            三点最终误差汇总
├── 02_节点列表.png            ros2 node list
├── 03_串口与总线舵机笔记.md   任务 3 笔记
├── README.md                  本文件
└── src/ball_tracker/          任务 1/2 源码（package.xml / CMakeLists.txt / msg / scripts）
```

## 六、评审复跑方法

```bash
cd ~/ros2_ws
colcon build --packages-select ball_tracker
source install/setup.bash
ros2 run turtlesim turtlesim_node          # 终端 1
ros2 run ball_tracker ball_publisher.py    # 终端 2（默认 2 Hz）
ros2 run ball_tracker ball_subscriber.py   # 终端 3
ros2 run ball_tracker nav_to_balls.py      # 终端 4，乌龟自动 1→2→3
```
