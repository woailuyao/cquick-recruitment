\# Week2 考核 README — duanyuanee



\## 一、第二周任务与坑的总结



\### 任务1：自定义 BallPos 消息

建了 ball\_pkg 包，写了 BallPos.msg（id, x, y），用 C++ 写了发布者和订阅者。

\- \*\*坑1\*\*：colcon build 报 `local\_setup.bash not found`。原因是 CMakeLists.txt 里漏了 `ament\_package()`，还有 install 目标写错了。改完就好了。

\- \*\*坑2\*\*：package.xml 里 `rosidl\_default\_runtime` 写了两次，一次 depend 一次 exec\_depend，直接报 redundant。删掉 depend 那行才过。

\- \*\*坑3\*\*：turtlesim 明明装了，但编译说找不到 `pose.hpp`。最后发现是 CMake 里 `ament\_target\_dependencies` 写了 `${PROJECT\_NAME}`（自己依赖自己），当时消息还没编完，删掉就过了。



\### 任务2：乌龟走三点

写了 turtle\_navigator.cpp，订阅 ball\_pos 和 /turtle1/pose，用 Twist 让乌龟按 1→2→3 走。

\- \*\*坑\*\*：一开始角度差没处理好，乌龟走到点附近会画圈。后来改成“角度差大于0.1就原地转，对准了才走，越近速度越小”，就不冲过头了。

\- 走完三个点后，因为之前忘了清屏又跑了一次，轨迹重叠了。后来重新 `ros2 service call /clear` 清屏，再跑一次，拿到干净的轨迹图。



\### 任务3：串口与舵机笔记

这周没接触到实物，只交了 `03\_串口与总线舵机笔记.md`。四参数、回环计划、波特率原理、指令拆解都写了，还提了一个自己的疑问（PULR 掉电后位置为啥会丢）。



\## 二、三种通信方式对比(个人见解)



| 方式 | 咋工作的 | 特点 | 这周哪儿用到了 |

|------|---------|------|---------------|

| 话题 Topic | 一个发、多个听，像广播 | 一直发，不管有没有人在听 | 任务1发球坐标、任务2听坐标，都是话题 |

| 服务 Service | 你问一次、它答一次 | 同步，必须有回应，只调一次 | 任务2开头清屏：`ros2 service call /clear std\_srvs/srv/Empty`，调一次就结束 |

| 动作 Action | 给个目标，过程中报进度，还能取消 | 适合长时间任务 | 这周没用上，下周可能用 |



\## 三、差速公式（乌龟是两轮差速）



乌龟左右轮速度不一样就能转弯，三个式子：

\- `v = (vL + vR) / 2` → 中间速度，也就是 linear.x

\- `ω = (vR - vL) / L` → 角速度，也就是 angular.z（L 是两轮之间的距离）

\- `r = v / ω` → 转弯半径（ω=0 就是走直线，r 无穷大）



我代码里没直接算轮速，是给的 linear.x 和 angular.z，乌龟底层自己会把这两个值变成左右轮转速。



\## 四、给下周自己的一句话



别一报错就慌着问 AI，先自己看报错最后几行，大部分坑都是少了个依赖或者名字写错。

