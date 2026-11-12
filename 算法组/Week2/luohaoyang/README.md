\## 任务 1：颜色目标检测 #针对目标为足球等圆形物体，因足球黑白色块较多，不使用圆的处理对我来说有些困难，调整参数也总是有问题

目标颜色：白色

HSV 阈值:{\[0,0,120] ,\[180,45,225]}

最小面积:500

中心点计算方法:圆心/轮廓矩

多个目标：取面积最大的

测试图片结果：06×（背景颜色接近） 07√  08√

&#x09;



\## 任务 2：可视化与参数对比

参数 HSV{ \[0, 0, 120],\[180, 45, 255]}    HSV{\[0, 0, 120],\[180, 70, 255]}

结果变化:07 的 mask 白色面积增加，

&#x09;浅色墙面/地板与足球连成一块。由于该区域贴到图像边界，被边界过滤，

&#x09;最终没有候选轮廓，07 从检出变为漏检，故S并非越大越好

最小面积area500   area12000

变化：候选轮廓被全部过滤，07 从检出变为漏检，故最小面积必须合理





\# 第 2 周任务三：ROS2 目标位置发布与订阅



\## 1. 环境



\- Ubuntu 22.04.5 LTS（WSL2）

\- ROS2 Humble

\- OpenCV 4.5.4

\- Python 3.10



\## 2. 代码结构



```text

ros2\_ws/src/

├── cquick\_interfaces/

│   ├── msg/TargetPosition.msg

│   ├── package.xml

│   └── CMakeLists.txt

└── football\_detector/

&#x20;   ├── football\_detector/

&#x20;   │   ├── detection.py

&#x20;   │   ├── target\_publisher.py

&#x20;   │   ├── target\_subscriber.py

&#x20;   │   └── images/

&#x20;   │       ├── 06.png

&#x20;   │       ├── 07.png

&#x20;   │       └── 08.png

&#x20;   ├── package.xml

&#x20;   └── setup.py

```



\## 3. 消息定义



`cquick\_interfaces/msg/TargetPosition.msg`：



```text

bool detected

float32 u

float32 v

float32 area

```



查看命令：



```bash

source /opt/ros/humble/setup.bash

source \~/ros2\_ws/install/setup.bash

ros2 interface show cquick\_interfaces/msg/TargetPosition

```



输出：



```text

bool detected

float32 u

float32 v

float32 area

```



\## 4. 编译



```bash

cd \~/ros2\_ws

colcon build --packages-select cquick\_interfaces football\_detector

source install/setup.bash

```



\## 5. 运行



终端 1，发布节点：



```bash

source /opt/ros/humble/setup.bash

source \~/ros2\_ws/install/setup.bash

ros2 run football\_detector target\_publisher

```



终端 2，订阅节点：



```bash

source /opt/ros/humble/setup.bash

source \~/ros2\_ws/install/setup.bash

ros2 run football\_detector target\_subscriber

```



\## 6. 话题检查



```bash

source /opt/ros/humble/setup.bash

source \~/ros2\_ws/install/setup.bash

ros2 topic info /target\_position

```



实际输出：



```text

Type: cquick\_interfaces/msg/TargetPosition

Publisher count: 1

Subscription count: 1

```



\## 7. 实际运行结果



发布端连续发布 10 条：



```text

publish 1: file=06.png, detected=False, u=0.0, v=0.0, area=0.0

publish 2: file=07.png, detected=True, u=345.0, v=252.0, area=10104.0

publish 3: file=08.png, detected=True, u=418.0, v=257.0, area=80615.5

publish 4: file=06.png, detected=False, u=0.0, v=0.0, area=0.0

publish 5: file=07.png, detected=True, u=345.0, v=252.0, area=10104.0

publish 6: file=08.png, detected=True, u=418.0, v=257.0, area=80615.5

publish 7: file=06.png, detected=False, u=0.0, v=0.0, area=0.0

publish 8: file=07.png, detected=True, u=345.0, v=252.0, area=10104.0

publish 9: file=08.png, detected=True, u=418.0, v=257.0, area=80615.5

publish 10: file=06.png, detected=False, u=0.0, v=0.0, area=0.0

```



订阅端收到 10 条：



```text

recv 1: detected=False, u=0.0, v=0.0, area=0.0

recv 2: detected=True, u=345.0, v=252.0, area=10104.0

recv 3: detected=True, u=418.0, v=257.0, area=80615.5

recv 4: detected=False, u=0.0, v=0.0, area=0.0

recv 5: detected=True, u=345.0, v=252.0, area=10104.0

recv 6: detected=True, u=418.0, v=257.0, area=80615.5

recv 7: detected=False, u=0.0, v=0.0, area=0.0

recv 8: detected=True, u=345.0, v=252.0, area=10104.0

recv 9: detected=True, u=418.0, v=257.0, area=80615.5

recv 10: detected=False, u=0.0, v=0.0, area=0.0

```

\## 10. 关键代码说明



\- `cquick\_interfaces` 使用 `ament\_cmake` 生成自定义消息；

\- `football\_detector` 使用 `ament\_python` 编写发布、订阅节点；

\- 发布节点每秒循环处理 06/07/08 三张图；

\- 检测到目标时发送 `u、v、area`；

\- 未检测到时发送 `detected=False` 和三个 0；

\- 订阅节点只负责接收并打印，不修改数据。













问题：任务二修改参数时S放宽反而找不到目标，百思不得其解

解决：把过程中的每一步处理的图片都show出来检查，发现形态学处理后小球与地面背景连接导致结构变化





问题：

对ubuntu理解较为浅显，多数依靠AI指导，对ubuntu完整的构建环境等操作一知半解



