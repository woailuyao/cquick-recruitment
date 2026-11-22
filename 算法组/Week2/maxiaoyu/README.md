```
# OpenCV图像处理实验作业

## 一、项目说明
本实验基于Python + OpenCV完成基础图像处理任务，在Ubuntu22.04虚拟机环境开发。
共包含3个图像处理任务 + ROS2消息通信任务：
- task1：读取3张图像（苹果apple.jpg、篮球basketball.jpg、足球football.jpg），输出图像宽高、通道信息，在图像上绘制目标中心点并保存结果。
- task2：读取3张图像，完成缩放、旋转、灰度转换、二值化、腐蚀、膨胀形态学操作，保存每一步中间结果。
- task3：读取3张图像，使用HSV颜色空间阈值分割，形态学去噪，轮廓检测并绘制目标包围框，保存原始掩码、去噪掩码、带检测框结果图。
- ROS2通信任务：将task3检测得到的目标坐标、面积信息通过ROS2话题发布，订阅节点接收并打印检测数据。

## 二、环境依赖
- 系统：Ubuntu 22.04
- Python版本：3.10
- 依赖包：opencv-python，rclpy（ROS2 Humble）

## 三、目录结构
maxiaoyu/
├── apple.jpg            # 输入图片：苹果
├── basketball.jpg       # 输入图片：篮球
├── football.jpg         # 输入图片：足球
├── task1.py             # 任务 1 代码
├── task2.py             # 任务 2 代码
├── task3.py             # 任务 3 代码
├── README.md            # 项目说明文档
├── output_task1/        # task1 输出结果
├── output_task2/        # task2 输出结果
├── output_task3/        # task3 输出结果
└── ros_screenshot/     # ROS2通信截图
    ├── msg_interface.png    # 自定义消息定义查看截图
    └── pub_sub_run.png      # pub/sub双终端通信运行截图

## 四、运行方式
### 图像处理任务
在项目根目录 maxiaoyu 下，终端依次执行：
```bash
python3 task1.py
python3 task2.py
python3 task3.py
```

运行完成后，对应的 output 文件夹内生成处理后的图像。

### ROS2 通信任务

1. 编译 ROS2 功能包

```
cd ~/ros2_ws
colcon build --packages-select target_detect
source install/setup.bash
```

2. 终端 1（发布节点，发送检测结果）

```
ros2 run target_detect detect_pub
```

3. 终端 2（订阅节点，接收并打印数据）

```
ros2 run target_detect detect_sub
```

> 
> ROS 相关截图放在 `ros_screenshot/` 文件夹，包含自定义消息定义查看截图、pub/sub 节点通信双终端截图；截图均带有姓名 maxiaoyu、系统时间信息。

## 五、任务简介

### task1

读取图片，获取图像尺寸信息；颜色分割，找到目标物体，绘制目标中心点，保存检测结果图。

### task2

对每张图像依次执行：图像缩放、图像旋转、转灰度图、灰度二值化、腐蚀、膨胀，保存每一步处理图片。

### task3

1. 图像转换到 HSV 颜色空间，通过颜色阈值生成掩码 mask
2. 使用开闭运算做形态学去噪，得到 clean_mask
3. 轮廓检测，筛选有效轮廓，绘制包围矩形框与目标中心点
4. 打印目标像素坐标，保存 mask、clean_mask、result 检测图

### ROS2 通信任务

将图像处理得到的目标位置、面积信息封装为 ROS 消息，发布节点持续向外发送检测结果；订阅节点接收话题消息并打印信息，完成节点间数据通信。

## 六、实验小结

本次实验练习 OpenCV 基础读写、几何变换、灰度与二值化、形态学操作、HSV 颜色分割和轮廓检测。通过颜色阈值分割分离目标物体，利用形态学操作去除噪点，轮廓提取获取目标位置并绘制包围框，掌握基础计算机视觉预处理流程。
同时学习了 ROS2 的包编译、节点编写、话题发布订阅机制，实现视觉检测结果在 ROS 节点间的传输，打通图像处理与机器人通信链路。
