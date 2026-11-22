# OpenCV图像处理实验作业
## 一、项目说明
本实验基于Python + OpenCV完成基础图像处理任务，使用uv搭建Python3.10虚拟环境。
共包含3个任务：
- task1：读取2张图像（蓝莓blueberry.jpg、橙子orange.jpg），输出图像宽高、通道信息，在图像上绘制像素白点并保存结果。
- task2：读取蓝莓图像，完成缩放、旋转、灰度二值化、腐蚀、膨胀形态学操作，保存每一步中间结果。
- task3：读取3张图像（蓝莓blueberry.jpg、足球soccer.jpg、树叶leaf.jpg），使用HSV颜色空间阈值分割，形态学去噪，轮廓检测并绘制目标包围框，保存掩码、去噪掩码、检测结果图。

## 二、环境依赖
- Python版本：3.10
- 依赖包：opencv-python
- 虚拟环境管理工具：uv

## 三、目录结构
M1_maxiaoyu/
├── data/
│   └── images/          # 原始输入图片
│       ├── blueberry.jpg
│       ├── orange.jpg
│       ├── leaf.jpg
│       └── soccer.jpg
├── src/
│   ├── task1_basics.py    # 任务 1 代码
│   ├── task2_processing.py# 任务 2 代码
│   └── task3_color_detect.py # 任务 3 代码
├── outputs/
│   ├── task1/            # task1 输出结果
│   ├── task2/            # task2 输出结果
│   └── task3/            # task3 输出结果
└── README.md

## 四、运行方式
1. 进入项目根目录 `M1_maxiaoyu`
2. 终端依次执行命令：
```bash
uv run python src/task1_basics.py
uv run python src/task2_processing.py
uv run python src/task3_color_detect.py
> 运行后图像窗口弹出，按任意键盘按键关闭图像窗口；处理结果自动保存至 outputs 对应子文件夹。

## 五、任务介绍
### Task1 图像基础读取与像素操作
循环读取两张图片，打印图像高度、宽度、通道数；复制原图，在固定坐标修改像素为白色圆点，保存处理后的图像。

### Task2 图像几何变换与形态学基础
对原图进行缩放、旋转操作；转灰度图并阈值二值化；使用5×5卷积核分别执行腐蚀、膨胀操作，保留全部中间结果用于对比效果。

### Task3 HSV颜色分割与轮廓检测
针对每张图片设置独立HSV颜色阈值，提取目标颜色掩码；使用闭运算消除掩码噪点；寻找图像外部轮廓，使用绿色矩形框标记检测到的目标物体。

## 六、实验小结
通过本次实验掌握了OpenCV图像读取、BGR/HSV颜色空间转换、图像几何变换、阈值分割、形态学运算、轮廓查找等基础API；理解了图像处理中间结果保存的意义，能够针对不同颜色物体调整HSV阈值完成目标提取。
