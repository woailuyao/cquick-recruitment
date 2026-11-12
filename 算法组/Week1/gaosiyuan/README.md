# 第 1 周提交说明 · <高思源>

> 复制本文件到你自己的文件夹，改名为 `README.md`，逐项填写。**方括号里的内容要替换掉，不要留着。**

## 一、基本信息

- 姓名拼音：<gaosiyuan>
- 姓名：<高思源>
- 完成时间：<2026-10-23>
- 系统 / Python 版本：<Python 3.10>
- OpenCV 版本：<`python -c "import cv2; print(cv2.__version__)"` 的输出>

## 二、环境与运行方式

**依赖安装**：

```bash
conda create -n cv_practice python=3.10 -y
conda activate cv_practice
pip install opencv-python numpy matplotlib
```

**运行命令**（在 `算法组/Week1/<gaosiyuan>/` 目录下执行）：

```bash
python src/task1_basics.py
python src/task2_processing.py
python src/task3_color_detect.py
```

**输入输出路径说明**：<输入读 data/images/，输出写 outputs/taskN/，>

## 三、任务完成情况

### 任务一 · 图像基础操作

- 用了哪几张图：<cat1.jpg>
- 各图尺寸与通道数：<cat1.jpg 500×750×3>
- 输出：`outputs/task1/` 下 <cat1_black.jpg、cat1_crop.jpg、cat1_resize.jpg >
- 遇到的问题：<结果一闪而过，而添加cv2.waitKey(0)>

### 任务二 · 基础图像处理对比

| 处理 | 关键参数 | 结果文件 | 现象观察 |
|---|---|---|---|
| 灰度 | — | car_black.jpg | 色彩消失 |
| 高斯滤波 | ksize=3 | car_gaussian3.jpg | <采用高斯滤波 (3,3)。对比原灰度图，滤波后的图像去除了高频噪点，表面变平滑，但目标边缘出现了一定程度的模糊。> |
| 阈值分割 | thresh=80/130 | car_thresh_low.jpg , car_thresh_high.jpg | <对比了阈值 80 和 130。阈值设为 80 时，大量暗色背景被误判为白色目标；调高至 130 时，能较好地分离明亮目标，但目标暗部丢失。> |
| Canny | low=45 / high=150 | car_edges_low.jpg | <采用 (45, 150) 。低阈值提取的边缘极多，包含大量噪点纹理。> |
| 形态学（开/闭） | kernel=3 , 迭代=1 | car_open.jpg , car_close.jpg | <开运算：针对阈值图背景中的孤立的白色小噪点，开运算成功将其消除，保持了主目标大小不变。闭运算：针对 Canny 边缘图中断裂的线条，闭运算成功将断开的轮廓线连接了起来，有利于后续闭合区域的提取。> |

**至少换的 2 组参数对比**：<对比了阈值 80 和 130。阈值设为 80 时，大量暗色背景被误判为白色目标；调高至 130 时，能较好地分离明亮目标，但目标暗部丢失。>

### 任务三 · 颜色目标提取与轮廓定位

- 目标颜色：<橙>
- HSV 范围：`lower = (<h>, <s>, <v>)`，`upper = (<h>, <s>, <v>)`
- 面积阈值：<area> —— **为什么这么设**：<完整的橘子果肉轮廓面积通常在 3000 以上，而噪点面积通常小于 200，因此将 contourArea 过滤阈值设定为 1000，既能滤除噪点，又不会漏掉小果肉。>
- 形态学处理：<开运算 ksize=3 / 闭运算 ksize=3>

| 测试图 | 背景/光照 | 中心坐标 (x, y) | 轮廓面积 | 稳定 / 误检 / 漏检 | 原因分析 |
|---|---|---|---|---|---|
| 01 | | ( , ) | | | |
| 02 | | ( , ) | | | |
| 03 | | ( , ) | | | |

- 输出文件：`outputs/task3/mask_easy.png`、`outputs/task3/mask_dark.png`、`outputs/task3/mask_noisy.png`、`outputs/task3/result_easy.png`、`outputs/task3/result_dark.png`、`outputs/task3/result_noisy.png`

### 任务四（选做）· 实时摄像头

<未做>

## 四、一次失败案例与改进思路

> 这一节是评分点，写具体的过程，别写「没什么问题」。

- **现象**：<比如「打印的轮廓定位图不符合预期」>
- **我是怎么查的**：<问deekseek>
- **原因**：<绘制轮廓用的二值掩膜图>
- **改进**：<在绘制前先复制原图，再在复制图上画轮廓>
- **改进后效果**：<效果近乎完美，但reslut_noisy里部分目标被遮挡，导致识别不全>

## 五、给下周自己的提醒

<别再对着单张图调参数了，先看 3 张图的 HSV 分布再定范围>
