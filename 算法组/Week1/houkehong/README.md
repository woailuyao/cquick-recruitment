# 第 1 周考核 · houkehong（视觉算法）

- 姓名（拼音）：houkehong
- 完成日期：2026-11-02（如果第 1 周是更早做完的，把这里改成那天的日期）
- 环境：Windows 10 + Python 3.10（Conda 环境 `cv_practice`）+ OpenCV 5.0.0 + NumPy

## 1. 目录结构与运行方式

```text
W1_Hkh/
├── README.md
├── data/images/               # 输入图片
│   ├── test1.jpg test2.jpg test3.jpg      # 任务一、二用
│   └── blue1.jpg blue2.jpg blue3.jpg      # 任务三（颜色提取）用
├── src/
│   ├── task1_basics.py        # 任务一：读取校验、缩放、中心裁剪、灰度化
│   ├── task2_processing.py    # 任务二：滤波 / 阈值 / 边缘 / 形态学 对比
│   └── task3_color_detect.py  # 任务三：HSV 颜色提取 + 轮廓 + 中心点
└── outputs/                   # 所有结果图（由脚本自动创建）
    ├── task1/   task2/   task3/
```

脚本里的图片路径写成的是相对路径 `../data/images`，所以**要先进入 `src/` 再运行**：

```bash
cd src
python task1_basics.py
python task2_processing.py
python task3_color_detect.py
```

结果分别写入 `outputs/task1`、`outputs/task2`、`outputs/task3`，所有写操作都在副本上做，原图不会被覆盖。

## 2. 结果速查

| 任务 | 脚本 | 输出 | 对应内容 |
| --- | --- | --- | --- |
| 任务一 | `task1_basics.py` | `outputs/task1/<图名>_resized.jpg` | 宽高减半的缩放结果 |
|  |  | `outputs/task1/<图名>_cropped.jpg` | 中心裁剪结果 |
|  |  | `outputs/task1/<图名>_gray.jpg` | 灰度化结果 |
| 任务二 | `task2_processing.py` | `01_gray.jpg` ~ `09_morph_close.jpg` | 灰度、5×5/15×15 高斯、阈值 127/50、Canny 高低阈值、开/闭运算 |
| 任务三 | `task3_color_detect.py` | `outputs/task3/blueN_mask.png` | HSV 阈值 + 形态学后的二值掩膜 |
|  |  | `outputs/task3/blueN_result.png` | 原图上画的绿色外接矩形 + 红色中心点 |

## 3. 任务记录

### 任务一：基础操作
完成了图片的读取与成功校验，打印了宽/高/通道数；实现了缩放（宽高减半）、中心裁剪和灰度化；结果已保存且原图未被覆盖。

### 任务二：基础处理对比
* **滤波**：核越大（如 15×15）去噪越强，但画面越模糊；5×5 相对保留细节。
* **阈值**：阈值降低（如 50），更多暗部被判为前景（白色区域变大）；阈值 127 分割相对适中。
* **边缘**：低阈值边缘完整但噪点多；高阈值去噪但边缘易断裂。
* **形态学**：开运算去除了背景的白色小噪点；闭运算填补了目标内部的黑洞。

### 任务三：颜色提取与轮廓
* **参数依据**：
  * **HSV 范围**：以蓝色为例，`H 100~130`，并把 S、V 的下限抬到 100 / 50，用来过滤发白的天空和偏暗的阴影，只留鲜艳的蓝色。
  * **面积阈值**：设为 500，过滤掉背景里面积太小的同色噪点。
* **中心点**：用图像矩 `M["m10"]/M["m00"]`、`M["m01"]/M["m00"]` 求轮廓重心（比外接矩形中心更稳）；`M["m00"] == 0` 时退回外接矩形中心。
* **测试情况**：背景干净时定位准确；光照过强（物体泛白）或阴影过重时容易漏检。

## 4. 失败案例与改进

* **失败案例**：环境配置时直接在全局 CMD 用 pip 安装，导致 PyCharm 报 `ModuleNotFoundError`。
* **改进思路**：在 Conda 激活的独立环境（`cv_practice`）内重新安装，并在 PyCharm 中把解释器指定到该环境的 `python.exe`。
* **算法改进**：针对强光导致颜色发白而漏检，后续可以引入直方图均衡化（CLAHE）增强对比度，或结合 Canny 边缘检测辅助定位。

## 5. 说明

- 本目录不含 PyCharm 的 `.idea/` 工程配置（本机环境相关，与作业无关）。
- 输入图片与结果图均为本机实际运行产生。
