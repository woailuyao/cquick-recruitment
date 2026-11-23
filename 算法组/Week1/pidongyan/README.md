姓名 ： 皮董彦
完成日期：2026.10.22
注：第一周的时候不会写README，22号飞书上提交的内容不是很完善，本周阅读考核细则之后，新写的README，其余代码文件和截图上周
没有添加自己的姓名和时间戳，这次新交的week1的文件补上了姓名和时间戳。

## 零、运行方式

依赖：Python 3.10+，`pip install opencv-python numpy`

在本目录（`算法组/Week1/pidongyan/`）下运行：

```bash
python src/task0_env_check.py     # 任务一·环境自检：打印系统/Python/OpenCV/NumPy 版本
python src/task1_basics.py        # 任务二·读图与信息分析：shape/dtype/像素值/显示（弹窗按任意键退出）
python src/task2_processing.py    # 任务三·图像处理：缩放→灰度→HSV通道→双阈值对比，结果存 outputs/task2/
```

- 所有路径都是相对本目录的相对路径，clone 后无需修改即可运行；
- 测试图在 `data/images/`（来源 Unsplash，作者 allec gomes，免费授权）；
- 因为目录名含中文，`cv2.imread/imwrite` 在 Windows 下会失败，脚本统一用 `np.fromfile + cv2.imdecode` 读、`cv2.imencode + tofile` 写来兜底；
- task3（颜色目标提取与轮廓定位）本周未完成，计划下周补上。

一、任务一：环境自检
1.环境信息记录
| 项目 | 版本 / 路径 |
| 操作系统 | Windows 10（64 位，build 26100） |
| Python | 3.10.22（Miniconda 虚拟环境 `py310`） |
| 解释器路径 | `D:\soft\Miniconda3\envs\py310\python.exe` |
| OpenCV | 5.0.0（`import cv2`） |
| NumPy | 2.2.6 |
| 开发工具 | PyCharm（项目名：Py3.10 week1） |


2.Python、OpenCV、NumPy 分别承担什么作用

Python 用于编程的最基本工具，opencv 和 numpy 都要在 python 中导入才能使用。

OpenCV 用于读图片文件，并对图片进行处理，如颜色空间转换、读取灰度值等。

NumPy 是一个数据库，用于存放opencv的图片处理数据。

二、任务二：图片读取
1.shape各维度含义：高度，行数，从上到下的像素行；宽度，列数，从左到右的像素列；通道数，彩色图通常为3，分别对应蓝、绿、红
2.图像像素数据类型：整数
3.opencv 通道顺序：B  G  R
4.HSV : H表示色调，范围0~179；S表示饱和度，范围0~255，0 是白色，越大越鲜艳；V表示明度，范围0~255
5.阈值：阈值调高 则 变白的像素变多
6、BGR和RGB：R红色，B蓝色，G绿色，RGB 和 BGR指三个颜色通道在内存里的排列顺序。opencv多用BGR

三、任务三：图像处理
1.灰度处理：节约内存，略去其他颜色信息，使分析更容易
2.阈值分割：可以生成掩膜，利于识别物体，可以方便分析要识别物体的形状

四、过程中遇到的问题
1.因为图片存放地址路径存在中文（文件夹是中文），在AI建议下使用 “np.fromfile(path, np.uint8)” “cv2.imdecode”，可以避免
 读图失败
2.验证颜色转换时，不太能理解AI给出的代码含义
3.打开图片时，只显示其中一个角落，询问AI后，给出相应建议，但对代码不是很理解
4.不知道怎么在图片上标注，借助AI帮忙

五、没搞懂问题
调整图片读取尺寸的代码不是很理解，对阈值处理参数没搞明白

注：第一周的时候不会写README，22号飞书上提交的内容不是很完善，本周阅读考核细则之后，新写的README，其余代码文件和截图上周
没有添加自己的姓名和时间戳，这次新交的week1的文件补上了姓名和时间戳。