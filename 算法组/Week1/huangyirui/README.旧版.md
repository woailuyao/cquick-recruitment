## 测试图
| 文件 | 场景 | 用途 |
|---|---|---|
| `sample_scene.png` | 灰条纹背景 + 蓝菱形 / 绿圆 / 橙三角 | 基准图 |
| `sample_blue.png` | 淡蓝灰干净背景 | 演示「背景干净时识别稳定」 |
| `sample_green.png` | 淡绿干净背景 | 换背景复测同一套阈值 |
| `sample_dark.png` | 同基准场景压到约 30% 亮度 | 暗光挑战：S/V 掉到阈值边缘 |

# W1 视觉算法 · 环境 / 运行方式 / 参数 / 问题解释

## 一、运行环境

- 操作系统：Windows（桌面会话，需有显示器才能弹 OpenCV 窗口）
- Python：3.10.0rc2，解释器 `C:\Users\31816\AppData\Local\Programs\Python\Python310\python.exe`
- OpenCV：4.14.0；NumPy：2.2.6（实测版本与 `requirements.txt` 完全一致）
- 编辑器：VSCode 1.142.0 + Python / debugpy 扩展（`launch.json` 用 debugpy 启动）
- 依赖安装：`python -m pip install -r requirements.txt`
- 纯 CPU 运行，不依赖 GPU、不依赖 matplotlib 绘图

编码与路径（已写进 `.vscode\settings.json`、`launch.json`、`tasks.json`，不用手动设）：

- `PYTHONUTF8=1`、`PYTHONIOENCODING=utf-8`：保证 `print` 中文不乱码
- `TMP` / `TEMP` / `TMPDIR` 指向 `D:\DeepSeek\_tools\tmp`，`MPLCONFIGDIR` 指向 `D:\DeepSeek\_tools\mpl-cache`
- 所有运行入口的 `cwd` 都锁在项目根（`${workspaceFolder}`），因此脚本里的 `data/images/...`、`outputs/...` 全是相对项目的路径

## 二、素材清单

| 类别 | 文件 | 说明 |
|---|---|---|
| 基准图 | `data/images/sample_scene.png` 等 4 张 | 灰条纹背景 + 蓝菱形 / 绿圆 / 橙三角，另含纯蓝、纯绿、压暗背景各一张 |
| 多条件测试图 | `data/images/test_*.png` 共 7 张 | 暗光、极暗、暖光、过曝、蓝背景、缩放一半、冷光 |
| 应用测试图 | `data/images/A1.png` ~ `A5.png` 共 5 张 | CS2 截图，用于暗色目标（人物）检测 |
| 视频 | `data/videos/test.mp4` | 约 3.3 MB，任务四逐帧读取 |

## 三、运行方式

1. 打开项目：VSCode →「文件 → 从文件打开工作区」→ 选 `W1视觉算法.code-workspace`；也可以直接打开文件夹，`.vscode` 里的配置同样生效。
2. 按 F5 运行，`launch.json` 里有四个配置：「运行当前文件」（跑你正打开的那个脚本）、「任务一：图像基础操作」、「任务二：图像处理」、「任务三：颜色识别」。
3. 任务面板：`Ctrl+Shift+B` 触发默认构建任务「Python: 运行当前文件」；`Ctrl+Shift+P` → `Tasks: Run Task` 可选「运行任务一/二/三」与「清理 outputs」。
4. 终端直接跑（必须先 cd 到项目根，脚本用的是相对路径）：

```bat
python -u src/task1_basics.py
python -u src/task2_processing.py
python -u src/task3_color_detect.py
python -u src/task4_timely.py

## 四、图像解释

   |滤波的变化主要是消除噪点，让图片变得更模糊了，可以看到gaus2明显比gaus模糊
   |阈值分割让颜色明显的色块变得非常明显，让模糊的或者较浅的色块直接变成白色，做了两级分割
   |canny边缘图的处理是将颜色和亮度变化剧烈的地方画一条线，然后变成一张只有线条的线稿
   |开处理是去除小白点，闭处理是去除小黑点，通过np设置的范围越大，处理得越明显，值较大的时候看起来就像散光一样

## 五、失败案例

   1.在图像识别过程中对于core的设置不适宜，导致识别出来完全对不上
      调整core参数，从11,11 ---> 7,7 ---> 3,3 ---> 5,5
      最后经过实验 5,5 作为core参数是最合适的
   
   2.在后续CS游戏实机验证时发现完全无法识别人物
      由于HSV对暗色的直接处理效果较差，导致人物和环境完全无法识别
      经过多次实验，我发现图片中某些横向的或者很小的阴影块也会影响图片识别，它会自动识别所有的阴影区块
      改进后加了限制，只有大于一定面积的区域才能被视作人物区块，然后要竖着的才能算作人物区块，即代码中写的cnt4>cnt3（长>宽）,否则将continue至下一个循环
      改进后仍存在问题，比如识别弹孔也符合这样的标准，在较近距离上弹孔看起来较大，且与匪门阴影共同形成一个竖着的长条形，导致识别失效
      狙击镜的问题也仍未解决，但有一定思路，可以仅裁取中间的圆形部分，将外围的黑色部分通过像素转换RGB至白色，就能规避这种影响
      目前仅能识别站立的全身角色