# 第 2 周考核 · 蒯振华

- 姓名：蒯振华（kuaizhenhua）
- 完成日期：2026-10-31；提交文件整理日期：2026-11-01。
- Ubuntu / ROS2：WSL2 中的 Ubuntu 22.04 / ROS2 Humble。
- Python / OpenCV / NumPy：3.10.12 / 4.5.4 / 1.21.5。
- 使用蓝色球或气球作为目标，没有摄像头，输入为公开照片。代码和整理使用AI辅助；本人实操截图、观察与助手自动验证分别记录。

目录按仓库现有规范放在 `算法组/Week2/kuaizhenhua/`。7张现场截图放在本目录，代码在 `src/`，输入在 `data/`，结果在 `outputs/`。

```text
kuaizhenhua/
├── README.md
├── 00_环境与练习.png
├── 01_检测输出.png
├── 02_结果可视化.png
├── 02_参数对比_HSV.png
├── 02_参数对比_面积.png
├── 03_interface_show.png
├── 03_pub_sub分屏.png
├── src/
│   ├── run_detection.py
│   ├── compare_parameters.py
│   ├── prepare_demo_images.py
│   ├── color_target_interfaces/
│   └── color_target_vision/
├── data/
│   ├── real_images/
│   ├── images/
│   └── demo_manifest.json
└── outputs/
    ├── detection/
    ├── comparison/
    └── ros2/
```

## 任务 1：颜色目标检测

输入是BGR图片。程序先检查读取成功，再转成HSV；`inRange`按颜色范围产生二值mask，经过开闭运算清理后提取外轮廓。按面积过滤和排序，选择最大的候选，用轮廓矩算中心，最后输出检测状态、中心和面积。这一结果同时供画图和ROS2发布使用。

| 参数或处理 | 设置与含义 |
| --- | --- |
| HSV下界 / 上界 | (95,80,70) / (130,255,255)；8位OpenCV的H范围是0～179，S/V为0～255 |
| 最小轮廓面积 | 500像素²；过小色块会被过滤 |
| 形态学 | 3×3椭圆核开运算去小噪声，5×5闭运算填小孔 |
| 多个候选 | 轮廓面积降序，选最大；同面积时按位置稳定排序 |
| 中心 | u=m10/m00，v=m01/m00；m00为0时跳过 |
| 未检测到 | detected=false，u、v、area均为0，并显示NO TARGET |

读图失败会报错退出，不当作“正常读图但无目标”。area是外轮廓几何面积，不是mask像素总数或真实物体面积。

正式输入为5张公开实拍照片，不是本人拍摄。作者、原始链接、许可、尺寸见[来源说明](data/real_images/来源说明.md)，下载文件的SHA256见[sources.json](data/real_images/sources.json)。另外保留6张标为SYNTHETIC的640×480练习图，用来单独检查小目标、暗光等条件；可由 `src/prepare_demo_images.py` 重新生成。

| 图片 | 尺寸 | 目标大致位置 | 默认结果(u,v)，面积 | 结果分析 |
| --- | --- | --- | --- | --- |
| 01_stress_ball | 1024×768 | 球心约(510,385) | (508.076,380.061)，80689.5 | 找到球，中心接近参考 |
| 02_table_ball | 960×623 | 球心约(219,370) | (233.379,358.374)，8134.5 | 找到球，但暗部丢失，中心偏右上 |
| 03_blue_flower | 960×951 | 没有目标球 | (408.670,194.876)，21896 | 把蓝紫色花朵误识别为目标 |
| 04_blue_balloon | 960×640 | 下半部气球被边界截断 | (442.589,493.961)，153421 | 检出可见区域，不能当作完整气球球心 |
| 05_red_ball | 960×640 | 没有蓝色目标 | false，0，0，0 | 正确报告未检出 |

位置参考来自助手看原图时的粗略估计，不是精确标注。实际输出见[完整统计](outputs/detection/summary.json)、[运行截图](01_检测输出.png)和[两图结果截图](02_结果可视化.png)。每张图的原始mask、形态学mask和标注文件均保留。

## 任务 2：可视化与参数对比

绿色框与轮廓表示选中的候选，红色十字表示中心，图中有u、v和area；浅色框是其他候选。未检出时明确写NO TARGET。结果保存为PNG，不只在窗口中显示。

下面两组均使用同一张图，只改变一项参数。对比图上排A、下排B，每排依次为原图、mask和标注结果。

| 对比 | A → B | 现象与解释 |
| --- | --- | --- |
| 玻璃球HSV | V下限70 → 40 | 面积8134.5→11916.5，中心(233.379,358.374)→(219.868,367.460)。暗部被保留，轮廓更完整，中心更接近完整球心。我的观察是“B的轮廓更完整，红点更集中”。 |
| 合成小圆面积 | 最小面积500 → 100 | A显示NO TARGET；B输出(330,250)，面积228，出现框和中心。球本身没有改变，只是面积门槛降低。 |

[HSV现场截图](02_参数对比_HSV.png) · [面积现场截图](02_参数对比_面积.png) · [HSV对比原图](outputs/comparison/01_v_min/01_02_table_ball.png) · [面积对比原图](outputs/comparison/02_min_area/01_02_small.png)

同目录的summary.json是原始运行统计的单图节选，参数和结果不变。两种典型失败是：蓝紫色花朵满足颜色条件，产生误检；玻璃球暗部被V门槛过滤，产生中心偏差。小目标也可能被面积门槛过滤造成漏检。降低阈值会保留更多背景，因此不能只凭一张图改善就认定整体效果更好。

坐标原点在左上角，u向右、v向下，单位为像素。它只是目标在画面中的位置，不是米制距离，也不是机器人世界坐标。同一个像素位置可能对应不同距离，转换还需要相机标定、深度或地面几何关系。本周没有实现这一步。

## 任务 3：ROS2 通信

`color_target_interfaces` 是ament_cmake接口包，`color_target_vision` 是ament_python节点包。消息类型为 `color_target_interfaces/msg/TargetPosition`，字段固定如下：

```text
bool detected
float32 u
float32 v
float32 area
```

发布节点按文件名循环读5张公开照片，每秒处理2张，每次调用与离线脚本相同的 `core.py` 重新检测，再把四个字段发到 `/target_position`。订阅节点从同一话题接收并打印四个字段，保存实际接收的JSONL。两端QoS均为reliable、keep_last、depth=10。

未检出消息为false和三个0，必须先看detected，不能理解成目标在左上角。读图或计算异常会报错，不发送伪造的“正常”结果。

本人分屏截图中，发送和接收均达到5581条，包含连续正常消息和未检出消息。程序随后继续运行，停止时[接收摘要](outputs/ros2/manual/summary.json)记录5843条，[完整接收记录](outputs/ros2/manual/received.jsonl)保留真实UTC时间。界面截图使用北京时间，两种时间相差8小时。

- [接口、topic info与实际消息截图](03_interface_show.png)：4个字段正确，发布者1个、订阅者1个。
- [发布/订阅分屏截图](03_pub_sub分屏.png)：蓝球消息为true / 508.076 / 380.061 / 80689.5；红球消息为false / 0 / 0 / 0。
- [动态参数验证](outputs/ros2/dynamic_test/report.json)：助手在本机独立验证默认V下限70时15条、运行中改成40后15条，两组都包含false，顺序和数值与逐图检测一致。实际命令及时间见[命令日志](outputs/ros2/dynamic_test/commands.log)。

动态参数项与5张照片测试已完成；摄像头实时输入未做。通信条数达标不表示所有图都识别正确，花朵的误检也会如实发布。

### 安装依赖、编译和复现

先安装Ubuntu22.04与ROS2 Humble。在本个人目录打开终端，使用Ubuntu系统Python：

```bash
sudo apt install python3-opencv python3-numpy python3-colcon-common-extensions ros-humble-rosidl-default-generators
source /opt/ros/humble/setup.bash
colcon build --packages-select color_target_interfaces color_target_vision
source install/setup.bash
```

图像检测与两组对比：

```bash
python3 src/run_detection.py
python3 src/compare_parameters.py --input data/real_images/02_table_ball.jpg --group 01_v_min
python3 src/compare_parameters.py --input data/images/02_small.png --group 02_min_area
```

默认结果分别写入 `outputs/detection/` 和 `outputs/comparison/`。复现时若想保留现有结果，可在命令后添加 `--output outputs/retest_detection` 或 `--output outputs/retest_comparison`。

ROS2实操时，两个终端都进入个人目录，执行上面两条source，并设置相同的本机实验域：

```bash
export ROS_DOMAIN_ID=89
export ROS_LOCALHOST_ONLY=1
echo "姓名: kuaizhenhua"; date "+%Y-%m-%d %H:%M:%S"
```

先在终端A运行订阅，再在终端B运行发布：

```bash
# 终端A：接收至少10条后，Ctrl+C结束并保存本次记录
ros2 run color_target_vision target_subscriber --ros-args -p output_dir:=outputs/ros2_retest
# 终端B：持续检测真实照片并发布，Ctrl+C结束
ros2 run color_target_vision target_publisher --ros-args -p image_dir:=data/real_images
```

第三个终端设置相同source与实验域后，可检查和调参：

```bash
ros2 interface show color_target_interfaces/msg/TargetPosition
ros2 topic info /target_position
ros2 topic echo /target_position --once
ros2 param set /target_publisher lower_v 40
```

运行中可调整HSV、min_area和形态学核，非法配置会被拒绝。min_area是浮点参数，例如 `ros2 param set /target_publisher min_area 100.0`。image_dir和publish_hz仅启动时设置。

## 问题与解决过程

安装时WSL在线下载和ROS软件源下载出现403。改用本地安装文件配置WSL与Ubuntu22.04，ROS源配置包从官方发行页下载并校验后本地安装，最后Humble及图像处理依赖能正常导入。

玻璃球默认结果中心偏右上。对比mask后发现暗部被过滤，只把V下限70调为40，轮廓更完整，中心也更接近球心。但蓝色花朵仍会误检，单靠颜色和最大面积不能判断物体身份。

本人16:47完成环境和合成图练习，观察到清晰蓝圆被框住且中心居中，无目标图显示NO TARGET；17:34—17:35运行公开照片检测和对比；23:02检查接口话题；23:56—23:57保存ROS2分屏及三张图像结果截图。截图原件未改动，图像预览里的时钟是查看结果时刻，检测运行时刻见终端截图。

## 仍未解决的问题

我还没完全理解：**为什么像素坐标不能直接用来控制机器人移动？** 目前知道它只是图像位置，具体怎么通过相机标定、深度或地面几何关系变成机器人坐标，还需要继续学习。

算法也不能可靠区分蓝球与同色花朵、背景，没有实现目标身份识别或跟踪。

代码采用Apache-2.0许可。照片和基于照片的标注、拼图按各自原许可使用，保留署名；具体见data中的来源说明。提交材料不含安装包、虚拟环境、缓存和编译产物。
