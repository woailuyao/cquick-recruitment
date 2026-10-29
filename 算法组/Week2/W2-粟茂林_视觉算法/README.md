 第 2 周提交说明 · 粟茂林

 一、基本信息

- 姓名拼音：sumaolin
- 姓名：粟茂林
- 完成时间：2026-10-29
- 系统 / Python 版本：Windows 11 / Python 3.11.9（任务 1、2）；WSL2 Ubuntu 22.04 / python3.10（任务 3）
- ROS2 版本：Humble
- OpenCV / numpy 版本：5.0.0 / 2.4.6

 二、环境与运行方式

本周有两套环境，分开说清楚：

任务 1、2 在 Windows 上跑（和 W1 同一套，商店版 Python）：

bash
pip install opencv-python numpy
python src/task1_target.py       三张图的检测与标注
python src/task2_compare.py      四组参数对比拼图


输入读 data/，输出写 outputs/taskN/。路径都用 Path(__file__).parent.parent 拼，不含绝对路径。

任务 3 在 WSL2 的 ROS2 Humble 里跑，源码仍然放在本仓库的 src/ 下，编译产物放到 WSL 本地盘：

bash
source /opt/ros/humble/setup.bash  //在我自己的电脑上，无需此行也可运行，原因未知
cd ~/ros2_ws
colcon build --base-paths /mnt/c/Users/cijiz/Desktop/W2-粟茂林_视觉算法/src \
             --build-base ~/ros2_ws/build --install-base ~/ros2_ws/install
source install/setup.bash
colcon build --base-paths /mnt/c/Users/cijiz/Desktop/W2-粟茂林_视觉算法/src \
             --build-base ~/ros2_ws/build \
             --install-base ~/ros2_ws/install \
             --packages-select target_detector
source install/setup.bash         //在每个终端上都先运行一遍基础加载

ros2 interface show target_interfaces/msg/TargetPosition   //终端A上运行

ros2 run target_detector target_publisher                  //终端B上运行

ros2 run target_detector target_subscriber                 //终端C上运行

ros2 topic info /target_position
ros2 topic echo /target_position                           //终端D上运行


 三、任务完成情况

 任务 1 · 颜色目标检测

- 测试图：football_1/2/3.jpg，都是 1920×1079
- 目标颜色：白色（足球球面）。白色在 HSV 里是 S 低 V 高，但草地白线也是白色，所以光靠阈值不够
- HSV 阈值：下限 (0, 0, 120)，上限 (180, 80, 255)
- 最小轮廓面积：5000 px
- 形态学：先 21×21 椭圆开运算（吃掉细白线），再 9×9 椭圆闭运算（补球面黑色五边形造成的洞）
- 筛选规则：面积 ≥ 5000 且外接矩形长宽比在 0.6~1.7 之间；多个候选按面积从大到小取最大
- 中心点：轮廓矩，u = m10/m00，v = m01/m00；m00 = 0 时退化成外接矩形中心

| 测试图 | 背景/光照 | 中心坐标 | 轮廓面积 | 结果 | 原因 |
|---|---|---|---|---|---|
| football_1 | 俯视全场，正面光，球压在中线上 | (958, 489) | 11243 | 检出 | 球是实心大块，21×21 开运算把细线抹掉后只剩球 |
| football_2 | 近距离，逆光，球底在阴影里 | (961, 594) | 66735 | 检出，中心略偏上 | V≥140 时球底阴影被切，降到 120 才够到；仍偏高约 25 px |
| football_3 | 同场景，画面里没有球 | — | — | 未检测到目标 | 只剩白线碎片，最大 1215 px，小于 5000 |

输出：outputs/task1/ 下 6 张，每张图一个 _mask.png 和一个 _result.png，左上角加了姓名拼音和运行时间戳。

失效场景：背景里有大面积白色物体（比如白墙、白车）时，面积阈值挡不住，会被当成球；球被完全遮挡或滚出画面时只能报“未检测到”，本次没有做跟踪，无法预测位置。

 任务 2 · 可视化与参数对比

标在图上的东西：绿色轮廓、黄色外接矩形、红色中心点、中心点坐标文字 u=958, v=489；未检出时图上只显示 NO TARGET。结果保存成文件，不依赖临时窗口。

参数对比用 football_1 做基准，每次只改一个参数，长宽比筛选保持不变：

| 变体 | 改动 | mask 像素 | 候选数 | 选中面积 | 中心点 | 判定 |
|---|---|---|---|---|---|---|
| A 基线 | — | 28423 | 1 | 11243 | (958, 489) | 正确 |
| B 放宽 HSV | S 80→120，V 120→100 | 151618 | 2 | 33755 | (920, 237) | 误检 |
| C 减小面积 | 5000→500 px | 28423 | 6 | 11243 | (958, 489) | 正确，候选变多 |
| D 不做形态学 | 开、闭都关掉 | 160447 | 0 | — | — | 漏检 |

B 的误检：阈值一放宽，mask 从 2.8 万涨到 15 万，球正下方那条白色中线跟球连成一块，绿色轮廓顺着中线窜到画面上边缘，中心点被拽到 (920, 237)，离球 250 多像素。结论：目标和背景同一色系时，放宽阈值只会让干扰长进目标里。

C 的结果不变：面积门槛从 5000 降到 500，候选从 1 个涨到 6 个（图上多出 5 个蓝框），但“取最大面积”这条规则让结果纹丝不动。说明面积阈值管的是噪声过滤，不管定位；只有背景里出现比目标更大的白块时才会出事，那时候该换规则而不是调阈值。

D 的漏检：没有开运算，白线保留原样，球又压在中线上，整张图的白线连成一张 94.8 万像素的网（外接矩形 1920×1048，几乎盖满画面），球被彻底吞掉；这张网的长宽比 1.83 超出 0.6~1.7 被筛掉，剩下的候选都小于 5000，于是 0 个候选。

坐标约定：图像左上角是原点 (0,0)，u 向右增大，v 向下增大。像素坐标只描述目标在图像里的位置，不是机器人坐标，也不是米制距离——同一个像素，球在 1 米和 5 米时坐标完全一样。

 任务 3 · ROS2 通信

消息定义（src/target_interfaces/msg/TargetPosition.msg）：

| 字段 | 类型 | 含义 |
|---|---|---|
| detected | bool | 本帧是否检测到目标 |
| u | float32 | 中心横坐标（像素） |
| v | float32 | 中心纵坐标（像素） |
| area | float32 | 轮廓面积（像素平方） |

约定 detected=False 时 u/v/area 一律填 0，但判断有效性只看 detected，不能用 u==0 推断——因为 (0,0) 本身是合法坐标。

两个包分工：

- target_interfaces（ament_cmake）：只负责用 rosidl_generate_interfaces() 把 .msg 翻译成 Python 类。必须是 cmake 包，因为 rosidl 是 CMake 宏。
- target_detector（ament_python）：detect.py 放纯函数（图进、(u,v,area) 出），target_publisher.py 和 target_subscriber.py 放节点。

数据流：

text
data/.jpg → cv.imdecode → detect() → (u,v,area) → TargetPosition → /target_position
                                                                          ↓
                                                              target_subscriber 打印


发布端用 1 Hz 定时器循环读 3 张图，跑 12 秒正好 12 条消息（细则要求至少 10 条）。定时器靠 rclpy.spin() 驱动，回调里读图+检测约几毫秒。

实际检测结果接进来了：detect.py 的参数和 task1_target.py 完全一致，同一张图在两边算出的坐标逐位相同（football_1 都是 u=958.2, v=489.3，面积 11243），不是写死的假数据。

命令与验证：

bash
ros2 interface show target_interfaces/msg/TargetPosition    应显示 4 个字段
ros2 topic info /target_position                             看 Type 和 Publish/Subscribe 数量
ros2 topic echo /target_position                             逐条看消息内容


为什么 u、v 不能直接当运动指令：像素坐标是图像平面上的位置，要变成场地坐标至少需要相机内参、安装外参和一个几何假设（比如球在地面上），还要知道距离。这周只要求把像素坐标稳定、可解释地传出去。

 四、问题与解决过程

白色球和草地白线同色，最大轮廓一开始是 94.8 万像素

- 现象：不做形态学处理时，最大轮廓的外接矩形几乎盖满画面（1920×1048），球在里面找不到
- 怎么查的：把 mask 存图看，发现白线在球场上是连通的，球压在中线上，三者连成一片
- 原因：白线的宽度在远处只有几个像素，但开运算核太小时抹不掉
- 改进：开运算核从 7 一路试到 31，21×21 时白线断成小点、球还是完整一块；再配合面积 5000 和长宽比筛选

 五、仍未解决的问题

1. football_2 的中心点还是偏高约 25 像素。球底阴影被 V 阈值切掉一部分，矩心天然往上偏。用最小外接圆圆心会更准（球是圆的），但换成任意形状就退化，本周没做。
2. 没有做相机标定和坐标转换。现在输出的像素坐标下发给底盘毫无意义，需要内参、外参和地面假设，这是第 3 周之后的事。
3. 参数还写死在代码里。HSV 阈值、最小面积、发布频率都还没做成 ROS2 参数，加分项里的“ros2 param set 运行时调整”没做。
4. 只有 3 张图，样本偏少。football_3 恰好是“完全没有球”的负样本，但“球被部分遮挡”“背景有白色物体”这两类难例没测过。

 六、给下周自己的提醒

- 白色目标的难点不在阈值，在背景同色。先看 mask 再定参数，不要对着终端数字调。
- 面积阈值只影响噪声过滤，不影响定位；改它之前先想清楚背景里有没有更大的同类目标。
- 形态学核大小绑分辨率，换图尺寸要跟着换。
- ROS2 里改 .msg 必须重新 colcon build 并重新 source，否则两个节点用的是不同版本的定义，不报错但数据错位。
- 定时器靠 spin 驱动，while True + sleep 会堵死订阅回调。