# 第2周考核 · 胡钰涵

- 姓名：胡钰涵 / Hu Yuhan
- 完成日期：2026-11-01
- 目标：使用OpenCV检测黄绿色网球，输出图像中心坐标和轮廓面积，再通过ROS2话题发布，由另一个节点订阅并打印。
- 输入：5张静态图片；本次未使用实时摄像头。

## 环境与目录

本次分别在Windows中进行图片实验，在WSL的Ubuntu中运行ROS2。两个环境的库版本分别记录，不混用。

| 项目 | Windows图片实验 | WSL中的ROS2实验 |
| --- | --- | --- |
| 系统 | Windows | Ubuntu 22.04 |
| ROS2 | 不使用 | Humble |
| Python | 3.10.11 | 3.10.12 |
| OpenCV | 5.0.0 | 4.5.4 |
| NumPy | 2.2.6 | 1.21.5 |

版本依据：图片实验的[运行记录](results/bonus_five_images/run_log.txt)和[ROS2环境截图](screenshots/14_ros2_environment.png)。

| 路径 | 内容 |
| --- | --- |
| `images/` | 5张输入图片 |
| `scripts/detect_color.py` | 单图检测与鼠标采样BGR/HSV |
| `scripts/compare_params.py` | HSV和最小面积参数实验 |
| `scripts/temperary.py` | 裁剪生成无目标图片 |
| `scripts/_image_io.py` | 支持中文路径的图片读写工具 |
| `bonus_five_images.py` | 使用同一组参数批量处理5张图片 |
| `results/` | 单图和参数实验结果 |
| `results/bonus_five_images/` | 五图测试结果、CSV及运行日志 |
| `screenshots/` | 实际运行截图 |
| `ros2_ws/src/target_interfaces/` | 自定义消息包 |
| `ros2_ws/src/target_detector/` | Python发布者和订阅者包 |

## 任务1：颜色目标检测

### 处理流程与参数

| 步骤 | 输入 | 处理与输出 |
| --- | --- | --- |
| 1. 读取图片 | 图片路径 | 读取为BGR图像；失败时给出提示并退出 |
| 2. 转换颜色空间 | BGR图像 | 转为HSV图像 |
| 3. 颜色分割 | HSV图像、阈值 | 使用`cv.inRange`生成二值mask |
| 4. 提取候选轮廓 | mask | 提取外轮廓，按轮廓面积降序排列 |
| 5. 筛选目标 | 候选轮廓 | 依次检查面积、宽高比和圆度，选第一个合格轮廓 |
| 6. 计算与输出 | 目标轮廓 | 计算中心和面积，打印结果，保存mask、分割图及标注图 |

固定基准参数如下：

| 参数 | 取值 | 作用 |
| --- | --- | --- |
| HSV下界 | `[20,40,50]` | 保留目标的黄绿色范围，并限制最低饱和度、亮度 |
| HSV上界 | `[45,255,255]` | 限定色调上界 |
| 最小轮廓面积 | `500`像素² | 排除小面积噪声 |
| 外接矩形宽高比 | `0.7～1.3` | 排除明显狭长的轮廓 |
| 最小圆度 | `0.6` | 排除形状过于不规则的轮廓 |

选取阈值时，先用鼠标点击目标和背景，查看对应的BGR、HSV值，再观察mask。`H=20～45`作为黄绿色网球的初始范围；`S≥40`和`V≥50`用于排除部分低饱和度和过暗区域。这是针对本次图片的经验参数，并不是适用于所有光照和背景的通用阈值。

本次基准检测未采用形态学处理；开、闭运算对分割结果的影响尚未进行系统对比，保留为后续测试方向。

圆度计算为`4πA/P²`，其中A是轮廓面积，P是闭合周长。代码在P为0时跳过，避免除以0。

### 思考与排查过程

**1. 先看分割结果，发现“颜色范围包含网球”还不够。**

处理clear.jpg时，球虽然进入mask，部分树木背景也变成了白色，说明HSV范围同时包含了目标和部分背景。进一步查看similar_background.jpg，草地与网球颜色接近，仅靠颜色很难将两者分开。因此需要同时检查目标是否保留、背景是否被排除；mask中的白色区域减少，并不一定意味着检测更准确。

**2. 用球面不同位置的采样，检查阈值依据。**

球面不同位置受光照影响，颜色可能存在差异，单个像素不足以代表整个目标。因此选取球的最左、最下、最右、最上四个位置进行颜色观察，通过鼠标点击回显坐标、BGR和HSV，比较不同位置的颜色。采样用于辅助选择阈值，当前没有实现自动计算最优阈值。

收窄H范围后，部分背景减少，但球的mask也出现缺失。这说明阈值需要在保留目标和排除背景之间取舍，不能只为清除背景而不断收窄。

**3. 当颜色规则不够时，再检查面积和形状；无球图推动了圆度筛选。**

颜色分割后仍存在多个候选轮廓，因此先按面积排序，再检查面积和外接矩形宽高比。此时无目标图片仍把树木背景中的一个轮廓选为目标，输出中心(719.5,236.5)、面积1909.5，说明面积与宽高比接近球并不足以确认目标。

为进一步排除不规则背景，增加圆度条件，最终无目标图返回False，见[05号误检截图](screenshots/05_no_target_before_circularity.png)与[04号最终结果](screenshots/04_no_target.png)。但圆度限制也会带来漏检：shadow.jpg中球的大部分区域已经进入mask，却因轮廓缺口和不规则而被过滤，说明形状约束需要兼顾真实目标的分割完整性。

**4. 用两张图片做控制变量实验，检查参数对不同大小目标的影响。**

只在一张图片上调整参数，难以判断变化来自参数本身还是图片特点。因此每组A/B都分别测试clear和small，并保持其余条件一致。两张图的目标大小不同，可以检查同一个参数是否会对大、小目标产生不同影响。

面积阈值是筛选下限，选值需要结合真实目标面积。对比500和10000时，clear的目标面积11966.0高于10000，仍被保留；small的目标面积4229.0低于10000，被过滤。由此可以判断，提高面积门槛虽然能过滤小噪声，也会漏掉真实小目标，不能只凭某张图的效果决定阈值。HSV对比同样保留了收窄范围后漏检的结果，具体参数与数据见任务2。

**5. 最后把程序结果与原图核对，保留没有解决的情况。**

终端的True或False只能说明程序是否选中了轮廓，正确性仍需对照原图和标注图判断。no_target.jpg没有球，返回False是正确排除；similar_background.jpg和shadow.jpg实际有球，返回False则是漏检。同色背景问题尚未解决，因此保留失败案例，用于分析颜色分割与形状筛选各自的局限。

### 中心坐标与失败处理

本次采用**外接矩形中心**，不是轮廓矩质心。对`x,y,w,h=cv.boundingRect(cnt)`：

```python
u = x + w / 2
v = y + h / 2
```

中心以浮点数保存，因此可能出现330.5这样的结果；只在绘制中心点时取整。多个候选目标存在时，选面积排序后第一个通过全部筛选条件的轮廓。

未检测到目标时，输出明确提示，并令`detected=False`、`u=v=area=0`。必须先判断detected，不能把默认的(0,0)解释为目标真的位于图像左上角。

### 五张图片的实际结果

5张图片使用相同的基准参数。表中“目测位置”仅用于人工核对，不是精确标注。

| 图片 | 尺寸（宽×高） | 目标目测位置 | 程序输出中心 | 轮廓面积（像素²） | 判断 |
| --- | --- | --- | --- | ---: | --- |
| clear.jpg | 960×631 | 约(330,440) | (330.5,438.5) | 11966.0 | 正确定位 |
| small.jpg | 1200×800 | 约(630,590) | (629.0,591.5) | 4229.0 | 正确定位 |
| similar_background.jpg | 1200×795 | 约(590,395) | 未检出 | 0 | 漏检 |
| no_target.jpg | 960×330 | 无球 | 未检出 | 0 | 正确排除 |
| shadow.jpg | 1600×2400 | 约(790,1140) | 未检出 | 0 | 漏检 |

结果见[运行日志](results/bonus_five_images/run_log.txt)、[数据表](results/bonus_five_images/summary.csv)和[标注图目录](results/bonus_five_images/)。CSV已在自动检测数据的基础上补全人工判断与原因分析，与本节表格及下文分析一致。重新运行批量脚本会重新生成CSV，人工结论需在核对新标注图后填写。

### 疑惑与易错点

1. **HSV阈值不能直接使用BGR采样值。** 先转换颜色空间；鼠标返回的是(x,y)，读取数组像素时写`image[y,x]`。OpenCV常用8位HSV中，H范围为0～179，S、V为0～255。
2. **mask里背景变白，不一定表示最终误检。** 白色只表示通过颜色阈值，还需要轮廓筛选与人工核对。
3. **最大轮廓不一定是目标。** 本次按面积排序后继续检查宽高比和圆度，不能直接选整张mask最大的白色区域。
4. **没有目标与目标漏检要区分。** no_target返回False是正确排除；有球的图片返回False才是漏检。

## 任务2：结果可视化与参数对比

### 可视化

输出图绘制红色目标轮廓、绿色中心点和像素坐标；未检出时标注`No target detected`。结果保存为文件，可以在程序退出后检查。

- [clear原图](images/clear.jpg)与[基准标注图](results/clear_detected.jpg)。
- [small原图](images/small.jpg)与[基准标注图](results/small_detected.jpg)。
- mask表示颜色筛选结果，res表示原图中被mask保留的区域，detected表示完成轮廓筛选后的结果。

### 对比一：调整HSV范围

保持最小面积500、宽高比及圆度条件不变，比较一组较宽与一组较窄的颜色范围。

| 组别 | HSV下界 | HSV上界 | clear结果 | small结果 |
| --- | --- | --- | --- | --- |
| HSV_A | `[20,40,50]` | `[45,255,255]` | 正确检出 | 正确检出 |
| HSV_B | `[25,40,50]` | `[40,255,255]` | 未检出 | 未检出 |

收窄H范围后，部分背景减少，但目标颜色也被排除了一部分，mask轮廓变得不完整。clear的候选圆度出现0.354、0.473；small的候选圆度为0.588，都低于0.6，因此最终漏检。不能只根据背景变少就认定阈值更好。

证据：[clear A](screenshots/06_clear_HSV_A.png)、[clear B](screenshots/07_clear_HSV_B.png)、[small A](screenshots/08_small_HSV_A.png)、[small B](screenshots/09_small_HSV_B.png)。对应结果以`*_HSV_A_*`和`*_HSV_B_*`保存。

### 对比二：调整最小面积

保持HSV基准范围、宽高比及圆度条件不变，只改变最小轮廓面积。

| 组别 | 最小面积 | clear结果 | small结果 |
| --- | ---: | --- | --- |
| AREA_A | 500 | 正确检出，面积11966.0 | 正确检出，面积4229.0 |
| AREA_B | 10000 | 正确检出 | 未检出 |

clear的目标面积大于10000，仍被保留；small的目标面积小于10000，被过滤。提高面积门槛能排除小噪声，也会排除真实小目标。

这两组的mask应相同，因为面积阈值在提取轮廓后才使用，不参与颜色分割；变化发生在最终筛选结果中。

证据：[clear A](screenshots/10_clear_AREA_A.png)、[small A](screenshots/11_small_AREA_A.png)、[clear B](screenshots/12_clear_AREA_B.png)、[small B](screenshots/13_small_AREA_B.png)。

### 真实误检、漏检与排查

| 场景 | 观察到的现象 | 原因与处理 |
| --- | --- | --- |
| 无目标图，加入圆度筛选前 | 背景轮廓被选中，中心(719.5,236.5)，面积1909.5 | 只依靠面积和宽高比仍可能误选背景，之后加入圆度筛选 |
| 无目标图，最终版本 | mask仍含背景白色区域，但最终返回False | 背景没有通过后续形状筛选，最终没有误检 |
| 同色背景 | 草地和球所在区域连成较大白色区域，最终返回False | 颜色阈值难以区分目标与同色背景，难以获得独立球形轮廓 |
| shadow | mask保留了球的大部分区域，但最终返回False | 深色球缝、文字等使保留区域产生缺口，轮廓不规则，未通过圆度门槛 |

历史误检证据：[加入圆度筛选前](screenshots/05_no_target_before_circularity.png)；最终无目标结果：[截图](screenshots/04_no_target.png)。

本次五图最终结果中未出现误选背景的情况，但存在两个漏检案例。形状限制有助于减少误检，同时也可能过滤真实目标；不能把限制加得越多简单理解为检测效果越好。

### 坐标约定与易错点

1. 图像左上角为原点(0,0)，u向右增大，v向下增大，单位为像素。数组读取顺序为`image[v,u]`，即先行后列。
2. 像素坐标只表示目标在图像中的位置，不是米制距离，也不是机器人世界坐标。转换成实际空间位置还需要相机标定、几何关系、深度或其他测量信息，本次未实现这部分。
3. u、v不能直接作为机器人运动指令，还需要将目标位置转换到控制所需的坐标系，并结合控制逻辑。
4. 未检出时的(0,0)是无效结果的默认值，必须先判断`detected`，不能将它当作目标的实际位置。

## 任务3：ROS2通信

### 消息与节点

接口包`target_interfaces`为ament_cmake类型，定义`TargetPosition.msg`：

```text
bool detected
float32 u
float32 v
float32 area
```

| 字段 | 含义 |
| --- | --- |
| detected | 是否选中了符合条件的目标轮廓 |
| u、v | 外接矩形中心的水平、垂直像素坐标 |
| area | 目标轮廓面积，单位像素² |

`CMakeLists.txt`配置消息代码生成，`package.xml`声明构建与运行依赖。编译并source工作区后，Python通过`from target_interfaces.msg import TargetPosition`使用消息类型。

程序包`target_detector`为ament_python类型，含两个节点：

| 节点 | 处理过程 |
| --- | --- |
| target_publisher | 读取image_path指定图片，执行检测，把结果填入消息，每秒发布到`/target_position` |
| target_subscriber | 订阅同名话题及同一消息类型，在回调中打印四个字段并累计接收条数 |

具体数据流：读取图片得到BGR数组；检测函数产生`detected/u/v/area`并填入TargetPosition对象；发布者调用publish发送；订阅者收到对象后，通过`msg.detected`、`msg.u`等读取字段。标注图用于核对消息中的坐标是否确实对应图片中的球。

当前发布者在**启动时检测一次静态图片**，之后每秒重复发布这次结果。因此连续消息相同是预期行为，坐标仍来自实际检测，并不是在代码中写死的测试坐标。更换图片后需重新启动发布者。

### 实际验证

使用clear.jpg得到`detected=True, u=330.5, v=438.5, area=11966.0`，与标注图一致。发布端和订阅端连续收发超过10条消息，独立运行记录展示了15条消息。

| 验证内容 | 证据 |
| --- | --- |
| 环境与依赖 | [14_ros2_environment.png](screenshots/14_ros2_environment.png) |
| 自定义消息四个字段 | [15_target_position_interface.png](screenshots/15_target_position_interface.png) |
| 实际检测消息发布 | [16_target_publisher.png](screenshots/16_target_publisher.png) |
| 连续接收至少10条 | [17_target_subscriber_10_messages.png](screenshots/17_target_subscriber_10_messages.png) |
| 发布与订阅同时运行 | [18_pub_sub_split.png](screenshots/18_pub_sub_split.png) |
| 话题类型及节点数量 | [19_target_topic_info.png](screenshots/19_target_topic_info.png) |
| 消息对应的标注图 | [clear_ros2_detected.jpg](results/clear_ros2_detected.jpg) |

topic info显示类型为`target_interfaces/msg/TargetPosition`，发布者与订阅者各1个。

### 学习中的疑惑与易错点

1. **Ubuntu、WSL、ROS2分别是什么？为什么又要安装Linux？** Ubuntu提供Linux运行环境，WSL使其可以运行在Windows中；ROS2是机器人软件通信框架。本次选择Ubuntu 22.04运行Humble，图片检测本身仍可在Windows完成。
2. **.msg文件为什么只有四行，没有具体坐标？** 它规定数据格式；具体值由检测程序填入。消息包需要配置、编译并加载环境，才能被节点使用。
3. **新终端为什么需要source？** 当前终端需要加载系统ROS2及自建工作区的环境，才能找到消息包和程序入口；`setup.py`中的console_scripts规定启动入口对应哪个main函数。
4. **订阅代码里的10、回调和spin是什么意思？** 10是队列深度，不是只收10条；`self.receive_target`不加括号，是把方法交给ROS2在收到消息时调用；spin负责持续处理消息等事件。

## 运行与复现

### Windows：安装依赖与图片检测

在PowerShell中先进入克隆得到的`cquick-recruitment`仓库根目录，再进入个人目录并创建环境：

```powershell
Set-Location ".\算法组\Week2\huyuhan"
py -3.10 -m venv .venv
.\.venv\Scripts\python.exe -m pip install opencv-python numpy
```

后文“项目根目录”指包含本README、images、scripts、results和ros2_ws的个人目录。图片脚本根据自身位置寻找输入和输出，不依赖固定盘符；读写方式兼容Windows中的中文路径。

先运行五图批量测试：

```powershell
.\.venv\Scripts\python.exe .\bonus_five_images.py
```

批量脚本生成5张PNG掩膜、5张JPG标注图、summary.csv及run_log.txt，保存在`results/bonus_five_images/`。提交的CSV已经人工核对；重新运行脚本后需重新填写人工判断和原因分析。

单图脚本需要先修改`name`，例如`"clear"`、`"small"`、`"similar_background"`、`"no_target"`或`"shadow"`，再运行：

```powershell
.\.venv\Scripts\python.exe .\scripts\detect_color.py
```

显示窗口中可点击原图，终端输出坐标、BGR和HSV值；查看完成后按键退出。输出目录不存在时自动创建。

### Windows：参数对比

在`compare_params.py`中同时检查以下变量，再运行脚本：

| 实验编号experiment | lower | upper | min_area |
| --- | --- | --- | ---: |
| HSV_A | [20,40,50] | [45,255,255] | 500 |
| HSV_B | [25,40,50] | [40,255,255] | 500 |
| AREA_A | [20,40,50] | [45,255,255] | 500 |
| AREA_B | [20,40,50] | [45,255,255] | 10000 |

每组分别令`name="clear"`、`name="small"`执行：

```powershell
.\.venv\Scripts\python.exe .\scripts\compare_params.py
```

**experiment目前只是实验标签和文件名组成部分，不会自动切换参数。** 不能只改标签而忘记修改lower、upper、min_area。本次脚本保留最后一次small、AREA_B配置。

### Ubuntu：构建ROS2软件包

前提是已安装并可使用ROS2 Humble、colcon、rclpy、OpenCV与NumPy。使用ROS2的系统Python；Windows虚拟环境不用于运行ROS2节点。

在Ubuntu或WSL终端中先进入本地克隆仓库的根目录，再执行：

```bash
cd "算法组/Week2/huyuhan"
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-up-to target_detector
cd ..
```

这里直接构建个人目录下自带的ros2_ws工作区，无需复制到固定的主目录工作区。构建结束后的当前目录应回到个人项目根目录。

### Ubuntu：运行与检查

以下每个终端都先进入个人项目根目录。终端一启动订阅者：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 run target_detector target_subscriber
```

终端二启动发布者：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 run target_detector target_publisher --ros-args -p image_path:=images/clear.jpg
```

image_path默认也是`images/clear.jpg`，相对于启动节点时的当前目录。输出自动保存在输入图片目录同级的results目录。检测只在节点启动时执行一次，此后每秒发布该结果。

终端三检查消息和话题：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 interface show target_interfaces/msg/TargetPosition
ros2 topic info /target_position
ros2 topic echo /target_position
```

保持发布者和订阅者同时运行时检查topic info。检查结束后使用Ctrl+C停止各程序。若换成no_target.jpg检查失败消息，应先停止原发布者，再以`image_path:=images/no_target.jpg`重新启动，避免多个发布者同时向该话题发送结果。代码约定未检出时发送False和三个0值。

关键截图保留姓名拼音、实际时间和对应输出。截图前可运行：

```bash
echo "姓名：胡钰涵 / Hu Yuhan"
TZ=Asia/Shanghai date '+%Y-%m-%d %H:%M:%S %Z'
```

### 输出格式与提交整理

为控制提交体积，彩色分割图与标注图以JPEG质量90保存，保持原始宽高；掩膜继续使用无损PNG。压缩只用于结果展示，不改变输入图片、HSV阈值、面积、圆度条件或中心计算。

早期运行截图、日志中可能显示原工作路径或PNG标注文件名；这些是实际实验记录，予以保留。当前代码与本文链接使用整理后的可移植路径和JPG结果文件名。

`.gitignore`排除了虚拟环境、Python缓存及ROS2的build、install、log目录，提交时只保留源码和实验材料。

## 图片来源

| 文件 | 来源与处理 |
| --- | --- |
| clear.jpg | Santeri Viinamäki，Wikimedia Commons：[Tennis ball on tennis court 20170619](https://commons.wikimedia.org/wiki/File:Tennis_ball_on_tennis_court_20170619.jpg)，使用960×631预览图；许可为[CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/) |
| no_target.jpg | 从上述clear.jpg裁剪第0～329行，保留全宽，生成无球负样本；处理脚本为scripts/temperary.py，来源作者与许可同上 |
| small.jpg | Nik（@helloimnik），Unsplash：[white and yellow tennis ball on green grass field](https://unsplash.com/photos/white-and-yellow-tennis-ball-on-green-grass-field-Bs9tJiCq930)，按[Unsplash License](https://unsplash.com/license)使用；下载的JPEG尺寸为1200×800，保留完整画面以体现较小的目标占比 |
| similar_background.jpg | Jeroen Bosch（@jeroenbosch），Unsplash：[green tennis ball on green grass](https://unsplash.com/photos/green-tennis-ball-on-green-grass-afcdwWh8BPI)，按[Unsplash License](https://unsplash.com/license)使用；下载的JPEG尺寸为1200×795 |
| shadow.jpg | Ahmed（@mutecevvil），[Unsplash原页面](https://unsplash.com/photos/a-tennis-ball-sitting-on-a-tennis-court-net-HL8ebMRzZiY)，按[Unsplash License](https://unsplash.com/license)使用，下载的JPEG尺寸为1600×2400 |

结果目录中的mask、分割图和标注图由相应输入图片计算生成。clear与no_target相关派生图片保留上述作者署名、来源与CC BY-SA 4.0许可说明；添加检测标注或进行颜色分割、裁剪是本次所做的处理。

clear、small与similar_background的来源信息来自最初测试图片包中的SOURCES.md；本次实验保留该包提供的图片版本。

## 问题与解决过程

1. 最初清晰图中也有背景进入mask，因此同时检查原图、mask和最终轮廓，而不是只看分割图颜色。
2. 无目标图曾被误检为面积1909.5的背景轮廓。检查后增加圆度筛选，最终版本能返回未检测到；历史失败截图保留，没有只保留成功结果。
3. 同色草地背景难以分开，当前尚未找到稳定的解决办法，因此保留漏检并分析颜色规则的限制；形态学处理的效果留待后续系统对比。
4. ROS2部分通过定义消息、构建、source、注册入口，再启动发布订阅两个节点逐步验证；每一步分别保留了运行证据。

## 仍未解决的问题与本周总结

目前ROS2通信已经按步骤跑通，但对`rclpy.spin`如何调度回调，以及两个节点如何在底层发现对方并建立通信，仍未完全理解。这部分需要继续结合简单发布订阅示例学习。

检测方面还存在一个取舍：降低圆度门槛可能恢复shadow中的目标，也可能重新放过不规则背景；目前没有找到对所有测试场景都稳定的参数。

本次完成了图片读取、HSV分割、轮廓筛选、中心与面积输出、结果可视化、两类参数实验、ROS2实际检测结果通信，以及5张图片的统一参数测试。五图结果保留了成功与失败情况，作为多图测试加分项的实验与分析材料；未实现实时摄像头和ROS2运行时HSV调参。

