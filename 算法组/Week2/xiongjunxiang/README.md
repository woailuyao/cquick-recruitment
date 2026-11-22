# 第 2 周考核 · 熊俊翔

- 姓名：熊俊翔
- 完成日期：2026-11-01
- 开发环境（Windows 11 23H2）：Python 3.15.0 / OpenCV 5.0.0 / numpy 2.5.4
- 开发环境（WSL2 Ubuntu 22.04.5）：Python 3.10.12 / OpenCV 4.5.4 / ROS2 Humble + colcon 0.21.3

## 目录与运行方式

```text
xiongjunxiang/
├── README.md
├── data/images/                       orange1.jpg ~ orange3.jpg（手机自拍，橘子）
├── src/
│   ├── task.py                        任务 1/2：HSV 检测 + 中心点 + 标注存图
│   └── ros2ws/                        任务 3：ROS2 工作空间
│       ├── src/target_interfaces/     ament_cmake 接口包（msg/TargetPosition.msg）
│       ├── src/target_pubsub/         ament_python 节点包
│       │   ├── images/                orange1.jpg ~ orange3.jpg（发布节点读这一份）
│       │   └── target_pubsub/         publisher_node.py / subscriber_node.py
│       └── build/ install/ log/       colcon 编译产物（可删，重新 build 会生成）
├── outputs/                           mask2/3、result2/3 共 12 张结果图
└── 屏幕截图 *.png                      终端输出与 ROS2 运行的截图
```

```bash
# 任务 1/2
python src/task.py      # 读 data/images/*.jpg，写 outputs/orangeX_{mask3,result3}.png

# 任务 3（在 WSL 的 Ubuntu 里）
cd src/ros2ws
colcon build --symlink-install
source install/setup.bash
ros2 run target_pubsub publisher_node      
ros2 run target_pubsub subscriber_node     
```

`outputs/` 里的 `_2` 和 `_3` 是**同一套流程、两组不同阈值**跑出来的，
所以能直接放在一起对比（`_2` 是上一版，`_3` 是现在这版）。

## 任务 1：颜色目标检测

| 项目 | 取值 |
| --- | --- |
| 目标颜色 | 橙色（橘子） |
| HSV 上下界 | `LOWER = (10, 170, 150)`，`UPPER = (20, 255, 255)` |
| 最小轮廓面积 | $1000 px^2$ |
| 形态学 | 开运算 椭圆 7×7 ×1（去散点）→ 闭运算 椭圆 7×7 ×2（补孔洞） |
| 多候选选择规则 | 按 `contourArea` 从大到小排序，取**面积最大**的当目标，其它只画灰色细线 |
| 中心点算法 | 轮廓矩 `u = m10/m00`、`v = m01/m00`；`m00 == 0` 的退化轮廓直接跳过，避免除零 |

阈值不是猜的：先在橘子上取 180×180 像素块统计，得到 **H 中位 14、S 中位 227~240、V 中位 216~247**；
而手掌的 S 只有 120~150、木桌面 V 多在 180 以下。把 S 下限提到 170、V 下限提到 150，
才能把橘子和手、桌面分开（这一步是调试出来的，见"问题与解决过程"）。

**三张测试图结果**（都是 3072×4096，手机竖拍，对应 `outputs/orangeX_result3.png`）：

| 测试图 | 场景 | 中心 (u, v) | 面积 px² | 候选数 | 结果 |
| --- | --- | --- | --- | --- | --- |
| orange1.jpg | 手拿着橘子，室内偏暗，旁边有橙色魔方 | (1507, 3024) | 166662 | 2 | 检测到 |
| orange2.jpg | 橘子放桌面，距离近、目标最大 | (1724, 2877) | 348902 | 5 | 检测到 |
| orange3.jpg | 橘子放桌面，距离远、目标最小，背景有瓶子橙色图案 | (2151, 2199) | 73188 | 3 | 检测到 |

代码里对没有目标的情况做了分支：候选为空时把 `detected` 置 False、`u/v/area` 置 0，
不会拿未初始化的坐标当结果。

**算法会失效的情况**：

1. **同色物体干扰**：橙色魔方、瓶子上的橙色图案和橘子色相几乎一样（H 都在 10 左右）。
   orange2 里魔方那块 164750 px、圆形度只有 0.26，仅仅是比橘子小才没被选成主目标；
   橘子再远一点，魔方就会被误判成目标。
2. **光照不足 / 背光**：橘子暗面 V 会掉到 150 以下，掩膜缺一块，轮廓缺口变大，中心点跟着偏。
3. **反光高光**：果皮上白亮斑点的 S 很低会被判成背景，掩膜里挖出小洞（闭运算能补一部分）。
4. **手部肤色**：本周实际踩到的坑——参数一放宽，手掌和橘子就粘成一片（见任务 2 的对比）。

## 任务 2：可视化与参数对比

标注内容（`outputs/orangeX_result3.png`，脚本生成，不是手工截图）：黄色轮廓、红色外接矩形、
红色实心中心点、文字 `u=..., v=...`；有多个候选时次要轮廓画灰色细线。掩膜另存 `orangeX_mask3.png`。

### 参数对比：`outputs/` 里上一版（`_2`）与现在这版（`_3`）

| 版本 | HSV 上下界 | 最大面积过滤 | 形态学 |
| --- | --- | --- | --- |
| `_2`（上一版） | `(5, 100, 100) ~ (25, 255, 255)` | 1000 px | 开 7×7 ×1 + 闭 7×7 ×2 |
| `_3`（现在） | `(10, 170, 150) ~ (20, 255, 255)` | 1000 px | 开 7×7 ×1 + 闭 7×7 ×2 |

**掩膜规模对比**

| 图片 | `_2` 白像素 / 轮廓数 / 通过面积过滤 | `_3` 白像素 / 轮廓数 / 通过面积过滤 |
| --- | --- | --- |
| orange1 | 9.16% / 53 / 17 | 1.38% / 11 / 2 |
| orange2 | 9.62% / 189 / 35 | 4.49% / 8 / 5 |
| orange3 | 4.94% / 113 / 17 | 0.73% / 5 / 3 |

**最大目标的位置与形状对比**

| 图片 | `_2`：面积 / 中心 / 圆形度 | `_3`：面积 / 中心 / 圆形度 |
| --- | --- | --- |
| orange1 | 612947 / (1127, 3287) / 0.11 → 框住"手掌 + 魔方 + 桌面" | 166662 / (1507, 3024) / **0.68** → 橘子 |
| orange2 | 374512 / (1722, 2885) / 0.58 → 外接框 711×672，偏大 | 348902 / (1724, 2877) / **0.86** → 橘子 |
| orange3 | 487050 / (1872, 2134) / 0.09 → 外接框 2074×769，横跨桌面 | 73188 / (2151, 2199) / **0.81** → 橘子 |

**结果差在哪**

- `_2` 的 S 下限只有 100、V 下限 100，**手掌（S≈120~150）、木桌面、橙色魔方全进了掩膜**：
  白像素占比 9% 左右，最大目标 48.7 万到 61.3 万 px、圆形度只有 0.09 到 0.11（形状是一大片，不是圆），
  orange1 的中心 (1127,3287) 直接落在手掌上，orange3 的中心 (1872,2134) 落在桌面上。
- `_3` 把 S 抬到 170、V 抬到 150 后，白像素占比降到 1.38% / 4.49% / 0.73%，
  通过面积过滤的候选从 17/35/17 个降到 2/5/3 个，最大目标变成圆形度 0.68~0.86 的橘子，
  orange1、orange3 的中心也回到果皮上（orange3 从 (1872,2134) → (2151,2199)）。
- 副作用：`_3` 在 orange2 里仍保留魔方（164750 px，圆形度 0.26）和瓶子图案（46520 px），
  只是都比橘子小，按"取最大"不会选它们——这也说明**只靠颜色 + 面积排不掉同色物体**。

**误检案例**：`_2` 的 orange1，面积 612947 px、中心 (1127,3287)，框住的是手掌和魔方；
肤色、木色、橙色魔方的 H 都在 5~25 之间，S/V 下限太低时全被放进来。

**定位偏差案例**：`_2` 的 orange3 检出 2074×769 的横条（面积 487050），中心 (1872,2134) 落在桌面中间；
真正的橘子实际中心约 (2151,2199)、面积约 7.3 万 px，只是这个大块里的一小部分，等于定位彻底偏了。

**坐标约定**：图像左上角是原点 (0,0)，`u` 向右增大，`v` 向下增大。像素坐标只描述目标在
**这一帧图像里的位置**，不是机器人坐标、也不是米制距离；要换成场地坐标必须做相机标定和外参，
本周不涉及，也不能把 `u、v` 直接当运动指令发给底盘。

## 任务 3：ROS2 通信

环境：**WSL2 Ubuntu 22.04.5 + ROS2 Humble + colcon 0.21.3**。节点用的 python3.10 里
`import cv2` 直接可用（系统包 `python3-opencv 4.5.4`），所以视觉检测和 ROS2 跑在同一个解释器里，
不需要跨进程搬运图片。

### 工作空间与包结构

```text
src/ros2ws/                       ← colcon 在这里执行
├── src/
│   ├── target_interfaces/        ament_cmake 接口包，只放消息
│   │   ├── package.xml  CMakeLists.txt
│   │   └── msg/TargetPosition.msg
│   └── target_pubsub/            ament_python 节点包
│       ├── package.xml  setup.py  setup.cfg
│       ├── images/               orange1~3.jpg（发布节点读这里）
│       └── target_pubsub/
│           ├── __init__.py
│           ├── publisher_node.py     检测 + 发布
│           └── subscriber_node.py    订阅 + 打印
├── build/  install/  log/        colcon 产物（提交时可删）
```

### 消息定义 `TargetPosition.msg`

```
bool detected
float32 u
float32 v
float32 area
```

| 字段 | 含义 |
| --- | --- |
| `detected` | 本帧是否检测到目标。**false 时 u/v/area 一律填 0**，用它判断这一帧有效无效 |
| `u` | 目标中心的图像横坐标（像素），向右增大 |
| `v` | 目标中心的图像纵坐标（像素），向下增大 |
| `area` | 目标轮廓面积（像素²，`cv2.contourArea` 的近似值） |

`package.xml` 里除了 `rosidl_default_generators` / `rosidl_default_runtime`，还必须有
`<member_of_group>rosidl_interface_packages</member_of_group>`；
`CMakeLists.txt` 里用 `rosidl_generate_interfaces()` 指向 `msg/TargetPosition.msg`，缺一项消息就生成不出来。

### 两个节点

| 节点 | 命令 | 做什么 |
| --- | --- | --- |
| 发布 | `ros2 run target_pubsub publisher_node` | 每 0.5 s 处理一张图（约 2 Hz），按 `orange1 → orange2 → orange3` 循环读 `images/` 下的图；检测流程和阈值与 `task.py` 完全一致（H 10~20 / S≥170 / V≥150、开 7×7×1 + 闭 7×7×2、面积≥1000、取最大轮廓、矩求中心），结果发到 `/target_position` |
| 订阅 | `ros2 run target_pubsub subscriber_node` | 订阅同一话题，打印 `Received: detected=..., u=..., v=..., area=...`；只用 `detected` 判断有效性，不看 u/v 是否为 0 |

图片路径用 `Path(__file__).resolve().parent.parent / "images"` 定位到**包内的 `images/` 目录**，
是相对包的位置，不依赖某台机器的绝对路径——只要按 `colcon build --symlink-install` 编译，
放在任何工作空间里都能直接跑（原因见上面编译一节的说明）。

### 编译与验证

```bash
cd src/ros2ws
colcon build --symlink-install   
source install/setup.bash          
ros2 interface show target_interfaces/msg/TargetPosition
ros2 topic info /target_position
ros2 topic echo /target_position
```

> `--symlink-install` 不能省：发布节点用 `Path(__file__).resolve().parent.parent / "images"`
> 找图片，符号链接安装时 `__file__` 仍指向**源码目录**，所以能找到 `src/target_pubsub/images/`；
> 如果用普通 `colcon build`，代码会被复制到 `install/target_pubsub/lib/python3.10/site-packages/`，
> 那个位置的同级没有 `images/`，`glob` 返回空，节点会一直发 `detected=False`。

**实测现象**（截图见根目录 `屏幕截图 *.png`）：

| 验证项 | 结果 |
| --- | --- |
| 消息定义 | `ros2 interface show` 显示 4 个字段（`bool` + 3 个 `float32`） |
| 编译 | 两个包都通过，`target_interfaces` 生成消息、`target_pubsub` 生成两个可执行节点 |
| 话题类型 | `Type: target_interfaces/msg/TargetPosition`，发布端与订阅端都在线 |
| 收发 | 发布端每 0.5 s 打印一条 `Publishing: detected=..., u=..., v=..., area=...`，订阅端同步打印 `Received: ...` |
| 数据来源 | 三条消息的数值就是 `task.py` 在对应图片上的检测结果：`(1507, 3024, 166662)`、`(1724, 2877, 348902)`、`(2151, 2199, 73188)` |
| 未检测到目标 | 某一帧没有候选轮廓时，`detected=False` 且 `u=v=area=0`，订阅端仍然照常打印，不会把"没检测到"当成"目标在左上角" |

### 为什么 `u、v` 不能直接当机器人运动指令

`u、v` 表示"目标落在这张图的第几列第几行"，它同时取决于目标真实位置、相机装在车上的位置与角度、
镜头焦距和畸变、以及目标离相机多远。同一颗球，相机转 10° 或者车往前开半米，`u、v` 就全变了。
要变成机器人能用的坐标，至少还需要相机内参标定、外参标定和深度信息。

## 问题与解决过程

**一开始检出的不是橘子，而是整只手。**

- **现象**：用 `(5,100,100)~(25,255,255)` 这组阈值，三张图都能"检出"，但 orange1 框住的是手掌，
  中心 (1127,3287)、面积 612947 px、圆形度 0.11。
- **怎么查的**：先看 `outputs/orangeX_mask2.png`，掩膜是连成一大片的；再写小脚本在橘子中心和手掌中心
  各取 180×180 像素块打印 HSV 分位数——橘子 H≈14、S≈227、V≈247，手掌 S≈120~150、V≈170~190，
  木桌面 V 也偏低。于是问题锁定在 **S、V 下限太低**，不是 H 范围的问题。
- **原因**：橙色、肤色、木色在色相上几乎重叠（H 都在 10~20），靠 H 分不开，必须靠饱和度和亮度区分。
- **改进**：S 下限 100 → 170，V 下限 100 → 150，H 收窄到 10~20；面积阈值保持 1000，改用"取最大轮廓"。
- **改进后效果**：三张图都能框住橘子（圆形度 0.68 / 0.86 / 0.81），中心落在果皮上；
  但橙色魔方仍在候选里（orange2 有 164750 px），只是面积较小没被选中。

**中文路径读图返回 `None`**（第 1 周遗留）：读图用 `np.fromfile` + `cv2.imdecode`，
写图用 `cv2.imencode` + `tofile`，所以目录名带中文也能正常跑。

**任务 3 的图片路径一开始写成了绝对路径**：发布节点里直接写 `/mnt/c/Users/.../data/images`，
在别人机器上 `glob` 返回空，节点每 0.5 s 发一条 `detected=False` 的消息，看起来"跑起来了但什么都没检测到"。
直接把工作区全部拷过来，改成 `Path(__file__).resolve().parent.parent / "images"`，把图片放进包内 `images/` 目录，
路径就跟着包走，换机器、换工作空间都不用改代码。

## 仍未解决的问题

1. **同色干扰排不干净**：橙色魔方和橘子同色，面积又卡在两个橘子之间
   （小橘子 73188 < 魔方 164750 < 大橘子 348902），纯 HSV/面积没有可用的分界。
   实测两条路可行：把开运算核 7×7 加到 15×15、最小面积提到 20000（魔方被侵蚀掉，但小目标会漏）；
   或者加一个圆形度判断（橘子 0.68~0.86 vs 魔方 0.26）——后者还没写进代码。
2. **阈值是固定的**：S/V 下限写死在 `publisher_node.py` 顶部，换到更暗的环境（整体 V 更低）就会漏检，
   需要按亮度自适应或改用 Otsu；也还没做成 ROS2 参数，现场调参得改代码重新 build。
3. **还没有摄像头实时输入**：现在发布节点是"逐张图片循环"，下一步要换成 `cv2.VideoCapture` 连续取帧。

## 截图

根目录已有的截图（建议按提交规范重命名成 `01_检测输出.png`、`02_结果可视化.png`、
`03_interface_show.png`、`03_pub_sub分屏.png`、`03_topic_info.png`、`03_消息输出.png`）：

```
屏幕截图 2026-11-01 014829.png
屏幕截图 2026-11-02 172157.png
屏幕截图 2026-11-02 194512.png
屏幕截图 2026-11-02 194543.png
屏幕截图 2026-11-02 194710.png
屏幕截图 2026-11-02 195130.png
屏幕截图 2026-11-02 195402.png
```
```
