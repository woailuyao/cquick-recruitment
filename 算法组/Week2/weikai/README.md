# 第 2 周考核 · Wei Kai

- 姓名：魏恺（个人目录 `weikai`）
- 完成日期：2026-11-01
- Ubuntu / ROS2 / Python / OpenCV 版本：
  - OS：Windows 10 Pro 26H2（任务 1/2 运行于 conda 环境）+ WSL2 Ubuntu 22.04（任务 3 运行 ROS2）
  - Ubuntu：22.04（WSL2）
  - ROS2：Humble（`ros-humble-ros-base` + `ros-dev-tools`）
  - Python：3.10.22（conda env `cv_practice`）
  - OpenCV：5.0.0.93（`opencv-python-headless`，所用 API 与 4.x 一致）
  - NumPy：2.2.6　Matplotlib：3.10.9

---

## 零、整体数据流

```text
[任务1 输入] data/images/*.jpg
      ↓  imread_safe 读图 → BGR2HSV → inRange 生成 mask → 开/闭运算 → findContours
      ↓  面积筛选 + 贴边轮廓过滤 → 轮廓矩算中心
[任务1 输出] detected / u / v / area   +   outputs/task1_*_mask.png、task1_*_result.png
      ↓  detected/u/v/area 被封装成 detect_target() 的返回值
[任务2 输入] 同一批图片 + detect_target()
      ↓  把轮廓/矩形/中心/坐标文字画回原图；改一个参数重跑做对比
[任务2 输出] outputs/task2_annotated_*.png（标注图）、task2_compare_*.png（参数对比图）
      ↓  同一条检测链路被 vision_detector 包 import
[任务3 输入] 图片路径 + HSV/面积参数（ROS2 参数）
      ↓  publisher 每 100ms 调一次检测，把结果填进 TargetPosition 消息
[任务3 输出] /target_position 话题上的 detected/u/v/area → subscriber 打印
```


---

## 数据与图片来源

测试图为公开图片（下载网站为https://www.pexels.com/zh-cn/，免费使用许可），已等比缩小到长边 1280 后存放在 `data/images/`：

| 本地文件名 | 原图文件 | 内容 / 用途 | 尺寸 |
| --- | --- | --- | --- |
| `test_clear.jpg` | `pexels-ozan-aldemir-2157508360-34912371.jpg` | 足球特写，目标大而清晰（「目标清晰」场景） | 853×1280 |
| `test_small.jpg` | `pexels-omar-ramadan-1739260-9377768.jpg` | 比赛中抢球远景，球在画面上只占几十像素（「目标较小」场景） | 1280×853 |
| `test_similar_bg.jpg` | `pexels-victor-chijioke-350220031-20472553.jpg` | 绿衣球员在绿茵场，画面里有黄底白字广告牌、白色球袜、白线（「背景存在相近颜色/干扰物」场景） | 933×1280 |

> 三张图的球都是**白色足球**，所以目标颜色定为白色而不是某个色相。

---

## 任务 1：颜色目标检测

代码：`src/task1_color_detect.py`（在 `weikai/` 根目录运行：`python src/task1_color_detect.py`）

### 1.1 输入 → 处理 → 输出

- **输入**：`data/images/` 下的 BGR 图片（`imread_safe` 读取，见「问题与解决过程」第 1 条）。
- **处理步骤**：`BGR→HSV` → `cv2.inRange` 生成二值 mask → 形态学**开运算**去噪 → 形态学**闭运算**把球面被黑色贴片切碎的白色块连回整体 → `findContours` 取外轮廓 → **面积筛选**（`>= MIN_AREA`）+ **贴边轮廓过滤**（被图像边缘裁切的物体丢弃）→ `cv2.moments` 算质心 → 按面积降序排序取最大者。
- **输出**：`detected / u / v / area / candidates`，以及每张测试图对应的掩膜与标注结果图：`task1_test_clear_mask.png`、`task1_test_clear_result.png`、`task1_test_small_mask.png`、`task1_test_small_result.png`、`task1_test_similar_bg_mask.png`、`task1_test_similar_bg_result.png`（均位于 `outputs/`）。
- **如何成为下一步输入**：检测逻辑封装为 `detect_target(img, lower, upper, min_area, kernel_size)`，返回字典。任务 2 直接 import 它做可视化和参数对比；任务 3 的 `vision_detector/detector.py` 复用同一套逻辑，把返回值逐字段填进 ROS2 消息。**三个任务共用同一套检测规则，所以三处结果一致。**

### 1.2 统一验收参数

- **目标颜色**：白色（足球本体为白色，含黑色贴片）
- **HSV 阈值上下界**：`lower = (0, 0, 170)`，`upper = (180, 80, 255)`
  - 白色没有固定色相，HSV 里表现为「饱和度低 + 明度高」，因此 **H 取全区间 0~180**；
  - **S 上界 80**：草地绿色的 S 通常在 100 以上，用 80 能把绿茵场排除掉；
  - **V 下界 170**：球面阴影部分实测 V 只有 150~190，所以不能取到 200 以上，否则球的一半会被切掉。
- **最小轮廓面积**：`MIN_AREA = 500`（像素平方），小于此值当噪声丢弃
- **多候选选择规则**：先丢弃 `面积 < 500` 的轮廓，再丢弃**外接矩形触碰图像边界**的轮廓（球门柱、围栏杆、延伸出画面的白线属于被裁切的不完整目标），剩下的按**面积从大到小排序，取面积最大者**为主目标，并打印候选数量
- **中心点计算方法**：`cv2.moments` 轮廓矩求质心 `u = m10/m00, v = m01/m00`；当 `m00 == 0`（面积近似为 0 的退化轮廓）时**退回外接矩形中心** `x + w/2, y + h/2`，避免除零
- **坐标约定**：图像左上角为原点 `(0,0)`，`u` 向右增大、`v` 向下增大；

### 1.3 测试图片结果

| 图片 | 尺寸 (宽×高) | 目标大致位置 | detected | (u, v) | area | 候选数 |
| --- | --- | --- | --- | --- | --- | --- |
| `test_clear.jpg`（特写） | 853×1280 | 画面中部偏下，球占画幅比例大（白色区域约 6 万像素） | True | (404, 682) | 59781 | 3 |
| `test_small.jpg`（远景） | 1280×853 | 画面下部中央，球直径约 50px | True | (598, 682) | 2000 | 5 |
| `test_similar_bg.jpg`（绿衣球员） | 933×1280 | 球在左下（约 u≈120, v≈900） | True → **见下方说明** | (207, 605) | 19494 | 22 |


第三张是**真实的误检案例**，必须如实说明：该图中的球被泥土污染且处于阴影中，实测 HSV 中位数 `S≈131, V≈154`——完全不落在白色阈值范围内，所以球本身检测不到；而画面左侧广告牌上的白色文字反而是「低饱和 + 高亮度」，面积 19494 最大，于是被当成了目标。

### 1.4 算法会在什么情况下失效

1. **光照/阴影**：球面暗部 V 掉到 170 以下就被 mask 切掉，轮廓只剩亮部，中心点会向亮侧偏移；强烈逆光时整个球丢失。
2. **球面图案/遮挡**：足球的黑色贴片本身不是白色，轮廓被切成几块（闭运算只是把它们勉强连起来）；球被腿或草遮住一半时，面积和中心都会失真。
3. **背景同色干扰**：球门柱、场地白线、广告牌白字、白色球袜都是「低饱和高亮度」，与白色球在同一阈值内，只能靠面积和贴边过滤勉强区分——`test_similar_bg.jpg` 就是被广告牌白字抢走了。
4. **目标过小**：远景下球只有 2000 像素面积，一旦 `min_area` 调到 5000 就直接漏检。

---

## 任务 2：可视化与参数对比

代码：`src/task2_visualize_compare.py`（复用任务 1 的 `detect_target` / `draw_result`，保证两任务检测逻辑完全一致）

### 2.1 输入 → 处理 → 输出

- **输入**：与任务 1 相同的图片 + `detect_target()` 的返回结果。
- **处理**：把轮廓（绿）、外接矩形（蓝）、中心点（红十字+红点）、坐标文字 `u=..., v=...` 画回原图；再**每次只改一个参数**重跑，生成左右并排对比图。未检测到时在图上打印 `NOT DETECTED`，不只靠终端提示。
- **输出**：
  - `outputs/task2_annotated_test_clear.png`、`task2_annotated_test_small.png`、`task2_annotated_test_similar_bg.png` —— 3 张图的标注结果（可视化验收点）
  - `outputs/task2_compare_hsv.png` —— 参数对比 A
  - `outputs/task2_compare_minarea.png` —— 参数对比 B
  - `outputs/task2_false_detection.png` —— 误检案例
  - `outputs/task2_miss_detection.png` —— 漏检案例


### 2.2 参数对比 A：HSV 的 S/V 宽 vs 窄（只改 S 上界和 V 下界，H 不变）

在 `test_clear.jpg`（特写）上实验：

| 参数 | 掩膜像素 | 候选数 | 中心 (u, v) |
| --- | --- | --- | --- |
| 宽：`S<80, V>170`（当前默认） | 50639 | 3 | (404, 682) |
| 窄：`S<30, V>210` | 16916 | 2 | (426, 617) |

**具体变化**：掩膜像素减少 **66%**，球体边缘和阴影区域被整片切掉，轮廓不再是一个完整圆而只剩高光部分；中心点从 (404,682) 跳到 (426,617)，**Δu=+22、Δv=−65**——位置误差接近球半径。结论：V 下界抬高会让「定位偏差」明显增大，这比「检测不到」更危险，因为它仍返回 detected=True，下游会以为坐标可信。

### 2.3 参数对比 B：最小轮廓面积 小 vs 大（只改 `min_area`）

| 参数 | detected | 说明 |
| --- | --- | --- |
| `min_area = 50` | True | 球被检出（特写球面积 59781，远大于阈值） |
| `min_area = 80000` | **False** | 阈值大于球的实际面积，球被当成噪声整块丢弃 → 漏检 |

**具体变化**：面积阈值是「一刀切」的硬门槛，超过目标自身面积就必然漏检；反过来取太小（见 2.4）会把噪点全放进来。

### 2.4 误检案例

**（a）真实照片误检**：`test_similar_bg.jpg` 中检测框落在左侧广告牌的白色文字上（area=19494，候选 22 个），真正的球在画面左下（约 u≈120, v≈900）却没有检出。原因：白色文字与球同为「低饱和高亮度」，而文字面积更大，按「取最大面积」规则胜出；球因泥土污染 `S≈131` 直接被阈值排除。

**（b）可控复现的误检**（`task2_false_detection.png`）：合成一张暗灰背景（V=120，不在白色范围）+ 一个白色大圆 + 10 个半径 7~10 的白色小噪点（模拟反光点）：

- `min_area = 20` → 框出 **9 个**（1 个真目标 + 8 个噪点误判）
- `min_area = 500` → 框出 **1 个**（只有真目标）

结论：`min_area` 过小会把反光/噪点当成目标，必须按目标尺度取合理阈值。

### 2.5 漏检 / 定位偏差案例

**（a）真实照片漏检**：同上 `test_similar_bg.jpg` 的球未检出（原因见 2.4a）。

**（b）可控复现的漏检**（`task2_miss_detection.png`）：合成一张只有半径 12 的小白球（面积约 452）的图：

- `min_area = 500` → 框出 **0 个**（小目标被当噪声漏掉）
- `min_area = 50` → 框出 **1 个**（检出）

**（c）定位偏差**：见 2.2，HSV 取窄后球心从 (404,682) 偏到 (426,617)，detected 仍为 True——这是「看起来正常、坐标其实错了」的典型情况。

### 2.6 坐标约定（验收点）

- 图像左上角是原点 `(0, 0)`；
- `u`（等价 `x_pixel`）向右增大，`v`（等价 `y_pixel`）向下增大；
- 像素坐标只描述目标**在图像中的位置**，**不是机器人世界坐标，也不是米制距离**；
- 想换算成场地坐标/距离，需要相机内参标定、安装位置与被摄平面的几何关系等信息，本周不要求实现——所以任务 3 里 `u/v` 只能作为「目标偏左还是偏右、偏上还是偏下」的**方向性依据**。

---

## 任务 3：ROS2 通信

代码：`src/target_position_msgs/`（接口包）+ `src/vision_detector/`（节点包）

### 3.1 消息字段（`src/target_position_msgs/msg/TargetPosition.msg`）

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `detected` | `bool` | 本帧是否检测到目标（有效性标志，最重要） |
| `u` | `float32` | 目标中心水平像素坐标（向右为正，左上角原点） |
| `v` | `float32` | 目标中心垂直像素坐标（向下为正，左上角原点） |
| `area` | `float32` | 目标轮廓面积（像素平方，近似值） |

**关键约定**：`detected = false` 时 `u / v / area` 一律填 **0**，订阅方**必须靠 `detected` 判断这一帧是否有效**，绝不能把 0 当成「目标位于图像左上角」——否则机器人会朝一个不存在的目标跑。

### 3.2 话题名

`/target_position`，类型 `target_position_msgs/msg/TargetPosition`（发布端与订阅端必须一致）。

### 3.3 输入 → 处理 → 输出

- **输入**：ROS2 参数 `image_path`（图片路径）+ `hsv_lower` / `hsv_upper` / `min_area` / `rate` / `demo`。
- **处理（发布节点 `publisher.py`）**：
  - 定时器每 `1/rate` 秒（默认 10Hz）触发一次回调；
  - `image_path` 非空 → 调 `detector.detect_from_file()` 读图并检测，把真实 `detected/u/v/area` 填进消息（**这就是「接入实际检测结果」**）；
  - `image_path` 为空 → **演示模式**发固定数据 `detected=True, u=320, v=240, area=26608`，并且**每第 10 条故意发一次 `detected=False`**，用来验证「未检测到」的约定能正确传出去。
- **输出**：`/target_position` 上的 `TargetPosition` 消息；终端同步打印 `发布 #n detected=True u=... v=... area=...`。
- **订阅（`subscriber.py`）**：收到后打印 4 个字段；`detected=False` 时用警告级日志明确写「无效目标，u/v/area 为默认 0，不能当作运动指令」。

### 3.4 包结构与构建

```text
src/
├── target_position_msgs/        # ament_cmake：只定义消息，不跑代码
│   ├── package.xml
│   ├── CMakeLists.txt
│   └── msg/TargetPosition.msg
└── vision_detector/             # ament_python：真正干活
    ├── package.xml
    ├── setup.py / setup.cfg
    ├── resource/vision_detector     # ament 索引标记文件（空文件，规范要求）
    └── vision_detector/             # Python 模块目录
        ├── detector.py              # 检测函数（复用任务1 逻辑，cv2 惰性导入）
        ├── publisher.py             # 发布节点
        └── subscriber.py            # 订阅节点
```

在 WSL2 的 Ubuntu 里编译：

```bash
# 1) 在 weikai/ 目录下执行（相对路径，不必改成自己机器的绝对路径）
mkdir -p ~/ros2_ws/src
cp -r src/target_position_msgs ~/ros2_ws/src/
cp -r src/vision_detector      ~/ros2_ws/src/

# 2) 编译
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

# 3) 验证消息定义（4 个字段）
ros2 interface show target_position_msgs/msg/TargetPosition
```


### 3.5 运行与验证

```bash
# 终端 A：发布（默认演示模式，先验证通信是否通）
ros2 run vision_detector publisher
# 终端 B：订阅
ros2 run vision_detector subscriber
# 终端 C：核对话题与类型
ros2 topic info /target_position          # Type: target_position_msgs/msg/TargetPosition
ros2 topic echo /target_position          # 应连续收到 ≥10 条
```

订阅端预期输出：

```text
收到 #1 detected=True u=320.0 v=240.0 area=26608.0
...
收到 #10 detected=False -> 无效目标，u/v/area 为默认 0，不能当作运动指令！
```

### 3.6 接入实际检测结果（任务 3 最终要求）

```bash
IMG=$(realpath data/images/test_clear.jpg)
ros2 run vision_detector publisher --ros-args -p image_path:="$IMG"
```

此时发布端按 `rate` 参数（默认 10Hz）读图并检测，`u/v/area` 来自任务 1 同一条 HSV 检测链路；若换成检测不到目标的图，则发出 `detected=False, u=v=area=0`。

**注意：WSL 实测值与任务 1（Windows）略有差异，属正常现象。** 同一张 `test_clear.jpg`：
- Windows（OpenCV 5.0.0.93）：`u≈404, v≈682, area≈59781`
- WSL Ubuntu（ROS2 自带 python3-opencv 4.5.x）：`u≈411.3, v≈678.0, area≈47835.0`

原因是两个平台的 OpenCV 版本不同，JPEG 解码与形态学实现有细微差别，而白球边缘存在大量处于阈值附近的像素，解码差异会让部分像素进出 mask，面积与质心随之偏移。算法、HSV 参数、图片完全一致——这说明数值结果依赖具体的库版本，部署时应在目标平台上重新验证阈值。

### 3.7 为什么 `u/v` 不能直接当机器人运动指令

1. `u/v` 是**像素**，随分辨率、镜头焦距、安装角度变化——同一目标在 853×1280 与 1280×853 的图上坐标完全不同，没有物理意义；
2. 像素位置只说明「目标在视野中的方向」，不含**距离**信息：目标在 u=400 时可能离机器人 1 米，也可能 3 米；
3. `detected=False` 时 `u/v` 是占位的 0，若直接当坐标用，机器人会朝左上角（图像原点）运动；
4. 正确做法是先用相机标定把像素映射到地面平面坐标（或结合目标实际尺寸估算距离），再由控制模块做决策——本周不要求实现。

---

## 问题与解决过程

**问题 1：`cv2.imread` 读不到图片，`cv2.imwrite` 也存不下（返回 True 但文件不存在）**

- 现象：脚本里 `cv2.imread('.../视觉组/Week2/weikai/data/images/test_clear.jpg')` 返回 `None`，报 `读取失败`；改成 `assert` 后定位到「文件确实存在但读不到」。写图时更隐蔽：`cv2.imwrite` 不报错，但 `outputs/` 里根本没有生成 `task2_annotated_*.png`。
- 排查：先 `os.path.exists()` 确认文件在（在），再写了一个最小脚本分别试 `cv2.imread(p)` 与 `np.fromfile(p)+cv2.imdecode()`——前者返回 None，后者正常；`cv2.imwrite` 同理，`cv2.imencode()+tofile()` 正常。
- 原因：路径中含中文「视觉组」，OpenCV 的 C++ 文件层按本地编码打开文件，与 Python 的 Unicode 路径不一致（终端里路径也被打成乱码可作旁证）。
- 解决：封装 `imread_safe()` / `imwrite_safe()`，统一用 `np.fromfile` + `cv2.imdecode` 读、`cv2.imencode` + `buf.tofile` 写。**所有读写都走这两个函数**，之后 13 张结果图全部正常落盘。

**问题 2：改成白色目标后，检测框总是落在球门柱 / 广告牌上，而不是球**

- 现象：`test_clear.jpg` 输出的中心点在 `(4, 300)`——贴着图像左边缘，明显不是球。
- 排查：打印所有候选的面积/中心，发现排第一的是球门柱（面积 3458，外接矩形左侧 x=0），球实际是第 2 候选（面积 2000）。
- 原因：球门柱、围栏杆、场地白线末端都被图像边界裁切，**轮廓不完整但面积仍然很大**，在「取最大面积」规则下会压倒真正的球。
- 解决：加一条「贴边轮廓过滤」——`boundingRect` 的任一方向触到图像边界就丢弃（被裁切的目标中心不可信）。加上之后球自然成为最大候选，`test_clear` / `test_small` 都能正确检出。


---

## 仍未解决的问题

1. **输出的中心点不在球的真实圆心上。** 这里算的是「白色像素的质心」，而球面黑色贴片（V 低）和下半球阴影/泥点（V<170）都被白色阈值挖掉了，白色区域集中在球的上半部，质心就被拉向亮部。实测 `test_clear.jpg` 中心落在 (404, 682)，位于球的上半部：绿色轮廓只描到球的上 2/3，蓝色外接矩形也罩不住整个球，十字偏离球心约 90 像素。本周的定位只用了颜色单特征，定位对象实际上是「白色区域」而不是「球」；改进方向是加入形状特征（例如在候选局部用霍夫圆或凸包最小外接圆做二级定位，把中心修正到几何中心），但会引入新的参数和失败情况，本周未实现，只在此记录。
2. **白色目标的 HSV 阈值对光照过于敏感。** 同一个白球，特写时光照明亮（V 中位数约 160~185），远景阴影下 V 掉到 132、S 反而升到 131（泥土+阴影），单一固定阈值无法同时覆盖两种场景。
3. **多帧之间中心点会跳变的问题。** 本周是逐张静态图片检测，如果换成摄像头连续帧，候选轮廓可能在「球」和「球旁边的高光点」之间来回切换，导致 `u/v` 抖动。

---

## 附：目录结构

```text
weikai/
├── README.md                          # 本文件
├── .gitignore                         # 忽略 __pycache__ / build / install / log
├── data/images/                       # 3 张测试图片（公开来源，见上表）
├── outputs/                           # 程序生成的掩膜图、标注图、参数对比图（共 13 张，超大图已等比缩小以控制仓库体积）
├── 01_检测输出.png                    # 提交截图：任务1 终端输出（含姓名+时间戳）
├── 02_结果可视化.png                  # 提交截图：原图 vs 标注图
├── 02_参数对比.png                    # 提交截图：参数 A/B 对比
├── 03_interface_show.png              # 提交截图：ros2 interface show
├── 03_pub_sub分屏.png                 # 提交截图：发布端与订阅端同时运行
└── src/
    ├── task1_color_detect.py          # 任务1：HSV 分割 + 轮廓 + 中心点
    ├── task2_visualize_compare.py     # 任务2：可视化 + 参数对比 + 误检/漏检演示
    ├── target_position_msgs/          # 任务3 接口包（ament_cmake）
    └── vision_detector/               # 任务3 视觉节点包（ament_python）
```
