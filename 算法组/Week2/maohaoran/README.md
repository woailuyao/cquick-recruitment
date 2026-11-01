# 第 2 周考核 · 毛浩然

- 姓名：毛浩然（Mao Haoran）
- 完成日期：2026-10-28
- 环境版本：
  - Ubuntu 22.04.5 LTS（WSL2，WSL 版本 3.0.3）
  - ROS2 Humble
  - Python 3.10.12（WSL）/ Python 3.14.7（Windows，任务 1/2 脚本）
  - OpenCV 4.5.4（WSL，`python3-opencv`）/ OpenCV 5.0.0（Windows，`opencv-python`）
  - NumPy 1.21.5（WSL）/ 2.x（Windows）

> 说明：任务 1/2 是纯 OpenCV 脚本，在 Windows 上运行；任务 3 的 ROS2 节点在 WSL2 的 Ubuntu 22.04 中编译运行。两边的检测逻辑是同一套（`src/color_detector.py` 与 `target_vision/detection.py`）。

---

## 目录结构

```text
算法组/Week2/maohaoran/
├── 01_检测输出.png           任务 1 终端输出（含姓名、时间戳）
├── 02_结果可视化.png         原图 vs 标注图
├── 02_参数对比.png           4 组参数对比汇总
├── 03_interface_show.png     消息定义
├── 03_pub_sub分屏.png        发布端 + 订阅端同时运行
├── 03_topic_info.png         话题信息与频率
├── 03_实际检测消息.png       topic echo 的实际消息
├── data/images/              7 张测试图片
├── outputs/
│   ├── task1/detect_report.txt
│   └── task2/compare_report.txt
├── src/
│   ├── color_detector.py     检测核心：HSV -> mask -> 轮廓 -> 中心点
│   ├── image_io.py           支持中文路径的读写封装
│   ├── make_test_images.py   生成 04~07 号测试图
│   ├── task1_detect.py       任务 1
│   ├── task2_visualize.py    任务 2
│   ├── make_summary_images.py 生成两张汇总图
│   ├── target_interfaces/    ament_cmake 接口包（TargetPosition.msg）
│   └── target_vision/        ament_python 节点包（发布 + 订阅，含测试图）
└── README.md
```

### 复现命令

```bash
# 任务 1、2：需要 Python + opencv-python + numpy
cd 算法组/Week2/maohaoran
python src/task1_detect.py        # 结果写入 outputs/task1/
python src/task2_visualize.py     # 结果写入 outputs/task2/
python src/make_summary_images.py # 结果写入 outputs/summary/

# 任务 3：ROS2 Humble
mkdir -p ~/cquick_ws/src
cp -r src/target_interfaces src/target_vision ~/cquick_ws/src/
cd ~/cquick_ws && colcon build && source install/setup.bash
ros2 launch target_vision target_vision.launch.py

# 另开终端
ros2 topic info /target_position
ros2 topic echo /target_position
ros2 param set /target_publisher h_min 110   # 运行时改阈值
```

---

## 任务 1：颜色目标检测

### 输入 → 处理 → 输出

| 环节 | 具体内容 |
| --- | --- |
| 输入 | `data/images/` 下的 7 张图片（BGR，`cv2.imdecode` 读入） |
| 步骤 1 | `cv2.cvtColor(BGR2HSV)` 转 HSV |
| 步骤 2 | `cv2.inRange` 按上下界生成二值 mask |
| 步骤 3 | 开运算 5×5 去噪点，闭运算 5×5 补小洞 |
| 步骤 4 | `cv2.findContours(RETR_EXTERNAL)` 取外轮廓 |
| 步骤 5 | 面积 `< 2000` 的轮廓丢弃；多个候选取**面积最大**的 |
| 步骤 6 | `cv2.moments` 求 `m10/m00`、`m01/m00` 得到中心点 |
| 输出 | `(u, v)`、面积、候选数；未检测到则输出 `detected=False` |

### 统一验收参数

- 目标颜色：**蓝色**
- HSV 下界：`(100, 80, 40)`，上界：`(130, 255, 255)`
  - OpenCV 的 H 范围是 0~179，不是 0~360
- 最小轮廓面积：`2000` 像素²
- 多候选规则：**取面积最大的轮廓**（面积最大通常最接近真实目标，也避免小噪点）
- 中心点方法：**轮廓矩** `(m10/m00, m01/m00)`；`m00 == 0` 时退化为外接矩形中心

### 测试图片与结果

| 图片 | 尺寸（宽×高） | 场景 | 结果 | 中心点 (u, v) | 面积 |
| --- | --- | --- | --- | --- | --- |
| sample.jpg | 640×480 | 蓝色方块 + 橙色圆，目标清晰 | 检测到 | (320.0, 240.0) | 39994 |
| sample2.jpg | 800×600 | 蓝方块 + 绿圆 + 橙圆，多颜色干扰 | 检测到 | (180.0, 190.0) | 52794 |
| sample3.jpg | 960×720 | 蓝球 + 强高光 + 暗背景 | 检测到 | (479.5, 359.9) | 70660 |
| 04_small.jpg | 800×600 | 半径 32 的小目标（面积约 3116） | 检测到 | (620.0, 140.0) | 3116 |
| 05_similar_bg.jpg | 900×600 | 蓝灰色背景 + 同色干扰块 | 检测到 | (300.0, 300.0) | 23026 |
| 06_no_target.jpg | 640×480 | 只有橙/绿物体 | **未检测到** | — | 0 |
| 07_noisy.jpg | 800×600 | 蓝目标 + 200 个蓝色噪点 | 检测到 | (400.0, 300.1) | 25211 |

图片来源：
- `sample.jpg`、`sample2.jpg`、`sample3.jpg`：第 1 周自制测试图，本次直接复用；
- `04~07`：本次由 `src/make_test_images.py` 程序生成（随机种子固定，可复现），用于覆盖「小目标」「背景同色」「无目标」「噪点」四种场景。

### 算法会在哪些情况下失效（至少 2 个）

1. **光照变化 / 强高光**：高光处饱和度 S 只有 28，低于下限 80，mask 出现空洞（本测试中占目标面积 7.4%）。外轮廓法能保住中心点，但高光再强、把目标边缘切断时就会漏检或中心偏移。
2. **背景同色**：`05_similar_bg.jpg` 的背景 S 约 51~68，一旦把 S 下限降到 40，背景会整体进入 mask，面积 14 万像素的背景成为最大轮廓，中心点跑到 (710, 300)，属于误检。
3. **目标过小 / 太远**：面积小于 2000 时会被面积阈值当成噪点过滤掉。
4. **遮挡**：目标被挡住一部分后，外轮廓面积变小，面积筛选可能失效，中心点也会偏向可见部分。
5. **反光材质颜色偏移**：蓝色表面出现镜面反射时，H 可能跳到别的区间（如偏青或偏白），直接丢出阈值范围。

### 截图

- `01_检测输出.png`：`task1_detect.py` 的终端输出（带姓名和时间戳）
- `outputs/task1/detect_report.txt`：每张图的检测记录（掩膜由脚本重新生成）

---

## 任务 2：可视化与参数对比

### 可视化

`src/task2_visualize.py` 对每张图输出 `outputs/task2/vis_<图名>.png`：

- 绿色：检测到的目标轮廓；
- 蓝色：外接矩形；
- 红色圆点 + 十字线：目标中心；
- 红色文字：`u=..., v=..., area=...`；
- 未检测到时，图中央用红色标注「未检测到目标」。

### 坐标约定

- 图像左上角是原点 `(0, 0)`；
- `u`（或 `x_pixel`）向右增大，`v`（或 `y_pixel`）向下增大；
- 像素坐标只描述目标在**图像**中的位置，不是机器人世界坐标，也不是米制距离；
- 要转成场地/机器人坐标，需要相机内参标定 + 外参 + 几何关系，本周不要求实现。

### 参数对比（4 组）

每组只改一个主要参数，数值来自 `outputs/task2/compare_report.txt`。

#### A1：S 下限 40 vs 80（误检案例）

图片：`05_similar_bg.jpg`，其余参数不变。

| S 下限 | 结果 | 中心点 | 面积 | 候选数 |
| --- | --- | --- | --- | --- |
| 40 | 检测到（**误检**） | (710.0, 300.2) | 144145 | 2 |
| 80 | 检测到（正确） | (300.0, 300.0) | 23026 | 2 |

现象：S 下限 40 时蓝灰色背景通过阈值，背景区域面积远大于目标，按「面积最大」规则被选中，中心点落到背景里。说明 S 下限是区分「彩色目标」和「低饱和背景」的关键。

#### A2：H 下限 100 vs 110（漏检案例）

图片：`05_similar_bg.jpg`。

| H 下限 | 结果 | 中心点 |
| --- | --- | --- |
| 100 | 检测到 | (300.0, 300.0) |
| 110 | **未检测到** | — |

现象：目标蓝色像素的 H 实测为 109，阈值收紧到 110 后整个目标被排除。阈值不是越「精确」越好，必须留出传感器噪声和光照引起的色相漂移余量。

#### B：最小面积 2000 vs 5000（小目标漏检）

图片：`04_small.jpg`，目标真实面积 3116。

| 最小面积 | 结果 |
| --- | --- |
| 2000 | 检测到 (620.0, 140.0) |
| 5000 | 未检测到 |

现象：面积阈值同时承担「去噪」和「保留真目标」两个职责。抬高阈值能压掉噪点，但会把远距离的小目标一起滤掉。

#### C：形态学开运算 关闭 vs 11×11

图片：`07_noisy.jpg`，为了让噪点不被面积阈值提前过滤，本轮把最小面积临时设为 10（这是唯一改动的第二参数，单独说明）。

| 开运算 | 候选轮廓数 | 选中目标面积 |
| --- | --- | --- |
| 关闭 | 100 | 25190 |
| 11×11 | 1 | 25188 |

现象：开运算把直径小于 11 像素的蓝色噪点腐蚀掉，候选从 100 个降到 1 个，同时基本不改变目标面积。代价是目标边缘也会被削掉一点，目标很小的时候要慎用大核。

#### D：目标清晰 vs 强高光

| 图片 | 结果 | 中心点 | 目标面积 | 轮廓内未被选中的像素 |
| --- | --- | --- | --- | --- |
| sample.jpg（清晰） | 检测到 | (320.0, 240.0) | 39994 | 0% |
| sample3.jpg（高光） | 检测到 | (479.5, 359.9) | 70660 | **7.4%** |

现象：高光区 S 低，阈值选不中，mask 内部出现洞。因为 `RETR_EXTERNAL` 只取外轮廓，面积和中心点没受影响，但这也意味着「检测到」不等于「mask 完整」——如果高光切断了目标边缘，结果会立刻变差。

### 截图

- `02_结果可视化.png`：原图与标注图对比（sample.jpg、sample3.jpg）
- `02_参数对比.png`：4 组参数对比汇总（A1、A2、B、C、D 纵向拼接）

---

## 任务 3：ROS2 通信

### 消息定义

接口包：`target_interfaces`（`ament_cmake`）

`msg/TargetPosition.msg`：

```text
bool detected    # 本帧是否检测到目标
float32 u        # 目标中心横坐标（像素）
float32 v        # 目标中心纵坐标（像素）
float32 area     # 目标轮廓面积（像素²）
```

话题：`/target_position`，类型 `target_interfaces/msg/TargetPosition`。

`detected=false` 时 `u`、`v`、`area` 全部填 `0.0`，下游必须用 `detected` 判断本帧是否有效，不能把 `(0,0)` 当成目标位置。

### 节点

- **发布节点** `target_publisher`：
  - 读取包内 `images/` 下的 7 张测试图片，按文件名顺序循环、每次发布 1 张；
  - 对每张图做 HSV 检测，把 `detected / u / v / area` 填入消息；
  - 默认 1 Hz，可用参数 `publish_rate` 调整；
  - HSV 阈值和最小面积全部是 ROS2 参数（加分项）。
- **订阅节点** `target_subscriber`：订阅 `/target_position`，逐条打印四个字段。

### 运行流程

```bash
# 1. 编译
cd ~/cquick_ws
colcon build --symlink-install
source install/setup.bash

# 2. 查看消息定义
ros2 interface show target_interfaces/msg/TargetPosition

# 3. 同时启动发布和订阅
ros2 launch target_vision target_vision.launch.py

# 4. 另开终端检查话题
ros2 topic info /target_position
ros2 topic echo /target_position

# 5. 运行时改阈值（加分项）
ros2 param set /target_publisher h_min 110
```

### 实际检测结果如何接入

发布节点没有写死坐标：`publisher_node.py` 调用 `detection.detect()`，把 `DetectionResult` 的四个字段原样填入 `TargetPosition`。因此改一张图、改一次阈值，订阅端收到的是**当前检测输出**，不是固定值。

### 为什么 u、v 不能直接当机器人运动指令

`u`、`v` 是像素坐标，与相机分辨率、安装角度、镜头焦距、目标距离都有关：同一个像素位置在不同距离下对应完全不同的场地位置。要得到机器人坐标，需要相机标定、坐标变换，并由控制模块结合机器人当前位姿决定运动。直接拿像素差值当速度指令，在目标距离变化时会产生严重偏差。

### 截图

- `03_interface_show.png`：`ros2 interface show` 输出
- `03_pub_sub分屏.png`：发布端（左）与订阅端（右）同时运行，订阅端收到 15 条
- `03_topic_info.png`：Publisher 1 / Subscription 1，实测频率 0.999~1.000 Hz
- `03_实际检测消息.png`：`ros2 topic echo` 收到的实际 `detected/u/v/area`

---

## 问题与解决过程

### 1. OpenCV 在 Windows 下读不了中文路径

- 现象：`cv2.imread(r"D:\W2_毛浩然_视觉算法\data\images\sample.jpg")` 返回 `None`，控制台报 `can't open/read file`，`cv2.imwrite` 也静默失败。
- 排查：用 `os.path.exists()` 确认真实文件存在，说明不是路径写错；把图片复制到纯英文目录后能正常读取，定位为 OpenCV 的 Windows 版本按 ANSI 编码处理路径。
- 解决：新增 `src/image_io.py`，用 `np.fromfile` 读二进制再 `cv2.imdecode` 解码；写出时 `cv2.imencode` + `tofile`。

### 2. ROS2 APT 源签名报错（GPG 过期）

- 现象：`apt-get update` 报 `EXPKEYSIG F42ED6FBAB17C654 Open Robotics`，ROS2 仓库被拒绝。
- 排查：`gpg --show-keys` 查看 keyring，发现公钥 `C1CF6E31...` 的过期时间是 2025-06-01；而 `mirrors.ustc.edu.cn/ros2` 上的 `InRelease` 仍由该 key 签名，签名时间早于过期时间，但 apt 认为 key 已过期。
- 解决：从 rosdistro 仓库拿到**续期版**同一把公钥（有效期到 2030-06-01）替换 keyring，再 `apt-get update` 恢复正常。

### 3. WSL 官方分发安装失败

- 现象：`wsl --install -d Ubuntu-22.04` 报 `raw.githubusercontent.com ... 超时 / 连接被重置`。
- 解决：改为从 `cloud-images.ubuntu.com` 下载官方 rootfs，用 `wsl --import` 手动导入，绕开 Microsoft Store 和 GitHub raw 的网络依赖。

---

## 仍未解决的问题

1. **像素坐标到场地坐标的转换**：本周只有像素 `(u, v)`，没有做相机标定，无法给出目标在场地中的真实位置。
2. **多目标选择规则过于简单**：现在固定选「面积最大」，当真实目标较小、干扰物较大时会被干扰物抢走（A1 对比已经暴露了这个问题）。更稳的做法是结合颜色相似度、形状、连续性或跟踪。
3. **强高光下的 mask 完整性**：目前靠 `RETR_EXTERNAL` 绕过了内部空洞，但如果高光切断目标边缘，检测会直接失败；可能需要自适应阈值或 HDR/多曝光。
