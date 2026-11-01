# CQuick 视觉组 · 第 2 周考核（重做版）

- 姓名：唐子一（tangziyi）
- 日期：2026-10-28
- 代码：`task1.py`（检测）、`task2.py`（可视化与参数对比）、`task3.py`（ROS2 通信），一个任务一个文件
- 检测部分只用 OpenCV + NumPy，没有引入别的库（task3 的 rclpy 是 ROS2 通信必需的）

---

## 0. 先说一件事：ROS2 Humble 装没装

**结论：这台电脑没装 ROS2 Humble，也没 source。** `check_env.py` 的输出就是证据：

```text
  Python           3.11.4  (D:\python311\python.exe)
  OpenCV           5.0.0
  NumPy            2.4.6
  ROS_DISTRO       （空，没有 source）
  ros2             未找到（命令不在 PATH 里）
  rclpy            不可导入（ROS2 的 Python 环境没准备好）
  TargetPosition   不可导入（接口包没编译或没 source）
```

还查了这些地方，都没有：

| 检查项 | 结果 |
| --- | --- |
| `echo $ROS_DISTRO` | 空 |
| `which ros2` | 找不到 |
| `C:\dev\ros2*` / `C:\opt\ros*` / `/opt/ros/` | 目录不存在 |
| `python -c "import rclpy"` | ModuleNotFoundError |
| WSL（本来可以装 Ubuntu 版） | `wsl.exe` 被系统安全策略拦截，调不起来 |
| choco / cmake / MSVC(cl) | 都没有 |

我这边的会话装不了它：装 Ubuntu 版要走 WSL（被安全策略拦住），
装 Windows 原生版需要管理员权限 + Visual Studio C++ 工具链 + 几个 GB 下载，
而且官方只把 Windows 列为 Tier-3 支持，`cquick_interfaces` 这种要生成 C 消息的包还得先编 MSVC。
**建议直接按下面的方案 A 装**，和考核要求的 Ubuntu 环境完全一致。

### 方案 A（推荐）：WSL2 + Ubuntu 22.04 + ROS2 Humble

**第一步，在 Windows 上（管理员 PowerShell，装完要重启）：**

```powershell
wsl --install -d Ubuntu-22.04
```

重启后会弹出 Ubuntu 终端，设一个用户名和密码（记住它）。

**第二步，在 Ubuntu 终端里装 ROS2 Humble：**

```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release

# 添加 ROS2 软件源
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
     | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

# 装 Humble 桌面版 + 编译工具 + OpenCV
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-opencv

# 每次开终端自动 source
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 验证
printenv ROS_DISTRO     # 应该输出 humble
```

> 国内网络慢的话，把软件源换成清华镜像再 `apt update`：
> `sudo sed -i 's|http://packages.ros.org/ros2|https://mirrors.tuna.tsinghua.edu.cn/ros2|' /etc/apt/sources.list.d/ros2.list`

**第三步，进到项目目录（WSL 里访问 Windows 桌面就是这个路径）：**

```bash
cd /mnt/c/Users/17986/Desktop/week2test
python3 check_env.py        # ROS_DISTRO 应是 humble，rclpy 应可导入
```

### 方案 B（不推荐）：Windows 原生版

要装 Chocolatey → `choco install ros-humble-desktop`，再装 Visual Studio 2022 的
“使用 C++ 的桌面开发”工作负载（`cquick_interfaces` 生成消息要用 MSVC），
然后 `call C:\dev\ros2_humble\local_setup.bat`。
坑多、体积大，而且和考核要求的 Ubuntu 环境不一致，**除非你只能用 Windows，否则别选这个**。

---

## 1. 目录结构

```text
week2test/
├── README.md                    本文档
├── check_env.py                 环境自检（ROS2 / OpenCV / 消息包）
├── task1.py                     ★ 任务1：HSV 分割 + 轮廓筛选 + 中心点
├── task2.py                     ★ 任务2：可视化 + 参数对比（复用 task1.detect）
├── task3.py                     ★ 任务3：ROS2 发布/订阅两个节点
├── image/                       测试图片 1.png / 2.png / 3.png
├── output/
│   ├── task1/                   掩膜图、结果图、task1_report.md
│   └── task2/                   原图对比图、三张参数对比图、task2_report.md
└── ros2_ws/
    └── src/cquick_interfaces/   任务3 的消息接口包（ament_cmake）
        ├── CMakeLists.txt
        ├── package.xml
        └── msg/TargetPosition.msg
```

## 2. 怎么跑

```bash
cd week2test

python3 check_env.py    # 环境自检
python3 task1.py        # 任务1：检测三张图，打印候选与中心点
python3 task2.py        # 任务2：生成对比图 + 参数对比表
```

`task1.py` 和 `task2.py` 只依赖 OpenCV，**Windows 和 Ubuntu 都能直接跑**（我这边跑通了）。
`task3.py` 需要 ROS2，见第 5 节。

---

## 3. 任务 1：HSV 颜色分割与目标中心点

### 3.1 流程（`task1.py` 的 `detect()` 函数）

```text
读取 BGR 图像（cv2.imdecode，兼容中文路径）
    ↓  cv2.cvtColor(BGR2HSV)
转 HSV
    ↓  cv2.inRange(lower, upper)
二值掩膜 mask（目标 255，背景 0）
    ↓  cv2.morphologyEx 开运算 → 闭运算（椭圆核 7x7）
去噪点 + 补上球面黑斑的小洞
    ↓  cv2.findContours(RETR_EXTERNAL)
外轮廓列表
    ↓  cv2.contourArea(c) >= 3000，并按面积降序排序
候选目标
    ↓  取第 1 个（面积最大）
    ↓  cv2.moments() → m10/m00, m01/m00
中心点 (u, v) 和面积 area
```

### 3.2 README 要求记录的 5 项参数

| 项目 | 取值 | 说明 |
| --- | --- | --- |
| **目标颜色** | 白色足球（低饱和度 + 高亮度） | 足球白底黑斑，白色在 HSV 里没有色相，只能靠“S 低、V 高”描述 |
| **HSV 阈值下界** | `[0, 0, 150]` | H 用全范围 0~180；S 下界 0；V≥150 用来排除阴影和暗草 |
| **HSV 阈值上界** | `[180, 60, 255]` | 关键在 S≤60：草坪的 S 一般在 90~150，S 卡在 60 才能把白球和绿草分开 |
| **最小轮廓面积** | `3000 px²` | 球约 5.6~9.6 万 px²；3000 能滤掉草叶高光噪点，又远小于球 |
| **多候选选择规则** | 按轮廓面积降序，取**面积最大**的那个 | 面积大 = 离镜头近、轮廓完整；比“第一个找到的”稳定 |
| 中心点方法 | 轮廓矩 `m10/m00, m01/m00`；`m00=0` 时退回外接矩形中心 | 比外接矩形中心更抗局部缺口 |

### 3.3 图片尺寸与目标实际中心大致位置

三张图都是 **960×1280（宽×高）**，球在画面水平居中、纵向约 50%~55% 的位置：

| 图片 | 尺寸 | detected | u | v | area (px²) | 候选数 | 目测位置 |
| --- | --- | :---: | ---: | ---: | ---: | ---: | --- |
| `1.png` | 960×1280 | True | 479.3 | 710.5 | 57529 | 3 | 球心在画面正中偏下，直径约 300 px |
| `2.png` | 960×1280 | True | 475.1 | 676.2 | 56549 | 3 | 与 1.png 接近，球略小 |
| `3.png` | 960×1280 | True | 484.5 | 646.6 | 96375 | 3 | 球更大（离镜头更近） |

三张都检测成功（要求至少 2 张），没有出现 `未识别到目标` 的情况。

### 3.4 终端输出（`python3 task1.py`）

```text
==========================================================================
任务 1：HSV 颜色分割与目标中心点检测
==========================================================================
目标颜色 : 白色足球（低饱和度 + 高亮度）
HSV 阈值 : [0, 0, 150] ~ [180, 60, 255]
最小面积 : 3000 px^2
形态学   : 开运算 + 闭运算，椭圆核 7x7
--------------------------------------------------------------------------
[1/3] 1.png  尺寸 960x1280
      候选目标 3 个（按轮廓面积降序）：
         1) area=   57529 px^2  bbox=(x=332, y=582, w=298, h=271)  中心=(479.3, 710.5)
         2) area=   47991 px^2  bbox=(x=0, y=0, w=960, h=312)  中心=(283.1, 187.8)
         3) area=    5184 px^2  bbox=(x=582, y=0, w=188, h=154)  中心=(628.6, 90.1)
      >>> 最终选择第 1 个（面积最大）：u=479.3, v=710.5, area=57529
...
==========================================================================
汇总：3 张图，检测成功 3 张，未识别 0 张
文件      尺寸(WxH)     结果      u         v         area(px^2)
1.png     960x1280      成功      479.3     710.5     57529
2.png     960x1280      成功      475.1     676.2     56549
3.png     960x1280      成功      484.5     646.6     96375
==========================================================================
```

### 3.5 多候选目标怎么处理

程序不是“找到第一个就用”，而是**先把所有面积 ≥3000 的轮廓按面积从大到小排序、逐个打印**，
再说清楚选了哪个。`1.png` 的 3 个候选是：

1. 足球本体，57529 px²，中心 (479, 710) —— 面积最大，**选它**
2. 画面上方的围栏/球网白区，47991 px²，中心 (283, 188) —— 面积第二大，是干扰
3. 右上角一块高光白块，5184 px² —— 面积最小

如果没有目标，会打印 `未识别到目标（detected=False，u=v=area=0）`，
并且 `u/v/area` 全部保持 0，不会返回一个“看起来像坐标”的假值。

### 3.6 会失效的情况

1. **光照变化**：V≥150 是按晴天定死的。傍晚/背光时球面 V 掉到 100 以下，球下半部分被切掉，
   面积变小、中心点上移（第 4.3 节 V≥200 那组就是这个现象的极端版）。
2. **背景同色**：跑道白线、围栏、球网、图上的白色水印，在 HSV 上和球**完全同类**，
   只能靠面积区分；一旦白色背景块比球大，就会误选（4.3 节 A/B 组已复现）。
3. **反光过曝**：阳光直射的草地会出现大片浅色高光，S 掉到 60 以下，整片草地进 mask，球被“吃掉”。
4. **目标被遮挡**：球被挡住一半时外轮廓不再是圆，质心偏向没被挡的一侧；挡太多则面积不够 3000 直接漏检。
5. **多个球同时出现**：本周规则是“只发最大的一个”，第二个球会被忽略。

---

## 4. 任务 2：可视化与参数对比

### 4.1 图上标了什么

`output/task2/<图片名>_original_vs_result.png` 是“原图 | 标注图”并排：

- **绿色轮廓**：真正参与计算的那个外轮廓
- **橙黄外接矩形**：`cv2.boundingRect` 的结果
- **红色十字**：质心 `(u, v)`
- **文字 `u=479, v=710`**：中心点像素坐标（写在框上方，带黑边，不挡球）
- 没检测到时整幅图写 `NO TARGET`（在 `task1.py` 的 `draw_result()` 里）

### 4.2 三组参数对比（每次只改一个参数）

```bash
python3 task2.py        # 三组全跑
python3 task2.py B      # 只跑 B 组
```

**A. 最小轮廓面积 min_area**（HSV 不变）

| min_area | 1.png | 2.png | 3.png | 候选数(1/2/3) | 现象 |
| ---: | --- | --- | --- | --- | --- |
| 300 | (479.3, 710.5) | (475.1, 676.2) | (484.5, 646.6) | 11/12/10 | 中心没变，但草地噪点全变成候选了 |
| 3000（基准） | 同上 | 同上 | 同上 | 3/3/3 | 正常 |
| 60000 | **未识别** | **未识别** | (484.5, 646.6) | 0/0/1 | 1.png(57529)、2.png(56549) 的球比 60000 小，被整个丢掉 |

**B. HSV 饱和度上界 S_max**（V≥150 不变）

| S_max | 1.png 中心 | 1.png 面积 | Δ中心(1/2/3) | 现象 |
| ---: | --- | ---: | --- | --- |
| 40 | (479.4, 710.7) | 57164 | 0.3 / 0.3 / 0.7 | 完全正常，说明基准不算贴边 |
| 60（基准） | (479.3, 710.5) | 57529 | 0 / 0 / 0 | 正常 |
| 100 | (499.2, 569.1) | 198390 | **142.8** / 2.8 / **26.4** | 草地高光也满足 S≤100，背景连成一片，中心点跳到上方 |

**C. 形态学处理**

| 处理方式 | 1.png | 2.png | 3.png | Δ中心(1/2/3) | 现象 |
| --- | --- | --- | --- | --- | --- |
| 不做 | (479.0, 710.8) 58058 | (477.0, 674.8) 61777 | (481.8, 644.4) 103808 | 0.4 / 2.3 / 3.6 | 轮廓边缘长毛刺，面积偏大 |
| 只做开运算 | (479.3, 710.0) 57320 | (475.2, 676.2) 56322 | (484.2, 646.2) 95646 | 0.4 / 0.1 / 0.6 | 噪点没了，和基准几乎一致 |
| 开+闭（基准） | (479.3, 710.5) 57529 | (475.1, 676.2) 56549 | (484.5, 646.6) 96375 | 0 / 0 / 0 | 稳定 |

对比图：`output/task2/param_A_minarea.png`、`param_B_hsv.png`、`param_C_morph.png`
（每张都是“行=参数、列=图片”，未检测到的格子一眼能看出来）。

### 4.3 误检 / 漏检 / 定位偏差的具体案例

- **误检（B 组 S≤100，1.png）**：中心从 (479.3, 710.5) 变成 (499.2, 569.1)，面积 198390（占整幅图 16%），
  比球大 3 倍多，`v` 从 710 跳到 569，框住的其实是上方的围栏/球网。原因：S 上界放太松，
  草地高光也进了 mask，背景连成一大片，比球还大。
- **漏检（A 组 min_area=60000）**：1.png、2.png 直接 `未识别`，`u=v=area=0`，图上只写 `NO TARGET`。
  原因和阈值无关，是面积下限比球本身还大（球只有 5.6 万）。
- **定位偏差（C 组“不做形态学”，3.png）**：中心偏 3.6 px、面积从 96375 涨到 103808。
  原因是掩膜边缘的毛刺把轮廓撑大了；闭运算补洞之后轮廓更接近整圆，质心就回来了。
  另外 1.png 的质心和外接矩形中心差 7.3 px，3.png 差 10.2 px——黑斑分布不均会让质心略偏，
  这也说明“质心法比矩形中心更敏感”。

### 4.4 像素坐标 ≠ 真实坐标

- 原点在**图像左上角**，`u` 向右增大，`v` 向下增大，单位是**像素**。
- `(u, v)` 只是“目标在 960×1280 的图像里占哪一格”，换分辨率、裁剪、缩放都会变：
  同一颗球在 640×480 里可能是 (320, 240)，在 1920×1080 里完全不是这个数。
- 要变成场地/机器人坐标，至少还需要：相机内参（去畸变、焦距、主点）+ 相机外参（装在哪、朝哪）
  + 球的实际尺寸或地面约束。本周没有标定，所以 `v=710` 不能读成“球离我几米”。

---

## 5. 任务 3：ROS2 发布目标位置并订阅验证

### 5.1 消息定义

`ros2_ws/src/cquick_interfaces/msg/TargetPosition.msg`（ament_cmake 接口包）：

```text
bool detected      # 本帧是否检测到目标
float32 u          # 目标中心横坐标（像素）
float32 v          # 目标中心纵坐标（像素）
float32 area       # 目标轮廓面积（像素平方）
```

`detected=false` 时 `u/v/area` 约定全为 0，用 `detected` 判断这一帧是否有效。

### 5.2 编译（在 Ubuntu / WSL 里）

```bash
# 1) 建工作空间，把接口包放进去
mkdir -p ~/ros2_ws/src
cp -r /mnt/c/Users/17986/Desktop/week2test/ros2_ws/src/cquick_interfaces ~/ros2_ws/src/

# 2) 编译 + source
cd ~/ros2_ws
colcon build --packages-select cquick_interfaces
source install/setup.bash

# 3) 检查消息字段
ros2 interface show cquick_interfaces/msg/TargetPosition
```

`ros2 interface show` 应该显示 `bool detected` / `float32 u` / `float32 v` / `float32 area` 四行
（文件开头的注释也会一起显示）。

### 5.3 运行（两个终端，一个文件两个角色）

终端 A（发布端，把任务 1 的检测结果发出去）：

```bash
cd /mnt/c/Users/17986/Desktop/week2test
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
python3 task3.py pub
```

终端 B（订阅端，打印收到的内容）：

```bash
cd /mnt/c/Users/17986/Desktop/week2test
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
python3 task3.py sub
```

预期输出（2 Hz，5 秒就够 10 条）：

```text
图片目录：/mnt/c/Users/17986/Desktop/week2test/image
  1.png    detected=True  u=  479.3 v=  710.5 area=   57529
  2.png    detected=True  u=  475.1 v=  676.2 area=   56549
  3.png    detected=True  u=  484.5 v=  646.6 area=   96375
  1.png[no-target] detected=False  （负样本，用于验证未识别到目标的情况）
[INFO] [target_publisher]: 发布话题 /target_position，类型 cquick_interfaces/msg/TargetPosition，频率 2.0 Hz，共 4 帧轮流发布

[INFO] [target_publisher]: #001 来源=1.png  detected=True  u=479.3 v=710.5 area=57529
[INFO] [target_subscriber]: #001 detected=True  u=479.3 v=710.5 area=57529  TARGET
[INFO] [target_subscriber]: #004 detected=False u=0.0 v=0.0 area=0  NO-TARGET
```

### 5.4 验证命令

```bash
ros2 topic info /target_position      # Type 应为 cquick_interfaces/msg/TargetPosition
ros2 topic echo /target_position      # 看原始消息
ros2 topic hz /target_position        # 应该是 ~2 Hz
```

### 5.5 实际检测结果是怎么接进去的

发布节点启动时**真的跑了一遍 `task1.detect()`**，不是写死的坐标：

```python
for name, img in images:
    res = detect(img)                       # 复用任务1的检测函数
    frames.append((name, res))              # 以后每帧就发这个结果
```

另外补了一帧“负样本”：把第一张图左下角（只有草地、没有球）裁出来当一张“无目标”图，
所以订阅端一定会收到 `detected=False` 的消息，能证明“没检测到”这件事被正确表达，
而不是发一个 `(0,0)` 的假目标。

### 5.6 为什么 u、v 不能直接当运动指令

- `u=479` 只说明球在图像第 479 列，**不是**“球在机器人前面 479 mm”。
- 想让底盘转向球，需要的是相对相机光轴的角度：`θ ≈ atan((u - cx) / fx)`，
  `cx, fx` 来自相机内标定。
- 再叠加相机安装位置/朝向（外参）才能换到机器人坐标系；距离还要靠球的真实尺寸或地面约束。
- 完整链路应该是：`像素坐标 → 标定 → 相机坐标 → 外参 → 底盘坐标 → 控制律 → 速度指令`。
  本周只做了第一步。

---

## 6. 遇到的问题 / 还没解决的问题

**遇到的**

1. `cv2.imread` 读中文路径返回 `None`。项目放在桌面（路径含中文）时必然踩到。
   解决：`task1.py` 里统一用 `np.fromfile` + `cv2.imdecode` 读。
2. `cv2.putText` 写中文全是方块。Hershey 字库没有中文字形。
   解决：图上文字一律用英文（`min_area=60000`、`NO TARGET`），中文只出现在终端和本文档。
3. 一开始把 `min_area` 设成 40000 想演示漏检，结果三张图都还能检测到。
   看了面积才发现球的面积是 5.6 万，得设到 60000 才会真的漏。参数要对着实际尺寸定，不能拍脑袋。
4. 一开始以为“不做形态学”更省事，实测轮廓边缘全是毛刺：3.png 面积从 96375 涨到 103808（多 7433），
   中心偏 3.6 px。闭运算能把球面黑斑造成的小洞补上、轮廓更接近整圆；但核太大又会把球旁边的高光连进来。
   所以最后定的是“先开后闭 + 椭圆核 7×7”。

**还没解决的**

1. 阈值是这三张图的阈值。换球场（人工草偏黄、夜间灯光）大概率要重调，我还没想清楚怎么做自适应阈值而不引入误检。
2. 背景里只要出现比球更大的白色块就会误检（B 组已复现）。想加圆度/长宽比约束，但球被遮挡时圆度也会掉，两者怎么兼顾还没定。
3. 质心和外接矩形中心差 7~10 px，哪个更稳需要多帧数据统计，本周样本量不够下结论。
4. `detected=false` 现在只是“没有面积达标的轮廓”，还没有“像不像一个球”的置信度判断。

---

## 7. 截图清单（对照考核细则）

截图前先跑一句，把姓名和时间戳带上（防抄袭要求）：

```bash
echo "姓名: tangziyi"; date "+%Y-%m-%d %H:%M:%S"
```

| 细则要求 | 用哪个文件/命令 |
| --- | --- |
| 任务1 终端输出 | `python3 task1.py` 的终端（含 3 个候选 + 最终选择） |
| 任务1 至少两张检测结果 | `output/task1/1_result.png`、`2_result.png`、`3_result.png` |
| 任务2 原图 vs 标注图 | `output/task2/1_original_vs_result.png` |
| 任务2 参数 A/B 对比 | `output/task2/param_A_minarea.png`、`param_B_hsv.png` |
| 任务3 `ros2 interface show` | `ros2 interface show cquick_interfaces/msg/TargetPosition` |
| 任务3 发布/订阅同时运行 | 两个终端并排，或 `task3.py pub` + `task3.py sub` |
| 任务3 话题信息 | `ros2 topic info /target_position` |
| 任务3 一条实际检测消息 | `ros2 topic echo /target_position` |
