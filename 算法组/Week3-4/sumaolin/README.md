# 第 3-4 周提交说明 · 粟茂林

## 一、基本信息

- 姓名拼音：`sumaolin`
- 姓名：粟茂林
- 完成时间：2026-11-20
- 系统：Windows 11
- Python：3.11.9
- 主要依赖：Ultralytics 8.4.215、PyTorch 2.15.0+cpu、OpenCV 5.1.0、NumPy 2.4.6
- ROS2：Humble（WSL2 Ubuntu 22.04，`scr/task3_ros2`、`scr/task3_interfaces`）

## 二、目录结构

```text
sumaolin/
├── README.md
├── scr/
│   ├── task1.py              # 足球检测
│   ├── task2.py              # 标志地面坐标
│   ├── task3.py              # 分屏定位 + PnP
│   ├── ball.pt（在网盘）
│   ├── marker.pt（在网盘）
│   ├── task3_interfaces/     # ROS2 消息包
│   └── task3_ros2/           # ROS2 节点/监听器
├── config/
│   ├── camera_intrinsics.yaml
│   ├── field.yaml
│   └── field_points.png
├── calibration/
│   ├── README.md
│   └── detected_*.png
├── dataset/
│   ├── ball_dataset/ （在网盘）
│   ├── marker_dataset/ （在网盘）
│   ├── train_summary.md
│   ├── ball_train.py
│   └── marker_train.py
├── tools/
│   ├── annotate_markers.py
│   └── 标注工具使用说明.md
├── 标注_train.bat
├── 标注_val.bat
└── 标注_test.bat
```

我用夸克网盘给你分享了「.pt文件及原始数据集.zip」，点击链接或复制整段内容，打开「夸克APP」即可获取。
/~25163cQdJ0~:/
链接：https://pan.quark.cn/s/991d94535d29

哔哩哔哩 https://b23.tv/kcqathp
## 三、环境与运行

Windows 任务使用仓库内虚拟环境：

```powershell
tools\venv\Scripts\python.exe scr\task1.py
tools\venv\Scripts\python.exe scr\task2.py
tools\venv\Scripts\python.exe scr\task3.py
```

三个任务都支持：

- `--source auto`：自动扫描摄像头，优先使用最后识别到的 USB 摄像头；
- `--source 0/1/...`：指定摄像头编号；
- `--source 视频路径`：离线视频；
- `--source http://...`：MJPEG/RTSP 流；
- `--list-cameras`：列出可用摄像头；
- `--device cpu`：CPU 推理。

所有输出画面带粟茂林拼音 `sumaolin` 和逐帧时间戳。

## 四、相机标定

标定文件：`config/camera_intrinsics.yaml`

| 参数 | 值 |
|---|---:|
| 分辨率 | 1280×720 |
| fx | 1563.6194 |
| fy | 1632.7168 |
| cx | 640.0 |
| cy | 360.0 |
| 畸变 `[k1,k2,p1,p2,k3]` | `[0.252780, -1.094336, -0.032371, 0.004958, 0.0]` |
| 重投影 RMS | 0.854 px（68 个视图） |

标定板：8×6 方格、7×5 内角点；每 2 帧抽帧，保留清晰视图，固定主点到图像中心。详细过程见 `calibration/README.md`。

**未完成的是安装外参**：光心高度、俯角、横滚、偏航、相对载体偏移没有实测。task2 的地面投影只能用占位外参做流程验证；task3 的 PnP 模式不依赖安装外参，只使用内参和畸变。

## 五、任务 1 · 足球检测

- 模型：`scr/ball.pt`，YOLOv8n；
- 数据：`dataset/ball_dataset`，216 张，train 153 / val 27 / test 36；
- 训练：640、100 epochs、batch 16；
- 验证集 mAP50：**0.995**。

运行：

```powershell
tools\venv\Scripts\python.exe scr\task1.py --source auto
tools\venv\Scripts\python.exe scr\task1.py --source "视频.mp4" --conf 0.25
```

输出：实时画面画球框、置信度、球心像素坐标，并显示姓名和时间戳。

限制：0.995 是同一批场地视频抽帧的验证结果，跨相机、强逆光、球被遮挡时不能保证。

## 六、任务 2 · 标志地面坐标

- 模型：`scr/marker.pt`；
- 输入：`marker.pt` 的 L/T/X 检测框；
- 使用内参 + 畸变 + 安装外参，把像素点投影到地面；
- 输出每个标志的场地坐标和类型；
- 点球点 `P` 已从类别、场地 YAML、代码和消息定义中删除。

运行：

```powershell
tools\venv\Scripts\python.exe scr\task2.py --source auto
tools\venv\Scripts\python.exe scr\task2.py --source "视频.mp4" --conf 0.10
```

注意：task2 依赖安装外参。当前只完成内参标定，外参仍是占位值，所以地面坐标只能作为流程验证，不能作为最终米制定位结果。

## 七、任务 3 · 分屏定位

- 左屏：原画面 + 标志/足球检测框 + 像素坐标；
- 右屏：场地地图 + 相机位姿/地图点；
- 每秒输出一行 JSON；
- 保留 `plane` 和 `pnp` 两种位姿模式；
- `pnp` 模式只使用内参/畸变，**不需要安装外参**；
- 点球点 `P` 已从 `config/field.yaml`、task2、task3 和消息定义中删除。

运行：

```powershell
tools\venv\Scripts\python.exe scr\task3.py --source auto --pose-mode pnp
tools\venv\Scripts\python.exe scr\task3.py --source "视频.mp4" --pose-mode pnp --marker-conf 0.10
```

场地坐标：`config/field.yaml`，共 14 个点：

- 6 个 T：中线/边线、球门线/禁区线、球门线/球门区线；
- 6 个 L：场角、禁区前沿、球门区前沿；
- 2 个 X：中圈与中线交点。

原点取两个 X 的中点，x 沿中线，y 指向球门线。半场宽 6.05 m、半场长 3.9 m、中圈直径 1.45 m、禁区宽 3.95 m、球门区宽 2.95 m，实测深度分别为 1.86 m 和 0.935 m。

### PnP 说明

`pnp` 模式枚举检测与地图点的 4 点组合，使用 `cv2.solvePnP`（ITERATIVE）求解，再用重投影匹配、内点数和场地边界筛选。

### 时序消歧

当前帧存在多个 4 点解、且上一帧有有效位姿时：

- 在所有内点 ≥4、残差 ≤8 px 的候选解里，找与上一帧位置差 ≤0.45 m、偏航差 ≤15° 的解；
- 有接近解就选它，输出 `valid=true`、`temporal=true`；
- 没有接近解才保持 `ambiguous` / `valid=false`，不强行输出错误位姿。

上一帧有效位姿最多保留 8 帧；超过 8 帧没有有效解就清空先验。离线统计中，423 个 ≥4 检测帧从 42 个 valid 提升到 44 个（1 s 采样间隔大，提升有限）；30 fps 连续视频上会按相邻帧生效。

当前限制：

- PnP 至少需要 4 个可靠标志；
- 4 个共面点存在镜像/多解，容易出现 `ambiguous`；
- 当前 marker 已经能在很多帧给出 4 个以上检测，但 4 点解仍大量歧义：647 个采样帧里 423 帧有 ≥4 个检测，只有 44 帧 `valid=true`（10.4%），285 帧 `ambiguous`；
- 手持视频 1555 帧里 marker 框 3434 个、363 帧有 ≥4 个检测、28 帧 PnP `valid=true`；
- 主要瓶颈已经从“检不出”变成“4 点共面多解 + 视频模糊”。

## 八、ROS2 封装

两个包都放在 `scr/` 下，不复制 `task3.py`、`marker.pt`、`ball.pt`：

- `task3_interfaces`（ament_cmake）：`Localization.msg`、`MarkerDetection.msg`、`BallDetection.msg`；
- `task3_ros2`（ament_python）：`task3_node` 发布 `/task3/localization`，`task3_listener` 订阅打印。

`task3_node` 通过参数 `task3_root` 找到源码目录，用 `importlib` 加载 `task3.py`，复用其中的标定、PnP 和绘制函数。

WSL 构建：

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --base-paths /mnt/c/Users/cijiz/cquick-recruitment/算法组/Week3-4/sumaolin/scr \
             --packages-select task3_interfaces task3_ros2
source install/setup.bash
```

运行：

```bash
# 终端 A
ros2 run task3_ros2 task3_listener

# 终端 B，在 scr 目录下运行，或显式指定 task3_root
ros2 run task3_ros2 task3_node --ros-args \
  -p task3_root:=$PWD \
  -p pose_mode:=pnp \
  -p source:=auto
```

说明：ROS2 包代码和 XML 已做语法检查；本次没有在 WSL 里完成 `colcon build` 和实机运行验收，WSL 侧摄像头也需要 usbipd 或 MJPEG 桥接。

## 九、数据与训练

完整数据卡、训练参数和指标见 [`dataset/train_summary.md`](dataset/train_summary.md)。

当前 `dataset/marker_dataset`：

| split | images | L | T | X |
|---|---:|---:|---:|---:|
| train | 116 | 263 | 190 | 108 |
| val | 13 | 39 | 33 | 13 |
| test | 17 | 36 | 29 | 19 |

当前 `scr/marker.pt`：重标后的全帧模型，val mAP50 **0.961**，mAP50-95 0.481；L 0.982、T 0.906、X 0.995；test mAP50 0.961、mAP50-95 0.444。训练集每帧最多 11 个标志，已经具备多标志同框样本。

实测手机视频 1555 帧：marker 框 3434 个，363 帧有 ≥4 个检测，PnP `valid=true` 28 帧。检测框数量已经够用，但 4 点共面多解仍让大量帧停在 `ambiguous`。

### 标注工具

```text
tools/annotate_markers.py
tools/标注工具使用说明.md
标注_train.bat / 标注_val.bat / 标注_test.bat
```

支持滚轮缩放、左键单击/拖拽画框、右键删除、`1/2/3` 切 L/T/X、`n` 下一帧、`d` 丢弃整帧、旧标签自动备份。
## 十、问题与解决过程

1. **白色球和白色场地线同色**：任务 1 用 YOLO 替代颜色阈值，减少白线干扰。
2. **摄像头默认选到笔记本内置摄像头**：增加摄像头扫描、`--list-cameras`、`--source`、`--camera-backend`，默认选最后识别到的 USB 摄像头。
3. **WSL 看不到 Windows 摄像头**：提供 `tools/windows_camera_bridge.py` MJPEG 桥接，以及 usbipd 方案说明。
4. **删除外参后 task3 跑不起来**：`build_camera` 只在 `plane` 模式检查安装外参，`pnp` 只用内参，已验证可启动。
5. **点球点残留**：从 task2/task3/场地 YAML/消息定义中全部删除。
6. **自动扩展标签污染**：曾经用光流/高置信框扩展，抽检发现大量 T 被标成 L、框偏离交点，已清空并改为全帧人工标注，提供 `tools/annotate_markers.py` 重新标。
7. **PnP 4 点多解**：保留 `ambiguous` 状态，不强行输出错误位姿；当前数据下无法稳定消除。

## 十一、仍未解决

1. PnP 4 点共面多解仍然严重：423 帧有 ≥4 个检测，只有 44 帧 valid（10.4%），285 帧 ambiguous；
2. 手机手持视频 1555 帧只有 28 帧 valid，主要受模糊、视角和 4 点歧义影响；
3. 个别标签框仍有偏移，需要继续抽检修正；
4. 安装外参未标定，task2 的地面坐标不是最终米制结果；
5. ROS2 包未在本环境完成 `colcon build` 和实机验收；
6. 手机手持视频与固定相机训练域差异大，跨域仍不稳定。

## 十二、提交清单

| 文件/目录 | 状态 |
|---|---|
| `scr/task1.py` + `scr/ball.pt` | 完成，球检测可用 |
| `scr/task2.py` | 完成，依赖未标定的安装外参 |
| `scr/task3.py` | 完成，PnP 去外参可运行，定位精度受 marker 模型限制 |
| `config/camera_intrinsics.yaml` | 内参完成，RMS 0.854 px |
| `config/field.yaml` | 实地尺寸，已删除点球点 |
| `scr/task3_interfaces` / `scr/task3_ros2` | 源码完成，但所处虚拟环境与USB摄像头不互通 |
| `dataset/ball_dataset` + `ball.pt` | 完成 |
| `dataset/marker_dataset` + `marker.pt` | 全帧重标模型，val mAP50 0.961；PnP 多解未解决 |
| `tools/annotate_markers.py` | 已完成 |