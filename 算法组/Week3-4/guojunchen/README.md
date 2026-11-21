# 第 3–4 周合并考核 · guojunchen

## 一眼看这里（视频 + 权重）
采用队长推荐的pnp算法
| 内容 | 链接 |
|---|---|
| 任务 1 录屏（足球检测） | <https://www.bilibili.com/video/BV1J4yezZEke/> |
| 任务 2 录屏（左右分屏定位） | <https://www.bilibili.com/video/BV1J4yezZERm/> |
| ROS2 录屏（加分项） | <https://www.bilibili.com/video/BV1J4yezZEyQ/> |
| 权重网盘（Week3-4guojunchen） | <https://pan.baidu.com/s/1UhbZ_PyChPVQwycBPv7Qaw?pwd=5408> 提取码 **5408** |

权重放置：`task1_best.pt` → `task1/weights/best.pt`；`task2_marker_best.pt` → `task2/weights/marker_best.pt`（详见「第 0 步」）。

| 项 | 内容 |
|---|---|
| 姓名 | guojunchen |
| 完成日期 | 2026-11-21 |
| 运行平台 | Windows 11（任务 1/2 主平台）、WSL2 Ubuntu 22.04 + ROS2 Humble（ROS2 节点） |
| 摄像头类型 | USB 单目 RGB 摄像头（UVC），OpenCV 索引 `1`；Windows 用 `CAP_MSMF`→`CAP_ANY` 兜底，Linux 用 V4L2 |
| 相机接口 | USB 2.0（UVC），即插即用 |
| 设备节点 | Windows 按索引枚举；Linux `/dev/video0`（V4L2） |
| 分辨率 | 1280×720（MJPG） |
| 实际帧率 | 相机 30.1 FPS；整链任务 1 约 23.1 FPS；任务 2 约 5 FPS |
| 镜头 | 单目 RGB 摄像头（杰锐微通），水平视场角约 44°，内参由棋盘格标定得出 |
| 实验环境 | Python 3.14.6 / torch 2.11.0+cu128 / Ultralytics 8.4.211 / OpenCV 5.0.0（ROS2 侧 Python 3.10.12 / torch 2.14.1+cpu / Ultralytics 8.4.215） |
| GPU 型号 | NVIDIA GeForce RTX 5060 Laptop GPU |

---

## 快速开始

### 准备 Python 环境（对应细则 2.3）

```bash
# Linux
sudo apt install -y python3-venv v4l-utils libgl1 libglib2.0-0
mkdir -p ~/vision_ws/k1_soccer_vision && cd ~/vision_ws/k1_soccer_vision
python3 -m venv .venv && source .venv/bin/activate
python -m pip install --upgrade pip
pip install ultralytics opencv-python numpy matplotlib pyyaml
# RTX 50 系（sm_120）另需 GPU 版：pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128

# Windows 等价
python -m venv D:\vision_ws\k1_soccer_vision\.venv
D:\vision_ws\k1_soccer_vision\.venv\Scripts\activate
pip install ultralytics opencv-python numpy matplotlib pyyaml

# 环境检查
python -c "import cv2, torch; from ultralytics import YOLO; print(cv2.__version__, torch.__version__, torch.cuda.is_available())"
# 期望：5.0.0 2.11.0+cu128 True
```

### 第 0 步：放权重（只做一次）

网盘 **Week3-4guojunchen**：<https://pan.baidu.com/s/1UhbZ_PyChPVQwycBPv7Qaw?pwd=5408> 提取码 **5408**

| 网盘文件 | 放到 |
|---|---|
| `task1_best.pt` | `task1/weights/best.pt` |
| `task2_marker_best.pt` | `task2/weights/marker_best.pt` |

仓库**不含** `.pt`（符合“权重放共享盘”要求）。校验：

| 文件 | SHA256 |
|---|---|
| `task1_best.pt` | `88974eb2a5ce720c01eb8599b0bbddc709c49ad85225c3dc1a973948e06fc437` |
| `task2_marker_best.pt` | `5edda17feca64453e9623f4b152d69b0b3665b7db4495048c339592d25101af1` |

缺权重时程序会打印网盘地址与放置路径后退出。

### 第 1 步：跑任务 1（足球检测）

```bash
cd task1
python src/detect_camera.py     # SOURCE=1（USB 相机）；编号/阈值/录像开关都在文件顶部常量里
```

### 第 2 步：跑任务 2（左右分屏定位）

```bash
cd task2
python src/live_localize.py                     # 左相机 / 右地图分屏
```

按键：`f` 翻转左右（场地 180° 对称，分不清时按一次）、`s` 存图、`q` 退出。

> 相机索引因机器而异：本机 `SOURCE = 1`（0 是 DroidCam 占位、2 是全黑的内置摄像头）。Linux 下用 `v4l2-ctl --list-devices` 查编号，改 `SOURCE` 即可；没有相机时把 `SOURCE` 改成视频路径同样能跑。
> 工具集：`python src/tools.py calib {record,capture,extract,calibrate,diag,focus}` 做标定、`data {extract,dataset,normalize,prelabel,flag,xml2txt}` 做数据、`map` 生成示意图。

### 第 3 步：ROS2
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch vision_comm localize.launch.py \
     source:=/mnt/d/guojunchen_code/tmp/ros_demo.mp4 show_echo:=true

ros2 topic list
ros2 topic echo /field_pose
ros2 topic echo /target_position
```
---

## 任务 1：足球检测

- **采集日期 / 视频 / 抽帧 / 标注实例**

  | 项 | 数值 |
  |---|---|
  | 采集日期 | 2026-11-10 ～ 2026-11-15 |
  | 原始视频 | 13 段（同一台 USB 单目相机，1280×720） |
  | 抽帧数量 | 3013 张（运动自适应 + 感知哈希去重） |
  | 标注实例 | 2491 个 `ball` 框 + 538 张背景负样本 |

- **划分方式**：按独立视频片段 70/20/10，不拆散相邻帧。

  | 划分 | 图片 | 标注框 | 视频来源 |
  |---|---|---|---|
  | train | 2009 | 1639 | 8 段整段 + 星纹长视频前段 |
  | val | 567 | 440 | 2 段整段 + 星纹长视频中段 |
  | test | 437 | 412 → 清洗后 391 | 2 段整段 + 星纹长视频后段 |

  星纹长视频（5807 帧）按时间分块并在分界各剔除 ±30 帧，无相邻帧泄漏。

- **模型与训练参数**

  | 项 | 值 |
  |---|---|
  | 模型 | YOLO11s（`yolo11s.pt` 预训练） |
  | imgsz / epochs | 640 / 80（`patience=20`、`close_mosaic=10`） |
  | batch / device / workers | 16 / 0 / 2 |
  | 优化器 / 增强 | 默认 SGD(lr0 0.01) / 默认在线增强 |

  改动与原因：① `yolov8n@640` 50 轮作合规基线；② 换 `yolo11s@640`：mAP50 +2.4、召回 +0.7；③ 试 `yolo11s@960`：召回 +2.6 但 mAP50-95 −1.8，8 GB 显存下框变松，**保持 640**；④ 训练前补回 32 个“有球却标成背景”的错标后重训。

  等价训练命令：

  ```bash
  yolo detect train model=yolo11s.pt data=dataset/ball_data.yaml epochs=80 imgsz=640 \
       batch=16 device=0 workers=2 patience=20 close_mosaic=10 project=runs name=strong_s640_v2
  ```

- **best.pt 保存位置**：网盘（见第 0 步），映射到 `task1/weights/best.pt`。

- **Precision / Recall / mAP50 / mAP50-95**（独立测试集 437 张 / 391 实例）

  | 模型 | P | R | mAP50 | mAP50-95 |
  |---|---|---|---|---|
  | yolov8n@640（基线） | 0.974 | 0.869 | 0.924 | 0.849 |
  | yolo11s@960 | 0.936 | 0.895 | 0.946 | 0.831 |
  | **yolo11s@640（最终）** | **0.975** | **0.936** | **0.966** | **0.877** |

  测试集原有 412 框，其中 21 个错标（17 个白背包、4 个草地/白鞋）已删除后重测；清理前同模型为 0.974 / 0.888 / 0.939 / 0.853。

- **推理耗时 / 实时 FPS**：端到端 9.3 ms/帧 ≈ 107 FPS（GPU）；整条相机链路 23.1 FPS（受相机 30 fps 限制）；纯推理 3.2 ms。

- **误检**：`task1_test_0400` 右下角椅子旁浅色鞋/包裹，conf 0.6–0.7 被当成球——白球最强干扰项，负样本覆盖不足。
- **漏检**：`task1_test_0424` 近处 110×115 px 绿球检出，右侧约 30 m 外 30×30 px 小球漏掉（640 输入下只剩 15 px）。
- 另有 3 个 conf 0.91–0.94 的“误检”其实是真球（当年漏标），**未回改**以保持测试集可复现。
- **处理链路**：相机帧 → `cv2.VideoCapture` → YOLO11s(640) → `xyxy` + 置信度 → 框中心 `(u,v)` → 画框/置信度/绿点/`(u,v)` + 左上角 FPS → `s` 存图、`q` 退出。

---

## 任务 2：单目场地自定位
<img width="2206" height="1887" alt="image" src="https://github.com/user-attachments/assets/0add2831-7240-4d0e-ac31-e13de77e2d1c" />


- **field.yaml / SHA256**：场地 7.80×6.00 m、25 个点全部卷尺实测。

  | 文件 | 内容 | SHA256 |
  |---|---|---|
  | `config/field.yaml` | 特征点 id/类型/米制 x,y + 场地尺寸（程序优先读它） | `f9c59572dbfa88855e236826c9b20097289f15a17633c2ae6f0dd4d451c98f3b` |
  | `config/field_map.png` | 俯视图，**仅界面显示、不参与计算** | `c8c4eaebf60e47bfa2da5881c7f0a39c4a4b61f9e3081c4c0f912615288cf163` |
  | `config/field_map.json` | 显示图比例/原点/画布尺寸：**186.92 px/m、原点像素 (1103.0, 943.5)、画布 2206×1887、四周 2 m 白边** | `883be0e8a77126cc2e2ba13f5bcb9c64cc9707003c5cc9a48d8c7bcf60fa2579` |
  | `calibration/intrinsics.yaml` | 相机内参 | `8e5d1834545904040522b7906380e305e329e806f5c190a55569be438dcbc546` |

- **field_geometry**：未单独建，尺寸写在 `field.yaml` 的 `field / zones / center_circle` 段。
- **地图原点 / X/Y 轴方向 / 单位**：原点 = 中圈圆心；**+X 向右（指向唯一球门）**；**+Y 向上**；**yaw 从 +X 逆时针为正（度）**；位置单位米，地面 Z=0。
- **尺寸**：总长 7.80 / 总宽 6.00；大禁区深 1.90 宽 4.00；小禁区深 0.94 宽 2.70；中线到小禁区前沿 2.96；中圈半径 0.75。
- **标志类别**：`L` 12 个、`T` 10 个、`X` 2 个。**PenaltyPoint 不参与检测**（圆心处无可见标记），由两个 X 的中点几何推导。
- **检测模型**：YOLO11s，544 张（train 478 / val 66），imgsz 640、150 epochs；val mAP50 **0.677**（L 0.568 / T 0.512 / X 0.949）；权重 `task2_marker_best.pt` → `task2/weights/marker_best.pt`。
- **几何特征点提取**（框中心与真交点差 20–40 px，必须细化）：框内白掩膜（`V>140 & S<70`）→ LSD 线段 → 按方向聚类（0.20 rad）→ `cv2.fitLine` → 两两求交（夹角 >20°）→ 取离框中心最近的交点（`score = 距离 − 0.5×段长和`）。成功则置信度 +0.2，失败退回框中心。
- **相机内参**：`fx=1630.237 fy=1625.296 cx=640.314 cy=412.602`（1280×720，21 张棋盘格，RMS **0.472 px**）。
- **畸变**：`k1=0.22881 k2=-1.59620 p1=0.00200 p2=0.00172 k3=2.20875`。
- **外参 / 高度 / 俯仰 / 滚转**：不单独存 `camera_extrinsics.yaml`；按细则 2.9 相机光心即载体原点。高度、俯仰、滚转**每帧由 PnP 解出**（实测高度中位 1.33 m，范围 1.16–1.52 m），不使用标称值当前验。
- **地面投影**：`(u,v)` → `K⁻¹` 射线 → 转世界系 → 与 z=0 求交（`s = -c_z/d_z`）；`s≤0` 或落点超出场地 0.5 m 丢弃。
- **输出坐标系**：测试架坐标系（原点=相机光心），输出 `x_camera_map / y_camera_map / yaw_camera_map`。
- **3 个测试点的参考位姿与误差**（实拍视频复现，非卷尺绝对精度；方法见「C0」）：

  | # | 参考位姿 (x,y,yaw) | 估计位姿 (x,y,yaw) | 位置误差 (m) | 航向误差 (°) |
  |---|---|---|---|---|
  | P1 | （现场未量，取窗口内位姿中位数作参考） | −5.294, +3.520, −14.9 | 0.317（重复性 2σ） | 1.2 |
  | P2 | 同上 | −2.143, +3.316, −20.7 | 0.494 | 3.0 |
  | P3 | 同上 | −1.917, +3.289, −53.4 | 0.343 | 9.5 |

  40 帧连续测试：`status=ok` 25 帧（62%），其余 `high_residual`，残差中位 8.8 px。
- **输出字段**：`marker_count`（去重后标志数）、`residual`（内点平均重投影误差 px）、`valid`（本帧是否可信）、`status`（`ok` / `insufficient_markers`（<3 点）/ `ambiguous` / `high_residual`（>15 px 或高度异常）/ `lost`（>2.5 s））。无解时不拿旧位姿冒充，只在地图画灰色虚影。
- **定位 FPS / 耗时**：约 5 FPS（≈200 ms/帧），双模型推理仅占约 11 ms，主要耗时在枚举打分。
- **处理链路**：相机帧 → 标志模型得 L/T/X 框 → 框内 LSD 细化 → 与 25 个地图点多假设 PnP → 内点评分+精修 → `R,t` → `yaw=atan2(前向y,前向x)` → 地图按 `px = ox + x·scale`、`py = oy − y·scale` 画蓝点与箭头；球框底边中点投到 z=0 得球地面坐标画黄点。
---

## ROS2 接口

| 项 | 内容 |
|---|---|
| 节点 | `localizer_publisher`（`localize`，同时发定位+足球，launch 中重命名 `field_localizer`）、`target_publisher`（`pub`）、`target_subscriber`（`sub`） |
| 足球话题 | `/target_position`（`vision_interfaces/msg/TargetPosition`） |
| 定位话题 | `/field_pose`（`vision_interfaces/msg/FieldPose`） |
| 消息字段 | `TargetPosition`: detected/u/v/area/confidence；`FieldPose`: valid/x/y/yaw/marker_count/residual/status（单位与坐标轴同上） |

包结构 `task2/ros2_ws/src/{vision_interfaces, vision_comm}`，写法沿用第 1–2 周（单类、`declare_parameter`、按面积排序、未检测到也发一条 `detected=False`）。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch vision_comm localize.launch.py \
     source:=/mnt/d/guojunchen_code/tmp/ros_demo.mp4 show_echo:=true


ros2 topic list
ros2 topic echo /field_pose
ros2 topic echo /target_position
```

| 节点 | 参数（默认值） |
|---|---|
| `localize` | `source`＝`1`、`conf`＝`0.20`、`r`＝`0.75`、`publish_ball`＝`true`、`conf_ball`＝`0.30`、`min_area`＝`300` |
| `pub` | `source`＝`1`、`conf`＝`0.30`、`min_area`＝`300` |

> 一台 USB 相机只能被一个进程打开，`pub` 与 `localize` 不要同时启动。
> 仓库不在 `~/guojunchen` 时用 `TASK1_DIR` / `TASK2_DIR` 指过去；节点若在系统 Python 下缺 `ultralytics`，会自动切到 `~/vision_ws/k1_soccer_vision/.venv/bin/python`（`VISION_VENV_PY` 可覆盖）。
> 若 `colcon build` 报 `canonicalize_version() got an unexpected keyword argument 'strip_trailing_zero'`，`pip install -U 'packaging>=24.0'` 即可。

**验证记录（WSL2 Ubuntu 22.04 + ROS2 Humble）**

```
$ colcon build --packages-select vision_interfaces vision_comm
Summary: 2 packages finished [45.9s]
$ ros2 topic list
/field_pose        /target_position
$ ros2 topic echo /field_pose --once
valid: true  x: -1.98  y: 3.41  yaw: -50.3  marker_count: 5  residual: 6.51  status: ok
$ ros2 topic echo /target_position --once
detected: true  u: 939.0  v: 246.0  area: 13883.33  confidence: 0.9379
$ ros2 topic hz /field_pose
average rate: 3.947
```

---

## B 站演示录屏

| 视频 | 链接 | 视频时长 |
|---|---|---|
| 任务 1 · 足球检测 | <https://www.bilibili.com/video/BV1J4yezZEke/> | 约 1 分 30 秒 |
| 任务 2 · 场地定位（左右分屏） | <https://www.bilibili.com/video/BV1J4yezZERm/> | 约 2 分钟 |
| ROS2 节点 · 加分项 | <https://www.bilibili.com/video/BV1J4yezZEyQ/> | 约 1 分钟 |

---

## 加分项

| 项 | 状态 | 证据 |
|---|---|---|
| ONNX 导出与对比（+3） | ✅ | test 100 张：pt-GPU 9.34 ms / 107 FPS、pt-CPU 49.83 ms、onnx-CPU 55.62 ms（本机 onnxruntime 为 CPU 版，价值在部署） |
| 稳健位姿（多假设 + 时序）（+2） | ✅ | 40 帧：单次最近邻 ok 1–2 帧 → 多假设 25 帧 → 时序 27 帧；关掉 `INIT_SIDE` 时如实报 `ambiguous` |
| ROS2 节点 + launch（+2） | ✅ | 见「ROS2 接口」，WSL 实编译并跑通 |

---

## 问题与解决过程

1. **解出的相机高度 4.4 m**：原内参是按 1910×1094 屏录标定的，与 1280×720 不一致；改用 21 张 1280×720 棋盘格重标（fx 1767→1630，RMS 0.472），高度回到 1.16–1.52 m。
2. **怎么调都解不出位姿**：枚举预算把每个 4 点子集截断在 200 次，正确组合永远跑不到；改为“总组合数小于预算则全枚举，否则随机采样”，并把高度先验从硬约束改成过滤条件。
3. **相机能打开但读不到帧**：同一台 USB 相机用 MSMF 打开会返回近全黑帧（亮度 0–9），改用“MSMF 与默认后端各试一次、取亮度 >20 的那路”后正常。

---
## 附：仓库内容

```
guojunchen/
├── README.md
├── task1/{dataset/{ball_data.yaml, dataset_summary.md}, src/detect_camera.py, weights/放置说明.txt}
└── task2/{calibration/intrinsics.yaml, config/{field.yaml, field_map.png, field_map.json},
          dataset/marker_data.yaml, src/{live_localize.py, tools.py},
          ros2_ws/src/{vision_interfaces, vision_comm}, weights/放置说明.txt}
```

仓库 27 个文件 / 499 KB，不含权重、不含数据集、不含视频（按细则要求）。

```bash
cd task1 && python src/detect_camera.py      # 任务 1
cd task2 && python src/live_localize.py      # 任务 2
cd task2/ros2_ws && colcon build --packages-select vision_interfaces vision_comm   # ROS2
```
