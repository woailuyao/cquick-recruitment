# 第 3–4 周合并考核 · yangtiancheng

- 姓名：yangtiancheng
- 完成日期：2026-11-20
- 任务一视频：https://www.bilibili.com/video/BV13ayxzpEei/
- 任务二视频：https://www.bilibili.com/video/BV1vWyxzZECM/
- 摄像头类型：单目 RGB 摄像头（USB/UVC，不限定品牌型号）
- 摄像头接口 / 设备节点：实时验收在 Windows 原生环境运行，相机为 OpenCV source 0（设备名 USB Camera）；数据采集与训练使用同一台相机
- 镜头：固定焦距、固定对焦（焦距数值未记录）
- 图像分辨率 / 实际帧率：1280 × 720；程序显示 FPS 约 10、处理 FPS 约 3（滚动值）；相机标称 30 fps，实际采集帧率未单独测量
- 环境版本：验收（Windows）Python 3.12.14 / OpenCV 5.1.0.95 / Ultralytics 8.4.215 / PyTorch 2.15.0+cpu；训练（Ubuntu 虚拟机）Python 3.10.12 / Ultralytics 8.4.208 / PyTorch 2.14.1+cu130；Ubuntu 版本未记录
- GPU 型号：无（全部 CPU 运行，Intel Core Ultra 9 275HX）
- ROS2：未使用（本地可视化输出，见第 10 节）

## 1. 提交内容

本包包含：

- 两个任务的最终检测/标定/定位/实时验收源码
- 足球与场地标志两个 `data.yaml`
- 数据集统计说明
- 实际在用的相机内参、临时外参和场地图 YAML
- 模型 SHA-256 校验值

本包不包含：

- `.pt` 模型权重
- 原始视频
- 原始数据图片
- YOLO 标签文件
- 训练输出图片、视频或缓存

原始数据、训练环境和权重保留在实验工作目录/虚拟机中。

## 2. 运行环境

### 2.1 Windows 实时验收环境

本项目实时验收实际使用的 Python 环境：

```text
C:\Users\Lenovo\Documents\Codex\2026-11-14\soccer-localization-continuation\.venv-acceptance\Scripts\python.exe
```

实测包版本：

```text
Python  3.12.14
opencv-python  5.1.0.95
ultralytics  8.4.215
torch  2.15.0+cpu
numpy  2.5.5
numba  0.68.1
```

还需安装：

```text
PyYAML
```

### 2.2 离线训练环境

足球和标志模型训练在 cquick Ubuntu 虚拟机的 `vision_ws/k1_soccer_vision` 目录内完成。离线训练环境的完整 Python 包版本记录未保留，标为：

```text
缺失
```

## 3. 路径约定

本提交包内：

```text
dataset/ball_data.yaml
dataset/marker_data.yaml
calibration/camera_intrinsics.yaml
calibration/camera_extrinsics.yaml
config/field_geometry.yaml
src/
```

模型与原始数据不在包内，实际存放位置：

```text
足球 best.pt:
C:\Users\Lenovo\Documents\Codex\2026-10-21\u-b\work\vm-share\ball_v3_best.pt

标志 best.pt:
C:\Users\Lenovo\Documents\Codex\2026-10-21\u-b\work\vm-share\task2\runs\detect\runs_markers\markers_v1\weights\best.pt
```

足球数据集原始路径：

```text
/home/obsession/vision_ws/k1_soccer_vision/datasets/ball_split_v3
```

标志数据集原始路径：

```text
C:\Users\Lenovo\Documents\Codex\2026-10-21\u-b\work\vm-share\task2\marker_dataset
```

## 4. 数据说明

详细统计见：

```text
dataset/dataset_summary.md
```

### 4.1 任务一足球数据

- 数据来源：本人实验室实拍
- 视频来源数：14 段
- 数据根目录：`ball_split_v3`
- 类别：`0=ball`

| split | 图片 | 有球 | 无球背景 |
|---|---:|---:|---:|
| train | 275 | 160 | 115 |
| val | 67 | 33 | 34 |
| test | 77 | 36 | 41 |
| 合计 | 419 | 229 | 190 |

划分方式：

- 有球 train 来源：2、3、4、5、6、9、10、11
- 有球 val 来源：12
- 有球 test 来源：1、14
- 无球 train/val/test 来源：15/16/17
- 按整段视频划分，同一视频不会跨 split

这些数字来自数据集构建脚本当时成功运行的输出和会话记录。当前 Windows 主机没有 `ball_split_v3` 图片/标签副本，无法重新逐文件统计，因此没有伪造“当前文件重统计”结果。

### 4.2 任务二标志数据

当前实际文件重新统计：

| split | 图片 | 标签 | 背景 | LCross | TCross | XCross | PenaltyPoint |
|---|---:|---:|---:|---:|---:|---:|---:|
| train | 150 | 150 | 99 | 28 | 3 | 20 | 0 |
| val | 82 | 82 | 24 | 0 | 58 | 0 | 0 |
| test | 40 | 40 | 5 | 35 | 0 | 0 | 0 |
| 合计 | 272 | 272 | 128 | 63 | 61 | 20 | 0 |

视频划分：

| 视频 | split |
|---|---|
| `01_normal.avi` | train |
| `02_glare.avi` | test |
| `03_goalpost.avi` | val |
| `04_invalid.avi` | train |

补充：

- 原始抽帧清单 321 张
- `22.avi` 的 49 张未进入当前训练轮次
- 活动数据 272 张
- 独立 XCross 评测视频 `23.avi` 抽帧 73 张，其中 31 个 XCross、42 张背景

## 5. 任务一：足球检测

### 5.1 模型与训练设置

```text
模型：YOLO11n
初始化：yolo11n.pt
epochs：30
patience：12
imgsz：640
batch：4
workers：2
device：CPU
cache：False
amp：False
seed：42
```

训练输出：

```text
/home/obsession/vision_ws/k1_soccer_vision/runs/ball_detect/ball_v3_hardneg/weights/best.pt
```

最终权重已备份为：

```text
C:\Users\Lenovo\Documents\Codex\2026-10-21\u-b\work\vm-share\ball_v3_best.pt
```

### 5.2 指标

训练结束时的验证集指标：

| 来源 | Precision | Recall | mAP50 | mAP50-95 |
|---|---:|---:|---:|---:|
| 验证集 val | 0.998 | 1.000 | 0.995 | 0.951 |

上述为验证集指标，不作为独立 test 指标。

独立 test 的标准 Precision / Recall / mAP50 / mAP50-95：

| 来源 | 图片 | 实例 | Precision | Recall | mAP50 | mAP50-95 |
|---|---:|---:|---:|---:|---:|---:|
| 独立 test | 77 | 36 | 0.994 | 0.861 | 0.877 | 0.806 |

该结果于 2026-11-20 运行，run 名为 `ball_v3_test_final`。

已有独立 test 图像级检出审计：

```text
部署推理：imgsz=640, conf=0.25, max_det=1
test 有球图片：35/36 检出
test 无球图片：0/41 误检
多余框：0
推理耗时：约 20 ms/张
```

据此计算的图像级指标：

```text
Image-level Precision = 35 / (35 + 0) = 1.0000
Image-level Recall    = 35 / (35 + 1) = 0.9722
```

这不是 YOLO 标准框级 P/R/mAP，报告中不混用。

### 5.3 误检案例

案例：`17_00005.jpg`、`17_00029.jpg`

- 来源：test 的无球来源 17
- 发生设置：诊断时使用 `imgsz=960, conf=0.25`
- 结果：两张无球图片出现错误球框
- 最终设置：`imgsz=640, conf=0.25, max_det=1`，无球误检恢复为 `0/41`

原因：更高输入分辨率放大了无球场景中的局部黑白/边缘纹理，模型把非球结构误判为足球。该案例说明不能简单依赖提高分辨率；最终配置仍选择 `640`。

具体误检框置信度：

```text
缺失
```

### 5.4 漏检案例

案例：`14_00091.jpg`

- 来源：test 有球来源 14
- `640, conf=0.25`：漏检
- `960, conf=0.25`：可以检出

原因：

- 足球紧贴球网
- 黑白花纹与球网、墙面线条混杂
- 画面倾斜并存在局部高亮
- 训练集中“足球贴近球网”的困难样本不足
- 640 缩放后细节不足

最终仍采用 640 以保证速度和零无球误检，通过连续帧容忍单帧漏检。

## 6. 任务二：场地标志检测与定位

### 6.1 相机内参

来源：`camera_intrinsics.yaml`，由本人实验室标定。

```text
图像尺寸：1280 × 720
棋盘内角点：9 × 9
单格：0.027 m
标定图片成功：17 / 18
RMS：0.6281101600619807 px

fx = 1490.3291634516265
fy = 1501.1247891348844
cx = 690.3055808809594
cy = 327.3470954106697

k1 =  0.15986872738605942
k2 = -0.9979708282542827
p1 = -0.002721193959994071
p2 = -0.0021791026978977722
k3 =  1.3694314276100565
```

### 6.2 外参当前状态

随包外参：

```text
calibration/camera_extrinsics.yaml
```

当前状态为未定版：

```text
calibration_status: provisional_height_mismatch
source image: IMG_0024.jpg
square_size_m: 0.029
reprojection RMS: 0.829163446597286 px
measured height: 0.725 m
PnP height: 0.6614644846921296 m
height difference: -0.06353551530787038 m
```

当前 camera→robot 外参：

```text
[ 0.0303106310, -0.4496752876,  0.8926776582,  0.0683219075]
[-0.9829225546,  0.1487660699,  0.1083139331, -0.0755593224]
[-0.1815062459, -0.8807160679, -0.4374867890,  0.6614644847]
[ 0.0,           0.0,           0.0,           1.0]
```

该外参来自 PnP 推导，实测镜头高度为 72.5 cm，但 PnP 高度为 66.15 cm，存在约 6.35 cm 高度差，因此没有定版。该 YAML 只适用于开发期定位。

### 6.3 场地图来源

随包配置：

```text
config/field_geometry.yaml
```

来源：`BoosterRobotics/robocup_demo` 的 `FD_KIDSIZE` 场地定义，经结构化转换得到；不是“队长官方修订版”。

当前实际修改：

```yaml
PenaltyPoint:
  CentrePoint: [0.0, 0.0]
```

原始 `PenalityPoint` 拼写通过别名映射为 `PenaltyPoint`。

### 6.4 标志检测路线

#### 路线 A：YOLO

训练模型：

```text
runs/detect/runs_markers/markers_v1/weights/best.pt
```

训练设置：

```text
模型：yolov8n.pt
imgsz：640
epochs：50
```

已有报告指标：

| 类别 | 真值实例 | 来源 split | Precision | Recall | mAP50 | mAP50-95 |
|---|---:|---|---:|---:|---:|---:|
| LCross | 35 | 原 test，40 张 | 0.8252 | 0.8096 | 0.9282 | 0.4513 |
| TCross | 58 | 原 val，82 张 | 0.7188 | 0.7931 | 0.7334 | 0.2479 |
| XCross | 31 | 独立 test，73 张 | 0.8525 | 0.5596 | 0.7590 | 0.2379 |
| PenaltyPoint | 0 | 无真值/未训练 | N/A | N/A | N/A | N/A |

注意：

- 三个类别来自不同评估集，没有强行计算跨集合总体 mAP。
- `PenaltyPoint` 在当前训练数据中为 0 实例，没有训练。

#### 路线 B：HSV 几何检测

实时验收实际使用 `detect_field_features.detect()` 作为标志前端，原因如下：

- YOLO 在验收当日实时画面上检出不足。
- `postcheck` 真实帧在 `conf=0.35` 和原始推理 `conf=0.25` 下均无输出。
- 训练视频同模型对照帧可以检出 LCross 0.8538，说明不是模型文件缺失，而是实时域差异和召回不足。
- HSV 前端可直接利用白线亮度和几何形态，在当前场地画面中更稳定。

HSV 路线不是纯颜色阈值：

1. 白线 HSV 掩膜
2. 形态学清理
3. Zhang-Suen 细化
4. L/T/X 角向分支判定
5. 草地区域支撑约束
6. 候选地面投影过滤
7. 多特征定位验证

当前性能优化只替换骨架化的执行实现，检测规则和阈值不变。

### 6.5 候选地面过滤

实时程序在 HSV 候选后、定位前增加：

- 使用实际内参和临时外参投影到地面
- 地平线以上或地面求交失败的候选丢弃
- 场地边界外扩 1 m 之外的候选丢弃

当前允许半边界：

```text
X: 5.5 m
Y: 4.0 m
```

未修改检测阈值、定位阈值或场地图。

### 6.6 显示分级

- 未通过定位验证的候选：小灰点
- 通过定位验证的 inlier：正式类别框
- 候选数据和定位结果不因绘制方式改变

### 6.7 定位链路

```text
像素特征点
→ 去畸变
→ 像素射线与 z=0 地面求交
→ field.yaml 多特征匹配
→ 两两假设 + RANSAC/刚体拟合
→ prior 消歧
→ 输出 x / y / yaw
```

如果可信标志不足两个、类型不匹配或几何上不可区分，程序输出：

```text
INVALID
```

不伪造位姿。

### 6.8 实测结果

有效位姿示例：

```text
x      = -3.422318 m
y      =  0.130053 m
yaw    = -126.044484 deg
residual = 0.074361 m
```

实时程序：

- 显示 FPS 约 10
- 处理 FPS 滚动值约 3
- 30 秒调度实测中处理线程不会积压旧帧；处理期间新帧覆盖旧待处理帧

说明：30 秒运行的显示 FPS 均值约 9.50，处理 FPS 末段滚动值约 2.88；处理均值约 1.38，因为每帧处理耗时高于显示间隔，处理线程主动跳过中间帧以保持最新帧。


### 6.9 细则模板补充项

- `field.yaml` / `field_geometry.yaml` 版本：组织方统一修订版未下发至个人目录；本提交使用自建合并配置 `config/field_geometry.yaml`（特征点与场地几何同一文件），SHA-256：`3a5c8688f224bfa77a25c251864f966ad4b93e0882f8ffbd9a0674f40a177b7c`
- 地图原点、X/Y 轴方向、单位：原点为场地中心 (0, 0)；+X 向右（场地长边方向，范围 ±4.5 m）；+Y 向上（范围 ±3.0 m）；单位 m；Yaw 自 +X 轴逆时针为正
- 场地实际尺寸是否与地图一致：实验室场地按 KidSize 9 m × 6 m 搭建；未逐点核验全部特征坐标与地图的一致性（队长修订版未下发）
- 输出坐标系：地图坐标系下的载体位姿 (x, y, yaw)，载体参考原点为相机测试架参考点（外参定义见 `calibration/camera_extrinsics.yaml`）；定位结果以该参考点输出，未另行换算 `camera_pose_map`
- 3 个测试点的参考坐标与估计误差：未使用卷尺等真值逐点测量（缺失）；已在多个位置验证可输出 VALID 位姿，示例见 6.8（residual ≈ 0.074 m）；标志不足时正确输出 INVALID

## 7. 离线流程与命令

以下命令默认当前目录为本提交包根目录 `yangtiancheng/`。如果命令需要在 Ubuntu 虚拟机中执行，请先把整个提交包放到虚拟机可访问的共享目录，或把 `src/` 下所需脚本复制到虚拟机项目目录。

### 7.1 任务一：构建足球数据集

在 Ubuntu 虚拟机、进入提交包根目录后运行：

```bash
python3 src/build_ball_split_v3.py
```

### 7.2 任务一：训练

```bash
cd ~/vision_ws/k1_soccer_vision
source .venv/bin/activate

yolo detect train \
  model=./yolo11n.pt \
  data=datasets/ball_split_v3/ball.yaml \
  epochs=30 \
  patience=12 \
  imgsz=640 \
  batch=4 \
  workers=2 \
  device=cpu \
  cache=False \
  amp=False \
  project="$PWD/runs/ball_detect" \
  name=ball_v3_hardneg \
  seed=42 \
  plots=True
```

### 7.3 任务一：独立 test 评估

```bash
cd ~/vision_ws/k1_soccer_vision
source .venv/bin/activate

yolo detect val \
  model=runs/ball_detect/ball_v3_hardneg/weights/best.pt \
  data=datasets/ball_split_v3/ball.yaml \
  split=test \
  imgsz=640 \
  batch=4 \
  device=cpu \
  project="$PWD/runs/ball_detect" \
  name=ball_v3_test_final
```

### 7.4 任务二：相机内参标定

```bash
python3 src/calibrate_camera.py \
  --images "C:\Users\Lenovo\Documents\Codex\2026-10-21\u-b\work\vm-share\calibration_images" \
  --output "C:\Users\Lenovo\Documents\Codex\2026-10-21\u-b\work\vm-share\calibration_images\calibration_output" \
  --max-view-rms 1.5
```

### 7.5 任务二：外参计算

```bash
python3 src/calibrate_extrinsics_robust.py \
  --intrinsics "C:\Users\Lenovo\Documents\Codex\2026-10-21\u-b\work\vm-share\camera_intrinsics.yaml" \
  --images "C:\Users\Lenovo\Pictures\IMG_0024.jpg" \
  --square 0.029 \
  --a-x 0.750 \
  --a-y 0 \
  --measured-height 0.725 \
  --output "calibration\camera_extrinsics.yaml"
```

该输入会输出 `provisional_height_mismatch`。这是实际门禁结果，不应改写成 accepted。

### 7.6 任务二：抽帧与划分

```bash
python3 src/extract_frames.py \
  --videos "C:\Users\Lenovo\Desktop\拍摄" \
  --out frames \
  --target-total 320 \
  --interval-sec 0.5 \
  --diff-thresh 2.0 \
  --min-sharpness 30.0

python3 src/split_dataset.py \
  --frames frames \
  --out marker_dataset \
  --seed 42 \
  --ratios 0.7,0.2,0.1
```

### 7.7 任务二：训练与评估

```bash
python3 src/train_markers.py \
  --data marker_dataset/data.yaml \
  --weights yolov8n.pt \
  --epochs 50 \
  --imgsz 640 \
  --batch 16 \
  --project runs_markers \
  --name markers_v1

python3 src/evaluate_markers.py \
  --weights runs_markers/markers_v1/weights/best.pt \
  --data marker_dataset/data.yaml \
  --split test \
  --imgsz 640 \
  --output marker_metrics_test.json
```

### 7.8 任务二：单帧检测与定位

```bash
python3 src/detect_field_features.py \
  --image test.jpg \
  --output features.json \
  --preview features_preview.jpg

python3 src/task2_localization.py \
  --intrinsics calibration/camera_intrinsics.yaml \
  --extrinsics calibration/camera_extrinsics.yaml \
  --field config/field_geometry.yaml \
  --features features.json \
  --threshold 0.30 \
  --prior -1.225 1.233 -51.36 \
  --allow-provisional \
  --output localization.json
```

## 8. 实时验收命令

两个任务已拆成两个入口，运行时二选一，不同时占用同一相机。

### 8.1 足球实时检测

```powershell
$pkg="C:\Users\Lenovo\Documents\Codex\2026-11-19\du-q\提交包\yangtiancheng"
$py="C:\Users\Lenovo\Documents\Codex\2026-11-14\soccer-localization-continuation\.venv-acceptance\Scripts\python.exe"

& $py "$pkg\src\run_ball_live.py" `
  --source 0 `
  --ball-model "C:\Users\Lenovo\Documents\Codex\2026-10-21\u-b\work\vm-share\ball_v3_best.pt" `
  --ball-conf 0.40 `
  --display-max-width 1400
```

### 8.2 场地定位实时验收

```powershell
$pkg="C:\Users\Lenovo\Documents\Codex\2026-11-19\du-q\提交包\yangtiancheng"
$py="C:\Users\Lenovo\Documents\Codex\2026-11-14\soccer-localization-continuation\.venv-acceptance\Scripts\python.exe"

& $py "$pkg\src\run_field_live.py" `
  --source 0 `
  --intrinsics "$pkg\calibration\camera_intrinsics.yaml" `
  --extrinsics "$pkg\calibration\camera_extrinsics.yaml" `
  --field "$pkg\config\field_geometry.yaml" `
  --threshold 0.30 `
  --initial-prior -1.225 1.233 -51.36 `
  --display-max-width 1600
```

### 8.3 兼容入口

`src/run_acceptance_live.py` 保留足球 + 场地组合版本，默认参数可按需覆盖；最终提交建议优先使用拆分后的两个入口。

## 9. 模型 SHA-256

| 任务 | 权重路径 | SHA-256 |
|---|---|---|
| 足球 | `C:\Users\Lenovo\Documents\Codex\2026-10-21\u-b\work\vm-share\ball_v3_best.pt` | `D0DB8C75622D5781374AA537F83D42D29DB76D9A7DEE7ABC5AD11ACB5E026062` |
| 标志 | `C:\Users\Lenovo\Documents\Codex\2026-10-21\u-b\work\vm-share\task2\runs\detect\runs_markers\markers_v1\weights\best.pt` | `44635F1110CA55B3FC765D1748BAAADDFE43706DF3FB38605B091B0E12089D29` |

权重文件没有上传进提交包。

## 10. ROS2 接口

未使用 ROS2。两个任务均为本地程序运行，结果在本地可视化界面输出（左 RGB / 右地图），未发布 ROS2 话题。

## 11. B 站演示录屏（必交）

- 任务一链接：https://www.bilibili.com/video/BV13ayxzpEei/（标题：CQuick 第3-4周考核 · 任务一 足球检测（yangtiancheng））
- 任务二链接：https://www.bilibili.com/video/BV1vWyxzZECM/（标题：CQuick 第3-4周考核 · 任务二 单目场地自定位（yangtiancheng））
- 视频时长：未记录
- 访问权限：公开
- 是否包含连续实时相机输入：是
- 是否展示足球检测框、置信度和 FPS：是
- 是否展示 L/T/X 标志识别与几何特征点：是
- 是否采用左 RGB / 右地图的左右分屏布局：是
- 是否展示 field.yaml 地图匹配和地图俯视图：是
- 是否在右侧地图标出相机光心位置与朝向箭头：是
- 是否展示相机地图坐标 x / y / yaw：是
- 是否展示至少 3 个不同位置 / 朝向的地图位姿：是
- 是否展示 marker_count、residual、valid / status：是
- GitHub 本目录是否已提交该 B 站链接：是

## 12. C0 现场测试

- 测试日期：实验室自测 2026-11-20；C0 正式现场复测待评审安排
- 测试点与误差：未做真值逐点测量（见 6.9）；自测中多个位置输出 VALID 位姿（示例 x = -3.42 m, y = 0.13 m, yaw = -126.0°, residual ≈ 0.074 m）
- 定位失败场景与处理：可信标志不足两个或几何不可区分时输出 INVALID，不伪造位姿

## 13. 问题与解决过程

问题：验收当日 YOLO 标志模型在实时相机画面上完全检不出标志（原始推理 conf = 0.25 仍零框），但同一模型在训练视频帧上可正常检出（LCross 0.854）。

排查：逐层定位——绕过几何细化直接跑原始 YOLO 仍为零框，排除细化模块问题；量化对比发现当日画面白线更细（宽度中位 31.6 px，训练帧 43.9 px）、草地更暗，判定为实时画面与训练分布不一致（域偏移）；将相机退至 3.5–4 m 仍零框，排除尺度因素。

解决：实时验收程序的标志前端切换为 HSV 几何检测（白线掩膜 + Zhang-Suen 细化 + 分支分类）；候选必须通过地面投影与 `field.yaml` 多特征几何验真才输出位姿，验不过输出 INVALID。YOLO 路线保留为训练与指标路线并如实说明。切换后多个位置获得 VALID 位姿（residual ≈ 0.074 m）。

## 14. 已知不足

- 当前外参为 `provisional_height_mismatch`，实测高度与 PnP 推导高度相差约 6.35 cm，未定版。
- `PenaltyPoint` 训练实例为 0，没有训练该类。
- 标志 YOLO 在实时画面上检出不足，当前验收依赖 HSV 前端 + 几何验真。
- 足球在贴网、高亮、黑白纹理混杂场景存在 640 输入漏检。
- 任务二定位依赖至少两个可信且几何上可区分的特征；不足时只能输出 INVALID。

## 15. ONNX 推理对比

导出与对比环境：

```text
Python            3.12.14
Ultralytics       8.4.215
torch             2.15.0+cpu
onnx              1.23.2
onnxruntime       1.32.0
opset             12
imgsz             640
device            CPU
```

ONNX 文件：

```text
C:\Users\Lenovo\Documents\Codex\2026-11-19\du-q\outputs\ball_onnx_comparison_20261122\ball_v3_best.onnx
SHA-256: DF1B87EEC374E9EC511E1F65E0030376FF81B18E8009F0A06A9FC52D429CC439
```

对比数据为同一批独立 test 的 77 张图片、36 个真值实例和 41 张无球背景，按 `ball_split_v3` 的视频来源 1、14、17 重建。

表和框一致性测试固定使用：

```text
imgsz=640
conf=0.25
iou=0.7
batch=1
device=cpu
```

| 模型 | FPS | 单张延迟 | Precision | Recall | mAP50 | mAP50-95 |
|---|---:|---:|---:|---:|---:|---:|
| PyTorch `.pt` | 36.93 | 27.080 ms | 0.9944 | 0.8611 | 0.8650 | 0.7974 |
| ONNX `.onnx` | 38.45 | 26.009 ms | 0.9887 | 0.8056 | 0.8315 | 0.7640 |

框一致性按同类、匹配 IoU ≥ 0.5 配对，再用 IoU ≥ 0.99 判定几何一致：

- 77 张图中 71 张框几何一致
- 6 张不一致
- ONNX 少 2 个框，没有多出框
- 已配对框中有 6 个 IoU < 0.99

差异图片：

```text
14_00010.jpg   1 个匹配框 IoU < 0.99
14_00045.jpg   ONNX 少 1 个框
14_00089.jpg   ONNX 少 1 个框，另有 1 个匹配框 IoU < 0.99
14_00092.jpg   1 个匹配框 IoU < 0.99
14_00093.jpg   1 个匹配框 IoU < 0.99
1_00003.jpg    2 个匹配框 IoU < 0.99
```

ONNX 输出存在数值差异，匹配置信度平均差 `0.0362`、最大差 `0.4475`。上述对比使用部署 conf=0.25；5.2 节标准 test mAP 仍使用 `conf=0.001, iou=0.7`，两节的 conf 口径不同，不能直接混用。

`.pt` 和 `.onnx` 权重本身都不进入提交包。
