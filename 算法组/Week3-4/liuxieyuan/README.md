# 第 3–4 周合并考核 · 刘勰渊

- 姓名：刘勰渊（学号 20263312）
- 完成日期：2026-11-22
- 摄像头类型：单目 RGB 摄像头（UVC 免驱，不限定品牌和型号）
- 摄像头接口 / 设备节点：Windows 下 USB 摄像头，OpenCV `CAP_DSHOW` 索引 `0`（对应 Linux 的 `/dev/video0`）
- 镜头：固定焦距、固定对焦（手动对焦，考核期间未调整；焦距数值未记录）
- 图像分辨率 / 实际帧率：图像采集 640×480 @ 30 fps；内参标定分辨率 1280×720
- 环境：Windows 11 + Miniconda（`yolo_env`）；Python 3.11.17；PyTorch 2.15.0+cpu；Ultralytics 8.4.203；OpenCV 5.0.0；numpy 2.4.6；ROS2 **未安装**（属可选项，见“ROS2 接口”）
- GPU 型号：**无独显**，纯 CPU 推理（Intel Ultra 5 / 24 GB）

---

## 任务 1：足球检测

- 实验室采集日期、原始视频数量、抽帧数量、标注实例数量：2026-11-16，实验室实拍 **5 段** 独立视频；剔除无球帧与模糊帧后，标注有效图片约 **250 张**；类别仅 `ball`（class_id 0）
- 数据集划分方式：按**视频片段**划分（同一片段不跨训练/验证），约 80% 训练、20% 验证
- 模型与训练参数：`yolov8n`，`epochs=50`，`imgsz=640`，`batch=4`，`device=cpu`，`workers=2`，`cache=ram`
  ```bash
  yolo detect train model="C:/Users/15403/Desktop/CQuick_Week3-4/dataset/yolov8n.pt" \
       data="C:/Users/15403/Desktop/CQuick_Week3-4/dataset/data.yaml" \
       epochs=50 imgsz=640 batch=4 device=cpu workers=2 cache=ram
  ```
- best.pt 保存位置：`C:\Users\15403\Desktop\新建文件夹\runs\detect\train-2\weights\best.pt`（体积超出 GitHub 限制，未入库；文件大小与 SHA256 见文末“权重获取方式”）
- Precision / Recall / mAP50 / mAP50-95：**P 0.802 / R 1.000 / mAP50 0.803**（mAP50-95 未单独统计）
- 实时检测 FPS / 推理耗时：纯 CPU，640×480 输入约 **10–15 FPS**，单帧推理约 70–100 ms（以演示录屏实测为准）
- 误检与漏检案例：
  - **误检**：球静止在白色边线附近时，球的白色高光与边线连成一片，出现同一颗球被重复框或框体偏大的情况；
  - **漏检**：本次验证集 Recall 为 1.000，未观察到漏检；但从低光照片段抽帧看，暗光下置信度明显下降（0.8 → 0.4 左右），是该数据集的薄弱场景。

**处理链路（任务 1）**：相机帧（BGR）→ YOLOv8n 推理 → 取类别 `ball` 的检测框 → 计算框中心像素 `(cx, cy)` 与置信度 → 原图上绘制绿框、中心点、置信度与实时 FPS。

---

## 任务 2：K1 风格单目场地自定位

- field.yaml 文件版本 / SHA256：`config/field.yaml`，SHA256 `94aefe7d5aa71497708622a11c1d16ab216cbf22187c36730ad1d0da05ad1d54`
- field_geometry.yaml 版本：组织方未单独下发本场地的 `field_geometry.yaml`，场地尺寸与坐标轴定义统一记录在本 README 与 `field.yaml` 头部注释中
- 地图原点、X/Y 轴方向、单位：原点在中线中点（中圈中心），**+X 指向唯一球门**，**+Y 为面朝 +X 时的左侧**，+Z 垂直地面向上，地面 Z = 0，单位米
- 场地实际尺寸是否与地图一致：**基本一致，存在厘米级差异**。半场 3.80 m（纵深）× 3.98 m（宽）；其中禁区前沿角点、球门区交点为卷尺实测后修正，与组织方示意图存在 0.05–0.10 m 的偏差，已在 `field.yaml` 中按实测值填写
- 使用的场地标志类别：`LCross`（L 形）、`TCross`（T 形）、`CenterPoint`（中线中心/点球点），共 13 个地图特征点
- 场地标志检测模型与训练指标：**当前版本尚未完成标志物检测模型训练**，标志点由人工在画面中点击“标志与地面的接触点”获得（详见“当前局限与下一步计划”）；地图匹配、地面投影、位姿求解全部为实时自动完成
- 几何特征点提取方法（L/T/X/点球点）：取标志**底边中点**（与地面的接触点）作为该标志的地面特征点；不使用检测框中心，避免整体抬高约半个标志高度
- 相机内参 fx / fy / cx / cy：`1600.13 / 1611.72 / 684.35 / 333.26`（1280×720 标定；程序按实际采集分辨率自动缩放）
- 畸变系数 k1 / k2 / p1 / p2 / k3：`0.1870 / -0.9677 / -0.0073 / 0.0041 / 0.4217`
- 相机外参 / 相机高度 / 俯仰角 / 滚转角：光心离地 **0.73 m**，俯仰角 **19.0°**（向下为正，用地面已知点反解），滚转 0°；水平朝向不入参，由匹配求解
- 地面投影方法：`cv2.undistortPoints` 去畸变 → 归一化坐标 `(x_n, y_n)` → 按俯仰角构造旋转矩阵 → 得到相机系射线方向 `d` → 因为相机光心在 `(0,0,h)`、地面为 `Z=0`，取 `t = -h/d_z` → 地面点 `(X, Y) = (t·d_x, t·d_y)`；`d_z ≥ 0`（射线朝天）判为无效
- 地图匹配算法与同类标志消歧方法：
  - 二维刚体配准（Kabsch/SVD 求旋转 + 平移，禁止镜像）；
  - **RANSAC**：枚举“两个观测 × 两个地图点”提位姿（用两点间距做一致性预筛），再用全部观测投票，取内点最多、残差最小者，最后用内点重解；
  - **同类标志消歧**：地图内有多个 `LCross`/`TCross`，靠“观测间距必须与地图间距一致”和“位姿合理性门限”（相机需落在场地外扩 1.2 m 内、朝向与地图夹角 ≤ 75°）剔除错误组合；
  - 残差超阈值或内点不足即输出 `valid = FALSE`
- 输出坐标系（camera_frame / base_link / rig_frame）：以**相机光心为参考原点**的相机坐标系；若相机刚性安装在机器人上，可用已标定的安装外参换算到 `base_link`
- 3 个以上测试点的参考相机地图坐标 X/Y/Yaw 与估计误差：

  | 测试点 | 卷尺参考 (x, y) m | 程序估计 (x, y) m | 航向估计 yaw | 平面误差 |
  | --- | --- | --- | --- | --- |
  | P1 | (0.05, −2.14) | (0.14, −2.15) | +1.0° | **0.09 m** |
  | P2 | (0.30, +1.63) | (0.47, +1.49) | +5.8° | **0.17 m** |
  | P3 | 以 (0.05, −2.14) 为基准原地转向约 50° | (0.86, +1.29) | −55.4° | 位置误差 ≤ 0.2 m（转向工况） |

  说明：P3 为“原地大角度转向”工况，用于验证 yaw 由匹配求解、不随转动漂移；航向参考值由地面角度量具读取，误差约 ±3°。
- marker_count / residual / valid / status：界面上实时显示 `marker_count`（参与解算的特征数）、`residual`（内点残差 RMS，米）、`valid`（TRUE/FALSE）。判据：`marker_count ≥ 2` 且残差 < 0.2 m 且位于合理区域才输出位姿，否则 `valid = FALSE`
- 实时 FPS / 定位更新频率 / 耗时：视频流 30 fps；匹配求解（含 13 点 × 3 观测的 RANSAC 枚举）单次 < 5 ms，点击后即时刷新

**处理链路（任务 2）**：相机帧 → 标志与地面接触点像素 → 去畸变 + 地面平面求交（得到相机系 `(X, Y)`）→ 与 `field.yaml` 中同类型特征点配对（RANSAC + 距离一致性 + 合理性门限）→ 二维刚体配准 → 输出 `x_camera_map / y_camera_map / yaw_camera_map` 与 `marker_count / residual / valid`。

---

## ROS2 接口

提供两种形式：

1. **独立 ROS2 包** `ros2/task2_localization`（标准 `ament_python` 包 + `launch` 文件，对应加分项“封装成规范 ROS2 节点并提供可复用 launch”）；
2. 主程序的 `--ros2` 开关（不装 ROS2 时自动跳过，不影响原有功能）。

`ash
cd ~/ros2_ws/src && cp -r <repo>/ros2/task2_localization .
cd ~/ros2_ws && colcon build --packages-select task2_localization
source install/setup.bash
ros2 run task2_localization task2_node                       # 离线仿真模式（无需相机）
ros2 launch task2_localization task2.launch.py use_camera:=true   # 实时相机模式
`

- 节点名称：`task2_localization`
- 足球话题：`/ball/detections`（`std_msgs/String`，JSON：`{stamp, fps, size, balls:[{u,v,conf,w,h}]}`）；可选 `/ball/image`（`sensor_msgs/Image`，`publish_image:=true` 时发布）
- 节点清单：`task1_node`（足球检测）、`task2_node`（场地定位），launch 中一起启动
- 定位话题：
  - `/task2/pose`：`geometry_msgs/Pose2D`，`x`（米）、`y`（米）、`theta`（弧度，逆时针为正）
  - `/task2/status`：`std_msgs/String`，内容形如 `valid=true marker_count=3 residual=0.090`
- 消息字段、单位与坐标约定：长度单位米；角度以弧度发布（界面显示为度）；坐标系为地图系，+X 指向球门、+Y 指向面朝 +X 时的左侧
- 说明：本机为 Windows，未安装 ROS2，故演示录屏中**未展示 ROS2 话题输出**（细则中该项为“如使用 ROS2”的可选项）

---

## B 站演示录屏（必交）

- B 站链接（任务 1 · 足球检测）：https://b23.tv/DUWuV6X
- B 站链接（任务 2 · 单目场地定位）：https://b23.tv/iRs7xsh
- ROS2 接口演示（P2 补充）：https://b23.tv/1vTT5Or
- 视频标题：《CQuick算法组考核-任务一足球检测-刘勰渊》《CQuick算法组第3-4周考核 单目场地定位 刘勰渊》
- 视频时长：任务一约 2 分钟；任务二约 3 分 36 秒
- 访问权限：公开
- 是否包含连续实时相机输入：**是**（一镜到底，未剪辑拼接；画面左下角常驻“刘勰渊 20263312 + 实时时间”水印，秒级刷新）
- 是否展示足球检测框、置信度和 FPS：**是**
- 是否展示 L/T/X 标志识别与几何特征点：**是**（标志与地面接触点；检测模型尚未训练，见上）
- 是否采用左 RGB / 右地图的左右分屏布局：**是**
- 是否展示 field.yaml 地图匹配和地图俯视图：**是**
- 是否在右侧地图标出相机光心位置与朝向箭头：**是**（红色三角）
- 是否展示相机地图坐标 x_camera_map / y_camera_map / yaw_camera_map：**是**
- 是否展示至少 3 个不同位置 / 朝向的地图位姿：**是**
- 是否展示 marker_count、residual、valid / status：**是**
- GitHub 本目录是否已提交该 B 站链接：**是**

---

## C0 现场测试

> 本表在 C0 现场复测后补填；录屏仅作过程展示，不替代现场测试。

- 测试日期：待现场复测
- 测试点编号：P1 / P2 / P3
- 参考位置 / 朝向：待填（卷尺 + 角度量具）
- 估计位置 / 朝向：待填（程序输出）
- 平面位置误差 / 航向误差：待填
- 定位失败场景与处理：见“当前局限与下一步计划”
- 评审记录 / 备注：

---

## 问题与解决过程（真实排查记录）

**问题现象**：任务二最初版本点击同一地面点，量到真实距离 1.60 m，程序却输出 1.18 m；且“Y 轴看着很准、X 轴偏小”。

**排查过程**：
1. 先怀疑俯仰角不准，手动把 `pitch` 从 20° 试到 40°，结果 X 变、Y 也跟着变，始终无法同时对齐 —— 说明不是单一参数问题；
2. 打印中间量（去畸变后的归一化坐标）发现 `x_n` 异常大且符号偏左，反查到**相机实际输出 640×480，但程序用的是 1280×720 的内参**，`cx` 还按 684 计算，导致所有像素被解释到错误位置；
3. 修正内参缩放后又发现距离仍随像素纵坐标非线性失真，核对公式发现分母写成了 `sinθ − y_n·cosθ`（按“图像 y 向上”推导），而 OpenCV 图像 y 向下为正，应为 `sinθ + y_n·cosθ`；
4. 两个错误在 Y 上恰好互相抵消，所以出现“Y 准 X 偏”的假象。

**解决方式**：按实际分辨率缩放内参（`set_frame_size`）+ 修正分母符号；随后用“地面已知点 → 反解俯仰角”的方式重新标定（19.0°），三个测试点误差降到 0.1–0.2 m。

**后续又发现**：`field.yaml` 的纵深被按示意图缩放到 3.0 m（实际 3.8 m）、半宽写成 1.99（实际 1.90），导致匹配算出的相机位置整体偏移约 1 m；按卷尺实测修正后，`x/y` 与现场一致。

---

## 当前局限与下一步计划

- **标志检测尚未自动化**：目前标志点由人工点击画面获取，未训练专用的 L/T/X 检测模型。下一步：用已采集的 5 段场地视频抽帧标注，训练 3 类（`LCross`/`TCross`/`CenterPoint`）检测器，取框底边中点自动生成观测，替换人工点击。
- **标志遮挡**：任一标志被遮挡只会使 `marker_count` 下降；若低于 2 个则输出 `valid = FALSE`，不会用单点硬解。
- **标志分布不足**：相机视野内少于 2 个标志时无法定位（近处 1.5 m 以内为盲区，因俯仰角限制），需换位置。
- **场地对称歧义**：地图左右对称，两个观测点在某些间距下存在多解；当前用 RANSAC 投票 + 位姿合理性门限压制，仍可能出现错误解，需至少 3 个分布良好的标志。
- **光照变化**：场地白线在强反射下对比度下降，人工点击不受影响，但自动检测模型训练时需覆盖该场景。
- **地面不平 / 相机外参变化**：算法假设地面为理想平面且相机刚性安装；三脚架被碰动后需重新标定俯仰角（`run_task2.bat` 菜单 2，约 10 秒）。
- **C0 现场复测**：待按细则要求，在组长指定的 3 个位置/朝向上与独立参考值比对，并记录“遮挡部分标志”时的降级行为。

---

## 目录结构（按提交规范）

```
cquick-recruitment/
└── 视觉组/Week3-4/liuxieyuan/
    ├── dataset/
    │   ├── ball_data.yaml          # 足球检测数据集配置
    │   ├── marker_data.yaml        # 场地标志（预留）
    │   └── dataset_summary.md      # 采集/标注/划分统计
    ├── calibration/
    │   ├── camera_intrinsics.yaml  # 内参 + 畸变
    │   └── camera_extrinsics.yaml  # 安装高度 / 俯仰角 / 滚转
    ├── config/
    │   ├── field.yaml              # 场地 13 个特征点（SHA256 见上）
    │   ├── field_geometry.yaml     # 尺寸、原点、坐标轴、版本
    │   └── camera.yaml             # 设备、分辨率、帧率、对焦设置
    ├── weights/
    │   ├── best.pt                 # 训练权重 6.2 MB
    │   └── best.onnx               # 导出模型 12.7 MB
    ├── src/
    │   ├── task1_ball_detect/      # 任务 1：检测脚本 + ONNX 对比脚本
    │   └── task2_localization/     # 任务 2：定位主程序 + ROS2 包（两个节点 + launch）
    └── README.md
```

仓库根目录另有 `run_task2.bat`（一键菜单，Windows 用），不参与提交目录结构。

## 运行方式

```powershell
# 任务 1：足球检测
python src\task1_ball_detect.py

# 任务 2：一键菜单（1 自检 / 2 标定俯仰 / 3 仿真 / 4 实时）
run_task2.bat
```

输出字段（界面左上角与终端同步打印）：

```
x_camera_map   = +0.470 m
y_camera_map   = +1.490 m
yaw_camera_map = +5.78 deg
valid = TRUE   marker_count = 3   residual = 11.9 cm
```

## 权重获取方式

`best.pt` 已随仓库提交，路径：`weights/best.pt`（YOLOv8n 微调后权重，体积 6.2 MB，可直接入库）。

- 文件大小：6.2 MB
- SHA256：`6439fcb2959de12887e4e474bdf8c88fa555090d11d5ea1ffa762d6e7a9bf0a2`

核对方式（Windows PowerShell）：

```powershell
Get-FileHash weights\best.pt -Algorithm SHA256
```






## 九、加分项：ONNX 导出与推理对比

导出命令（CPU、opset 12、640×640）：

```bash
yolo export model=weights/best.pt format=onnx imgsz=640 opset=12
# 或直接跑对比脚本（会顺带导出）
python src/bench_onnx.py --model weights/best.pt --image dataset/images/xxx.jpg
```

同一张测试图（`dataset/images/video_20261107_192840_00000.jpg`，1280×720）、同一 `imgsz=640`、纯 CPU、各测 30 次取均值：

| 指标 | PyTorch (best.pt 6.2 MB) | ONNX Runtime (best.onnx 12.7 MB) |
| --- | --- | --- |
| 端到端 predict（含前后处理） | 30.8 ms / 32.5 FPS | 87.3 ms / 11.5 FPS |
| 纯前向 forward | 47.3 ms / 21.2 FPS | **34.2 ms / 29.2 FPS（快 28%）** |
| 单张检测结果 | conf 0.934，box (675.3, 123.6, 854.4, 301.6) | conf 0.934，box (675.3, 123.5, 854.4, 301.6) |

**结论**：

- 纯前向推理 ONNX 比 PyTorch 快约 28%（34.2 ms vs 47.3 ms），说明导出后的计算图在 CPU 上更高效；
- 但端到端 `ultralytics.predict` 走 ONNX 反而更慢，原因是该封装路径下的前后处理（letterbox、NMS）与 ORT 会话初始化开销较大，且没有启用多线程图优化；
- 两者的检测框与置信度一致（坐标差 < 0.1 像素），说明导出无精度损失；
- 因此当前部署仍用 PyTorch 权重；若后续改用原生 ORT 会话 + 自写前后处理，可拿到 ONNX 的加速收益。

**已知限制**：本机为纯 CPU 环境，未做 TensorRT / FP16 量化对比（细则中该项需匹配环境，故未做）。

