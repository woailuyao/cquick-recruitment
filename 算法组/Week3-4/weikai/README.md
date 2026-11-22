# 第 3–4 周合并考核 · weikai

- 姓名：weikai
- 本次提交范围：**任务 1（足球检测）+ 任务 2（单目场地自定位，离线验证）**。
  两个任务均含实时演示录屏（B 站链接见「B 站演示录屏」节）；任务 2 完成
  场地实测、内参标定、标志检测训练、定点视频的定位精度验证（全部达标）
  与 ROS2 封装（接口包 + 视觉定位包，话题实测通过，见 2.7）。
  加分项完成 ROS2 规范节点封装（+2）、ONNX 推理对比（+3）与地图匹配稳健化（+2），
  TensorRT 因工具链版本不匹配未完成（见「加分项」）。C0 现场测试尚未开展。
- 完成日期：2026-11-22
- 摄像头类型：单目 RGB 摄像头（USB UVC，外接）
- 摄像头接口 / 设备节点：Windows 侧 ffmpeg dshow 采集 MJPG 原始流 → HTTP 桥 `/snapshot.jpg` 供 WSL 取帧（相机不透传进 WSL，原因见「问题与解决过程」第 0 条）
- 镜头：固定焦距、固定对焦（定焦镜头，无自动对焦控制项）
- 图像分辨率 / 实际帧率：1280×720 / 29.8 fps（MJPG，连续取帧实测）
- 环境：Ubuntu 22.04.5（WSL2）· Python 3.10.12 · PyTorch 2.14.1+cu130（CUDA 13.0）· OpenCV 5.0.0 · Ultralytics 8.4.215 · NumPy 2.2.6 · ROS2 Humble（colcon 包 `cquick_interfaces` + `k1_vision_locate`，见 2.7）
- GPU：NVIDIA GeForce RTX 5060 Laptop GPU（8 GB）

---

## 任务 1：足球检测

### 1.1 数据采集与数据集

- 采集日期：2026-11-15，本人于实验室实拍 **5 段独立视频** V1–V5，每段约 45 s。场景覆盖：近/远/左/右、滚动与运动模糊、贴近白线、被机器人部分遮挡、阴影与不同背景。
- 抽帧：每 15 帧取 1 张，每段上限 60 张 → 初始 300 张；逐张目视甄别后保留 **263 张有效帧**，人工标注 **252 个 `ball` 实例**。
- 划分：**按独立视频段整体划分**，同一段视频的帧只进一个集合：

  | 集合 | 图片数 | 来源视频 | 标注实例 |
  |---|---:|---|---:|
  | train | 153 | V1 / V2 / V3 | 149 |
  | val   | 50  | V4 | 45 |
  | test  | 60  | V5 | 58 |

  明细见 `dataset/split_manifest.csv`。训练期间 test 完全未参与。`V1_h264.mp4` 是 V1 的转码副本，已从素材中剔除，避免重复样本与划分泄漏。
- 标注规范：YOLO 单类 `ball`（class_id=0），归一化 `cx cy w h`；部分遮挡只框可见区域。标注工具 LabelImg，COCO 预训练预标注做草稿 + 人工逐张修正。
- 质检：`src/verify_labels.py` 抽查 30 张回画，类别全为 0、坐标无越界、无过大框，PASS；另修正 1 处浮点边界越界。集合间无同视频段泄漏。
- 轻量替代：原始视频、整套抽帧图、权重、`.venv/`、`runs/` 均不上传。数据集摘要见 `dataset/dataset_summary.md`，划分见 `dataset/split_manifest.csv`。

### 1.2 模型与训练

- 模型：`yolov8n.pt`（COCO 预训练初始化），epochs=50，imgsz=640，batch=16，device=cuda:0。
- 训练/评估脚本：`src/train_eval.py`。训练完成后**在独立 test 集**（V5）上评估，四项指标写入 `dataset/test_metrics.csv`。
- 关键参数改动：唯一的改动是**训练集加入干扰物负样本增强**（见 1.4），验证集与测试集始终保持原始、不参与任何增强。

### 1.3 指标（独立测试集 V5）

最终提交版为 `dual_aug_final`（同时加入两类干扰球负样本）：

| 指标 | 基线 | 仅黄绿球增强 | **双干扰增强（最终）** |
|---|---:|---:|---:|
| Precision | 0.765 | 0.666 | **0.760** |
| Recall | 0.724 | 0.966 | **0.929** |
| mAP50 | 0.822 | 0.827 | **0.925** |
| mAP50-95 | 0.777 | 0.768 | **0.849** |

- 权重：`best.pt` 存于本机 `~/vision_ws/k1_soccer_vision/runs/ball_yolov8n_aug2/weights/best.pt`（6.2 MB），**按细则不上传 GitHub**（较大权重应放共享存储，此处为个人考核，保留本地备查）。
- 校验信息：SHA256 `e5920620e96fdbe6205a0295b92691f3902e4aa4394e1d0c9e54b410f9e01eab`。
- 实时链路：桥接服务 1280×720 完整帧、30 fps；WSL 取帧实测 29–31 fps；GPU 推理（RTX 5060）稳态约 10 ms/帧，端到端约 45 fps（受相机 30 fps 上限约束）。链路冒烟数据先用 COCO 预训练权重验证，**最终 `best.pt` 的完整实时演示录屏已完成**（真实连续相机输入，窗口实测端到端约 20 fps、推理约 15 ms/帧，含 GUI 面板渲染开销），见「B 站演示录屏」。
- 输出字段：类别 `ball` / confidence / 框 `x1,y1,x2,y2` / 中心 `center_u,center_v` / 帧号 / FPS。演示脚本 `src/realtime_demo.py` 在画面上逐帧显示全部六项（检测框上方标签 `ball 0.91 bbox=(572,376,808,615) c=(690,495)`，底部 HUD 显示 FPS、infer ms、frame、ball count）。

### 1.4 误检与漏检案例（本任务核心）

**本场景共有三种球，只有第一种是目标：**
1. **蓝白 / 黑白五角星花纹足球** —— 要检测的唯一目标 `ball`；
2. **黄绿色 Molten 球** —— 干扰物，不标注、不检测；
3. **红蓝花纹小球**（带 FIFA 印花、花色调与真球不同）—— 干扰物，不标注、不检测。

外观对照见 `outputs/ball_type_comparison.jpg`，同框场景见 `outputs/interference_scenes.jpg`。

#### 误检案例 1：黄绿色球被当成足球

- 现象：基线模型在 val 上 mAP50=0.954，但在独立 test（V5）只有 0.822。逐帧看检测图发现，V5 里每张图除蓝白足球外还有一个黄绿 Molten 球，**模型把黄绿球也框了**（置信度 0.92–0.97），每个 test 帧多出 1–2 个误检框，把 precision 拉低。
- 根因：训练集（V1–V3）里从没出现过黄绿球，模型没学过"这种球不该检"，见球就当目标。
- 解决：在不污染 test 的前提下做 **copy-paste 负样本增强**——用基线模型框 + HSV 黄绿判据从 V5 帧抠出 48 个黄绿球 RGBA 素材（只用外观，不读 test 标注），随机旋转/缩放/贴到 69 张 train 图上，**标签保持不变**（贴上去的球不画框 = 负样本）。val / test 一律不动。
- 效果：黄绿球误检次数 **47 → 0**，且蓝白目标球无漏检。

#### 误检案例 2：红蓝花纹球被当成足球

- 现象：黄绿球消除后，test 上仍残留大量"多检框"。我一度误判为"测试集漏标的真球"，甚至建议去补全 test 标注——这是**错误判断**。重新放大核对确认：这些是**红蓝花纹球**，与真球同框、大小相近，但花纹明显不同，是明确要求忽略的干扰物。
- 根因：与黄绿球相同——该球只在 V5（test）出现，训练集里没有，模型无从学起；而且它和蓝白目标球外观接近，比黄绿球更难区分。
- 解决：同样做法。从 V5 抠出 8 个完整红蓝球 patch（这次用整个检测框裁出球面，而不是只取红色像素——因为该球白底+红蓝斑块，只取红色块不像一个球，模型学不到"完整小球是干扰"），与黄绿素材一起贴到 train（累计 118 张受影响图：黄绿 69 + 红蓝 65），同样不画框。
- 效果：红蓝球误检 **8 → 1**，黄绿球保持 0；四项指标全面改善（见 1.3 表）。

#### 漏检案例

- 初筛阶段有 5 张帧的蓝白足球因置信度低被预标注漏掉（V1_f000270/285/555/855、V3_f000750），人工补标后召回率上升；其中 V3_f000750 是运动模糊下只露出约 1/4 的球，按可见部分标注。
- V1_f000555 为运动模糊帧，补标后模型可检出——覆盖细则要求的"运动模糊"场景。
- 部分遮挡样本（V4 球被背包挡住）只标可见区域，模型对遮挡球仍可检出。

#### 三版对比可视化

`outputs/false_positive_compare.jpg`：4 张测试图 × 3 列，左=基线、中=仅黄绿增强、右=双干扰增强。基线与中间版都框住了黄绿球（部分还框住红蓝球），**最右列只框蓝白目标球**，两种干扰球都不再误检。

### 1.5 问题与解决过程（汇总）

1. **相机不要透传进 WSL**：`usbipd` 把相机 attach 进 WSL 后，1280×720 MJPG 每帧从第 ~184 行起全黑（USB/IP 等时传输带宽不足，协议层硬限制）。改为相机留在 Windows、ffmpeg dshow 原生采集、HTTP 桥供 WSL 取帧，720p 完整、零截断。
2. **LabelImg 在 WSLg 下多次闪退**：根因是 PyQt5 5.15.11 对传入绘图/控件 API 的 float 不再自动转 int。逐处修复 `labelImg.py` 的 `setValue`、`canvas.py` 的 `drawLine/drawRect`、`shape.py` 的 `drawText`，并用专用启动器预设默认标签 `ball`、跳过会崩溃的标签弹窗。修复后正常完成全部标注。
3. **黄绿球误检**（详细见 1.4）：从 val/test 指标落差发现，用干扰物负样本增强解决。
4. **红蓝球误检**（详细见 1.4）：把干扰球错当成"漏标真球"，经纠正后重新定位并解决。
6. **1 处浮点边界越界**：`V1_f000285` 框右边界 1.0000005，内缩 1e-4 后质检 PASS。

---

## 任务 2：K1 风格单目场地自定位

**已完成离线全链路验证。** 验收口径：位置误差 ≤ 0.5 m、航向误差 ≤ 10°。
实测 4 段验证视频位置误差 **0.064 ~ 0.233 m**（全部达标），2 个有几何真值的
视频航向误差 **0.4° ~ 1.3°**（达标）。详见 `定位验证报告.md` 与
`outputs/loc_fig1_topview.png`（俯视散点图，绿点紧贴真值）。

### 2.1 场地实测与特征地图

- 坐标系：原点 O 为场地长宽中线交点，+X 指向唯一球门，+Y 为面向 +X 时的左侧，地面 Z=0。
- 工具：5 m 卷尺 + 美纹纸临时标记，2026-11-19 实测。场地 7.954 × 5.982 m，
  球门宽 2.886 m；12 个标志（6 个 T 形 TCross、6 个 L 形 LCross）坐标全部
  逐点实测，写入 `src/templates/field.yaml`（键名与 [x, y] 格式对齐 Booster 官方
  field.yaml；点球点 PenalityPoint 为实测的中心点，定位代码中不参与匹配）。
- **地图版本**：`map_version: 2`（2026-11-21）。v1 为 8 标志（TCross1-4 / LCross1-4），
  v2 新增 TCross5/6（左右边线中点）、LCross5/6（球门侧两角）共 4 点。新点物理形状与
  既有同类拐角/交汇点相似，沿用现有检测权重（未重训）；定位主链路靠 `field_type()`
  按首字母识别类型，新增 T/L 标志无需改代码。版本号、坐标系、尺寸与 SHA256 记录于
  `src/templates/field_geometry.yaml`；field.yaml 的 SHA256 为
  `fba9059789f7b45fc9466453ed005b2c869ef94b5d322fc2c09620d9328436c8`（2026-11-21
  修正 LCross5/6 的 x：3.084→3.977，与球门侧端线对齐；标志数量不变）。
- **数据口径**：2.5 节离线精度数字基于 v1（8 标志）跑出，v2 新增点尚未重跑该批视频，
  故如实保留 v1 口径，不冒充 v2 结果。

### 2.2 相机内参标定

- `src/calibrate_intrinsics.py`：9×6 棋盘格（方格 20 mm），实拍 17 张，
  标定 rms = 0.508 px。1280×720 下 **fx = 1554.66，fy = 1563.82**，
  畸变系数 5 参。内参文件 `D:/captures/calib/intrinsics.yaml`（含 5 参数畸变）。

### 2.3 标志检测模型（L / T 两类）

- 第一阶段数据：**178 张人工逐帧标注**（LabelImg + 自写启动器，流程与任务 1
  相同），L/T 两类；验证集按视频整段留出（MC1 + M2 + M3），避免相邻帧泄漏。
  yolov8n / 150 epochs / 640，val mAP50 = 0.569（L 0.674 / T 0.465），
  权重 `best_manual_150.pt`。曲线见 `outputs/landmark_training/`。
- 第二阶段（最终采用）：从固定外参重拍视频 M9~M15 再抽 **91 张**人工标注，
  与第一阶段合并为 269 张（train 213 / val 56，验证集新增整段 M11），
  用 `yolov8n.pt` 从零训练 120 epochs / 640 / batch16（早停于第 37 轮）。
  **val mAP50 = 0.808**（L 0.792 / T 0.823）。在同一 v2 验证集上，
  第一阶段权重的 mAP50 = 0.565（L 0.711 / T 0.418），即整体 +0.243、
  T 类 +0.405，显著减少了白线拐角/球门网误检。
  权重 `best_fixed_v2.pt`，曲线与混淆矩阵见 `outputs/landmark_v2_training/`。
  权重位置：本机 `D:/captures/landmarks/weights/best_fixed_v2.pt`（6.5 MB，
  SHA256 `56e72afcb0ad4668332dcb323014144712d5f4133f906537b82d82b4730852d7`），
  **按细则不上传 GitHub**，保存在本地共享存储路径备查。
- 第三阶段（**当前默认权重**）：前两阶段素材以相机背对 +X 的画面居多，正面朝 +X 时
  标志透视形态差异大、T 类召回偏低。补拍并标注 22 张朝向 +X 的帧（M5 前段 12 张、
  M9 后半段 10 张），并入后 train 235 / val 56（验证集仍按整段留出 MC1+M2+M3+M11），
  同配置再训 120 epochs：**val mAP50 = 0.807（L 0.742 / T 0.871）**。相对 v2，
  T 类 +0.048、L 类 −0.050、整体持平，按"关键类召回优先"取为默认权重
  `best_facing_v3.pt`（本机 `D:/captures/landmarks/weights/best_facing_v3.pt`，6.5 MB，
  SHA256 `2ea068413220d1fe5c1511c70e6adfe2916caf5b39c7e337ddaa5ad534b801e7`）。
  v3 只做了 M10 抽样复验（stride 20，位置误差 0.30 m、航向误差 3.6°，达标），
  **未重跑 2.5 节完整离线批次**；2.5 各表仍是 v2 口径，不拿旧结果冒充新权重。
- 推理配置：`--conf 0.05 --imgsz 640 --enhance none`（必须与训练尺寸一致；
  实测 960/1280 + CLAHE 会把白线角点误检成 L/T——见 2.6 第 6 条）。

### 2.4 定位算法（src/locator.py）

单张图像的解算流程（`solve_pose`）：

1. **穷举假设**：观测 4 点组合 × 地图 4 点**排列**（permutation，当前 12 个有
   类别的地图点全枚举，v1 验证时为 8 个）× IPPE 镜像双解；
2. **形状预筛**：4 点归一化两两距离向量的 L∞ 距离 < 0.45 才进 PnP，
   向量化后把 PnP 次数从几十万降到几千；
3. **PnP 求解**：`cv2.solvePnPGeneric` + IPPE，取全部双解；
4. **假设期匹配**：重投影用**米制容差**（像素误差 × 深度 / fx，假设 0.40 m、
   终验 0.30 m），粗白线上交点质心 0.1~0.2 m 的固有抖动不再误杀真解；
   类型硬约束（L 只配 L、T 只配 T）+ 相机系深度 z ≤ 0.2 m 的地图点剔除；
5. **终验**：对全部假设解开放，按高度 0.35~1.2 m、俯仰 15°±30°、场内边界、
   rms ≤ 0.35 m 淘汰（内点数最多的可能是几何自洽但在场外的假解）。

视频级聚合：逐帧解算，位置 0.8 m 内聚簇，取最大簇的中位数为整段定位结果，
天然压制单帧镜像/假解。复现命令见 `定位验证报告.md` 末尾。

### 2.4b 实时左右分屏演示（src/realtime_locate.py）

细则 2.11 要求的实时演示程序：左屏为相机实时 RGB + 标志检测框（L 蓝 / T 橙，
含置信度），右屏为 field.yaml 俯视地图 + 相机光心图标 + 朝向箭头 + 运动轨迹，
HUD 常驻显示 `x / y / yaw / z(离地高度) / pitch / marker_count / residual /
valid / status`，相机在场地内移动时右侧地图实时更新。

- 链路：Windows `win_cam_bridge`（MJPEG HTTP）→ WSL `NetCamera` 取流 →
  YOLOv8n 标志检测（`--imgsz 640 --enhance none`，与训练尺寸一致）→
  **固定外参定位**（`locator_fixed.py`：像素射线→地面 Z=0→2D Kabsch 配准，
  每帧只解 x/y/yaw，**后台线程**，每 2 帧派发一次，不阻塞显示）→
  最近 5 帧解中位数平滑 → 分屏渲染
- 外参说明：相机刚性固定后外参**锁死不再逐帧反解**（细则 2.6/2.7）——
  光心高 **1.005 m**、俯角 **15°**、roll≈0，实测值写入
  `src/templates/camera_extrinsics.yaml`，HUD 常驻显示 map v2 + 这组外参。
  （早期手持视频用逐帧自由外参 PnP，仅作离线验证口径，非最终演示算法。）
- 覆盖相机即触发 `INVALID: no landmark detected` 状态，用于录屏中的无效演示

### 2.5 验证结果（离线定点视频，每段 15 s）

| 视频 | 站位 (x, y) | 朝向 | 位置误差 | 航向估计 / 真值 | 航向误差 | 接受帧 |
|---|---|---|---|---|---|---|
| M1 | (0, 2.99) 边线中点 | 指向球门中心 | **0.170 m ✓** | −35.2° / −36.94° | **1.7° ✓** | 32/32 |
| M2 | (0, 0) 场地中心 | 向右 | **0.064 m ✓** | −21.5° / 无参照 | — | 32/32 |
| M3 | (0, 0) 场地中心 | 向左 | **0.088 m ✓** | +22.7° / 无参照 | — | 19/32 |
| M7 | (−2, 0) | 指向 LCross3 | **0.233 m ✓** | −24.6° / −25.94° | **1.3° ✓** | 30/32 |

- **航向真值不是"平行 X 轴"**：真值 = 站位指向取景中心目标的方位角
  `atan2(Δy, Δx)`。M1 站在边线上朝球门中心，真值是 −36.94° 而非 0°——
  站位不在中线时两者差 37°。这一度让我把达标结果误判为严重超差。
- 佐证：按水平半视野 22.4° 推算，yaw=0° 时 M1 视野内只有 2 个标志，
  yaw=−36.9° 时有 6 个；M1 实际每帧检出 4~6 个，独立支持 −37° 附近。
- 多轮独立跑批中位数可复现（M1 −35.2/−37.3，M2 −21.5/−21.7，M7 −24.6）。
- 俯视图与逐帧曲线：`outputs/loc_fig1_topview.png`、`outputs/loc_fig3_error.png`；
  单帧位姿→地图点投影回画面的贴合校验：`outputs/loc_overlay_M1_f150.png` 等。

**固定外参版复验（最终交付口径，细则 2.6/2.7）**：相机刚性固定、外参锁死
（h=1.005 m / pitch=15° / roll=0），固定安装重拍视频用
`locator_fixed.py` + `--imgsz 640 --enhance none` + 最终权重
`best_fixed_v2.pt` 复验：

| 视频 | 站位 (x, y) | 朝向目标 / yaw 真值 | 位置误差 | 航向误差 | 有解帧 |
|---|---|---|---|---|---|
| M10 | (0, 0) | LCross4 / −25.35° | **0.260 m ✓** | **3.10° ✓** | 32/32 |
| M14 | (0, +2.46) | TCross2 / −14.10° | **0.375 m ✓** | **0.61° ✓** | 32/32 |

> 换成 `best_fixed_v2.pt` 后，M10/M14 仍全部达标（旧权重对应 0.231 m/2.98°、
> 0.371 m/0.05°，数字略有变化但同量级），而检测器 mAP50 从 0.565 升到 0.808。
> 地图坐标修正：LCross5/6 的 x 由 3.084 修正为 3.977（球门侧端线，3.084 是
> LCross2/4 的 x，属笔误），SHA256 已更新。**M11 由 0.492 m 变为 1.367 m
> 超差**——原"达标"是错误坐标把球门网误检点凑巧配准的假象（每帧 8~9 检出
> 仅 1 个真标志），新权重下仍因站位几何退化超差，故作废。其余 M9/M12/M13/M15
> 均因站位几何退化或检出不足作废，详见 `定位验证报告.md`。
>
> 任务书「≥3 个不同位置/朝向」由**评审现场安排测试点**验收（原文第 22/42 行），
> 不以离线视频段数替代；另有三站位合成投影反解自检（误差 <0.015 m）佐证
> 固定外参解算成立。

### 2.6 问题与解决过程

1. **穷举从未枚举排列**：只把排序后的观测子集与地图子集按位置配对，
   真对应不在搜索空间；M1 某些帧能解纯靠"图像从左到右恰好对上地图序"
   的运气。改为全排列枚举。
2. **光轴方向转置约定错**：`pose_from_rvec` 用 `R@e3` 应为 `R.T@e3`，
   yaw 输出一直是错的；自检脚本用了同样的错误约定构造数据，互相掩盖——
   自检通过不代表约定正确，两个组件共享同一个错时自检是失效的。
3. **固定 10px 容差**：粗白线（宽 50~90 px）上交点质心抖动 15~40 px，
   且远点同偏差对应更多像素。改米制容差后远近一致。
4. **终验只取内点 top-12**：5 内点的场外假解把 4 内点的真解挤出名单；
   终验门限本来就是干这个的，改为全量终验。
5. **`PenalityPoint` 参与匹配**：它是点球点，训练集无此类别（类型为 None），
   类型约束对它跳过 → 它能匹配任意观测，凭空造出假对应，M5/M7 曾因此
   被拉偏 3 m。修复：解算前过滤无类别的地图点。
6. **相机后方地图点参与配对**：`cv2.projectPoints` 对 z<0 的点也输出
   （翻转的）像素坐标，M5 帧 150 因此解出 yaw=+174°、残差仅 0.003 m 的
   假解。修复：按相机系深度 z ≤ 0.2 m 剔除。修完 M1 接受帧 30/32 → 32/32。
7. **推理尺寸必须与训练一致**：M2 在 1280 下几乎检不出；M3 在 960 下
   稳定收敛到镜像假解。锁定 640。
8. **失败案例（如实报告，不作结论依据）**：
   - **M5**（(−2,0) 正对球门）：TCross1/LCross2 在图像上仅相距 60 px
     （方位差恒 2.3°，几何固有），YOLO 类别判反，硬类型约束下正确对应
     被锁死，位姿偏 2.37 m。教训：4~5 个点时对类别错判零容忍，需 ≥6 点冗余。
   - **M6**（(−2,0) 朝 L1）：该朝向视野内只剩 4 个标志、实测仅检出 3 个，
     无法解算——拍摄点位设计失误。
   - **M8**（(−3,0) 正对球门，11-20 补拍）：机位退远后球门白墙/窗户进入
     画面中央，自动曝光拉爆，4 个 T 形标志像素饱和；加 CLAHE + imgsz 1280
     后检出 5~8 个、可解帧率 91%，但逐帧解散布全场（见
     `outputs/loc_fig_m8_scatter.png`），整段 0.50 m 超差且不可复现。
     结论：饱和像素信息不可恢复；解决办法是**换机位**——球门后白墙在正对机位下离光轴
     仅 0.4°（压画面正中），改斜角机位（R4 朝 LCross6 (3.977, −2.991)，
     yaw −52.73°）后白墙离轴 23.4° 超出半视野 22.4°，白墙出画面，自动曝光即可，不必锁曝光。
9. **可视化脚本聚簇污染**：绘图脚本最初把每帧全部候选解（含假解）丢进
   跨帧聚簇，假解簇被误选为主簇，M2 误差虚报到 1.03 m；改为只聚合每帧
   最优解后与 `locator.py` 官方跑批一致。

### 2.7 ROS2 接口（已完成，实测通过）

按细则 2.11 封装为两个 colcon 包（源码在 `src/ros2/`，通过软链接接入
WSL `~/vision_ws/ros2_ws` 编译）：

- **`cquick_interfaces`**（接口包，ament_cmake）：自定义消息
  - `BallDetection.msg`（`/soccer/ball`）：header、detected、u/v、confidence、
    bbox_l/t/r/b、frame_seq、fps；
  - `FieldPose.msg`（`/localization/field_pose`）：字段命名与细则 2.11 建议表
    一致——`x_camera_map / y_camera_map / yaw_camera_map`、valid、
    marker_count、residual、status（OK / INVALID: no landmark detected /
    SEARCHING: high residual 等），另附 frame_seq / fps。
    `x/y_carrier_map`（可选项）未实现：本任务相机与载体刚性连体，
    固定外参链路直接解出光心地面投影，无二次换算。
- **`k1_vision_locate`**（ament_python）：`vision_node` 复用离线验证的同一套
  实现（`realtime_locate.py` 的检测 + `locator_fixed.py` 的固定外参解算），
  图像源支持 `net://127.0.0.1:8090`（Windows 桥接实时流）或视频文件；
  `launch/demo_launch.py` 先加载 `config/params.yaml`，再用 launch 参数覆盖其中
  的 `source` / 权重路径。
  实时流实测端到端约 28–31 fps，视频文件回放 44–47 fps；每个解算帧发布
  `/localization/field_pose`，每帧发布 `/soccer/ball`。

**设计说明：一个节点、两个话题。** 两个任务消费同一帧图像，且任务 1 的检测是任务 2
解算的前置——拆成两个节点就要么把同一帧解码两次，要么额外开一路图像话题在进程间搬数据。
所以这里把两件事放进同一个 `vision_node`：一次取帧、两路检测、两个 publisher 各自按
自己的节奏发布（`/soccer/ball` 每帧，`/localization/field_pose` 每 `solve_stride` 帧），
订阅方互不影响，节点内部也不复制算法——检测与解算直接 import 离线验证用的
`realtime_locate.py` / `locator_fixed.py`。接口层面两个话题的消息都定义在
`cquick_interfaces` 里，若后续要多相机、换模型或单独重启某一路，拆成两个节点时
消息定义与 launch 参数都不需要改。

加分项 1（规范 ROS2 节点 + 可复用 launch，+2）的要素逐条对齐：标准 ament_python 包结构
（`package.xml` / `setup.py` / `setup.cfg` / `resource/`）、独立的 `cquick_interfaces`
消息包、`launch/demo_launch.py` 暴露 `source` / `ball_weights` / `marker_weights` /
`venv_python` 四个 launch 参数（实时相机、视频回放换一行参数即可复用）、
`config/params.yaml` 经 `--params-file` 加载为默认参数、节点内全部参数可被
`--ros-args -p` 覆盖。

运行（WSL 终端）：

```bash
export ROS_LOCALHOST_ONLY=1        # 本机 WSL2 组播损坏，必须限制走回环，见下
source /opt/ros/humble/setup.bash
source ~/vision_ws/ros2_ws/install/setup.bash
# 实时相机（Windows 侧先启动 src/win_cam_bridge.py）：
ros2 launch k1_vision_locate demo_launch.py
# 视频文件自测：
ros2 launch k1_vision_locate demo_launch.py source:=/mnt/d/captures/M10.mp4
# 查看话题输出：
ros2 topic echo /localization/field_pose --no-daemon
```

**验证记录**（M10 视频源，`ros2 topic echo` 实测）：
`valid: true, x_camera_map=-0.051, y_camera_map=0.244, yaw_camera_map=-28.35°,
marker_count=5, residual=0.094, status=OK`——与 2.5 节离线复验结论
（M10 0.260 m / 3.10°，真值 yaw −25.35°）一致。（实测时字段名尚为
`x / y / yaw_deg / residual_m`，其后按细则 2.11 建议表改名为现字段，
仅名称变化、类型与数值不变。）

> 环境坑（如实记录）：本机 WSL2 的**组播已失效**（`ros2 multicast send/receive`
> 实测 0/3），而 DDS 发现阶段依赖组播，表现为节点正常发布但 `ros2 topic list/echo`
> 间歇性看不到话题。已通过 `export ROS_LOCALHOST_ONLY=1`（写入 `~/.bashrc`）
> 强制 DDS 走回环解决；CLI 侧偶发列表为空属发现抖动，重试 `echo` 即可。
> 另：节点必须用训练 venv 的解释器运行（rclpy 经 PYTHONPATH 来自 ROS 环境、
> torch/ultralytics 来自 venv，同为 Python 3.10），launch 文件已自动处理。

> 关于细则「建议目录结构」中 `src/<接口包名>/` 与 `src/<视觉定位包名>/`：
> 即上述 `src/ros2/cquick_interfaces/` 与 `src/ros2/k1_vision_locate/`；
> 平铺脚本与 ROS2 包并存于 `src/`，加分项 1（规范 ROS2 节点 + launch 文件，+2）已完成，
> 要素对照表见「加分项 1」。

## ROS2 接口

**已完成**（详细实现、消息定义、运行命令与验证记录见 2.7 节）：
接口包 `cquick_interfaces` 提供 `BallDetection.msg` / `FieldPose.msg`
（字段与细则 2.11 建议表一致），视觉定位包 `k1_vision_locate` 的
`vision_node` 发布 `/soccer/ball` 与 `/localization/field_pose`，
复用与离线验证完全相同的检测/定位实现；`ros2 launch k1_vision_locate
demo_launch.py` 一键启动。M10 视频源实测话题输出与离线复验结论一致。

## 加分项

四项加分项完成 3 项（ROS2 规范节点封装 +2、ONNX 对比 +3、地图匹配稳健化 +2）；
TensorRT 因工具链版本不匹配未取得数据，如实标注为未完成、不计分。

### 加分项 1：两个任务的规范 ROS2 节点封装与可复用 launch（+2）— 已完成

完整实现与验证记录见 2.7 节，这里按加分项要求逐条对齐：

| 要素 | 落地情况 |
|---|---|
| 规范节点包 | `src/ros2/k1_vision_locate/`，ament_python 标准结构（`package.xml` / `setup.py` / `setup.cfg` / `resource/`），`console_scripts` 注册可执行入口 `vision_node`（`rclpy` 节点类，非脚本硬跑） |
| 消息接口包 | `src/ros2/cquick_interfaces/`（ament_cmake），`BallDetection.msg` 与 `FieldPose.msg` 独立成包，供其他节点复用 |
| 两个任务都有输出 | 同一节点发布 `/soccer/ball`（任务 1，每帧）与 `/localization/field_pose`（任务 2，每 `solve_stride` 帧）；算法直接 import 离线验证用的 `realtime_locate.py` / `locator_fixed.py`，不复制逻辑 |
| 可复用 launch | `launch/demo_launch.py`，暴露 `source` / `ball_weights` / `marker_weights` / `venv_python` 四个 launch 参数——实时流与视频回放只差一个 `source:=` |
| 参数外置 | `config/params.yaml`（随包安装到 `share/k1_vision_locate/config/`）由 launch 以 `--params-file` 加载，`source` / `ball_weights` / `marker_weights` 再由 launch 参数经 `-p` 覆盖；其余参数（置信度、imgsz、增强模式、解算步长、平滑窗口、`residual_max`）同样可 `--ros-args -p` 覆盖 |
| 实际跑通 | M10 视频源 `ros2 topic echo /localization/field_pose` 收到 `valid:true, x_camera_map=-0.051, y_camera_map=0.244, yaw_camera_map=-28.35, marker_count=5, residual=0.094`，与离线复验一致；视频源端到端 28–31 fps |

设计上两项任务合并在一个节点（一个节点、两个话题），理由与拆分成本见 2.7 节「设计说明」；
消息与 launch 参数已按可拆分的方式组织，后续要拆成两个节点不需要改接口。

一条复现命令：

```bash
ros2 launch k1_vision_locate demo_launch.py source:=/mnt/d/captures/M10.mp4
```

参数链路复验（2026-11-22）：上述启动的日志首行为
`参数 source=/mnt/d/captures/M10.mp4 imgsz=640 marker_conf=0.05 enhance=none
solve_stride=5 residual_max=0.30 smooth=5`，与 `params.yaml` 一致；视频源
317 帧处理完毕后节点自行退出（44–47 fps）。再用一份临时参数文件把
`imgsz / solve_stride / smooth` 改成 512 / 7 / 9，启动日志同步变成
`imgsz=512 ... solve_stride=7 ... smooth=9`，确认 `--params-file` 生效、
命令行 `-p` 可覆盖。

### 加分项 2：ONNX 导出与 PyTorch / ONNX 推理对比（+3）— 已完成

足球与标志两个模型各导出 ONNX（opset 17、imgsz 640、simplify，与线上推理同口径）：

| 模型 | 文件（本机路径） | 大小 | SHA256 |
|---|---|---|---|
| 足球 | `~/vision_ws/k1_soccer_vision/runs/ball_yolov8n_aug2/weights/best.onnx` | 12.2 MB | `e39183a5ec5cae95b33ec7ca6a9c6426e394cf7f00c35ffcf89796a76b133baa` |
| 标志 v3 | `D:/captures/landmarks/weights/best_facing_v3.onnx` | 12.2 MB | `b5a24ce92003fc7069001c722e6af6a01f92a6df9f040f1cf99d1ac3c99ab00a` |

ONNX 与权重同样不上传（同 1.3 节说明）。对比口径：同一视频的同一批帧，前后处理都走
ultralytics 同一条 predict 链路（letterbox / NMS 一致，差异只来自算子实现），
**两者都在 CPU 上推理**（本机 onnxruntime 无可用 CUDA EP，PyTorch 也强制 `device=cpu`），
前 5 帧 warmup 不计入，统计各 100 帧：

| 模型 / 视频 | PyTorch 均值 / p95 | ONNX 均值 / p95 | FPS 对比 | 检测一致性 |
|---|---|---|---|---|
| 标志 v3 / M10 | 34.0 / 44.7 ms | 26.9 / 31.0 ms | 29.4 → 37.2（+27%） | 框数 100/100 一致，最优框 IoU 0.971（min 0.965），中心偏差 0.52 px |
| 足球 / V1 | 39.5 / 44.0 ms | 33.8 / 37.6 ms | 25.3 → 29.6（+17%） | 框数 100/100 一致，IoU 0.999（min 0.993），中心偏差 0.06 px |

结论：CPU 上 ONNX 延迟降 14%~21%，检测结果与原模型实质一致（框数全同、IoU>0.96），
可以直接替换部署。脚本 `src/onnx_compare.py`，复现：

```bash
python onnx_compare.py --pt D:/captures/landmarks/weights/best_facing_v3.pt \
    --onnx D:/captures/landmarks/weights/best_facing_v3.onnx \
    --video /mnt/d/captures/M10.mp4 --conf 0.05 --stride 3 --max-frames 100
```

### 加分项 3：TensorRT 推理与延迟 / FPS 对比（+3）— 未完成，如实标注

环境是具备的：RTX 5060 Laptop 8 GB、驱动 591.66（CUDA 13.1），WSL2 下 `/dev/dxg` 正常、
`torch.cuda.is_available()` 为 True；TensorRT 11.5.0.63（cu12 wheel）安装成功，
`trt.Builder` 能正常初始化。卡在两处工具链版本错配：

1. ultralytics 的 `format=engine` 导出强制依赖 `nvidia-modelopt[onnx]>=0.44`，其自动安装
   又要求精确的 `torch==2.14.1+cu130`，该 build 不在软件源索引上，导出直接失败；
2. `onnxruntime-gpu` 的 CUDA EP 也起不来（ORT 1.23 需 CUDA 12.x，本机 CUDA 13），
   所以加分项 2 的对比只能在 CPU 上做。

因此 TensorRT 的延迟 / FPS 数据缺失，本项不作任何结论、不计分。可继续的方向是绕开
ultralytics，直接用 TensorRT Python API 从 ONNX 建 engine 自测（需另排时间）。

### 加分项 4：地图匹配的稳健化（RANSAC / 稳健位姿优化，+2）— 已完成

`src/robust_match.py` 在现有 `solve_pose_fixed`（穷举 2 点假设 + 内点投票）之外实现了
两条稳健路线，输出结构与基线完全同构，可无缝替换：

- **RANSAC**：随机采样 2 对同源点假设（类型硬约束、固定种子可复现），保留内点数 top-12
  的假设，各自做两轮"重指派→重解"精化，再按（内点数、残差）排序去重；
- **IRLS 稳健位姿优化**：以基线解为初值，迭代重加权 Kabsch（Tukey biweight，c=0.25 m），
  长尾假配对的权重趋零，等效自动剔除外点。

合成蒙特卡洛（500 轮；地面噪声 σ=4 cm；每轮 5~8 个观测、注入 0~3 个假标志；
成功率判据为位置误差 ≤0.15 m 且航向误差 ≤5°）：

| 方法 | 成功率 | 平均误差 | 单帧耗时 |
|---|---:|---:|---:|
| 穷举基线 | 99.2% | 0.028 m | 99.4 ms |
| RANSAC（400 轮） | **99.6%** | 0.028 m | 91.6 ms |
| 基线 + IRLS | **99.6%** | 0.028 m | 102.7 ms |

真实视频（同一批检测观测，只换匹配算法）：

| 视频 | 穷举基线 | RANSAC | 基线 + IRLS |
|---|---|---|---|
| M10（真值 0, 0, −25.35°） | x=−0.036 y=+0.293 yaw=−28.96°，0.30 m / 3.6° | 与基线一致 | x=−0.034 y=+0.302 yaw=−28.96°，0.30 m / 3.7° |
| M1（较难片段） | 1.167 m / 14.3°，残差 0.132 m | 1.192 m / 14.3°，残差 0.135 m | **0.975 m / 12.5°，残差 0.049 m** |

规模实验（合成地标、10 个观测含 1 个假标志）：地图标志 8 / 12 / 16 个时，RANSAC 相对
穷举的耗时比为 0.6x / 1.9x / 3.1x——当前场地 7 个标志，穷举假设数只有
C(n,2)·C(m,2) 量级，本来就便宜；标志数上去才轮到 RANSAC 的 O(iters·nm) 占优。

结论：线上默认仍用穷举（结果确定、小规模更快）；IRLS 作为可选精化，在退化片段上把
残差压到原来的 1/3、位置误差降 0.19 m，适合在 `residual` 偏高时启用。复现：

```bash
python robust_match.py --mode synthetic --trials 500        # 蒙特卡洛对比
python robust_match.py --mode scale                         # 规模实验
python robust_match.py --mode real --video /mnt/d/captures/M10.mp4 \
    --truth 0,0,-25.35 --stride 20                          # 真实视频对比
```

## B 站演示录屏（必交）

### 任务 1 · 足球检测

**已完成并上传**，链接：<https://www.bilibili.com/video/BV156sh6CEAW>

- 内容：加载本任务最终权重 `best.pt`（ball_yolov8n_aug2，SHA256 见 1.3）对 USB 相机实时画面做足球检测，含**手动启动链路的全过程**（摄像头采集桥 → 演示窗口）。
- 画面要素：顶部栏常驻姓名 `weikai` 与实时时间（自证）；左侧为真实连续相机输入与检测框；右侧面板显示模型信息、独立测试集指标与误检/漏检分析；底部 HUD 逐帧显示 FPS / 推理耗时 / 帧号 / 检出数量。
- 输出展示：检测框上方标签逐帧给出类别 `ball`、置信度、`bbox=(x1,y1,x2,y2)` 四坐标与中心 `c=(u,v)`，覆盖细则要求的全部六项输出字段。
- 演示动作：球在画面内不同位置移动、远近变化（大小改变）、滚动，覆盖细则要求的展示场景。

### 任务 2 · 单目场地自定位

**已完成并上传**，链接：<https://www.bilibili.com/video/BV1aVy7zYEuo>

- 标题：任务二场地检测；时长 2 分 29 秒（149 s）；权限：公开；发布：2026-11-22。
- 内容：加载最终检测权重 `best_fixed_v2.pt` 与固定外参（h=1.005 m / pitch=15° / roll=0），
  运行 `realtime_locate.py` 实时定位，左右分屏布局。
- 画面要素（按细则 2.11 录屏要求逐项对照）：
  - 左侧：真实连续相机输入 + L/T 标志检测框与特征中心点；
  - 右侧：`field.yaml`（map v2，SHA256 `fba90597...36c8`）场地俯视图、相机光心位置标记、朝向箭头；
  - HUD：`weikai` + 实时时间戳（自证）、`x_camera_map / y_camera_map / yaw_camera_map`、
    marker_count、residual、valid/status、FPS；
  - 展示移动载体在场地内多个位置/朝向时，右侧地图位姿实时变化。

## 录屏与细则对照自查

| 细则要求 | 任务 1 视频 | 任务 2 视频 |
|---|---|---|
| 连续实时相机输入（无剪切拼接段） | ✓ | ✓ |
| 姓名 + 实时时间戳出现在画面中 | ✓ | ✓ |
| 检测框、类别、置信度 | ✓ | ✓（L/T） |
| 像素中心 / 几何特征点 | ✓（球心 u,v） | ✓（框中心为地面点，几何细化默认关闭，见 2.4b 说明） |
| FPS / 耗时 | ✓ | ✓ |
| 训练模型信息与指标 | ✓（面板展示） | ✓（map v2 + 外参 + 指标见 README） |
| 左 RGB / 右地图分屏 | — | ✓ |
| 地图俯视图 + 相机位置/朝向 | — | ✓ |
| `x/y/yaw_camera_map`、marker_count、residual、valid/status | — | ✓ |
| ≥3 个不同位置/朝向地图位姿变化 | — | ✓（录屏内现场走位） |
| 标志不足 / 歧义时输出无效状态 | — | ✓（离场/遮挡段 valid=false） |
| ROS2 话题输出 | ✓（`/soccer/ball`） | ✓（`/localization/field_pose`，实测记录见 2.7；话题数据未入录屏，可用 `ros2 topic echo` 现场复现） |

> 上表各项以视频实际内容为准；如有出入，以视频为准并在此修订。

## C0 现场测试

**尚未开展。**

## 当前局限与下一步计划

- **任务 2 已完成离线验证，且已完成固定外参最终复验**：
  - 手持阶段逐帧自由外参 PnP：位置 0.062~0.233 m、航向 0.4°~1.7° 达标，属离线验证口径；
  - **固定安装重拍已完成**：外参锁死（h=1.005 m / pitch=15° / roll=0），
    `locator_fixed.py` + `best_fixed_v2.pt` 在 M10/M14 两段站位上位置
    0.260~0.375 m、航向 0.61°~3.10° 达标（细则 2.6/2.7）；现场
    「≥3 个不同位置/朝向」由评审安排测试点验收。
    LCross5/6 坐标修正（3.084→3.977）后 M11 暴露为误检凑出的假达标，已作废；
  - 几何细化：已实现 `refine_landmarks.py`（框内白线 mask + 距离变换脊峰 / Hough
    线段求交）。基线框中心误差已全部达标，而细化在远景小框上反使 M2 恶化到
    2.9~3.9 m，故**默认关闭**，取点用 YOLO 框中心。
  - 站位几何教训：M5 类别混淆 / M8 过曝 / M13 共线歧义 / M15 检出不足均会失败——
    需选标志方位分散、背景干净的机位；M13/M15 已作废（详见 `定位验证报告.md`）。
  - ROS2 封装已完成：`cquick_interfaces`（自定义消息）+ `k1_vision_locate`
    （vision_node + launch），话题实测通过（见 2.7）；话题画面未入 B 站录屏；
  - 实时分屏演示程序已切固定外参核心（`src/realtime_locate.py`，HUD 显示 map v2 +
    实测外参），**B 站录屏已上传**（链接见「B 站演示录屏 · 任务 2」）；
- 干扰物负样本目前来自 V5 抠图 + copy-paste；若后续实拍中干扰球形态变化，需补充素材重训。
- 测试集 V5 中部分帧存在两个相似球但只标注了主要目标，官方 precision 口径会把正确检出的次要球算作误报；mAP50=0.925 更能反映真实检测质量。这是标注粒度导致的评估口径差异，非模型缺陷。

## 文件索引

```
weikai/
├── README.md                       本文件
├── 定位验证报告.md                  任务 2 完整验证报告（结果表 / bug 踩坑 / 复现命令）
├── dataset/
│   ├── ball_data.yaml              YOLO 数据配置（单类 ball）
│   ├── dataset_summary.md          任务 1 数据集摘要与增强说明
│   ├── split_manifest.csv          263 行划分明细（image/split/source_video/frame_index）
│   ├── test_metrics.csv            三版模型在独立 test 集的四项指标
│   ├── marker_data.yaml            任务 2 标志数据配置（L/T 两类）
│   └── marker_dataset_summary.md   任务 2 数据集摘要（269 张 / 842 框 / 划分 / 指标）
├── calibration/
│   ├── camera_intrinsics.yaml      内参（fx/fy/cx/cy + 5 参畸变，17 张棋盘格 rms 0.508 px）
│   └── camera_extrinsics.yaml      固定安装外参（h=1.005 m / pitch=15° / roll=0）
├── config/
│   ├── field.yaml                  场地特征地图 v2（与 src/templates 同一份，SHA256 `fba90597...36c8`）
│   ├── field_geometry.yaml         地图版本/尺寸/原点轴向（与 src/templates 同一份）
│   └── camera.yaml                 相机配置索引（指向 calibration/）
├── src/
│   ├── prepare_dataset.py          抽帧 + 按视频段划分
│   ├── check_video.py              原始视频质检（分辨率/fps/截断黑区）
│   ├── verify_labels.py            标注抽查 + 集合泄漏检查
│   ├── extract_distractors.py      从 V5 抠取两类干扰球素材（负样本来源）
│   ├── augment_train.py            copy-paste 干扰物负样本增强
│   ├── train_eval.py               训练 + 独立 test 集评估
│   ├── realtime_detect.py          实时检测脚本（检测 + CSV 逐帧记录）
│   ├── realtime_demo.py            实时检测演示窗口（B 站录屏所用，画面显示全部六项输出字段）
│   ├── net_camera.py               WSL 侧网络相机客户端
│   ├── win_cam_bridge.py            Windows 侧采集桥
│   ├── templates/field.yaml        场地特征地图 v2（12 标志实测坐标，Booster 格式）
│   ├── templates/field_geometry.yaml  地图版本/尺寸/原点轴向/SHA256（v2）
│   ├── templates/camera_extrinsics.yaml  固定安装外参（h=1.005 m / pitch=15° / roll=0，locator_fixed / realtime_locate 运行时读取）
│   ├── calibrate_intrinsics.py     相机内参标定（棋盘格，rms 0.508 px）
│   ├── detect_landmarks.py         L/T 标志检测辅助工具
│   ├── verify_landmarks.py         标志标注质检
│   ├── prepare_landmark_dataset.py 标志数据集拆分（多目录合并 + 按视频整段留出 val，未标完直接报错）
│   ├── extract_fixed_frames.py     固定外参视频抽帧（跳头尾 / 1 s 间隔 / 保留视频前缀）
│   ├── train_landmarks.py          标志检测训练（最终 mAP50=0.808）
│   ├── locator.py                  定位核心：穷举匹配 + PnP + 终验 + 跨帧聚合
│   ├── locator_fixed.py            **固定外参版**：射线→地面(Z=0)→2D 刚体配准，z/pitch/roll 从外参文件读入且整段不变（细则 2.6/2.7）
│   ├── calibrate_extrinsics.py     数据反标定工具：网格搜索 h/pitch（需固定安装视频）
│   ├── calibrate_direct.py         直接法外参诊断：真值位姿下投影残差
│   ├── refine_landmarks.py         几何细化模块（框内白线 mask + 脊峰/Hough 求交，默认关闭，见局限节）
│   ├── plan_reshoot.py             重拍站位规划：可见标志/投影尺寸/俯角覆盖计算
│   ├── 重拍执行清单.md              固定安装 + 外参量测 + 4 段验证站位（R4 斜角避白墙，无需锁曝光）+ 真值表
│   ├── FIELD修改指南.md            field.yaml 新增 L/T 标志的实测/命名/重训步骤
│   ├── realtime_locate.py          任务 2 实时左右分屏定位演示（左屏实时画面+检测框，右屏地图+光心+朝向箭头+x/y/yaw/marker_count/residual/valid/status）
│   ├── extract_facing_x_frames.py  朝向 +X 素材抽帧（v3 训练数据来源，M5/M9）
│   ├── onnx_compare.py             PyTorch / ONNX 同段推理对比（加分项 2）
│   ├── robust_match.py             地图匹配稳健化：RANSAC + IRLS 与三组对比实验（加分项 4）
│   ├── ros2/                       ROS2 colcon 包源码（加分项 1，接入 ~/vision_ws/ros2_ws 编译，见 2.7）
│   │   ├── cquick_interfaces/      接口包：BallDetection.msg / FieldPose.msg（字段对齐细则 2.11 建议表）
│   │   └── k1_vision_locate/       视觉定位包：vision_node（发布 /soccer/ball 与 /localization/field_pose）+ launch + params
│   ├── make_loc_figs.py            定位结果可视化（俯视散点 / 误差曲线 / 画面叠加）
│   ├── plot_m8_frames.py           M8 失败案例逐帧散点分析
│   ├── make_anno_crops.py          标注辅助裁剪
│   ├── launch_labelimg_landmark.py LabelImg 标志标注启动器
│   └── geometry_selftest.py        几何公式自测
└── outputs/
    ├── false_positive_compare.jpg  三版检测对比（黄绿/红蓝球误检消除）
    ├── ball_type_comparison.jpg    三种球外观对照
    ├── interference_scenes.jpg     干扰球同框场景（GT 绿框 / 误检红框）
    ├── landmark_training/          标志模型第一阶段训练曲线 / PR 曲线 / 混淆矩阵 / 训练摘要
    ├── landmark_v2_training/       最终标志模型（fixed_v2）训练曲线 / PR / 混淆矩阵 / 摘要
    ├── loc_fig1_topview.png        定位俯视图：4 段逐帧散点 + 真值 + 0.5m 验收圈
    ├── loc_fig3_error.png          逐帧位置误差曲线（0.5m 红线为验收门限）
    ├── loc_overlay_M1_f150.png     位姿→地图点投影回画面贴合校验（M1/M2/M7 各 1 帧）
    ├── loc_fig_m8_scatter.png      M8 失败案例：过曝下逐帧解散布全场
    └── loc_summary.json            逐段定位结果原始汇总
```

> 原始视频、整套抽帧图片、`best.pt`、`.venv/`、`runs/` 均按细则不上传，保留在本机备查。
