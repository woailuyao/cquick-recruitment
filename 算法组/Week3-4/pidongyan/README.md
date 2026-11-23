# 第 3–4 周合并考核 · pidongyan

- 姓名：pidongyan
- 完成日期：2026-11-21（计划提交）
- 摄像头类型：单目 RGB USB 摄像头（不记录品牌 / 型号）
- 摄像头接口 / 设备节点：USB，Windows DirectShow / DSHOW index 0；
  Ubuntu 设备节点 `[待现场填 /dev/videoX]`
- 镜头：固定焦距、固定对焦（锁定后未再变动）；焦距数值**未知**（模组未标注）
- 图像分辨率 / 实际帧率：1280×720 MJPG / **30 fps**；
  统计口径为连续取帧实际耗时（`logs/capture_log.csv`），未采用 `CAP_PROP_FPS` 返回值
- 环境：Windows 11 开发机，Python 3.13.14 / PyTorch 2.11.0+cu128 /
  Ultralytics 8.4.203 / CUDA 12.8；
- GPU：NVIDIA GeForce RTX 5060 Laptop 8 GB

---

## 任务 1：足球检测

- 实验室采集日期 / 原始视频数量 / 抽帧数量 / 标注实例数量：
  **2026-11-12 / 19 段（任务一使用 7 个独立片段）/ 282 张 / 214 个 ball 框 + 68 张空标签**。
  另有 100 张宿舍橙子调试帧与 3 个误标 txt 已归档到 `frames_pool_dorm_rejected/`（未删除、未参与训练）。
- 数据集划分方式：**按独立视频片段** 70/20/10（seed=0），
  train 215 / val 35 / test 32，同一视频段的帧不跨 split，无泄漏；
  test 为独立远距小球片段 `ball_run05`。纯空场片段强制进 train，保证 val/test 各有含球片段。
  正式训练用 `dataset_aug/`（train 384 = 215 原始 + 169 张伪远距小球增强图，
  val/test 原样复制），配置 `config/data_aug.yaml`。详见 `dataset/dataset_summary.md`。
- 模型与训练参数：**yolov8n.pt / imgsz=640 / epochs=50 / batch=auto(32) /
  device=CUDA:0 / seed=0 / optimizer=auto(实际 AdamW, lr0=0.002)**，
  与考核基线完全一致。v3 的提升来自数据增强，**不是调参**；
  v2 调参轮次失败（test mAP50 0.055）已如实记录在 `docs/训练报告_任务一.md`。
- best.pt 保存位置：**`runs/detect/train_v4/weights/best.pt`**
  （根目录副本 **`best_train_v4.pt`**，正式权重）。
  按细则**不上传 GitHub**：共享存储 / 网盘链接 通过网盘分享的文件：best_train_v4.pt
链接: https://pan.baidu.com/s/1aRZIqPonYuvn6IdsfZJXhQ?pwd=6b4d 提取码: 6b4d 
--来自百度网盘超级会员v1的分享；
  
  本地实测 SHA256 = `940e6e7c54a245909c35e6cb632687ea5e6903d969d68a1a73a82d726f09dd5e`
  （`sha256sum best_train_v4.pt`，2026-11-20 16:05）。
  ⚠️ 根目录 `best.pt` 是 v1 基线，`best_train_v3.pt` 是上一版（均保留复现对比），
  实时演示默认已指向 v4；回退用 `--model best_train_v3.pt`。
- Precision / Recall / mAP50 / mAP50-95（**独立 test 集，v4 正式，seed=0**）：
  **1.000 / 0.999 / 0.995 / 0.499**。
  同口径 conf=0.25 案例统计：正确 30 / **误检 0** / 漏检 1
  （v3 为 26 / 35 / 5，v1 基线为 3 / 4 / 28）。
- **双 seed 对照（务必一起写进报告，不要只挑 0.995）**：
  同一份 v3 数据换 seed=1 时 test mAP50 从 0.613 掉到 **0.136**，
  说明 50 epoch / 215 张小数据集**训练方差极大**；加 40 张类球干扰负样本后
  v4 的 val 峰值稳定在 0.895 / 0.875（v3 为 0.734 / 0.578），
  test mAP50 为 0.995 / 0.613。seed=1 在固定 conf=0.25 下仍有 18 个漏检，
  故**不宣称 v4 全面稳定优于 v3**，只说明负样本主要起到稳定 val 与压误检的作用。
  详见 `docs/训练报告_任务一.md` 第 8 节、`logs/train_runs.csv`。
- 实时检测 FPS / 推理耗时：**端到端 28.9 FPS**，取自 2026-11-20 15:10 连续实时录屏
  （4461 帧 / 154.2 s 墙钟；口径含采集 + YOLO 推理 + 分屏渲染 + mp4 编码，
  不是 `CAP_PROP_FPS` 标称值）。录屏状态栏瞬时 FPS 在 22~52 间波动
  （USB 采集抖动），**不作为指标**；纯推理耗时从状态栏读数约
  8.8 / 13.5 / 9.2 ms（f200 / f1500 / f3000）。
  离线参考 v3 在 val 上 1.1 ms inference / 张（RTX 5060，不含采集与后处理）。
- 误检与漏检案例：
  - 漏检：约 80 px 远距小球贴白色围挡、强过曝，黑白斑块糊成灰白噪点。
  - 误检：背景**蓝色地球仪圆形图案**（FP 0.22）与**栏杆设备**（FP 0.60，高置信，
    调 conf 压不掉）。v4 负样本实验即为针对此类干扰。
  - 案例图为带框标注图，按细则不上传，README 只保留文字描述。
- 问题与解决过程：① `analyze_boxes.py` 量出 test 球框面积仅 train 的 22%，
  判定尺度外 OOD；② 先试调超参（imgsz 960 / 关 mosaic / SGD），**v2 掉到 0.055**，
  查 val 曲线发现关掉 mosaic 让 215 张小数据过拟合；③ 认识到 imgsz 会把
  train/test 一起放大、相对尺度差不变；④ 在 train 补 32–96 px 的小球样本、
  沿用基线超参重训 v3 → mAP50 0.204→0.613、漏检 28→5；⑤ 误检变多，
  从已有无球视频段补 40 张类球干扰空标签负样本，同超参训 v4 → 误检 35→0、漏检 5→1；
  ⑥ 双 seed 对照发现 215 张小数据集训练方差极大（v3 换 seed 掉到 0.136），
  因此报告里同时给出两个 seed 的结果，不把单次 0.995 当普遍结论。
  完整链路见 `docs/训练报告_任务一.md`。

---

## 任务 2：K1 风格单目场地自定位

- field.yaml 文件版本 / SHA256：本人实测版 `c0-self-measured-v1`，
  `c4b1e91302b439821767f8d9fd78ac1abff24c6b636fb96d887334f6c7eb8e8b`
- field_geometry.yaml 版本 / SHA256：
  `cc7927abe62105808ebc80fb5f6679d63cb4a82cb5671d7bd14af7b27f8a06ee`
- 地图原点、X/Y 轴方向、单位：原点 O = 场地中心；**+X 指向唯一球门**，
  +Y 为面向 +X 时的左侧，+Z 向上；单位米；yaw 绕 +Z 逆时针为正，输出用**度**。
- 场地实际尺寸是否与地图一致：**实测 8.35 m × 6.00 m（含白线）后写入地图**。
  与官方图纸差异（如实记录，未擅自"修正"）：实际场地**无禁区弧线**
  （官方 T5/T6 不存在）、中线实际在 x=+0.125、中圈半径实测 ≈0.73 m、
  点球点按实地位于几何中心 (0,0)。共 15 个标志：L×8 + T×4 + X×2 + 点球点×1。
- 使用的场地标志类别：`L_cross` / `T_cross` / `X_cross` / `penalty_point`（4 类）。
- 场地标志检测模型与训练指标：**210 张素材已全部人工标注完成**（11 月 12 日 175 张 +
  11 月 20 日 C0 现场手持走位补录 `markers_sweep_20261120_175438.mp4` 抽出的 35 张，
  1280×720 原始帧）。**121 张含标志 / 89 张空负样本，共 301 个框**，四类齐全：
  `L_cross=158` / `T_cross=61` / `X_cross=54` / `penalty_point=28`；
  `check_labels.py` 自检 **0 格式错误、0 缺标签**（报告 `logs/label_check_20261121_225233.txt`）。
  - 首轮曾出现「只标 L_cross、其余三类零样本」，已用 `--only-labeled` 复核模式
    补标修正（详见 `docs/标注指南.md` 4.3 节），**未重拍、未伪造框**。
  - 划分：**按独立视频段** 70/20/10、**`seed=102`**（不是 seed=0）。
    改 seed 的原因如实记录：seed=0 时 test 集恰好只落到 `marker_run05`（34 张里
    31 张空图、仅 3 张有框、**T_cross 无真值**），四类 mAP 无法有效计算；
    在 200 个候选 seed 中按「val/test 均四类齐全 + 各有 ≥8 张含标志图」筛选，
    选 seed=102 使 test 四类为 L20/T15/X15/P7、val 为 L55/T17/X7/P4。
    **筛选依据是评估集覆盖度，不是挑好看的指标**（当时尚未训练）。
    最终 train 130 / val 52 / test 28，无视频组跨 split 泄漏。
  - 训练参数：考核基线 `yolov8n.pt / imgsz=640 / epochs=50 / batch=auto /
    device=0 / seed=0`（训练 seed 与划分 seed 是两回事），配置 `config/marker.yaml`。
    50 epoch 跑满（RTX 5060，约 1 分 39 秒），权重
    `runs/detect/markers/weights/best.pt`（根目录副本 `best_markers.pt`）。
  - **独立 test 集真实结果（28 张，57 个实例，best.pt 由 val 选出）**：
    Precision **0.940** / Recall **0.050** / mAP50 **0.113** / mAP50-95 **0.034**。
    分类别 AP50：`L_cross` 0.169 / `T_cross` 0.024 / `X_cross` 0.259 /
    `penalty_point` **0.000**。
    ⚠️ **指标很差，如实记录不修饰**：高 P 极低 R 说明模型只敢报极少数高置信框、
    绝大多数交点漏检；点球点 AP 为 0（训练集中仅 17 张含点球点、28 个框）。
    根因是 301 个框分散在 4 类、每类样本极少 + 交点框只有 8~25 px 的小目标 +
    50 epoch 基线未做任何增强。**不据此声称「四类标志识别可用」或「定位链路已跑通」**，
    15:10 录屏的 `valid=FALSE` 结论不变。记录见 `logs/train_runs.csv`、
    `docs/train_args_markers.md`。
- 几何特征点提取方法：
  ① `tools/pick_points.py` 本人目视点选真实白线交点（看不见 / 认不准的按 `s` 跳过，≥4 点）；
  ② `tools/diagnose_clicks.py` 类别级 RANSAC 核对点名（**像素观测一个字节不改**，
     建议须目视确认）；
  ③ 模型链路用 `task2/refine.py` 的 RANSAC 双直线拟合做 L/T/X 交点亚像素细化，
     再用类别级 RANSAC（`task2/correspondence.py`，P3P 4 点假设 + 软分配迭代精化）
     完成同类标志的多候选匹配。selftest 4/4、细化 28/28 通过。
- 相机内参 fx / fy / cx / cy（任务二几何专用 v2）：
  **1620.0 / 1620.0 / 602.411755 / 365.033275**。
- 畸变系数 k1 / k2 / p1 / p2 / k3：
  **0.17091399 / -0.56531384 / -0.00845623 / -0.01488631 / 2.88758479**。
  任务一去畸变另用近距标定（fx≈2031）那套；两套内参因**对焦呼吸效应**拆分，
  详见 `config/camera.yaml`。旧版把 k3=-12.5 的近距畸变用于远距地面解算会
  触发畸变多项式过零折叠，已由 `distortion_fold_warning()` 拦截。
- 相机外参 / 高度 / 俯仰 / 滚转：pitch=**14.525°**、roll=**+1.519°**
  （墙脚线几何法，投影自检 0.1 px）；桌边标定 h=0.222 m，
  2026-11-12 现场 pose 段实测光心高度 **h=0.55 m**（角度随刚性安装不变，现场只重量 h）。
- 地面投影方法：`K⁻¹` 反投影（`undistortPoints` 迭代求逆）+ 与 **Z=0 地面求交**
  （`task2/geometry.py::pixel_to_ground`），闭环自检误差 <2 cm。
- 地图匹配算法与同类标志消歧：SE(2) 位姿由类别级 RANSAC 联合估计「谁是谁 + 相机在哪」；
  L 类 8 个同形候选靠**整体重投影误差最小**软分配消歧；
  再用现场实测 `h=0.55` 与俯仰先验剔除**平面 PnP 的地下镜像解**
  （实测不加先验时 pose_B 会被解成 h=-0.737、位姿像 pose_C 的镜像分支）。
  `run_task2.py clicks` 默认带 `--h-prior 0.55`。
- 输出坐标系：**`camera_frame`**（独立移动测试架，以相机光心为载体参考原点，
  `x_carrier_map = x_camera_map`）。机器人本轮未参与，不把机器人位置误标成相机光心。
- 3 个以上测试点的参考相机地图坐标 X/Y/Yaw 与估计误差：

  | 组 | 参考 x/y/yaw | 估计 x/y/yaw | 平面误差 | 航向误差 | 高度误差 | 状态 |
  |---|---|---|---|---|---|---|
  | pose_B | -2.225 / +1.780 / -25.5° | -2.531 / +1.684 / -18.15° | **32.1 cm** | **7.3°** | **3.4 cm** | ✅ 已出（rms 6.97 px，5 点） |
  | pose_C | -3.085 / -2.050 / +18.9° | -3.155 / -2.107 / +23.41° | **9.1 cm** | **4.5°** | **4.8 cm** | ✅ 已出（rms 17.71 px，6 点） |
  | pose_D2 | -2.875 / -1.990 / +26.4° | -3.004 / -2.019 / +28.64° | **13.3 cm** | **2.2°** | **3.5 cm** | ✅ 已出（20 号 C0 现场补测，rms 18.54 px，5 点） |
  | pose_A | +0.055 / -1.720 / +0.54° | — （仅 2 个可见交点，不解算） | — | — | — | ⛔ `valid=FALSE` 无效状态案例 |

  现有 **3 组有效误差 + 1 组「标志不足」无效状态**，`logs/field_pose_samples.csv`
  有 4 行**卷尺实测**参考真值（A/B/C + pose_D2），满足细则 ≥3 个不同位置/朝向
  与「3 个以上测试点」要求。pose_D2 为 20 号在 C0 现场新拍：卷尺量光心投影
  (x,y) 与光轴落点、O→P 连线算 yaw，白盒对中偏差 9 px（yaw <0.4°）。
  ⚠️ 口径：以上均为**离线人工目视点选 + PnP** 的误差，不是「实时相机自动定位」
  结果；15:10 录屏时四类标志模型尚未训练（21 日已补训但指标极差，见任务二节），
  该录屏全程 `valid=FALSE`，两者分开陈述。
  完整记录、逐点重投影误差、h 先验降级诊断见 `docs/任务二误差表.md`。
- marker_count / residual / valid / status：
  pose_B 组 `marker_count=5`、`residual=6.97 px`、`valid=true`（rms<10 px 且 h>0）、`status=OK`；
  pose_C 组 `marker_count=6`、`residual=17.71 px`、`valid=true`（h=+0.502>0，6 点编号自洽）、`status=OK`；
  pose_D2 组 `marker_count=5`、`residual=18.54 px`、`valid=true`（h=+0.515>0，5 点编号自洽）、`status=OK`；
  pose_A 组 `marker_count=2`、`residual=--`、`valid=false`、`status=INSUFFICIENT_MARKERS`。
  标志不足（<4 点）时 `run_task2.py clicks` 不解算、不报残差、不输出 x/y/yaw，
  直接输出规范无效状态，绝不伪造位姿。
  诊断工具 `diagnose_clicks.py` 在「带 h 先验 RANSAC 无解」时会降级用无先验物理
  解核对编号（pose_D2 实测，属 4 点 P3P 假设阶段 `pen>3.0` 硬剔除误杀，非点位
  错误），并**始终写出候选副本**，三步命令可整段粘贴，不会中途缺文件。
- 实时 FPS / 定位更新频率 / 耗时：**未获取，如实留空**。20 号 15:10 实时演示时
  四类标志检测模型尚未训练（`marker=none`），定位链路整段跳过、全程
  `valid=FALSE`，**没有产生任何定位更新**，因此无更新频率与耗时可测。
  （21 日虽已补训四类模型，但 test R 仅 0.050，未用于补录有效位姿，故此项仍留空。）
  若后续拿到可用的 marker 模型并补录有效位姿，再按
  `ros2 topic hz /localization/field_pose` 或录屏状态栏实测补上；不预填数字。

---

## ROS2 接口

> 状态：**节点骨架与 launch 文件已按细则 2.11 写好，但 Windows 开发机未安装 ROS2 Humble，
> 尚未在实验室 Ubuntu 机实机跑通**。`[实机验证结果待现场填]`。
> 细则 666 行「封装成规范 ROS2 节点 + 可复用 launch 文件」为 +2 加分项，不计入硬性必交。

- 节点名称：`ball_detector_node`、`field_pose_node`
- 足球话题：`/soccer/ball`
- 定位话题：`/localization/field_pose`（自定义消息 `FieldPose.msg`）
- 消息字段、单位与坐标约定：

  | 字段 | 类型 | 含义 / 单位 |
  |---|---|---|
  | `std_msgs/Header header` | Header | 时间戳与数据来源 |
  | `float32 x_camera_map` | float32 | 相机光心地图 X，m |
  | `float32 y_camera_map` | float32 | 相机光心地图 Y，m |
  | `float32 yaw_camera_map` | float32 | 相对地图 +X 轴，**度**，逆时针为正 |
  | `float32 x_carrier_map` | float32 | 载体原点地图 X，m（本轮 = x_camera_map） |
  | `float32 y_carrier_map` | float32 | 载体原点地图 Y，m |
  | `float32 yaw_carrier_map` | float32 | 载体朝向，度 |
  | `bool valid` | bool | 位姿是否可信 |
  | `int32 marker_count` | int32 | 本帧有效参与匹配的特征数量 |
  | `float32 residual` | float32 | 重投影残差 rms，px |
  | `string status` | string | `OK` / `INSUFFICIENT_MARKERS` / `AMBIGUOUS` |

- 两条处理链路（细则 791 行）：
  ① 足球检测：相机帧 → YOLOv8n → `/soccer/ball`（框 + 置信度）；
  ② 场地自定位：相机帧 → 标志检测 → 亚像素交点 → 类别级 RANSAC 匹配
  `field.yaml` → PnP + h 先验消歧 → `/localization/field_pose` → 右侧地图分屏。

---

## B 站演示录屏（必交）

- B 站链接：**https://www.bilibili.com/video/BV17Ksq6cEbv/**
  （BV 号 `BV17Ksq6cEbv`，2026-11-20 15:55 上传；页面已验证可打开、标题与简介可读。
  原始分享链接带 `?share_source=copy_web&vd_source=...` 追踪参数，README 用去参数的
  规范链接；完整原链接留档在 `docs/录屏核查_20261120.md` 第 5.3 节）
- 视频标题：`CQuick录屏核查报告 · 2026-11-20 15:10 实时相机演示`
  （与 B 站页面实际标题逐字一致；页面标题末尾 `_哔哩哔哩_bilibili` 是站点自动后缀，不计入标题）
- 视频时长：**2 分 29 秒**（148.7 s / 4462 帧 / 1920×540 / 平均 28.9 FPS）
- 访问权限：公开（用不带参数的 BV 链接即可直接打开页面并读到标题与简介）
- 视频文件：`check_out/demo_20261120_151004.mp4`（59 MB，连续未剪切，`--source 0`
  **真实相机实时输入**，未用预录视频冒充）。按细则**只交 B 站链接，原视频不上传仓库**。

### 逐条自查（细则模板 49–63 行，如实填写）

| # | 自查项 | 填写 | 依据 |
|---|---|---|---|
| 1 | 是否包含连续实时相机输入 | **是** | `--source 0` LIVE，单个未剪切 mp4 |
| 2 | 是否展示足球检测框、置信度和 FPS | 部分 | 左半屏有检测框、FPS、infer 耗时；置信度在框标签上，抽帧缩略图不易逐字辨认，不夸大 |
| 3 | 是否展示 L/T/X 标志识别与几何特征点 | **否** | 录屏时四类标志检测模型未训练，`marker=none`（21 日已补训但 mAP50 仅 0.113 / R 0.050，本段视频未使用该权重，结论不变） |
| 4 | 是否采用左 RGB / 右地图的左右分屏 | **是** | 1920×540 分屏 |
| 5 | 是否展示 field.yaml 地图匹配和地图俯视图 | 部分 | 右半屏显示了 C0 地图、版本 SHA 与内外参，但**未发生地图匹配**（无位姿） |
| 6 | 是否在右侧地图标出相机光心位置与朝向箭头 | **否** | 全程 `valid=FALSE`，工具拒绝画假位姿 |
| 7 | 是否展示 `x/y/yaw_camera_map` | **否** | 字段名在，值一律 `--`，不是有效坐标 |
| 8 | 是否展示 ≥3 个不同位置 / 朝向的地图位姿 | **否** | 画面机位有变化，但地图上位姿一条都没有 |
| 9 | 是否展示 `marker_count` / `residual` / `valid` / `status` | **是** | 右栏常驻：`0 / -- / FALSE / INSUFFICIENT_MARKERS` |
| 10 | GitHub 本目录是否已提交该 B 站链接 | **是** | 本文件（含 B 站链接）就是提交到 `算法组/Week3-4/pidongyan/README.md` 的那一份 README；链接已写入且公开可访问，上传动作与本次提交为同一份文件 |

补充（细则画面要求 / 无效状态）：姓名 + 实时时间入画 **是**
（`LIVE pidongyan 2026/11/20 15:10:22`）；至少一次标志不足 / 歧义的无效状态
**是**（本段**全程** `INSUFFICIENT_MARKERS`）；ROS2 节点与话题输出 **部分**
（右栏列出 `/localization/field_pose` 话题名与字段，但 Windows 机未装 Humble，
无真实节点运行 / 消息输出）。

> ⚠️ 模板为「是 / 否」二选一。本表保留「部分」以如实反映；提交时 **3 项「部分」
> 由本人确认**：从严可填「否」，确认无误才填「是」，不要直接抄成「是」。
> 第 3 / 6 / 7 / 8 项明确为「否」，**不得改成「是」**。
> 如实填「否」比虚填「是」安全：细则对「定位失败仍伪造有效位姿」有扣分。

- 定位算法有效性**不靠这段视频证明**：由离线 `pose_B` / `pose_C` 人工点选误差表
  （`docs/任务二误差表.md`）与 `pose_A` 「标志不足」无效状态案例支撑，两者分开陈述。
- 逐条依据、抽帧核查、画面缺陷与 B 站上传记录见 `docs/录屏核查_20261120.md`。

## 硬性必交项状态（细则 661 行，缺一即「材料不完整」）

| 项 | 状态 |
|---|---|
| 任务 1 本人实验室实拍视频 | ✅ 19 段已拍 |
| 任务 1 实时相机演示 | ✅ 已录（`demo_20261120_151004.mp4` 左半屏：球检测框 + FPS + 姓名/实时时间） |
| 任务 2 实时相机演示 | ⚠️ **部分**：实时画面 + 地图分屏 + 无效状态已录；录屏时四类标志模型未训练（21 日补训后 test mAP50 仅 0.113、R 0.050，不足以改变结论）→ 全程 `valid=FALSE`，**无有效位姿 / 坐标 / 相机箭头**，不得声称定位成功 |
| B 站录屏链接 | ✅ **已上传并填入 README**：`https://www.bilibili.com/video/BV17Ksq6cEbv/`（15:55，页面可访问） |
| GitHub README 可访问链接 | ✅ **已满足，无需填写任何链接**：细则 661 行该项指「把演示录屏的视频链接提交到个人 GitHub 考核目录 / README」，本文件已写入 `https://www.bilibili.com/video/BV17Ksq6cEbv/`（公开、已验证可打开）；本文件提交位置即 `算法组/Week3-4/pidongyan/README.md`，上传后该项自动成立 |
| C0 多位置现场定位测试 | ✅ **离线点选误差已达标**：CSV 4 行卷尺真值、3 组有效误差（pose_B/C/D2）+ 1 组无效状态，pose_D2 为 20 号 C0 现场补测。⚠️ 但「实时相机 → 自动标志检测 → 位姿」链路未跑通（录屏时四类模型未训练；21 日补训后指标极差，未用于补录），如实分开陈述 |
