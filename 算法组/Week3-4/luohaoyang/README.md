# 第 3–4 周合并考核 · 罗颢洋

- 姓名：罗颢洋
- 完成日期：2026-11-22
- 摄像头类型：单目 RGB 摄像头（USB 免驱）
- 摄像头接口 / 设备节点：Windows DirectShow 索引 0（Linux 对应 `/dev/video0`）
- 图像分辨率 / 实际帧率：1280×720 / 30 fps
- 环境版本：Windows 11/Python 3.10.22 / PyTorch 2.11.0+cu128 / Ultralytics 8.4.210 / ROS2：未使用
- GPU 型号：NVIDIA GeForce RTX 5060 Laptop GPU

## 演示视频（B 站）
| 内容 | 链接 |
| 任务 1 · 足球检测（实时） | https://t.bilibili.com/1273561419365744663?share_source=pc_native |
| 任务 2 · 单目场地自定位（实时） |https://t.bilibili.com/1273561814536290306?share_source=pc_native|
【补充：由于本人没有良好的摄像头支架，任务二在测量过程中出现不稳定，有所误差的情况，原因为摄像头固定简陋，求得的外参在移动过程中变化，测算对外参的要求又极其严格，同时模型训练量不是很够，后续均可改进】

## 权重文件
两个任务各有一个 `best.pt`，均为本人训练（YOLOv8n 预训练初始化，基于本人实验室实拍数据训练）。
通过网盘分享的文件：ball_v1_best.pt等2个文件
链接: https://pan.baidu.com/s/1kpT_H1OS_vnQFi2DCnHH1w?pwd=1111 提取码: 1111

## 任务 1：足球检测

- 实验室采集日期、原始视频数量、抽帧数量、标注实例数量：2026-11-21；6 段原始视频（`1.avi`~`6.avi`）；抽帧并标注 **272 张**，标注实例 **272 个**（每帧 1 个球）
- 数据集划分方式：**按视频段划分，无帧间泄漏**
  - train = 视频 1/2/3/4，148 张
  - val = 视频 5，47 张
  - test = 视频 6，77 张
- 模型与训练参数：YOLOv8n（预训练），imgsz=640，batch=8，50 epoch，单卡训练
- best.pt 保存位置：见文末「权重文件」一节（本地在 `weights/ball_v1_best.pt`）
- Precision / Recall / mAP50 / mAP50-95：**0.911 / 0.979 / 0.966 / 0.702**（test 集 77 张，独立视频段）
- 实时检测 FPS / 推理耗时：单帧推理 **15 ms**（中位 13.5 ms，imgsz=640，RTX 5060），仅推理约 67 fps；接摄像头后的端到端帧率 30fps上下
- 误检与漏检案例：
  - 高度运动模糊的球
  - 强反光/白色物体：白色鞋容易误检
推测训练模型时运动模糊喂的较少，后续可多多训练改进

## 任务 2：K1 风格单目场地自定位

- field.yaml 文件版本 / SHA256：`measured-upper-only-2026-11-22-r3` / `489380d58f71e0e6ca6139424c500b9cc12c001fcda2775ecf52fcd7bb4ea111`
- field_geometry.yaml 版本：`measured-2026-11-21`（同目录 `config/field_geometry.yaml`）
- 地图原点、X/Y 轴方向、单位：原点 = 中圈中心；+X 向右、+Y 向上；单位 m；yaw 自 +X 逆时针为正
- 场地实际尺寸是否与地图一致：**不一致**。官方 `field.yaml` 为 8.95×6.94 m，与 C0 实测不符。按细则 2.10 条改用本人实测版本：**6.00 × 7.80 m**，收录已实测的 +Y 半场 14 个标志（LCross×6、TCross×6、XCross×2）
- 使用的场地标志类别：LCross、TCross、XCross（球门仅作背景，不作检测类别）
- 场地标志检测模型与训练指标：YOLOv8n，imgsz=1280，batch=4，100 epoch；训练/验证/测试 168/45/37；**P 0.535 / R 0.651 / mAP50 0.599 / mAP50-95 0.217**
- 几何特征点提取方法：检测框 → HSV 白线掩膜 + HoughLinesP 角度聚类 + 直线拟合 → 两线交点；拟合失败时退回框中心并计数
- 相机内参：fx 1612.90 / fy 1614.95 / cx 669.63 / cy 352.07（9×6 内角点，45 帧，标定 RMS 0.608 px；留出帧交叉验证 0.434 px）
- 畸变系数：k1 0.1360 / k2 −0.9501 / p1 −0.0033 / p2 0.0014 / k3 0.0
- 相机外参：地面 6 点 solvePnP（IPPE），重投影 RMS 6.89 px；相机高度 **0.643 m**；俯仰 **9.42°** / 滚转 **3.13°** / 偏航 **0.33°**
- 地面投影方法：像素去畸变 → 相机射线 → 按外参旋转到载体系 → 与地面 z=0 求交
- 地图匹配算法与同类标志消歧：标志对生成候选位姿 + 内点打分（阈值 0.30 m、≥3 内点）→ 高斯-牛顿精修；多解分数比 >0.75 判 `ambiguous` 不输出；位姿越出地图 0.6 m 判 `outside_map`；序列中带上一帧先验（1.5 m / 25°）辅助消歧。地图仅含 +Y 半场，无 180° 对称，**单帧即可唯一求解**
- 输出坐标系：`base_link`（x 前 / y 左 / z 上，地面 z=0），同时输出相机光心位姿
- 测试点误差（参考坐标未逐点复测，含站位误差）：

  | 点 | 参考 (x, y) | 估计 (x, y) | 估计 yaw | 位置偏差 |
  | --- | --- | --- | --- | --- |
  | P1 | −0.90, +1.10 | −0.243, +0.165 | 117.2° | 1.14 m |
  | P2 | +3.00, 0.00（站在 TCross1 上） | +2.879, +0.133 | 102.4° | **0.18 m** |
  | P3 | +0.50, +3.80 | +0.746, +4.217 | −157.7° | 0.48 m |

  P2 重复测量 3 次，结果一致性 0.46~0.50 m（相对设计点位）/ 0.18 m（相对 TCross1），重复性良好。

- marker_count / residual / valid / status：2~4 / 0.03~0.17 m / `valid=True|False` / 出现 `ok`、`ok_prior`、`ok_low_markers`、`ambiguous`、`outside_map`、`insufficient_markers`
- 实时 FPS / 定位更新频率 / 耗时：10~25 fps / 1.5~5 Hz / 单帧推理 20~50 ms（imgsz=1280）

## ROS2 接口
由于本次考核使用的单目工业相机在 Windows 原生环境下兼容性最好，因此目前的 YOLO 推理、坐标解算与地图匹配全部在 Windows (Python/OpenCV/PyTorch) 环境下实时运行，后续可将windows作为视觉端发布数据，WSL接受数据流封装为标准ROS2消息


## 运行
```bash
python src/field_localize/run_localize.py --camera 0 --csv run_test3.csv   # 左右分屏实时定位
python src/field_localize/acceptance_check.py                              # 内参/外参/地图/模型/链路自检
python src/ball_detect/detect_camera.py --camera 0                         # 任务 1 实时足球检测
```