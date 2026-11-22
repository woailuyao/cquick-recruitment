# 第 3–4 周合并考核 · <你的姓名拼音>

> 本 README 为考核提交模板，请按实际填写。所有 `<...>` 字段替换为你自己的真实值。

- 姓名：
- 完成日期：
- 摄像头类型：单目 RGB 摄像头（不限定品牌和型号）
- 摄像头接口 / 设备节点：`/dev/video0`
- 镜头：固定焦距、固定对焦（焦距数值如已知：___mm）
- 图像分辨率 / 实际帧率：`1280 × 720` / ___ fps
- Ubuntu / Python / PyTorch / Ultralytics / ROS2 版本：Ubuntu 24.04 / Python 3.12 / PyTorch ___ / Ultralytics ___ / ROS2 `jazzy`
- GPU 型号：___

## 任务 1：足球检测

- 实验室采集日期、原始视频数量、抽帧数量、标注实例数量：
- 数据集划分方式：按独立视频片段划分 train/val/test ≈ 70% / 20% / 10%
- 模型与训练参数：`yolov8n.pt`，imgsz=640，epochs=50，batch=___，device=___
- best.pt 保存位置：`runs/ball/weights/best.pt`（不提交 GitHub，见"权重获取方式"）
- Precision / Recall / mAP50 / mAP50-95：___ / ___ / ___ / ___
- 实时检测 FPS / 推理耗时：___ fps / ___ ms
- 误检与漏检案例：
  - 误检：___（描述 + 原因 + 截图分析）
  - 漏检：___（描述 + 原因 + 截图分析）

## 任务 2：K1 风格单目场地自定位

- field.yaml 文件版本 / SHA256：`C0-v1.0` / ___
- field_geometry.yaml 版本：`C0-v1.0`
- 地图原点、X/Y 轴方向、单位：原点___；+X 向右、+Y 向上；单位米
- 场地实际尺寸是否与地图一致：是 / 否（组长已核实）
- 使用的场地标志类别：LCross / TCross / XCross / PenaltyPoint
- 场地标志检测模型与训练指标：`yolov8n.pt`，各类别 Precision/Recall/mAP50 = ___
- 几何特征点提取方法（L/T/X/点球点）：
  - LCross：二值化 + Shi-Tomasi 角点检测取 L 顶点
  - TCross：二值化 + 前景质心（横竖线交点）
  - XCross：二值化 + 前景质心（两线交会中心）
  - PenaltyPoint：二值化 + 质心（点球标记中心）
- 相机内参 fx / fy / cx / cy：___ / ___ / ___ / ___
- 畸变系数 k1 / k2 / p1 / p2 / k3：___ / ___ / ___ / ___ / ___
- 相机外参 / 相机高度 / 俯仰角 / 滚转角：___ / ___ m / ___° / ___°
- 地面投影方法：像素去畸变 → 反投影射线 → 外参变换到载体坐标 → 与地面平面 Z=0 求交
- 地图匹配算法与同类标志消歧方法：RANSAC + SVD 最小二乘 SE(2) 位姿估计；同类标志多候选匹配，用内点数和残差筛选，歧义时输出 `valid=false`
- 输出坐标系（camera_frame / base_link / rig_frame）：camera_frame（相机光心定义为载体参考原点）
- 3 个以上测试点的参考相机地图坐标 X/Y/Yaw 与估计误差：

  | 测试点 | 参考 X/Y/Yaw | 估计 X/Y/Yaw | 平面误差(m) | 航向误差(°) |
  | --- | --- | --- | --- | --- |
  | P1 | | | | |
  | P2 | | | | |
  | P3 | | | | |

- marker_count / residual / valid / status：___ / ___ / ___ / ___
- 实时 FPS / 定位更新频率 / 耗时：___ fps / ___ Hz / ___ ms

## ROS2 接口

- 节点名称：`ball_detection_node`、`field_localization_node`
- 足球话题：`/soccer/ball`（`soccer_vision/msg/BallDetection`）
- 定位话题：`/localization/field_pose`（`soccer_vision/msg/FieldPose`）
- 消息字段、单位与坐标约定：见 `msg/FieldPose.msg`、`msg/BallDetection.msg`

## B 站演示录屏（必交）

- B 站链接：
- 视频标题：
- 视频时长：
- 访问权限（公开 / 仅链接可见）：
- 是否包含连续实时相机输入：是 / 否
- 是否展示足球检测框、置信度和 FPS：是 / 否
- 是否展示 L/T/X 标志识别与几何特征点：是 / 否
- 是否采用左 RGB / 右地图的左右分屏布局：是 / 否
- 是否展示 field.yaml 地图匹配和地图俯视图：是 / 否
- 是否在右侧地图标出相机光心位置与朝向箭头：是 / 否
- 是否展示相机地图坐标 x_camera_map / y_camera_map / yaw_camera_map：是 / 否
- 是否展示至少 3 个不同位置 / 朝向的地图位姿：是 / 否
- 是否展示 marker_count、residual、valid / status：是 / 否
- GitHub 本目录是否已提交该 B 站链接：是 / 否

## C0 现场测试

- 测试日期：
- 测试点编号：P1 / P2 / P3 / ...
- 参考位置 / 朝向：
- 估计位置 / 朝向：
- 平面位置误差 / 航向误差：
- 定位失败场景与处理：
- 评审记录 / 备注：

## 权重获取方式

`best.pt`、`best_marker.pt` 等较大权重不提交 GitHub，存放于实验室共享存储 / 网盘：
- 路径 / 链接：___
- SHA256 校验值：___

## 问题与解决过程

至少记录一个真实问题、排查过程和最终解决方式。

## 当前局限与下一步计划

说明当前在标志遮挡、标志分布不足、场地对称歧义、光照变化、地面不平或相机外参变化情况下的局限。
