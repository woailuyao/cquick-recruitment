# 第 3–4 周合并考核 · maxiaoyu

- 姓名：马晓宇（拼音 `maxiaoyu`，待核对）
- 完成日期：2026-11-22
- 摄像头类型：单目 RGB 摄像头（不限定品牌和型号）
- 摄像头接口 / 设备节点：USB UVC，`/dev/video0`（待现场确认）
- 镜头：固定焦距、固定对焦（焦距数值已知时填写）
- 图像分辨率 / 实际帧率：1280 × 720，目标 30 fps（实际帧率以连续取帧计时统计为准）
- Ubuntu / Python / PyTorch / Ultralytics / ROS2 版本：Ubuntu 22.04 / Python 3.10 / 待补充 / 待补充 / ROS2 Humble
- GPU 型号：待补充（无 GPU 时使用 CPU 推理）

## 任务 1：足球检测

- 实验室采集日期、原始视频数量、抽帧数量、标注实例数量：2026-11，5 段视频，抽帧间隔 15 帧，有效图片 ≥ 200 张 / 实例 ≥ 200 个（数量待补充）
- 数据集划分方式：按独立视频片段划分 train / val / test（约 70% / 20% / 10%），避免相邻帧泄漏；配置见 `dataset/ball_data.yaml`
- 模型与训练参数：YOLOv8n，`imgsz=640`，`epochs=50`，`data=./soccer_dataset/data.yaml`，device 按实际选择 0 / cpu
- `best.pt` 保存位置：实验室共享存储 / 指定网盘（README 记录获取方式与校验值，**不提交 GitHub**）
- Precision / Recall / mAP50 / mAP50-95：待补充（独立测试集评估）
- 实时检测 FPS / 推理耗时：待补充（以连续取帧计时统计）
- 误检与漏检案例：待补充（至少分析一个白色场地线/圆形物体误检、一个远处小足球/遮挡漏检）

## 任务 2：K1 风格单目场地自定位

- `field.yaml` 文件版本 / SHA256：待组长发放公共版本后记录（个人目录不复制、不改坐标）
- `field_geometry.yaml` 版本：待补充
- 地图原点、X/Y 轴方向、单位：地图 `+X` 向右、`+Y` 向上，单位米；`Yaw` 从 `+X` 逆时针为正
- 场地实际尺寸是否与地图一致：待现场核对
- 使用的场地标志类别：`LCross` / `TCross` / `XCross` / `PenaltyPoint`
- 场地标志检测模型与训练指标：YOLOv8n，各类别 Precision / Recall / mAP50 待补充
- 几何特征点提取方法（L/T/X/点球点）：YOLO 检测框内做线段拟合 / 角点交点细化，输出精确 `landmark_pixel=(u,v)`；取点规则待补充说明
- 相机内参 fx / fy / cx / cy：待标定后写入 `calibration/camera_intrinsics.yaml`
- 畸变系数 k1 / k2 / p1 / p2 / k3：待标定后写入
- 相机外参 / 相机高度 / 俯仰角 / 滚转角：见 `calibration/camera_extrinsics.yaml`（由棋盘标定 / 安装标定得到，标定方法与单位待补充）
- 地面投影方法：内参去畸变 → `K⁻¹[u,v,1]ᵀ` 反投影得射线 → 经外参转到载体坐标系 → 与地面平面 `Z=0` 求交；射线接近水平 / 交点越界时判无效
- 地图匹配算法与同类标志消歧方法：多标志空间分布 + 几何一致性 / RANSAC 候选位姿搜索，按整体残差评价；处理同类标志与场地对称歧义，歧义无法消除时 `valid=false`（具体实现待补充）
- 输出坐标系：camera_frame（相机光心为参考原点；如装在机器人上则先输出 base_link 再经外参换算）
- 3 个以上测试点的参考相机地图坐标 X/Y/Yaw 与估计误差：待 C0 现场测试后填写
- marker_count / residual / valid / status：运行时输出，状态含 `ok / insufficient_markers / ambiguous / high_residual / lost`
- 实时 FPS / 定位更新频率 / 耗时：待补充

## ROS2 接口

- 节点名称：待补充
- 足球话题：`/soccer/ball`
- 定位话题：`/localization/field_pose`
- 消息字段、单位与坐标约定：`x_camera_map / y_camera_map`（m）、`yaw_camera_map`（°）、`x_carrier_map / y_carrier_map / yaw_carrier_map`（可选）、`valid`(bool)、`marker_count`(int32)、`residual`、`status`(string)；坐标系为地图坐标系，Yaw 逆时针为正

## B 站演示录屏（必交）

- B 站链接：待补充（录屏上传 B 站后粘贴可访问链接，**视频原文件不上传 GitHub**）
- 视频标题：待补充
- 视频时长：待补充
- 访问权限（公开 / 仅链接可见）：待补充
- 是否包含连续实时相机输入：是 / 否（待录屏确认）
- 是否展示足球检测框、置信度和 FPS：是 / 否
- 是否展示 L/T/X 标志识别与几何特征点：是 / 否
- 是否采用左 RGB / 右地图的左右分屏布局：是 / 否
- 是否展示 field.yaml 地图匹配和地图俯视图：是 / 否
- 是否在右侧地图标出相机光心位置与朝向箭头：是 / 否
- 是否展示相机地图坐标 x_camera_map / y_camera_map / yaw_camera_map：是 / 否
- 是否展示至少 3 个不同位置 / 朝向的地图位姿：是 / 否
- 是否展示 marker_count、residual、valid / status：是 / 否
- GitHub 本目录是否已提交该 B 站链接：是 / 否

> 录屏开头先在终端执行：
> ```bash
> echo "姓名: maxiaoyu"; date "+%Y-%m-%d %H:%M:%S"
> ```

## C0 现场测试

- 测试日期：待补充
- 测试点编号：P1 / P2 / P3 / ...
- 参考位置 / 朝向：待补充（组长提供，不作为算法输入）
- 估计位置 / 朝向：待补充
- 平面位置误差 / 航向误差：待补充（基础目标：位置 ≤ 0.5 m，航向 ≤ 10°）
- 定位失败场景与处理：待补充
- 评审记录 / 备注：待补充

## 问题与解决过程

待记录至少一个真实问题、排查过程与最终解决方式（例如相机打不开 / 分辨率不生效 / 标定角点识别失败 / 帧泄漏排查等）。

## 当前局限与下一步计划

- 标志遮挡、标志分布不足、场地对称歧义、光照变化、地面不平、相机外参松动等情况下的局限：待补充
- 下一步计划：待补充

## 目录结构

```
算法组/Week3-4/maxiaoyu/
├── dataset/
│   ├── ball_data.yaml          # 足球检测数据集配置
│   ├── marker_data.yaml        # 场地标志检测数据集配置
│   └── dataset_summary.md      # 数据集统计（视频数/抽帧数/类别分布/划分方式）
├── calibration/
│   ├── camera_intrinsics.yaml  # 内参与畸变系数（标定后生成，本地待补）
│   └── camera_extrinsics.yaml  # 安装外参
├── config/
│   ├── field.yaml              # 引用公共统一版本，不复制、不改坐标
│   ├── field_geometry.yaml     # 引用公共统一版本
│   └── camera.yaml             # 相机配置
├── README.md
└── src/
    ├── cam_check.py            # 相机接入与帧率/分辨率自检
    ├── camera_calib.py         # 棋盘内参标定（9×6）
    ├── camera_calib_live.py    # 棋盘内参标定（7×5，实时采集角点）
    ├── extract_frame.py        # 场地标志视频抽帧
    └── extract_sep.py          # 足球视频抽帧
```

> 不进仓库：原始视频 / 录屏、抽帧与标注图片整套、`best.pt` 权重、`.venv/`、`runs/`、`__pycache__` 等缓存。
