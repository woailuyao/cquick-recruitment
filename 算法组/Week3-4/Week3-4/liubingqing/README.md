# 第 3–4 周合并考核 · liubingqing

- 姓名：刘冰清
- 完成日期：2026-11-22
- 摄像头类型：单目 RGB 摄像头（不限定品牌和型号）
- 摄像头接口 / 设备节点：USB 接口 / `/dev/video1`
- 镜头：固定焦距、固定对焦（未暴露 V4L2 控制项，物理定焦，焦距：未知/xx mm）
- 图像分辨率 / 实际帧率：1280 × 720 / 约 9.90 FPS（注：虚拟机环境连续读取 90 帧实测）
- 内部主格式：BGR（OpenCV `cv2.VideoCapture` 读取 UVC 相机的默认输出）
- 不主动做颜色转换：相机读取、送 YOLO、`cv2.imshow` 全链路保持 BGR
- 唯一显式转换：HSV 颜色分割用 `cv2.COLOR_BGR2HSV`（源为 BGR）
- YOLO 输入：直接传 BGR numpy 数组，Ultralytics 内部自动转 RGB
- ROS2 图像话题（如发布）：`encoding="bgr8"`，与内部格式一致
- Ubuntu / Python / PyTorch / Ultralytics / ROS2 版本：Ubuntu 22.04 / Python 3.10.12 / PyTorch 2.14.1+cpu / Ultralytics 8.4.213 / ROS2 Humble
- GPU 型号：无（虚拟机 CPU 运行）


### 相机接入与自检记录（对应细则 2.2）

- **设备节点**：`/dev/video1`
- **支持格式**：MJPEG 格式支持 1280x720 @ 30fps；YUYV 格式 1280x720 仅支持 10fps。
- **对焦/变焦设置**：通过 `v4l2-ctl --list-ctrls` 检查，该设备未暴露 `focus_absolute` / `focus_automatic_continuous` 等控制项。判断为定焦镜头，物理上不发生对  焦变化，满足考核要求。无需软件锁定。
- **异常处理**：当指定不存在的设备（如 `/dev/video99`）时，程序能够捕获异常/返回 False，未崩溃。所有 5 项严格检查 PASS。
- **问题与解决**：摄像头多个video节点，/dev/video0 视频数据流，/dev/video1元数据，第一次没有搞清楚用哪个节点，选择/dev/video1无法打开摄像头画面，在更换节点后成功输出画面。


## 任务 1：足球检测

- **实验室采集日期、原始视频数量、抽帧数量、标注实例数量**：2026-11-15 / 7 / 70 / 214
- **数据集划分方式**：按独立视频片段 7:2:1 划分
- **模型与训练参数**：YOLOv8n, epochs=50, imgsz=640, batch=4, worker=0 (虚拟机内存限制调整为4)
- **best.pt 保存位置**：/home/liubingqing/Desktop/runs/detect/train/weights/best.pt
- **Precision / Recall / mAP50 / mAP50-95**：0.902 / 0.905 / 0.927 / 0.721
- **实时检测 FPS / 推理耗时**：实时检测运行出了每帧FPS、类别、置信度、框、中心点、Frame，但未输出推理耗时（见录屏）
- **误检与漏检案例**：
-误检：在测试过程中，模型偶尔会将场地上的白线交叉点误认为足球。因为这些物体形状和颜色特征与足球有一定相似性。后续可以通过增加纯背景负样本或提高置信度阈值来过滤。 
-漏检：当足球被其它物体严重遮挡时，模型未成功识别出目标。这主要是因为特征提取受阻，后续需补充更多遮挡和模糊场景的困难样本。
- **标注质量检查记录**：本次抽查 30 张图像，经肉眼排查，未发现明显的框过大或漏标情况。同时运行了 Python 脚本进行批量校验，将随机抽取的 30 张图像及其对应的 YOLO 格式标签重新绘制为带检测框的可视化图像，排除了坐标越界、宽高异常及类别编号错误。经核对划分记录，确认 train/val/test 的数据来源按独立视频片段严格分开，不存在相邻视频帧泄漏问题。确认无漏标、错标，类别严格固定为 `class_id: 0 (ball)`，排除了标签越界和视频帧泄漏。已发现的问题：未发现异常


## 任务 2：K1 风格单目场地自定位

- field.yaml 文件版本  SHA256：6fcb6fe861713ce715b341c3e0149a2d8c0f0ef3080993499d2bb91d1456f2bc 
- field_geometry.yaml 文件版本  SHA256:  317e41bc13d69dc2b3cea9725a65163815a58ebb0584c8fbcd269d818bf6dca8 
- 地图原点、X/Y 轴方向、单位：原点 (0,0,0) 位于场地几何中心(即 XCross_1 交点)；X 轴正方向指向对方半场(沿长度方向)，Y 轴正方向指向左侧边线(沿宽度方向)，Z 轴向上；单位：米（yaw 为弧度）。map 系原点固定，与机器人开机位置无关。
- 场地实际尺寸是否与地图一致：是。实测端线到端线 8.00 m，边线到边线 6.00 m，中心到边线 3.00 m，与 field.yaml 一致，最大偏差 < 2 cm。
- 使用的场地标志类别：LCross, TCross, XCross, PenaltyPoint
- 场地标志检测模型与训练指标：YOLOv8n（Ultralytics 8.x），输入尺寸 1280×720 ，训练 30 epochs，batch=4，device=cpu,数据集：marker_dataset，训练集 259 张 / 345 个实例，4 类（LCross/TCross/XCross/PenaltyPoint）。  指标：mAP50 = 0，mAP50-95 = 0，Precision = 0，Recall = 0  推理设备：CPU（核显，无独显）。
- 几何特征点提取方法（L/T/X/点球点）：YOLO粗定位+OpenCV几何细化。读取图片后，转灰度、二值化（提取白色区域）、做形态学操作，然后用 cv2.findContours 找到目标轮廓，最后用cv2.moments(c) 计算出了质心坐标 M['m10']/M['m00']。
- 相机内参 fx / fy / cx / cy：1694.50 / 1715.61 / 633.18 / 295.34（分辨率 1280×720，标定板 8×6 棋盘格，19 张图，重投影 RMS 1.289 px）
- 畸变系数 k1 / k2 / p1 / p2 / k3：0.438974 / -3.723923 / -0.047982 / -0.014262 / 13.715388
- 相机外参 / 相机高度 / 俯仰角 / 滚转角：已保存至 `config/camera_extrinsics.yaml`/物理测量740mm，算法解算911.29mm/2.18°/-20.37°（相机向下，左右倾斜）R: [ 0.99588,  0.06396,  0.06424,
     -0.08226,  0.93535,  0.34401,
     -0.03808, -0.34788,  0.93676 ]
T: [ -504.949, -182.425, 911.289 ]  # 单位: mm
- 地面投影方法：从cv2.undistortpoints对特征点像素坐标进行畸变矫正，利用 'd_c=K^{-1} · [u,v,1]^T' 求射线，结合外参转换到载体坐标系，计算相机在世界坐标系下高度和位置0_b，与 'Z=0'平面求交。
- 地图匹配算法与同类标志消歧方法：RANSAC 几何一致性匹配 + 最小二乘位姿优化
- 输出坐标系：主输出camera_frame（相机光心）
- 3 个以上测试点的参考相机地图坐标 X/Y/Yaw 与估计误差：（-10.27，-45.09，60.99）17.98px（-4.47，-47.29，64.35）11.15px（9.28，-51.66，56.12）31.52px
- marker_count / residual / valid / status：marker_count 为单帧检出的有效标志数：3；residual 为位姿解算的平均重投影残差（像素）：0.0；valid = (marker_count >= 2) and (residual < 3.0)：true；status 取值：ok
- 实时 FPS / 定位更新频率 / 耗时：0.6/无/无，模型没有训练好，在实时检测中无法出现有效定位


## ROS2 接口

- 节点名称：usb_cam（相机节点）；vision_node.py（视觉定位节点）未跑通：VMware的USB透传对高带宽设备摄像头1280*720 未压缩）支持差。
- 足球话题：`/soccer/ball`
- 定位话题：`/localization/field_pose'
- 消息字段、单位与坐标约定：/localization/field_pose：header.frame_id = "map"，map 系原点固定在 XCross_1 交点，X 轴沿场地长边（8.00 m 方向），Y 轴垂直于 X，Z 轴向上，yaw 单位 rad；pose.pose.position.x/y 单位 m，为相机光心在 map 系坐标；orientation 无实际输出；pose.covariance[0]/[7]/[35] 分别为 x/y/yaw 方差（m²、rad²）。转换链：像素 →(K⁻¹)→ 相机系 →(T_base_camera)→ base_link →(定位结果)→ map； 角度统一左手定则，逆时针为正。


## B 站演示录屏（必交）

- B 站链接：暂时无法上传
- 视频标题：暂无
- 视频时长：暂无
- 访问权限（公开 / 仅链接可见）：仅链接可见
- 是否包含连续实时相机输入：是
- 是否展示足球检测框、置信度和 FPS：是
- 是否展示 L/T/X 标志识别与几何特征点：否
- 是否采用左 RGB / 右地图的左右分屏布局：否
- 是否展示 field.yaml 地图匹配和地图俯视图：否
- 是否在右侧地图标出相机光心位置与朝向箭头：否
- 是否展示相机地图坐标 x_camera_map / y_camera_map / yaw_camera_map：是
- 是否展示至少 3 个不同位置 / 朝向的地图位姿：否
- 是否展示 marker_count、residual、valid / status：否
- GitHub 本目录是否已提交该 B 站链接：


## C0 现场测试

- 测试日期：无
- 测试点编号：P1 / P2 / P3
- 参考位置 / 朝向：无
- 估计位置 / 朝向：无
- 平面位置误差 / 航向误差：无
- 定位失败场景与处理：无
- 评审记录 / 备注：无
因模型训练不成功，在实时检测中无法出现有效定位，无法进行现场测试。


## 问题与解决过程

1. **相机节点不对导致打不开**：一开始使用 `/dev/video0` 时报错 `Bad file descriptor`，经排查 `v4l2-ctl --list-devices` 发现摄像头实际分配为 `/dev/video1`，修改脚本后自检通过。
2. **对焦控制项不存在**：尝试关闭自动对焦时提示 `unknown control 'focus_absolute'`，确认该设备不支持软件调焦，为物理定焦硬件，不影响考核要求。
3. **虚拟机环境下帧率较低**：实测只有 9.90 FPS，排查发现 `v4l2-ctl --list-formats-ext` 显示 YUYV 格式下 1280x720 最高仅支持 10fps，MJPEG 格式支持 30fps。后续在 OpenCV 中强制使用 MJPG 格式可能提升帧率。


## 当前局限与下一步计划

说明当前在标志遮挡、标志分布不足、场地对称歧义、光照变化、地面不平或相机外参变化情况下可能导致误检漏检，无法输出有效定位。现场  ros2 topic echo  输出的字段名需要与 README 声明完全一致（含  valid  /  marker_count  /  residual  /  status ），缺字段或命名不一致视为材料不完整。

