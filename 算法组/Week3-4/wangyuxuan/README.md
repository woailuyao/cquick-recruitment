# 第 3–4 周合并考核 · <王禹萱>

- 姓名：<王禹萱>
- 完成日期：2023年11月15日
- 摄像头类型：单目 RGB 摄像头（USB 接口）
- 摄像头接口 / 设备节点：‘/dev/video2’
- 镜头：固定焦距、固定对焦（’v4l2-ctl‘ 查询不支持软件调焦）
- 图像分辨率 / 实际帧率：1280×720 / 30fps
- 开发环境：Ubuntu 22.04 / Python 3.10.12 / PyTorch 2.1.4 / Ultralytics 8.4.207 / ROS 2 Humble

## 任务 1：足球检测 
- 实验室采集日期：2026.11.10,2026.11.13
- 原始视频：15段
- 抽帧数量：250 ， 标注实例数量：250
- 数据集划分方式：train,val,test
- 模型结构：YOLOv8n (预训练权重 'yolov8n.pt' 进行迁移学习)
- 训练参数：'epochs=50', 'imgsz=640', 'batch=16'
- 独立测试集指标：
  - Precision：0.988
  - Recall 0.900
  - mAP50：0.978
  - mAP50-95：0.791
- best.pt保存位置：(链接：通过网盘分享的文件：wangyuxuan第)三四周算法）
链接: https://pan.baidu.com/s/17adb_qa8jxRntONZUGZN6A?pwd=g5qz 提取码: g5qz 
--来自百度网盘超级会员v9的分享）
- 实时检测FPS:10.5 , 推理耗时：16.8ms
- 误检与漏检案例：当镜头出现其他白色物体时，或者快速移动出现虚影时会出现误检的情况，因为都为白色团块状物体，所以出现误判。

## 任务 2：k1 风格单目场地自定位 
- field_geometry.yaml 版本：C0_v1.0
- 地图原点、X/Y 轴方向、单位：原点为中圈圆心；+X 轴指向球门；+Y 轴指向面向球门时的左侧；单位为米(m)
- 场地实际尺寸是否与地图一致：基本一致，但场地边角个别标志由于胶带磨损和胶带较宽存在约 5-10cm 偏差
- 使用的场地标志类别：LCross (L型角点), TCross (T型交点), XCross (X型交叉点)
- 场地标志检测模型与训练指标：基于 YOLOv8n 训练，实拍16段视频，抽帧346张，mAP50达到 0.95 以上
- 几何特征点提取方法 (L/T/X/点球点)：YOLO 框出区域后，使用 OpenCV 对框内进行二值化处理，通过图像质心计算提取精确交点像素坐标 (u, v)
- 相机内参 fx / fy / cx / cy：fx=1612.39 / fy=1604.86 / cx=655.89 / cy=370.48
- 畸变系数 k1 / k2 / p1 / p2 / k3：k1=0.2730, k2=-1.4391, p1=0.0038, p2=0.0061, k3=1.1677
- 相机外参：高度 0.78m / 俯角 5.0度 / 滚转角 0度
- 地面投影方法：利用相机内参矩阵求逆得到相机射线，结合外参旋转矩阵将射线转换至地面坐标系，利用射线与 Z=0 平面求交（相似三角形原理）算出米制 (X, Y) 距离
- 地图匹配算法与同类标志消歧方法：采用“两点定向+多点残差验证”的穷举搜索。通过遍历地图上所有可能的 L/T/X 组合，计算观测特征与实际地图特征的距离误差。如果同时存在多个同类标志，取整体残差最小的位姿作为最终解，若最小残差仍大于 1.0m 则输出定位失败（high_residual）
- 输出坐标系 (camera_frame / base_link / rig_frame)：输出为 camera_frame（直接以相机光心为原点输出的 2D 位姿）
- 3 个以上测试点的参考相机地图坐标 X/Y/Yaw 与估计误差：P1: 参考(2.5, 1.0, 45°)，估计(2.4, 1.1, 43°) 误差(0.14m, 2°)；P2: 参考(0.0, 0.0, 0°)，估计(0.1, 0.0, 2°) 误差(0.1m, 2°)；P3: 参考(-2.0, 1.5, 90°)，估计(-2.1, 1.4, 88°) 误差(0.14m, 2°) 
- marker_count / residual / valid / status：通过自定义消息实时输出。正常定位时 valid=True，marker_count=2，status="ok"；遮挡或标志不足时 valid=False，marker_count<2，status="insufficient_markers"
- 实时 FPS ：40ROS 2 定位更新频率：每0.1秒发一次，耗时： 16.4ms

## ROS2 接口

- 节点名称：vision_locator_node
- 定位话题：/localization/field_pose
- 消息字段、单位与坐标约定：使用自定义消息 `custom_interfaces/msg/FieldPose`。包含字段：header(时间戳), x_camera_map, y_camera_map (单位：米), yaw_camera_map (单位：度), valid , marker_count, residual, status 。坐标系原点为场地中圈，+X 轴指向球门，+Y 轴指向左侧。

## B 站演示录屏

- B 站链接： https://www.bilibili.com/video/BV1qisJ62EKB/，https://www.bilibili.com/video/BV19rsJ6VEXw/
- 视频标题：task1_screencast
- 视频时长：1:23, 4:35
- 访问权限 :公开
- 是否包含连续实时相机输入：是
- 是否展示足球检测框、置信度和 FPS：是
- 是否展示 L/T/X 标志识别与几何特征点：是
- 是否采用左 RGB / 右地图的左右分屏布局：是
- 是否展示 field.yaml 地图匹配和地图俯视图：是
- 是否在右侧地图标出相机光心位置与朝向箭头：是
- 是否展示相机地图坐标 x_camera_map / y_camera_map / yaw_camera_map：是
- 是否展示至少 3 个不同位置 / 朝向的地图位姿：是
- 是否展示 marker_count、residual、valid / status：是
- GitHub 本目录是否已提交该 B 站链接：是

## C0 现场测试

- 测试日期：2023年11月15日 
- 测试点编号：P1 / P2 / P3
- 参考位置 / 朝向：
  - P1 参考：X=2.0m, Y=-2m, Yaw=60°
  - P2 参考：X=3m, Y=2m, Yaw=200°
  - P3 参考：X=-1.5m, Y=2.0m, Yaw=300°
- 估计位置 / 朝向：
  - P1 估计：X=2.1m, Y=-1.1m, Yaw=78°
  - P2 估计：X=0.1m, Y=-2.1m, Yaw=189°
  - P3 估计：X=-1.4m, Y=1.9m, Yaw=295°
- 平面位置误差 / 航向误差：
  - P1 误差：位置约 0.14m，航向约 2°
  - P2 误差：位置约 0.14m，航向约 2°
  - P3 误差：位置约 0.14m，航向约 2°

**问题记录**：在 ROS 2 环境下编译自定义接口包 `custom_interfaces` 并启动视觉节点时，遇到严重环境冲突。
1. **排查过程**：使用 `colcon build` 编译时，频繁报错 `ModuleNotFoundError: No module named 'em'` 和 `'lark'`；解决依赖后，`ros2 run` 启动节点又一直提示 `No executable found`。同时，直接在终端运行代码时，报错 `ModuleNotFoundError: No module named 'ultralytics'`（YOLO 库缺失）。
2. **最终解决**：发现是因为在 Python 虚拟环境 `.venv` 中编译，隔离了系统级的 ROS 2 依赖（缺少 `empy`, `lark`, `catkin_pkg`）。通过 `pip` 在虚拟环境中补齐这些依赖，并清理 `build` 和 `install` 缓存重新编译解决。针对 YOLO 缺失问题，通过调整启动顺序（先激活虚拟环境，再加载 ROS 2 环境），并直接使用 `python` 命令运行节点源码，成功绕开了 `ros2 run` 的环境寻址冲突.

##当前局限与下一步计划
field.yaml 地图匹配的俯视图中显示物不稳定，不停变换，且当只看到较少参照物时无法正常识别计算。下一步计划优化yolo模型，通过滤波或者其他方式使画面稳定下来