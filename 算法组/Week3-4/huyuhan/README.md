# CQuick 算法组第 3–4 周考核交付

**姓名：** 胡钰涵（huyuhan）

**项目范围：** 任务一足球检测、任务二单目场地标志检测与二维自定位。

## 一、演示视频

| 任务 | 视频 |
|---|---|
| 任务一：足球检测 | [B站考核视频](https://www.bilibili.com/video/BV1yesU6rEMp/) |
| 任务二：YOLO足球场地自动定位 | [B站考核视频](https://www.bilibili.com/video/BV14zyGzgEcH/) |

## 二、交付目录

```text
huyuhan/
├── README.md
├── task1/
│   ├── README.md
│   ├── weights/
│   ├── training/
│   ├── evaluation/
│   ├── visualizations/
│   └── error_cases/
└── task2/
    ├── README.md
    ├── bonus/
    │   ├── robust_pose/
    │   └── ros2_offline/
    └── links/
        └── video_links.md
```

## 三、任务一：足球检测

采用 YOLOv8s 进行单类别足球检测。

- 正式权重：`task1/weights/best.pt`
- 类别：`ball`
- Test mAP50：99.50%
- Test mAP50-95：94.786%
- 固定置信度 0.50 时 Precision：97.3684%
- 固定置信度 0.50 时 Recall：100%

交付包含正式权重、训练参数、训练日志、模型选择记录、
独立测试结果、可视化和典型错误案例。

当前轻量包不是完整的从零复现实验包，详细边界见任务一 README。

## 四、任务二：场地定位

基于单目 RGB 相机完成 L/T/X 场地标志检测、
几何特征细化、相机内参标定、地面投影、
地图匹配和二维位姿输出。

主要输出：

- `x_camera_map`
- `y_camera_map`
- `yaw_camera_map`
- `marker_count`
- `residual`
- `valid`
- `status`

2026-11-22 实时自动定位记录：

- 总帧数：2230
- 有效帧：631
- 有效率：28.30%

**重要限制：** 该次运行使用历史近似外参，
`current_installation_verified=false`。
因此不能将上述有效帧数量解释为当前安装下的米制定位精度通过。

P1/P2/P3 现场测量已经完成，但坐标符号、
半场对应关系及航向参考仍需核对后才能计算正式独立误差。

## 五、加分实验

### 5.1 Huber IRLS 稳健位姿优化

独立完成普通 SVD 与 Huber IRLS 对照实验，包括：

- 单异常点实验
- 150 次单异常点偏移扫描
- 360 次多异常点实验
- 收敛性专项诊断
- 五张实验结果图

在本次合成数据实验中，IRLS 在所有 12 组多异常点条件下
取得低于 SVD 的位置误差和航向误差。

该实验尚未接入实时定位链路，不能宣称已经部署。

### 5.2 ROS2 双话题离线验证

建立：

- `/soccer/ball`
- `/localization/field_pose`

完成 2230 帧历史 JSONL 双话题离线回放审计。
原始数据统计与订阅端统计一致，JSON 解析错误为 0。

本项尚未完成实时相机 ROS2 集成。

## 六、交付边界

1. 任务一和任务二的模型、指标及实验结果分别统计，不混用。
2. 任务二当前外参未完成独立核验。
3. 任务二场地数据为 96 张，尚未达到任务书建议的 300 张。
4. Huber IRLS 为合成数据独立实验，不代表真实场地精度。
5. ROS2 为历史数据离线回放，不代表实时相机集成。
6. 所有未完成项目按实际状态披露，不将其标记为已通过。

## 七、详细报告

- [任务一 README](task1/README.md)
- [任务二 README](task2/README.md)
- [视频链接清单](task2/links/video_links.md)
- [Huber IRLS 实验报告](task2/bonus/robust_pose/README.md)
- [ROS2 离线审计报告](task2/bonus/ros2_offline/README.md)

## 最终交付说明（2026-11-23）

### 任务一：足球检测
- 正式模型：YOLOv8s。
- Test mAP50：99.50%。
- Test mAP50-95：94.79%。
- 固定 conf=0.50：Precision 97.37%，Recall 100%。
- B站演示：[任务一足球检测](https://www.bilibili.com/video/BV1yesU6rEMp/)。

### 任务二：单目场地定位
- 场地模型：YOLOv8n formal_v1。
- 场地数据：96 张图片、307 个标注实例、13 个独立视频源。
- 完成个人现场测量地图、内参标定、外参拟合、几何细化、地面投影及多候选地图匹配。
- B站演示：[任务二场地自动定位](https://www.bilibili.com/video/BV14zyGzgEcH/)。
- 当前定位有效帧率和现场独立误差仍有改进空间，详见 task2/README.md。

### 模型权重校验
- Task1 SHA256：
  `732ecee9f41639b6888632e1cd8417569807f570a680423b29547c9752824eae`
- Task2 SHA256：
  `2603112118bcaf24b050f3870292dd8f23ba491bd407f7eae83f516ff6c82418`
- 大型权重和原始视频不纳入 Git，权重本机源路径及校验值见各任务 README。
- 共享存储下载地址尚未配置，不以本机路径冒充公开下载链接。

### 加分成果
- Huber IRLS 稳健位姿优化：包含实验脚本、异常点诊断、对照日志及可视化。
- ROS2：两个任务的节点封装、launch 文件及离线 DDS 验证材料。
- ROS2 实时相机端到端链路尚未验证；不申报 TensorRT 或任务二 ONNX 对照成果。

### 交付边界
本目录保留工程源码、配置、指标、实验报告及必要证据。
未完成的现场误差核验、当前安装外参验证和实时链路验证均如实披露。
