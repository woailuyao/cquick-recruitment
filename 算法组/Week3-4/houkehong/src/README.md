# src/ 模块说明

本目录是一个 Python 包（`src`），按**职责**分两组模块。细则模板里的
`src/<接口包名>/` + `src/<视觉定位包名>/`，本项目在同一个包内用模块划分实现，
避免为了目录形式而引入一层无意义的跳转（`run.py` 与 `scripts/` 都直接 `from src.xxx import`）。

## 视觉定位层

| 模块 | 职责 |
|---|---|
| `geometry.py` | 相机模型、畸变、外参、像素→射线→地面求交、SE(2) 变换 |
| `landmarks.py` | L/T 标志的几何细化：框内线段拟合求交点，取亚像素特征点 |
| `matching.py` | 与 `field.yaml` 的地图匹配：同类点对假设 + Kabsch 求解 + 内点投票 + **歧义判定** |
| `config.py` | YAML 加载与启动校验（地图/相机/外参）、`FIELD_MARKER_CLASSES` 单一来源 |
| `app.py` | 主流程：检测 → 几何细化 → 地面投影 → 地图匹配 → 位姿；含位姿滤波与多帧观测聚合 |
| `viz.py` | 左右分屏渲染、地图俯视图、相机光心与朝向箭头 |
| `demo.py` | 合成场景（仅用于链路自检，不参与实拍评估） |

## 接口层

| 模块 | 职责 |
|---|---|
| `ros_bridge.py` | 向 ROS2 发布三个话题：`/localization/field_pose`（JSON）、`/localization/field_pose_2d`（Pose2D）、`/soccer/ball`（JSON） |

WSL 侧的 ROS2 节点在 `scripts/ros_bridge_node.py`（TCP 接收 → 发布话题）。

## 依赖方向

```
run.py / scripts/*     ← 入口与工具
      ↓
    src.app            ← 编排
      ↓
src.matching ← src.landmarks ← src.geometry
      ↓
   src.config          ← 配置与类别
```

`src.app` 不反向依赖 `run.py`；`src.matching` 不依赖视觉库（纯几何），可单独测试。
