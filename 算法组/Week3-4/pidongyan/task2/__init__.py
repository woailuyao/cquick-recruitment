"""任务二：单目足球定位流水线。

链路：标志检测(YOLO) → 白线交点亚像素细化 → 类别候选 + RANSAC 对应关系
      → solvePnP 求相机位姿 → K⁻¹ 反投影射线 → Z=0 地面求交 → 世界坐标。

坐标系约定（与 config/field_geometry.yaml 一致）：
  场地系 F：原点=场地中心，+X 指向唯一球门，+Y 面向 +X 左侧，+Z 向上，米。
  相机系 C：OpenCV 约定，x 右、y 下、z 前向光轴。
  位姿参数：(x, y, h, yaw, pitch)：
    (x,y) = 光心在地面投影的场地坐标，h = 光心高度；
    yaw = 光轴水平投影方向（+X 为 0°，逆时针/朝 +Y 为正）；
    pitch = 光轴俯角（下俯为正）。
"""

from .geometry import (
    load_intrinsics, load_field, field_points_by_class,
    camera_R, pose_from_Rt, project_field_points,
    undistort_ray, ground_intersect, pixel_to_ground,
)
from .correspondence import ransac_pose_from_detections, solve_pose_pnp
from .refine import refine_junction
from .pipeline import PoseResult, FieldLocalizer

__all__ = [
    "load_intrinsics", "load_field", "field_points_by_class",
    "camera_R", "pose_from_Rt", "project_field_points",
    "undistort_ray", "ground_intersect", "pixel_to_ground",
    "ransac_pose_from_detections", "solve_pose_pnp",
    "refine_junction", "PoseResult", "FieldLocalizer",
]
