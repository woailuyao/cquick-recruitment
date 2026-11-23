"""流水线：把各模块串起来。上层入口见 tools/run_task2.py。

FieldLocalizer 提供三种解算方式：
  1. clicks      —— 人工点选的「点名: 像素」，直接 PnP（最准，用于自证/评估）
  2. detections  —— YOLO 检测的「类别+框」，亚像素细化后 RANSAC 对应解算
  3. video       —— 逐帧 detections + 球检测 -> 世界坐标 CSV + 叠加可视化
"""
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

from .geometry import (camera_R, field_points_by_class, load_field,
                       load_intrinsics, pixel_to_ground, pose_from_Rt,
                       project_field_points)
from .correspondence import ransac_pose_from_detections, solve_pose_pnp
from .refine import refine_junction

GREEN = (80, 200, 80)
RED = (60, 60, 230)
YELLOW = (60, 220, 230)
WHITE = (255, 255, 255)


@dataclass
class PoseResult:
    C_F: np.ndarray            # 光心场地坐标 (x,y,z)
    yaw_deg: float
    pitch_deg: float
    rms_px: float
    names: list = field(default_factory=list)     # 每个观测点被分配的点名
    errs_px: np.ndarray = None
    inlier: np.ndarray = None

    @property
    def pos_str(self):
        return f"x={self.C_F[0]:+.3f} y={self.C_F[1]:+.3f} h={self.C_F[2]:.3f}"

    @property
    def ori_str(self):
        return f"yaw={self.yaw_deg:+.2f}° pitch={self.pitch_deg:+.2f}°"


class FieldLocalizer:
    def __init__(self, intrinsics_yaml, field_yaml, px_thresh=10.0):
        from .geometry import distortion_fold_warning
        self.K, self.dist = load_intrinsics(intrinsics_yaml)
        self.field = load_field(field_yaml)
        self.field_by_class = field_points_by_class(self.field)
        self.px_thresh = px_thresh
        ok, msg = distortion_fold_warning(self.K, self.dist)
        if not ok:
            raise ValueError(f"内参文件 {intrinsics_yaml} 的畸变系数病态：{msg}")

    # ------------------------------------------------------------------
    # 解算入口
    # ------------------------------------------------------------------
    def solve_from_clicks(self, clicks, prefer_h=None):
        """clicks: {点名: (u,v)} -> PoseResult（直接对应，无 RANSAC）。

        prefer_h: 现场实测相机高度(m)。给定时启用 IPPE 多解选优，
                  消掉平面 PnP 的地下镜像解；None 保持单解行为（selftest 用）。
        """
        items = [(n, self.field[n]) for n in clicks if n in self.field]
        if len(items) < 4:
            raise ValueError("点选已知点至少 4 个")
        uvs = [clicks[n] for n, _ in items]
        ok, R, C, err, rms = solve_pose_pnp(items, uvs, self.K, self.dist,
                                            prefer_h=prefer_h)
        if not ok:
            raise RuntimeError("PnP 求解失败")
        _, yaw, pitch = pose_from_Rt(R, (-R @ C).reshape(3, 1))
        return PoseResult(C_F=C, yaw_deg=yaw, pitch_deg=pitch, rms_px=rms,
                          names=[n for n, _ in items], errs_px=err,
                          inlier=np.ones(len(items), bool))

    def solve_from_detections(self, gray, detections, refine=True, prior=None):
        """detections: [{'cls','box','conf'}, ...]（box=[x1,y1,x2,y2] 像素）
        prior: {'h': ..., 'h_sigma': ..., 'pitch_deg': ..., 'pitch_sigma': ...}
        返回 (PoseResult | None, refined 列表 [(cls,uv,method)])。"""
        refined = []
        for det in detections:
            box = det["box"]
            if refine:
                u, v, method = refine_junction(gray, det["cls"], box)
            else:
                u, v, method = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2, "box_center"
            refined.append((det["cls"], (u, v), method))

        sol = ransac_pose_from_detections([(c, uv) for c, uv, _ in refined],
                                          self.field_by_class, self.K,
                                          self.dist, self.px_thresh,
                                          prior=prior)
        if sol is None:
            return None, refined
        return PoseResult(C_F=sol["C_F"], yaw_deg=sol["yaw_deg"],
                          pitch_deg=sol["pitch_deg"], rms_px=sol["rms"],
                          names=sol["names"], errs_px=sol["errs"],
                          inlier=sol["inlier"]), refined

    # ------------------------------------------------------------------
    # 世界坐标
    # ------------------------------------------------------------------
    def pixel_to_world(self, pose: PoseResult, uv, ball_radius=0.0):
        """像素 -> 地面世界坐标 (x,y)，含可选球半径接触点补偿。"""
        R_CF = camera_R(pose.yaw_deg, pose.pitch_deg)
        return pixel_to_ground(R_CF, pose.C_F, self.K, self.dist, uv,
                               ball_radius=ball_radius)

    # ------------------------------------------------------------------
    # 可视化
    # ------------------------------------------------------------------
    def draw(self, frame, pose: PoseResult, refined=None, ball=None,
             ball_world=None):
        out = frame.copy()
        if refined:
            for cls, (u, v), method in refined:
                color = GREEN if method != "box_center" else YELLOW
                cv2.circle(out, (int(u), int(v)), 4, color, -1)
                cv2.putText(out, f"{cls}:{method}", (int(u) + 6, int(v) - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)
        if ball:
            x1, y1, x2, y2 = [int(v) for v in ball["box"]]
            cv2.rectangle(out, (x1, y1), (x2, y2), RED, 2)
        if pose:
            cv2.putText(out, f"C0 {pose.pos_str} {pose.ori_str}",
                        (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, WHITE, 1)
            cv2.putText(out, f"rms={pose.rms_px:.2f}px pts={pose.names}",
                        (8, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.45, WHITE, 1)
        if ball_world is not None:
            cv2.putText(out, f"ball@C0: x={ball_world[0]:+.2f} y={ball_world[1]:+.2f} m",
                        (8, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.45, RED, 1)
        return out
