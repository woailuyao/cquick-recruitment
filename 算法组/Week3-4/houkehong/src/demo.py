# -*- coding: utf-8 -*-
"""
合成场景模拟器（**仅用于验证几何与匹配链路**，不用于训练、不作为考核数据）

原理：给定一个"真值位姿"，把 field.yaml 里的已知特征按真实投影关系打到图像上，
      叠加足球，得到一张有真值的模拟相机画面。
      程序再把这张图当成相机输入，跑一遍 检测 -> 细化 -> 地面投影 -> 地图匹配，
      最后和真值对比 —— 这样在**没有相机、没有标定、没有数据集**的情况下，
      也能确认几何公式和匹配算法是对的。

真值链路（与 geometry.py 严格互逆）：
    P_map --R(-yaw)-->> P_carrier --R_cb^T-->> P_camera -> 像素
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

from .geometry import CameraModel, Extrinsics, rot2, transform_points


@dataclass
class DemoDetection:
    bbox: tuple
    label: str
    conf: float


class SyntheticScene:
    def __init__(self, field_map, camera: CameraModel, extrinsics: Extrinsics,
                 seed: int = 0, max_range: float = 14.0, drop_rate: float = 0.0):
        self.field = field_map
        self.cam = camera
        self.ex = extrinsics
        self.rng = np.random.default_rng(seed)
        self.max_range = max_range
        self.drop_rate = drop_rate

    # ------------------------------------------------------------ 真值投影
    def map_to_pixel(self, pts_map: np.ndarray, pose_xyyaw) -> tuple[np.ndarray, np.ndarray]:
        """地图点 -> 像素。返回 (uv (N,2), visible (N,))"""
        pts_map = np.asarray(pts_map, dtype=np.float64).reshape(-1, 2)
        x, y, yaw_deg = pose_xyyaw
        yaw = math.radians(yaw_deg)

        # 地图 -> 载体（逆变换）
        R = rot2(yaw)
        p_rel = pts_map - np.array([x, y])
        p_carrier_xy = (R.T @ p_rel.T).T

        # 铺到地面 z=0，再转到相机系
        p_carrier = np.hstack([p_carrier_xy, np.zeros((len(p_carrier_xy), 1))])
        p_cam = (self.ex.R.T @ (p_carrier - self.ex.t).T).T

        uv, in_front = self.cam.project(p_cam)
        H, W = self.cam.height, self.cam.width
        visible = in_front & np.isfinite(uv[:, 0]) & np.isfinite(uv[:, 1])
        visible &= (uv[:, 0] > 2) & (uv[:, 0] < W - 2) & (uv[:, 1] > 2) & (uv[:, 1] < H - 2)
        dist = np.linalg.norm(p_cam[:, :2], axis=1)
        visible &= dist < self.max_range
        return uv, visible, p_cam[:, 2]

    # ------------------------------------------------------------ 造画面
    def render(self, pose_xyyaw, ball_map_xy: Optional[tuple] = None,
               noise: bool = True):
        H, W = self.cam.height, self.cam.width
        img = np.zeros((H, W, 3), np.uint8)

        # 草地背景（上远下近的渐变）
        for yy in range(H):
            t = yy / H
            b = int(38 + 26 * t)
            g = int(96 + 44 * t)
            r = int(40 + 24 * t)
            img[yy, :] = (b, g, r)

        # 地平线以上压暗
        horizon = int(H * 0.32)
        img[:horizon] = (58, 52, 46)

        if noise:
            n = self.rng.normal(0, 4, (H, W, 3))
            img = np.clip(img.astype(np.float32) + n, 0, 255).astype(np.uint8)

        dets: list[DemoDetection] = []
        gt_pixels = {}

        # 场地标志
        pts = np.array([[f.x, f.y] for f in self.field.features], dtype=np.float64)
        if len(pts):
            uv, visible, depth = self.map_to_pixel(pts, pose_xyyaw)
            for i, f in enumerate(self.field.features):
                if not visible[i]:
                    continue
                if self.drop_rate > 0 and self.rng.random() < self.drop_rate:
                    continue
                u, v = uv[i]
                size = int(np.clip(170.0 / max(1.0, depth[i]), 6, 46))
                self._draw_glyph(img, int(u), int(v), f.type, size)
                half = size + 5
                dets.append(DemoDetection(
                    (u - half, v - half, u + half, v + half), f.type,
                    float(np.clip(0.95 - 0.02 * depth[i], 0.5, 0.99))))
                gt_pixels[f.fid] = (float(u), float(v))

        # 足球
        ball_px = None
        if ball_map_xy is not None:
            buv, bvis, bdep = self.map_to_pixel(np.array([ball_map_xy]), pose_xyyaw)
            if bvis[0]:
                bu, bv = buv[0]
                r = int(np.clip(120.0 / max(1.0, bdep[0]), 5, 40))
                cv2.circle(img, (int(bu), int(bv)), r, (245, 245, 245), -1, cv2.LINE_AA)
                cv2.circle(img, (int(bu), int(bv)), r, (40, 40, 40), 2, cv2.LINE_AA)
                dets.append(DemoDetection(
                    (bu - r - 2, bv - r - 2, bu + r + 2, bv + r + 2), "ball",
                    float(np.clip(0.96 - 0.02 * bdep[0], 0.5, 0.99))))
                ball_px = (float(bu), float(bv))

        return img, dets, gt_pixels, ball_px

    @staticmethod
    def _draw_glyph(img, cu, cv, kind, s):
        """在像素 (cu,cv) 画一个白色标志符号，交会点正好落在 (cu,cv)。"""
        s = max(5, int(s))
        white = (250, 250, 250)
        th = max(2, s // 6)
        if kind == "LCross":
            # 两条腿从角点 (cu,cv) 出发分别向右、向下伸出，交会顶点就是角点
            cv2.line(img, (cu, cv), (cu + s, cv), white, th, cv2.LINE_AA)
            cv2.line(img, (cu, cv), (cu, cv + s), white, th, cv2.LINE_AA)
        elif kind == "TCross":
            # 横线以 (cu,cv) 为中心，竖线从交点向下，交会点就是 (cu,cv)
            cv2.line(img, (cu - s, cv), (cu + s, cv), white, th, cv2.LINE_AA)
            cv2.line(img, (cu, cv), (cu, cv + s), white, th, cv2.LINE_AA)
        elif kind == "XCross":
            # 两条对角线都过 (cu,cv)
            cv2.line(img, (cu - s, cv - s), (cu + s, cv + s), white, th, cv2.LINE_AA)
            cv2.line(img, (cu - s, cv + s), (cu + s, cv - s), white, th, cv2.LINE_AA)
        elif kind == "PenaltyPoint":
            cv2.circle(img, (cu, cv), max(4, s // 2), white, -1, cv2.LINE_AA)
