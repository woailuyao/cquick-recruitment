"""YOLO 检测 + 标志几何细化 + 像素->地面坐标。

依赖 cv2 / ultralytics；matcher 与 geom 仍是纯 numpy。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from .geom import ray_to_ground, sigma_from_range, weight_from_sigma
from .matcher import Observation

_NAME_ALIASES = {
    "l": "LCross", "lcross": "LCross", "l_cross": "LCross",
    "t": "TCross", "tcross": "TCross", "t_cross": "TCross",
    "x": "XCross", "xcross": "XCross", "x_cross": "XCross",
    "penalitypoint": "PenaltyPoint", "penaltypoint": "PenaltyPoint",
    "penalty": "PenaltyPoint", "penality": "PenaltyPoint", "p": "PenaltyPoint",
    "ball": "ball",
}


def norm_name(name: str) -> str:
    return _NAME_ALIASES.get(str(name).strip().lower().replace(" ", "_"), str(name))


@dataclass
class Detection:
    cls_name: str
    conf: float
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def center(self) -> Tuple[float, float]:
        return (self.x1 + self.x2) / 2.0, (self.y1 + self.y2) / 2.0

    @property
    def diag(self) -> float:
        return math.hypot(self.x2 - self.x1, self.y2 - self.y1)

    @property
    def wh(self) -> Tuple[float, float]:
        return self.x2 - self.x1, self.y2 - self.y1


class YoloDetector:
    """ultralytics 的薄封装。weights 不存在时返回空检测（便于无模型调试）。"""

    def __init__(self, weights: str, conf: float = 0.25, iou: float = 0.45,
                 imgsz: int = 640, device: str = "0", verbose: bool = True):
        import os
        # 防止配置里写 GPU 但机器只有 CPU（ultralytics 会直接抛 ValueError）
        if str(device).isdigit():
            try:
                import torch
                if not torch.cuda.is_available():
                    device = "cpu"
            except Exception:
                device = "cpu"
        self.enabled = bool(weights) and os.path.exists(weights)
        self.conf, self.iou, self.imgsz, self.device = conf, iou, imgsz, device
        self.model = None
        if not self.enabled:
            if verbose:
                print(f"[warn] 找不到权重 {weights}，该检测器禁用（界面仍可运行）")
            return
        from ultralytics import YOLO
        self.model = YOLO(weights)
        names = getattr(self.model, "names", None) or {}
        self.names = {int(k): norm_name(v) for k, v in names.items()} if names else {}
        if verbose:
            print(f"[det] {weights} classes={self.names}")
        self._warmup(verbose)

    def _warmup(self, verbose: bool) -> None:
        try:
            dummy = np.zeros((self.imgsz, self.imgsz, 3), np.uint8)
            self.model.predict(dummy, imgsz=self.imgsz, device=self.device,
                               verbose=False)
            if verbose:
                print("[det] warmup ok")
        except Exception as e:                       # pragma: no cover
            print(f"[warn] warmup 失败: {e}")

    def __call__(self, frame_bgr: np.ndarray) -> List[Detection]:
        if not self.enabled or self.model is None:
            return []
        res = self.model.predict(frame_bgr, conf=self.conf, iou=self.iou,
                                 imgsz=self.imgsz, device=self.device,
                                 verbose=False)
        out: List[Detection] = []
        for r in res:
            if r.boxes is None:
                continue
            xyxy = r.boxes.xyxy.cpu().numpy()
            conf = r.boxes.conf.cpu().numpy()
            cls = r.boxes.cls.cpu().numpy().astype(int)
            for (a, b, c, d), cf, ci in zip(xyxy, conf, cls):
                out.append(Detection(self.names.get(int(ci), str(int(ci))),
                                     float(cf), float(a), float(b), float(c), float(d)))
        return out


# ----------------------------------------------------------- 标志几何细化

def _white_mask(roi_bgr: np.ndarray, v_min: int, s_max: int) -> np.ndarray:
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    m = ((hsv[:, :, 2] > v_min) & (hsv[:, :, 1] < s_max)).astype(np.uint8) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    return m


def _line_angle(seg) -> float:
    return math.atan2(seg[3] - seg[1], seg[2] - seg[0])


def _fit_line_from_segs(segs: Sequence[np.ndarray]) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    pts: List[List[float]] = []
    for s in segs:
        pts.append([float(s[0]), float(s[1])])
        pts.append([float(s[2]), float(s[3])])
    if len(pts) < 2:
        return None
    arr = np.asarray(pts, np.float32).reshape(-1, 1, 2)
    vx, vy, x0, y0 = cv2.fitLine(arr, cv2.DIST_L2, 0, 0.01, 0.01).ravel()
    d = np.array([float(vx), float(vy)], float)
    n = np.linalg.norm(d)
    if n < 1e-9:
        return None
    return np.array([float(x0), float(y0)]), d / n


def _intersect(p1, d1, p2, d2) -> Optional[np.ndarray]:
    A = np.array([[d1[0], -d2[0]], [d1[1], -d2[1]]], float)
    if abs(np.linalg.det(A)) < 1e-6:
        return None
    lam, _ = np.linalg.solve(A, p2 - p1)
    return p1 + float(lam) * d1


def refine_cross(roi_bgr: np.ndarray, cfg: dict
                 ) -> Optional[Tuple[np.ndarray, int, float]]:
    """L/T/X 细化：白线掩膜 -> 两个方向分组 -> 拟合两条直线 -> 求交点。

    返回 (像素点(ROI 坐标), 质量=两方向线段数较小者, 两线夹角deg)。
    """
    m = _white_mask(roi_bgr, int(cfg.get("white_v_min", 140)),
                    int(cfg.get("white_s_max", 70)))
    if int(m.sum()) // 255 < int(cfg.get("min_area_px", 40)):
        return None

    segs = cv2.HoughLinesP(m, 1, np.pi / 180.0, threshold=12,
                           minLineLength=int(cfg.get("min_line_len", 15)),
                           maxLineGap=int(cfg.get("max_line_gap", 5)))
    if segs is None or len(segs) < 2:
        return None
    segs = np.asarray(segs).reshape(-1, 4).astype(float)

    # 方向按 2*angle 做圆周聚类，分成两组
    a2 = np.array([_line_angle(s) * 2.0 for s in segs])
    z = np.exp(1j * a2)
    c1 = float(np.angle(z.mean()))
    c2 = float(np.angle(z.mean() + np.pi))
    g1 = np.zeros(len(segs), bool)
    for _ in range(8):
        d1 = np.abs(np.angle(np.exp(1j * (a2 - c1))))
        d2 = np.abs(np.angle(np.exp(1j * (a2 - c2))))
        g1 = d1 <= d2
        if g1.all() or (~g1).all():
            return None                    # 只有一个方向，不是十字/T/L
        c1 = float(np.angle(np.exp(1j * a2[g1]).mean()))
        c2 = float(np.angle(np.exp(1j * a2[~g1]).mean()))

    l1 = _fit_line_from_segs(segs[g1])
    l2 = _fit_line_from_segs(segs[~g1])
    if l1 is None or l2 is None:
        return None

    P = _intersect(l1[0], l1[1], l2[0], l2[1])
    if P is None:
        return None

    angle = math.degrees(math.acos(min(1.0, abs(float(np.dot(l1[1], l2[1]))))))
    if angle < float(cfg.get("min_cross_angle_deg", 20.0)):
        return None
    return P, int(min(g1.sum(), (~g1).sum())), angle


def refine_blob(roi_bgr: np.ndarray, cfg: dict) -> Optional[Tuple[np.ndarray, int, float]]:
    """点球点细化：取 ROI 内最亮的圆形块质心。"""
    v = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)[:, :, 2]
    if v.size == 0:
        return None
    thr = int(max(cfg.get("white_v_min", 140), np.percentile(v, 90)))
    mask = (v >= thr).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    area = float(cv2.contourArea(c))
    if area < float(cfg.get("min_area_px", 40)):
        return None
    M = cv2.moments(c)
    if M["m00"] <= 0:
        return None
    return np.array([M["m10"] / M["m00"], M["m01"] / M["m00"]]), 1, 90.0


def refine_landmark(frame: np.ndarray, det: Detection, cfg: dict
                    ) -> Tuple[Optional[Tuple[float, float]], str, float]:
    """返回 (像素点, 方法, 质量)。失败时方法为 bbox_center。"""
    w, h = det.wh
    if w <= 1 or h <= 1:
        return det.center, "bbox_center", 0.0
    pad = int(round(0.20 * max(w, h)))
    H, W = frame.shape[:2]
    x1 = max(0, int(det.x1) - pad)
    y1 = max(0, int(det.y1) - pad)
    x2 = min(W, int(det.x2) + pad)
    y2 = min(H, int(det.y2) + pad)
    if x2 - x1 < 6 or y2 - y1 < 6:
        return det.center, "bbox_center", 0.0
    roi = frame[y1:y2, x1:x2]

    fn = refine_blob if det.cls_name == "PenaltyPoint" else refine_cross
    out = fn(roi, cfg)
    if out is None:
        return det.center, "bbox_center", 0.0

    P, quality, angle = out
    gx, gy = float(P[0]) + x1, float(P[1]) + y1

    # 交点必须落在 ROI 内且不能离 bbox 中心太远
    if not (0 <= P[0] < roi.shape[1] and 0 <= P[1] < roi.shape[0]):
        return det.center, "bbox_center", 0.0
    off = math.hypot(gx - det.center[0], gy - det.center[1])
    if off > float(cfg.get("max_offset_ratio", 0.60)) * max(det.diag, 1.0):
        return det.center, "bbox_center", 0.0
    return (gx, gy), ("line_cross" if det.cls_name != "PenaltyPoint" else "blob"), float(quality)


# --------------------------------------------------- 像素 -> 载体系地面坐标

def observations_from_detections(frame: np.ndarray, dets: Sequence[Detection],
                                 intr: dict, extr: dict, geom_cfg: dict,
                                 refine_cfg: dict, field_map) -> Tuple[List[Observation], List[dict]]:
    """把标志检测变成载体系地面观测；失败的观测带原因返回用于界面统计。"""
    K, dist = intr["K"], intr["dist"]
    R, t = extr["R"], extr["t"]
    fy, h_cam = intr["fy"], float(extr["height"])
    max_range = float(geom_cfg.get("max_ground_range_m", 8.0))
    min_down = float(geom_cfg.get("min_ray_downward", 0.001))
    delta_px = float(geom_cfg.get("delta_px", 1.0))

    obs: List[Observation] = []
    dbg: List[dict] = []
    if not dets:
        return obs, dbg

    pix: List[Tuple[float, float]] = []
    use_refine = bool(refine_cfg.get("enabled", True))
    for d in dets:
        if use_refine:
            p, method, quality = refine_landmark(frame, d, refine_cfg)
        else:
            p, method, quality = d.center, "bbox_center", 0.0
        pix.append((float(p[0]), float(p[1])))
        dbg.append({"cls": d.cls_name, "conf": d.conf, "bbox": (d.x1, d.y1, d.x2, d.y2),
                    "pixel": (float(p[0]), float(p[1])), "method": method,
                    "quality": quality, "reason": None, "distance": None})

    pts = np.asarray(pix, np.float64).reshape(-1, 1, 2)
    norm = cv2.undistortPoints(pts, K, dist, P=None).reshape(-1, 2)   # 归一化坐标

    for i, d in enumerate(dets):
        nx, ny = float(norm[i, 0]), float(norm[i, 1])
        d_c = np.array([nx, ny, 1.0], float)
        d_c /= np.linalg.norm(d_c)
        d_b = R @ d_c
        P, why = ray_to_ground(d_b, t, min_down=min_down, max_range=max_range)

        if P is None:
            dbg[i]["reason"] = why
            continue
        if field_map is not None and not field_map.in_field(float(P[0]), float(P[1])):
            dbg[i]["reason"] = "outside_field"
            continue

        D = float(math.hypot(P[0], P[1]))
        sigma = sigma_from_range(D, delta_px * (2.5 if dbg[i]["method"] == "bbox_center" else 1.0),
                                 fy, h_cam)
        dbg[i]["reason"] = None
        dbg[i]["distance"] = D
        obs.append(Observation(
            type=d.cls_name, x=float(P[0]), y=float(P[1]),
            sigma=sigma, confidence=float(d.conf),
            weight=weight_from_sigma(sigma) * max(float(d.conf), 0.05),
            pixel=(float(pix[i][0]), float(pix[i][1])),
            distance=D,
        ))
    return obs, dbg
