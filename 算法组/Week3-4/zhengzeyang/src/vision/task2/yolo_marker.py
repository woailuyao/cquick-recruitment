# -*- coding: utf-8 -*-
"""
yolo_marker.py —— YOLO 检测 L/T/X，框内几何细化落到真实交点
流程：YOLO 出框 → 框内白线中线 + 霍夫直线 → 两线求交 → 交点作为特征点 (u,v)。
点球点 P 不在此检测（无画出的特征），由调用方用两个 X 交点的地面中点导出。
"""
from __future__ import annotations

import math

import cv2
import numpy as np

from line_utils import (DEFAULT_PARAMS, _line_segments, _seg_intersect,
                        white_mask)

# 模型类别名 -> 内部单字母类型。兼容 3 类与 4 类权重。
CLASS_MAP = {
    "lcross": "L", "l": "L",
    "tcross": "T", "t": "T",
    "xcross": "X", "x": "X",
    "penaltypoint": "P", "penalitypoint": "P", "p": "P",
}


def _norm(name: str) -> str:
    return str(name).strip().lower().replace("_", "").replace(" ", "")


class YoloMarkerDetector:
    """对外只暴露 detect(frame) -> (marks, mask)。

    marks 里每一项：
        type  : 'L' / 'T' / 'X'
        u, v  : 特征点像素（框内细化后的交点）
        conf  : 置信度（0~1）
        box   : YOLO 框 (x1,y1,x2,y2)
        refined: True 表示点来自几何细化，False 表示退化成了框中心
    """

    def __init__(self, model_path, conf=0.25, iou=0.45, imgsz=640,
                 device="", params=None, verbose=True):
        from ultralytics import YOLO

        self.model = YOLO(model_path)
        self.names = {int(k): str(v) for k, v in self.model.names.items()}
        self.conf = float(conf)
        self.iou = float(iou)
        self.imgsz = int(imgsz)
        self.device = device or None
        self.p = dict(DEFAULT_PARAMS, **(params or {}))
        if verbose:
            print(f"[yolo] 权重 {model_path}")
            print(f"[yolo] 类别 {self.names}   conf={self.conf} iou={self.iou} imgsz={self.imgsz}")

    # ------------------------------------------------------------------ 检测
    def detect(self, frame):
        # 1) 一次白线掩膜 + 距离变换 + 全图中线直线段：
        #    细化交点、扣直线、拟合中圈都复用这批结果，避免重复计算
        mask = white_mask(frame, self.p)
        D = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
        segs = _line_segments(mask, self.p, D)

        # 2) YOLO 出框
        res = self.model.predict(frame, conf=self.conf, iou=self.iou,
                                 imgsz=self.imgsz, device=self.device, verbose=False)[0]
        marks = []
        if res.boxes is not None and len(res.boxes) > 0:
            xyxy = res.boxes.xyxy.detach().cpu().numpy()
            confs = res.boxes.conf.detach().cpu().numpy()
            clss = res.boxes.cls.detach().cpu().numpy()
            for box, cf, cl in zip(xyxy, confs, clss):
                t = CLASS_MAP.get(_norm(self.names.get(int(cl), "")))
                if t is None:
                    continue
                x1, y1, x2, y2 = (float(v) for v in box)
                if t == "P":                       # 4 类权重才有；点本身就在框中心
                    u, v, refined = 0.5 * (x1 + x2), 0.5 * (y1 + y2), False
                else:
                    u, v, refined = self._refine(segs, D, mask, (x1, y1, x2, y2))
                marks.append({"type": t, "u": float(u), "v": float(v),
                              "conf": float(cf), "arms": 4 if t == "X" else (3 if t == "T" else 2),
                              "box": (x1, y1, x2, y2), "refined": bool(refined)})

        # 3) P（场地中心）由调用方用两个 X 交点的地面中点导出
        return marks, mask

    # ------------------------------------------------------- 框内几何细化
    def _refine(self, segs, D, mask, box):
        """在框内找两条线的交点；找不到就退回框中心。"""
        x1, y1, x2, y2 = box
        cx, cy = 0.5 * (x1 + x2), 0.5 * (y1 + y2)
        w, h = x2 - x1, y2 - y1
        pad = max(8.0, 0.55 * max(w, h))

        # 先试全图直线段（已经算好了，零额外开销）
        q = _best_intersection(_segs_near(segs, x1 - pad, y1 - pad, x2 + pad, y2 + pad),
                               cx, cy, (x1, y1, x2, y2))
        if q is not None:
            return q[0], q[1], True

        # 框内线段不够时，在局部裁剪图里重跑一次霍夫（尺寸小，代价可忽略）
        H, W = mask.shape[:2]
        ex1, ey1 = int(max(0, x1 - pad)), int(max(0, y1 - pad))
        ex2, ey2 = int(min(W, x2 + pad)), int(min(H, y2 + pad))
        if ex2 - ex1 < 12 or ey2 - ey1 < 12:
            return cx, cy, False
        sub = mask[ey1:ey2, ex1:ex2]
        dsub = D[ey1:ey2, ex1:ex2]
        local = [(a + ex1, b + ey1, c + ex1, d + ey1)
                 for (a, b, c, d) in _line_segments(sub, self.p, dsub)]
        q = _best_intersection(local, cx, cy, (x1, y1, x2, y2))
        if q is not None:
            return q[0], q[1], True
        return cx, cy, False


def _segs_near(segs, x1, y1, x2, y2):
    """保留与给定矩形相交的线段（用 clipLine 判交，横贯画面的长线不会被误删）。"""
    rect = (int(round(x1)), int(round(y1)),
            max(1, int(round(x2 - x1))), max(1, int(round(y2 - y1))))
    out = []
    for s in segs:
        ok, _, _ = cv2.clipLine(rect, (int(s[0]), int(s[1])), (int(s[2]), int(s[3])))
        if ok:
            out.append(s)
    return out


def _best_intersection(segs, cx, cy, box, min_deg=20.0):
    """两两求交，取离框中心最近、且落在框内（允许 25% 外扩）的交点。
    两线夹角限制在 [min_deg, 180-min_deg]，避免近共线时数值爆炸。"""
    x1, y1, x2, y2 = box
    bw, bh = max(x2 - x1, 1.0), max(y2 - y1, 1.0)
    gx, gy = 0.25 * bw, 0.25 * bh
    best = None
    for i in range(len(segs)):
        for j in range(i + 1, len(segs)):
            s1, s2 = segs[i], segs[j]
            a1 = math.atan2(s1[3] - s1[1], s1[2] - s1[0])
            a2 = math.atan2(s2[3] - s2[1], s2[2] - s2[0])
            deg = abs(math.degrees((a1 - a2 + math.pi) % (2 * math.pi) - math.pi))
            if deg < min_deg or deg > 180.0 - min_deg:
                continue
            q = _seg_intersect(s1, s2)
            if q is None:
                continue
            qx, qy = q
            if not (x1 - gx <= qx <= x2 + gx and y1 - gy <= qy <= y2 + gy):
                continue
            d = math.hypot(qx - cx, qy - cy)
            if best is None or d < best[0]:
                best = (d, qx, qy)
    return None if best is None else (best[1], best[2])


# --------------------------------------------------------------------- 自测
if __name__ == "__main__":
    import argparse
    import os

    from ocv_field import CameraModel, GroundModel

    # 仓库根目录（zhengzeyang/）：默认 config 路径相对仓库根定位，不依赖运行时工作目录
    ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

    def ensure_weights(path):
        """权重文件存在性检查：缺失时给出明确的获取指引后退出，不进入算法流程。"""
        if not os.path.isfile(path):
            raise SystemExit(
                "[error] 权重未就位：" + path + "\n"
                "请从 README 中的百度网盘链接（提取码 27vp）下载 best2.pt，"
                "放到对应位置，或用 --model 指定已有权重路径。")

    ap = argparse.ArgumentParser(description="YOLO 标志检测 + 交点细化 自测")
    ap.add_argument("--image", required=True)
    ap.add_argument("--model", default=r"C:\Users\wen\Desktop\best2.pt")
    ap.add_argument("--config", default=os.path.join(ROOT, "calibration", "camera.yaml"))
    ap.add_argument("--out", default="out")
    ap.add_argument("--conf", type=float, default=0.25)
    a = ap.parse_args()

    ensure_weights(a.model)
    cam = CameraModel(a.config)
    gm = GroundModel(cam)
    det = YoloMarkerDetector(a.model, conf=a.conf)
    img = cv2.imread(a.image)
    if img is None:
        raise SystemExit("读不到图片 " + a.image)
    marks, mask = det.detect(img)

    colors = {"L": (0, 140, 255), "T": (255, 140, 0), "X": (200, 60, 140), "P": (60, 170, 60)}
    vis = img.copy()
    for m in marks:
        col = colors[m["type"]]
        if m.get("box"):
            x1, y1, x2, y2 = [int(v) for v in m["box"]]
            cv2.rectangle(vis, (x1, y1), (x2, y2), col, 2)
        u, v = int(m["u"]), int(m["v"])
        cv2.drawMarker(vis, (u, v), col, cv2.MARKER_CROSS, 16, 2)
        tag = "" if m.get("refined") else " (box-center)"
        cv2.putText(vis, f"{m['type']} {m['conf']:.2f}{tag}", (u + 10, v - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2, cv2.LINE_AA)
    os.makedirs(a.out, exist_ok=True)
    dst = os.path.join(a.out, "yolo_markers_" + os.path.basename(a.image))
    cv2.imwrite(dst, vis)
    print(f"检出 {len(marks)} 个：" + ", ".join(f"{m['type']}({m['u']:.0f},{m['v']:.0f})" for m in marks))
    print("结果图 ->", dst)
