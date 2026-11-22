#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
field_view.py —— 右侧"场地俯视图 + 相机位姿"面板

任务书要求的分屏 UI：
    左：RGB 检测叠加（bbox、类别、置信度、中心点 (u,v)、帧号/时间戳、FPS）
    右：field.yaml 俯视图 + 相机图标 + 朝向箭头 + 定位读数

这个文件只管右半边（以及把两半拼起来）。左半边在 localize.py 里画，
因为那里才拿得到原始帧和检测框。

坐标换算只做一次：map(米) → panel(像素)，
    px = (x - x_min + pad) * scale
    py = (y_max + pad - y) * scale     # y 轴翻转，符合"俯视图上方是 +Y"
"""

import math

import cv2
import numpy as np

# 配色（BGR）：四个类别用四个容易区分的颜色
CLASS_COLORS = {
    0: (255, 170, 60),     # LCross  橙
    1: (80, 220, 80),      # TCross  绿
    2: (60, 200, 255),     # XCross  黄
    3: (90, 90, 255),      # Penalty 红
    4: (255, 255, 255),    # Ball    白
}

STATUS_COLORS = {
    "ok": (80, 220, 80),
    "ambiguous": (60, 200, 255),
    "insufficient_markers": (255, 170, 60),
    "high_residual": (255, 170, 60),
    "lost": (90, 90, 255),
}


class FieldView(object):
    """把 map 坐标画成俯视图面板。初始化时算好缩放，之后每帧只画。"""

    def __init__(self, field_map, geometry=None, width=640, height=720, pad_m=0.6):
        self.field = field_map
        self.geom = geometry or {}
        self.W = int(width)
        self.H = int(height)
        self.pad_m = float(pad_m)

        x0, y0, x1, y1 = field_map.bounds(margin=0.0)
        self.x_min, self.x_max = x0 - self.pad_m, x1 + self.pad_m
        self.y_min, self.y_max = y0 - self.pad_m, y1 + self.pad_m
        self.span_x = self.x_max - self.x_min
        self.span_y = self.y_max - self.y_min
        # 留出顶部标题区和底部状态条
        self.margin_top = 54
        self.margin_bottom = 96
        usable_h = self.H - self.margin_top - self.margin_bottom
        self.scale = min(self.W / self.span_x, usable_h / self.span_y)
        self.off_x = (self.W - self.span_x * self.scale) / 2.0
        self.off_y = self.margin_top + (usable_h - self.span_y * self.scale) / 2.0
        self.trail = []

    # ---- map → 像素 ----
    def to_px(self, x, y):
        px = self.off_x + (x - self.x_min) * self.scale
        py = self.off_y + (self.y_max - y) * self.scale
        if not (math.isfinite(px) and math.isfinite(py)):
            return None
        return int(round(px)), int(round(py))

    def to_map(self, px, py):
        x = self.x_min + (px - self.off_x) / self.scale
        y = self.y_max - (py - self.off_y) / self.scale
        return x, y

    # ---- 场地底图 ----
    def _draw_field(self, img):
        g = (self.geom or {}).get("field", {}) if isinstance(self.geom, dict) else {}
        lines = (g.get("lines") or {}) if isinstance(g, dict) else {}

        # 地图范围框（一定画）：这是 field.yaml 定义的有效区域，不是"白线场地"
        x0, y0, x1, y1 = self.field.bounds(margin=0.0)
        p1, p2 = self.to_px(x0, y1), self.to_px(x1, y0)
        cv2.rectangle(img, p1, p2, (95, 95, 95), 1)
        cv2.putText(img, "map frame", (p1[0] + 4, p1[1] + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.34, (120, 120, 120), 1, cv2.LINE_AA)

        # 白线场地（只有真正实测过才画，避免画出一个假尺寸的球场）
        if lines.get("available"):
            xl = float(lines["x_left"])
            xr = float(lines["x_right"])
            yb = float(lines["y_bottom"])
            yt = float(lines["y_top"])
            a = self.to_px(xl, yt)
            b = self.to_px(xr, yb)
            cv2.rectangle(img, a, b, (60, 110, 70), -1)          # 草皮
            cv2.rectangle(img, a, b, (215, 215, 215), 2)         # 边线
            x_mid = 0.5 * (xl + xr)
            y_mid = 0.5 * (yb + yt)
            cv2.line(img, self.to_px(xl, y_mid), self.to_px(xr, y_mid), (190, 190, 190), 1)

            cc = g.get("center_circle") or {}
            if cc.get("available"):
                cx = float(cc["x"]) if "x" in cc else x_mid
                cy = float(cc["y"]) if "y" in cc else y_mid
                cv2.circle(img, self.to_px(cx, cy), int(float(cc["radius_m"]) * self.scale),
                           (190, 190, 190), 1)

            for key in ("penalty_area", "goal_area"):
                box = g.get(key) or {}
                if not box.get("available"):
                    continue
                depth = float(box.get("depth_m", 0.0))
                width = float(box.get("width_m", 0.0))
                if depth <= 0 or width <= 0:
                    continue
                for sign, xline in ((-1, xl), (+1, xr)):
                    xa, xb2 = xline, xline + sign * depth
                    ya, yb2 = y_mid - width / 2.0, y_mid + width / 2.0
                    cv2.rectangle(img, self.to_px(xa, ya), self.to_px(xb2, yb2),
                                  (180, 180, 180), 1)

        # 坐标轴提示（贴在地图区左下角，避开底部读数区与顶部标题）
        ay = int(self.H - self.margin_bottom - 24)
        cv2.arrowedLine(img, (18, ay), (58, ay), (200, 200, 200), 1, tipLength=0.3)
        cv2.putText(img, "+X", (62, ay + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    (200, 200, 200), 1, cv2.LINE_AA)
        cv2.arrowedLine(img, (18, ay), (18, ay - 40), (200, 200, 200), 1, tipLength=0.3)
        cv2.putText(img, "+Y", (22, ay - 44), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    (200, 200, 200), 1, cv2.LINE_AA)

    def _draw_markers(self, img, inlier_names=None):
        inlier_names = set(inlier_names or [])
        for name, (x, y) in zip(self.field.names(), self.field.points):
            cid = int(self.field.class_id_of(name))
            col = CLASS_COLORS.get(cid, (255, 255, 255))
            p = self.to_px(x, y)
            if p is None:
                continue
            used = name in inlier_names
            r = 7 if used else 4
            cv2.drawMarker(img, p, col, cv2.MARKER_CROSS, r * 2, 2 if used else 1)
            if used:
                cv2.circle(img, p, r + 4, col, 1)
                cv2.putText(img, name, (p[0] + 8, p[1] - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.32, col, 1, cv2.LINE_AA)

    # ---- 主绘制 ----
    def render(self, pose=None, observations=None, status="lost", fps=0.0,
               frame_id=0, stamp="", extra_text=None):
        """
        pose        : dict(valid,x,y,yaw_deg,residual,marker_count,status,inliers) 或 None
        observations: [(cls_id, p_map_x, p_map_y, inlier_bool), ...] 观测点投影到 map 后的坐标
        """
        img = np.full((self.H, self.W, 3), 26, dtype=np.uint8)
        self._draw_field(img)

        inlier_names = pose.get("inliers") if pose else None
        self._draw_markers(img, inlier_names)

        # 观测点（把这一帧看到的地面点用 map 坐标画出来，和地图点连线 = 残差可视化）
        if observations:
            for obs in observations:
                cls_id, x, y = obs[0], obs[1], obs[2]
                ok = bool(obs[3]) if len(obs) > 3 else False
                col = CLASS_COLORS.get(int(cls_id), (255, 255, 255))
                p = self.to_px(x, y)
                if p is None:
                    continue
                if 0 <= p[0] < self.W and 0 <= p[1] < self.H:
                    cv2.circle(img, p, 5 if ok else 3, col, 2 if ok else 1)

        # 位姿
        col_status = STATUS_COLORS.get(status, (200, 200, 200))
        if pose and pose.get("valid") and self.to_px(pose["x"], pose["y"]) is not None:
            cx, cy = self.to_px(pose["x"], pose["y"])
            yaw = math.radians(pose.get("yaw_deg", 0.0))
            # 视场扇形（半透明，示意 ±35°，帮助一眼看出朝向）
            wedge = np.array([[cx, cy],
                              [cx + 62 * math.cos(yaw - math.radians(35)),
                               cy - 62 * math.sin(yaw - math.radians(35))],
                              [cx + 62 * math.cos(yaw + math.radians(35)),
                               cy - 62 * math.sin(yaw + math.radians(35))]], np.int32)
            overlay = img.copy()
            cv2.fillConvexPoly(overlay, wedge, (70, 120, 150))
            cv2.addWeighted(overlay, 0.35, img, 0.65, 0, img)
            cv2.polylines(img, [wedge], True, (90, 160, 200), 1, cv2.LINE_AA)
            # 轨迹
            if self.trail is not None:
                self.trail.append((cx, cy))
                if len(self.trail) > 120:
                    self.trail = self.trail[-120:]
            for a, b in zip(self.trail[:-1], self.trail[1:]):
                cv2.line(img, a, b, (110, 110, 60), 1)
            # 朝向箭头 + 相机本体
            L = 40
            tip = (int(round(cx + L * math.cos(yaw))), int(round(cy - L * math.sin(yaw))))
            cv2.arrowedLine(img, (cx, cy), tip, (60, 200, 255), 2, tipLength=0.3, line_type=cv2.LINE_AA)
            cv2.circle(img, (cx, cy), 6, (60, 200, 255), -1, cv2.LINE_AA)
            cv2.circle(img, (cx, cy), 6, (20, 20, 20), 1, cv2.LINE_AA)
            cv2.putText(img, "camera", (cx - 20, cy + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.34, (60, 200, 255), 1, cv2.LINE_AA)

        # 顶部标题
        cv2.putText(img, "field.yaml  top view", (14, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, (230, 230, 230), 1, cv2.LINE_AA)
        cv2.putText(img, "%s  (n=%d)" % (self.field.shorthash, len(self.field.markers)),
                    (14, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (150, 150, 150), 1, cv2.LINE_AA)

        # 底部读数
        y0 = self.H - self.margin_bottom + 6
        def put(line, dy, color=(225, 225, 225), scale=0.45):
            cv2.putText(img, line, (14, y0 + dy), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)

        if pose:
            if pose.get("valid"):
                put("x_camera_map=%7.3f m   y_camera_map=%7.3f m" %
                    (pose.get("x", 0.0), pose.get("y", 0.0)), 16, (255, 255, 255), 0.46)
                put("yaw_camera_map=%7.2f deg" % pose.get("yaw_deg", 0.0),
                    33, (255, 255, 255), 0.46)
            else:
                put("x_camera_map=  ---    y_camera_map=  ---", 16, (150, 150, 150), 0.46)
                put("yaw_camera_map=  ---", 33, (150, 150, 150), 0.46)
            put("valid=%s   marker_count=%d   residual=%.3f m" %
                (pose.get("valid"), pose.get("marker_count", 0), pose.get("residual", float("nan"))), 50)
            _why = pose.get("reason") or ""
            put("status=%s%s   fps=%.1f   update=%.1f Hz   %.1f ms" %
                (pose.get("status", "?"), (" (%s)" % _why) if _why else "", fps,
                 pose.get("update_hz", 0.0),
                 pose.get("processing_ms", 0.0)), 67, col_status, 0.46)
        else:
            put("x_camera_map=  ---    y_camera_map=  ---", 16, (150, 150, 150), 0.46)
            put("yaw_camera_map=  ---", 33, (150, 150, 150), 0.46)
            put("valid=False  marker_count=0  residual=---", 50, (150, 150, 150))
            put("status=%s   fps=%.1f" % (status, fps), 67, col_status, 0.46)

        put("frame=%d  %s" % (frame_id, stamp), 84, (170, 170, 170), 0.4)

        if extra_text:
            for i, line in enumerate(extra_text):
                cv2.putText(img, line, (self.W - 250, 26 + 18 * i),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 120), 1, cv2.LINE_AA)
        return img


# ---------------------------------------------------------------------------
def make_split_view(frame_bgr, right_panel, total_w=1280, total_h=720):
    """
    左：检测画面（已经由 localize.draw_left_panel 合成好，尺寸为 total_h 高）；
    右：俯视图面板。两半等宽。
    如果传进来的是原始帧（高宽比不是 tid），会自动等比缩放。
    """
    half_w = total_w // 2
    h, w = frame_bgr.shape[:2]
    if abs(h - total_h) < 2 and abs(w - half_w) < 2:
        left = frame_bgr
    else:
        r = min(half_w / float(w), total_h / float(h))
        nw, nh = int(round(w * r)), int(round(h * r))
        left = cv2.resize(frame_bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((total_h, total_w, 3), 18, dtype=np.uint8)
    if left.shape[0] == total_h and left.shape[1] == half_w:
        canvas[:, :half_w] = left
    else:
        oy = (total_h - left.shape[0]) // 2
        ox = (half_w - left.shape[1]) // 2
        canvas[oy:oy + left.shape[0], ox:ox + left.shape[1]] = left
    rp = cv2.resize(right_panel, (total_w - half_w, total_h), interpolation=cv2.INTER_LINEAR)
    canvas[:, half_w:] = rp
    cv2.line(canvas, (half_w, 0), (half_w, total_h), (70, 70, 70), 1)
    return canvas
