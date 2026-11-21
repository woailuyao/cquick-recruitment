# -*- coding: utf-8 -*-
"""
main_live.py —— 实时单目场地自定位（纯 OpenCV + YOLO 标志检测）
左屏：RGB 实时画面 + 白线掩膜 + L/T/X/P 检测 + 像素特征点
右屏：场地俯视图 + 相机光心 + 朝向箭头 + 视野扇形
输出：x_camera_map / y_camera_map / yaw_camera_map / marker_count / residual / valid / status

运行:
    python src/vision/main_live.py --cam 0
    python src/vision/main_live.py --image test.jpg
    python src/vision/main_live.py --video clip.mp4
按键: q/ESC 退出  s 截图  d 掩膜叠加  r 复位滤波  m 切换布局
"""
import argparse, os, time, math, collections
import cv2, numpy as np
from ocv_field import CameraModel, GroundModel
from yolo_marker import YoloMarkerDetector
from ocv_match import load_field, load_field_meta, solve_pose, field_bounds

# 仓库根目录（zhengzeyang/）：默认 config 路径相对仓库根定位，不依赖运行时工作目录
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

FONT = cv2.FONT_HERSHEY_SIMPLEX


def ensure_weights(path):
    """权重文件存在性检查：缺失时给出明确的获取指引后退出，不进入算法流程。"""
    if not os.path.isfile(path):
        raise SystemExit(
            "[error] 权重未就位：" + path + "\n"
            "请从 README 中的百度网盘链接（提取码 27vp）下载 best2.pt，"
            "放到对应位置，或用 --model 指定已有权重路径。")


def derive_center_from_x(marks, cam, circle_r=0.75, min_sep=0.4, max_sep=6.0):
    """点球点（场地中心）= 两个 X 交点（中线∩中圈）的地面中点，兜底路径。

    返回 None，或 dict(a, b, u, v, sep)：
      a, b : 圆心在相机局部地面系下的坐标 (m)
      u, v : 两个 X 交点的像素中点
    """
    xs = [m for m in marks if m["type"] == "X"]
    if len(xs) < 2:
        return None
    pts = []
    for m in xs:
        a, b, ok, _ = cam.pixel_to_local_ground(m["u"], m["v"])
        if not ok:
            continue
        pts.append((a, b, m))
    if len(pts) < 2:
        return None
    best = None
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            a1, b1, m1 = pts[i]
            a2, b2, m2 = pts[j]
            sep = math.hypot(a2 - a1, b2 - b1)
            if not (min_sep <= sep <= max_sep):
                continue
            # 多个 X 候选时取间距最接近圆直径(2r) 的一对
            err = abs(sep - 2.0 * circle_r)
            if best is None or err < best[0]:
                best = (err, a1, b1, a2, b2, m1, m2, sep)
    if best is None:
        return None
    _, a1, b1, a2, b2, m1, m2, sep = best
    return {"a": 0.5 * (a1 + a2), "b": 0.5 * (b1 + b2),
            "u": 0.5 * (m1["u"] + m2["u"]), "v": 0.5 * (m1["v"] + m2["v"]),
            "sep": sep}


class Rate:
    """用到达时刻反算真实帧率"""
    def __init__(self, n=30):
        self.t = collections.deque(maxlen=n)
    def tick(self):
        self.t.append(time.perf_counter())
    def value(self):
        if len(self.t) < 2:
            return 0.0
        span = self.t[-1] - self.t[0]
        return (len(self.t) - 1) / span if span > 1e-9 else 0.0


class PoseFilter:
    """指数滤波：只在定位有效时更新；失效超过 5 帧就彻底丢弃，绝不伪造位姿"""
    def __init__(self, alpha=0.45):
        self.a = alpha
        self.pose = None
    def reset(self):
        self.pose = None
    def update(self, x, y, yaw):
        if self.pose is None:
            self.pose = [x, y, yaw]
            return self.pose
        dy = (yaw - self.pose[2] + 180.0) % 360.0 - 180.0
        self.pose[0] += self.a * (x - self.pose[0])
        self.pose[1] += self.a * (y - self.pose[1])
        self.pose[2] = (self.pose[2] + self.a * dy) % 360.0
        return self.pose


class MapView:
    """右栏 = 场地俯视图（主区） + 侧边信息卡（位姿/状态/匹配清单），按最终显示像素尺寸渲染。"""
    COLORS = {"L": (185, 95, 30), "T": (30, 140, 200), "X": (200, 60, 140), "P": (60, 170, 60)}
    BG = (250, 250, 252)
    FIELD_FILL = (216, 236, 216)
    INFO_BG = (244, 244, 248)

    def __init__(self, field, w=960, h=540, fov_deg=40.0, orient="v", info_w=None):
        self.w, self.h = int(w), int(h)
        self.field = field
        self.fov = math.radians(fov_deg)
        self.orient = orient                       # "v": +X 向上(球门在上)；"h": +X 向右
        self.xmin, self.xmax, self.ymin, self.ymax = field_bounds(field, 0.8)
        self.mx = (self.xmin + self.xmax) / 2.0
        self.my = (self.ymin + self.ymax) / 2.0
        self.k = max(0.55, self.h / 540.0)           # 字号/图标缩放：以 540px 高为基准

        span_x = max(self.xmax - self.xmin, 1e-6)    # 地图 X 跨度（长边 10.55 m）
        span_y = max(self.ymax - self.ymin, 1e-6)    # 地图 Y 跨度（6.94+1.6 ≈ 8.54 m）
        self.h_span, self.v_span = (span_y, span_x) if orient == "v" else (span_x, span_y)

        # 侧边信息卡宽度
        if info_w is None:
            info_w = int(min(max(0.28 * self.w, 200), 300)) if self.w > 520 else 0
        self.info_w = int(info_w)
        self.map_area_w = self.w - self.info_w

        self.title_h = int(44 * self.k)
        self.bot_h = int(46 * self.k)
        pad = int(16 * self.k)
        avail_w = self.map_area_w - 2 * pad
        avail_h = self.h - self.title_h - self.bot_h - 2 * pad
        self.scale = min(avail_w / self.h_span, avail_h / self.v_span)

        self.map_w = self.h_span * self.scale
        self.map_h = self.v_span * self.scale
        self.origin_x = self.map_area_w / 2.0        # 地图区中心
        self.origin_y = self.title_h + pad + self.map_h / 2.0
        self.base = self._render()

    # ---------- 坐标变换 ----------
    def to_px(self, x, y):
        """场地坐标(m) -> 面板像素"""
        if self.orient == "v":
            # 屏幕横 = -Y(地图 +Y 向左), 屏幕纵 = -X(地图 +X 向上)
            return (self.origin_x - (y - self.my) * self.scale,
                    self.origin_y + (self.mx - x) * self.scale)
        # 横置：屏幕横 = +X, 屏幕纵 = -Y
        return (self.origin_x + (x - self.mx) * self.scale,
                self.origin_y - (y - self.my) * self.scale)

    def yaw_vec(self, ang, L):
        """yaw 方向(地图系，逆时针为正) -> 面板像素偏移"""
        if self.orient == "v":
            return (L * math.sin(ang), -L * math.cos(ang))
        return (L * math.cos(ang), -L * math.sin(ang))

    # ---------- 底图 ----------
    def _render(self):
        k = self.k
        img = np.full((self.h, self.w, 3), self.BG, np.uint8)

        def P(x, y):
            px, py = self.to_px(x, y)
            return (int(px), int(py))

        corners = np.array([P(self.xmin, self.ymin), P(self.xmin, self.ymax),
                            P(self.xmax, self.ymax), P(self.xmax, self.ymin)], np.int32)
        cv2.fillPoly(img, [corners], self.FIELD_FILL)
        cv2.polylines(img, [corners], True, (150, 150, 150), max(1, int(2 * k)))

        for gx in range(int(math.floor(self.xmin)), int(math.ceil(self.xmax)) + 1):
            cv2.line(img, P(gx, self.ymin), P(gx, self.ymax), (198, 218, 198), 1)
        for gy in range(int(math.floor(self.ymin)), int(math.ceil(self.ymax)) + 1):
            cv2.line(img, P(self.xmin, gy), P(self.xmax, gy), (198, 218, 198), 1)

        # 球门（+X 端）
        g0 = P(self.xmax, self.my + 0.6)
        g1 = P(self.xmax, self.my - 0.6)
        cv2.rectangle(img, (g0[0] - int(8 * k), g0[1] - int(6 * k)),
                      (g1[0] + int(8 * k), g1[1] + int(6 * k)), (90, 90, 90), max(1, int(2 * k)))
        glx, gly = g0[0] + int(12 * k), g0[1]
        (gtw, _), _ = cv2.getTextSize("goal", FONT, 0.42 * k, 1)
        if glx + gtw > self.map_area_w - int(4 * k):
            glx = g0[0] - int(14 * k) - gtw
        cv2.putText(img, "goal", (glx, gly), FONT, 0.42 * k, (90, 90, 90), 1, cv2.LINE_AA)

        # 原点 O 与坐标轴
        o = P(0.0, 0.0)
        cv2.circle(img, o, max(3, int(5 * k)), (0, 0, 0), -1)
        (otw, _), _ = cv2.getTextSize("O(0,0)", FONT, 0.45 * k, 1)
        olx = o[0] - otw - int(8 * k)
        if olx < int(4 * k):
            olx = o[0] + int(8 * k)
        cv2.putText(img, "O(0,0)", (olx, o[1] + int(20 * k)),
                    FONT, 0.45 * k, (30, 30, 30), 1, cv2.LINE_AA)
        ax = int(66 * k)
        if self.orient == "v":
            cv2.arrowedLine(img, o, (o[0], o[1] - ax), (0, 0, 220), max(1, int(2 * k)), cv2.LINE_AA)
            cv2.arrowedLine(img, o, (o[0] - ax, o[1]), (0, 120, 255), max(1, int(2 * k)), cv2.LINE_AA)
            cv2.putText(img, "+X", (o[0] - int(24 * k), o[1] - ax - int(6 * k)),
                        FONT, 0.48 * k, (0, 0, 220), 1, cv2.LINE_AA)
            cv2.putText(img, "+Y", (o[0] - ax - int(14 * k), o[1] + int(5 * k)),
                        FONT, 0.48 * k, (0, 120, 255), 1, cv2.LINE_AA)
        else:
            cv2.arrowedLine(img, o, (o[0] + ax, o[1]), (0, 0, 220), max(1, int(2 * k)), cv2.LINE_AA)
            cv2.arrowedLine(img, o, (o[0], o[1] - ax), (0, 120, 255), max(1, int(2 * k)), cv2.LINE_AA)
            cv2.putText(img, "+X", (o[0] + ax - int(18 * k), o[1] - int(8 * k)),
                        FONT, 0.48 * k, (0, 0, 220), 1, cv2.LINE_AA)
            cv2.putText(img, "+Y", (o[0] + int(8 * k), o[1] - ax - int(4 * k)),
                        FONT, 0.48 * k, (0, 120, 255), 1, cv2.LINE_AA)

        # 地标（标签靠近边界时翻到另一侧，避免被面板裁掉）
        for t, lst in self.field.items():
            for x, y, name in lst:
                q = P(x, y)
                cv2.circle(img, q, max(3, int(5 * k)), self.COLORS[t], -1)
                lx, ly = q[0] + int(8 * k), q[1] - int(6 * k)
                if abs(x) < 0.05 and abs(y) < 0.05:
                    continue          # 与原点重合，原点已标 O(0,0)，不再重复标注
                (tw, _), _ = cv2.getTextSize(name, FONT, 0.38 * k, 1)
                if lx + tw > self.map_area_w - int(16 * k):
                    lx = q[0] - int(8 * k) - tw
                if lx < int(4 * k):
                    lx = q[0] + int(8 * k)
                if ly < self.title_h + int(8 * k):
                    ly = q[1] + int(16 * k)
                if ly > self.h - self.bot_h - int(4 * k):
                    ly = q[1] - int(6 * k)
                cv2.putText(img, name, (lx, ly), FONT, 0.38 * k, self.COLORS[t], 1, cv2.LINE_AA)

        # 标题栏
        hint = ("+X up (to goal)   +Y left   yaw CCW from +X" if self.orient == "v"
                else "+X right   +Y up   yaw CCW from +X")
        cv2.rectangle(img, (0, 0), (self.map_area_w, self.title_h), (255, 255, 255), -1)
        cv2.putText(img, "field map  (top view)", (int(12 * k), int(24 * k)),
                    FONT, 0.5 * k, (40, 40, 40), 1, cv2.LINE_AA)
        cv2.putText(img, hint, (int(12 * k), int(40 * k)), FONT, 0.4 * k, (95, 95, 95), 1, cv2.LINE_AA)
        return img

    # ---------- 每帧叠加：位姿 + 视野扇形 + 信息卡 ----------
    def draw_pose(self, x, y, yaw_deg, valid, residual=0.0, matched=None):
        img = self.base.copy()
        k = self.k
        ang = math.radians(yaw_deg)

        # 视野扇形（只在地图区内绘制，避免溢出到信息卡）
        R = 1.9 * self.scale
        half = self.fov / 2.0
        p0 = self.to_px(x, y)
        p1 = self.to_px(x + R * math.cos(ang - half), y + R * math.sin(ang - half))
        p2 = self.to_px(x + R * math.cos(ang + half), y + R * math.sin(ang + half))
        wedge = np.array([[int(p0[0]), int(p0[1]) - self.title_h],
                          [int(p1[0]), int(p1[1]) - self.title_h],
                          [int(p2[0]), int(p2[1]) - self.title_h]], np.int32)
        roi = img[self.title_h:self.h - self.bot_h, 0:self.map_area_w]
        sub = np.zeros_like(roi)
        cv2.fillPoly(sub, [wedge], (196, 232, 196) if valid else (228, 228, 228))
        img[self.title_h:self.h - self.bot_h, 0:self.map_area_w] = cv2.addWeighted(roi, 0.82, sub, 0.18, 0)

        q = (int(p0[0]), int(p0[1]))
        col = (60, 170, 60) if valid else (130, 130, 130)
        L = int(72 * k)
        dx, dy = self.yaw_vec(ang, L)
        tip = (int(q[0] + dx), int(q[1] + dy))
        cv2.arrowedLine(img, q, tip, col, max(2, int(3 * k)), cv2.LINE_AA)
        s = int(11 * k)
        tri = np.array([[q[0] + dx / L * s, q[1] + dy / L * s],
                        [q[0] + dx / L * s - dy / L * s * 0.5 - dx / L * s * 0.5,
                         q[1] + dy / L * s + dx / L * s * 0.5 - dy / L * s * 0.5],
                        [q[0] + dx / L * s + dy / L * s * 0.5 - dx / L * s * 0.5,
                         q[1] + dy / L * s - dx / L * s * 0.5 - dy / L * s * 0.5]], np.int32)
        cv2.fillPoly(img, [tri], col)
        cv2.circle(img, q, max(2, int(4 * k)), col, -1)

        # 底部信息条（地图区）
        cv2.rectangle(img, (0, self.h - self.bot_h), (self.map_area_w, self.h), (255, 255, 255), -1)
        cv2.putText(img, f"camera ({x:.2f}, {y:.2f})", (int(12 * k), self.h - int(28 * k)),
                    FONT, 0.5 * k, col, 1, cv2.LINE_AA)
        cv2.putText(img, f"yaw {yaw_deg:.1f} deg    residual {residual:.3f} m",
                    (int(12 * k), self.h - int(10 * k)), FONT, 0.44 * k, (60, 60, 60), 1, cv2.LINE_AA)

        if self.info_w > 0:
            self._draw_info(img, x, y, yaw_deg, valid, residual, matched, ang)
        return img

    def _draw_info(self, img, x, y, yaw_deg, valid, residual, matched, ang):
        """右侧信息卡：位姿大字 + 罗盘 + 状态 + 匹配清单"""
        k = self.k
        x0 = self.map_area_w
        cv2.rectangle(img, (x0, 0), (self.w, self.h), self.INFO_BG, -1)
        cv2.line(img, (x0, 0), (x0, self.h), (205, 205, 212), max(1, int(1 * k)))
        pad = int(12 * k)
        y = int(26 * k)
        cv2.putText(img, "POSE", (x0 + pad, y), FONT, 0.5 * k, (110, 110, 120), 1, cv2.LINE_AA)
        y += int(24 * k)

        def kv(label, value, unit="", big=True, vcol=None):
            nonlocal y
            cv2.putText(img, label, (x0 + pad, y), FONT, 0.4 * k, (130, 130, 140), 1, cv2.LINE_AA)
            cv2.putText(img, value, (x0 + pad, y + int(22 * k)), FONT,
                        (0.62 if big else 0.48) * k, vcol or (30, 30, 35), 1, cv2.LINE_AA)
            if unit:
                cv2.putText(img, unit, (x0 + pad + int(8 * k), y + int(40 * k)),
                            FONT, 0.38 * k, (130, 130, 140), 1, cv2.LINE_AA)
            y += int((56 if big else 44) * k)

        kv("x_camera_map", f"{x:.3f}", "m")
        kv("y_camera_map", f"{y:.3f}", "m")

        # yaw + 罗盘
        cv2.putText(img, "yaw_camera_map", (x0 + pad, y), FONT, 0.4 * k, (130, 130, 140), 1, cv2.LINE_AA)
        cv2.putText(img, f"{yaw_deg:.1f}", (x0 + pad, y + int(24 * k)), FONT, 0.62 * k,
                    (30, 30, 35), 1, cv2.LINE_AA)
        cv2.putText(img, "deg", (x0 + pad + int(52 * k), y + int(24 * k)),
                    FONT, 0.38 * k, (130, 130, 140), 1, cv2.LINE_AA)
        # 罗盘
        R = int(26 * k)
        ccx = self.w - R - int(16 * k)
        ccy = y + int(12 * k)
        cv2.circle(img, (ccx, ccy), R, (170, 170, 178), max(1, int(1 * k)))
        cv2.circle(img, (ccx, ccy), max(2, int(3 * k)), (120, 120, 128), -1)
        cv2.putText(img, "+X", (ccx - int(8 * k), ccy - R - int(4 * k)), FONT, 0.36 * k,
                    (120, 120, 128), 1, cv2.LINE_AA)
        dx, dy = self.yaw_vec(ang, R - 3)
        cv2.arrowedLine(img, (ccx, ccy), (int(ccx + dx), int(ccy + dy)),
                        (60, 170, 60) if valid else (140, 140, 148), max(2, int(2 * k)), cv2.LINE_AA)
        y += int(46 * k)

        cv2.line(img, (x0 + pad, y - int(8 * k)), (self.w - pad, y - int(8 * k)),
                 (210, 210, 218), 1)
        y += int(8 * k)
        cv2.putText(img, "STATE", (x0 + pad, y), FONT, 0.5 * k, (110, 110, 120), 1, cv2.LINE_AA)
        y += int(20 * k)
        cv2.putText(img, f"residual  {residual:.3f} m", (x0 + pad, y), FONT, 0.42 * k,
                    (60, 60, 66), 1, cv2.LINE_AA)
        y += int(18 * k)
        cv2.putText(img, f"matched   {len(matched) if matched else 0}",
                    (x0 + pad, y), FONT, 0.42 * k, (60, 60, 66), 1, cv2.LINE_AA)
        y += int(24 * k)

        # 匹配到的标志清单
        cv2.putText(img, "MATCHED MARKERS", (x0 + pad, y), FONT, 0.42 * k,
                    (140, 140, 150), 1, cv2.LINE_AA)
        y += int(18 * k)
        for name, dist in (matched or [])[:6]:
            cv2.putText(img, f"{name}", (x0 + pad, y), FONT, 0.4 * k, (70, 70, 78), 1, cv2.LINE_AA)
            cv2.putText(img, f"{dist:.2f}m", (self.w - pad - int(44 * k), y),
                        FONT, 0.38 * k, (140, 140, 150), 1, cv2.LINE_AA)
            y += int(16 * k)


def draw_markers(img, marks, obs_map, sx=1.0, sy=1.0):
    """在画面上标注检测到的标志。sx/sy = 画面被缩放的比例（像素坐标换算）"""
    for mk in marks:
        u, v = int(mk["u"] * sx), int(mk["v"] * sy)
        col = {"L": (0, 140, 255), "T": (255, 140, 0), "X": (200, 60, 140),
               "P": (60, 170, 60)}.get(mk["type"], (200, 200, 200))
        bx = mk.get("box")                       # YOLO 检测框；P 来自几何反算，没有框
        if bx is not None:
            x1, y1, x2, y2 = (int(q * sx) for q in (bx[0], bx[1], bx[2], bx[3]))
            cv2.rectangle(img, (x1, y1), (x2, y2), col, 1)
        cv2.circle(img, (u, v), 7, col, 2)
        cv2.drawMarker(img, (u, v), col, cv2.MARKER_CROSS, 14, 2)
        txt = f"{mk['type']} {mk['conf']:.2f}"
        if bx is not None and not mk.get("refined", True):
            txt += " ctr"                          # 细化失败、退回框中心的标记
        rm = mk.get("r_m")
        if isinstance(rm, float) and rm == rm:      # P 额外显示中圈半径/残差，便于判断可信度
            txt += f"  r{rm:.2f}m {mk.get('cov_deg', 0):.0f}deg"
        cv2.putText(img, txt, (u + 10, v - 8), FONT, 0.5, col, 2, cv2.LINE_AA)
        gid = obs_map.get(id(mk), "")
        if gid:
            cv2.putText(img, gid, (u + 10, v + 10), FONT, 0.42, col, 1, cv2.LINE_AA)
    return img


def draw_hud(img, st):
    """左上角精简状态条：位姿数值在右栏信息卡里已完整给出，这里只留运行状态，避免遮挡画面"""
    H, W = img.shape[:2]
    k = max(0.62, W / 1280.0)
    pw = int(min(270 * k, W * 0.34))
    x0 = W - pw
    ov = img.copy()
    cv2.rectangle(ov, (x0, 0), (W, int(112 * k)), (18, 18, 18), -1)
    cv2.addWeighted(ov, 0.75, img, 0.25, 0, img)
    y = int(22 * k)

    def line(txt, col=(235, 235, 235), sc=0.45):
        nonlocal y
        cv2.putText(img, txt, (x0 + int(10 * k), y), FONT, sc * k, col, 1, cv2.LINE_AA)
        y += int(19 * k)

    line("FIELD LOCALIZATION", (255, 255, 255), 0.46)
    line(f"FPS {st['fps']:5.1f}   {st['ms']:5.1f} ms   #{st['idx']}", (120, 255, 120), 0.42)
    line(f"raw {st['mk_n']}  proj {st['obs_n']}  match {st['cnt']}", (150, 220, 255), 0.42)
    sc = {"ok": (120, 255, 120)}.get(st["status"], (60, 200, 250))
    line(f"{st['status']}   valid={str(st['valid'])}", sc, 0.44)
    if st.get("sym"):
        line("SYMMETRY 180deg (by continuity)", (255, 170, 60), 0.40)
    cv2.putText(img, "q quit  s snap  d mask  r reset  m layout",
                (x0 + int(10 * k), H - int(12 * k)), FONT, 0.36 * k, (140, 140, 140), 1, cv2.LINE_AA)
    return img


def main():
    ap = argparse.ArgumentParser(description="纯 OpenCV 单目场地自定位")
    ap.add_argument("--cam", default="0")
    ap.add_argument("--video", default="")
    ap.add_argument("--image", default="")
    ap.add_argument("--config", default=os.path.join(ROOT, "calibration", "camera.yaml"))
    ap.add_argument("--field", default=os.path.join(ROOT, "config", "field.yaml"))
    ap.add_argument("--out", default="out")
    ap.add_argument("--alpha", type=float, default=0.45, help="位姿滤波系数")
    ap.add_argument("--inlier-thr", type=float, default=0.30)
    ap.add_argument("--min-inliers", type=int, default=3)
    ap.add_argument("--max-residual", type=float, default=0.25)
    ap.add_argument("--layout", choices=["split", "minimap"], default="split",
                    help="split=左右分屏(地图占比大)；minimap=地图叠加在画面左下角(画面全幅)")
    ap.add_argument("--width", type=int, default=1920, help="split 模式显示总宽(px)")
    ap.add_argument("--map-ratio", type=float, default=0.5, help="split 模式下右栏地图占总宽比例")
    ap.add_argument("--init", default="",
                    help="初始位姿 'x,y,yaw'(米,度)。实测地图左右对称，"
                         "首帧无法判断朝向，用它定方向后靠时序连续性保持")
    ap.add_argument("--map-orient", choices=["v", "h"], default="h",
                    help="h=场地横放(+X 向右，宽面板利用率最高，地图更大)；"
                         "v=场地竖放(+X 指向球门，与示意图一致)")
    ap.add_argument("--model", default=r"C:\Users\wen\Desktop\best2.pt", help="YOLO 权重路径")
    ap.add_argument("--conf", type=float, default=0.25, help="YOLO 置信度阈值")
    ap.add_argument("--imgsz", type=int, default=640, help="YOLO 推理尺寸")
    ap.add_argument("--device", default="", help="YOLO 设备，空=自动(cuda:0/cpu)")
    ap.add_argument("--cam-height", type=float, default=0.0,
                    help="临时覆盖相机光心高度(m)，仅用于排查外参；0=用 calibration/camera.yaml 的值")
    ap.add_argument("--pitch", type=float, default=0.0,
                    help="临时覆盖俯仰角(度)，仅用于排查外参；0=用 calibration/camera.yaml 的值")
    ap.add_argument("--prefer-yaw", type=float, default=None,
                    help="对称歧义时的朝向偏好(度)，例如球门在 +X 端、相机通常朝向球门就填 0")
    ap.add_argument("--max-range", type=float, default=0.0,
                    help="地面投影最大距离(m)；0=按场地半对角自动取值（推荐）")
    ap.add_argument("--keep-unrefined", action="store_true",
                    help="保留未细化成交点、退回框中心的检测；默认丢弃（它们多为压在直线上的误检）")
    ap.add_argument("--dump", default="", help="单帧处理后写到该路径并退出（无界面自测）")
    ap.add_argument("--headless", action="store_true", help="不弹窗，只跑流程并打印状态（回归自测用）")
    ap.add_argument("--max-frames", type=int, default=0, help="headless 下最多处理多少帧")
    ap.add_argument("--roi-top", type=float, default=-1.0,
                    help="白线掩膜顶部裁切比例；<0=用 line_utils 默认值(0.18)，"
                         "0=不裁。远端标记落在画面上部时需调小，否则几何细化失败")
    ap.add_argument("--csv", default="", help="把逐帧位姿写到该 CSV（相机坐标测试用）")
    args = ap.parse_args()

    cam = CameraModel(args.config)
    if args.cam_height > 0:
        print(f"[warn] 临时覆盖光心高度: {cam.cam_height:.3f} -> {args.cam_height:.3f} m（仅排查用）")
        cam.cam_height = args.cam_height
    if args.pitch > 0:
        print(f"[warn] 临时覆盖俯仰角: {math.degrees(cam.pitch):.1f} -> {args.pitch:.1f} deg（仅排查用）")
        cam.pitch = math.radians(args.pitch)
    field = load_field(args.field)
    meta = load_field_meta(args.field)
    gm = GroundModel(cam)                     # 像素<->地面米制 的批量映射
    det_p = {}                                # 检测器参数（中圈半径等）
    if meta.get("circle_r", 0.0) > 0:
        det_p["circle_r_fixed"] = meta["circle_r"]
    if args.roi_top >= 0:
        # 掩膜顶部裁切比例：过大会把画面上部的远端 L/T 标记切掉，导致细化失败
        det_p["roi_top"] = args.roi_top
    circle_r = float(meta.get("circle_r") or 0.75)   # 中圈半径，用于配对两个 X 交点
    _fb = field_bounds(field, 0.8)
    _auto_range = 0.5 * math.hypot(_fb[1] - _fb[0], _fb[3] - _fb[2]) + 1.5
    max_range = args.max_range if args.max_range > 0 else _auto_range
    print("=" * 70)
    print(f"内参 fx={cam.K[0,0]:.1f} fy={cam.K[1,1]:.1f} cx={cam.K[0,2]:.1f} cy={cam.K[1,2]:.1f}")
    print(f"畸变 {cam.dist.ravel()[:5]}")
    print(f"光心高度 {cam.cam_height:.3f} m   俯仰 {math.degrees(cam.pitch):.1f} deg")
    print(f"地图 {args.field}: " + "  ".join(f"{k}={len(v)}" for k, v in field.items()))
    print(f"中圈半径 {circle_r:.3f} m；点球点 = 中圈圆心（弧拟合，失败时用两个 X 交点中点兜底）")
    print("坐标系: O=球场中心, +X 指向球门, +Y 面向 +X 的左侧, yaw 自 +X 逆时针")
    print(f"地面投影量程 {max_range:.1f} m（场地半对角 + 1.5 m 余量）")
    print(f"检测器 YOLO   权重 {args.model}")
    print("=" * 70)

    # 检测器：YOLO 出框 + 框内交点细化（这是唯一的检测路径）
    ensure_weights(args.model)
    detector = YoloMarkerDetector(args.model, conf=args.conf, imgsz=args.imgsz,
                                  device=args.device, params=det_p)

    # 地图视图延后到拿到第一帧后再建，因为布局尺寸依赖实际帧分辨率
    view = None
    layout = args.layout
    hfov = 2.0 * math.degrees(math.atan(cam.width / 2.0 / max(cam.K[0, 0], 1e-6)))
    cap, still = None, None
    if args.image:
        still = cv2.imread(args.image)
        if still is None:
            raise SystemExit("读不到图片: " + args.image)
    else:
        if args.video:
            cap = cv2.VideoCapture(args.video)
        else:
            cap = cv2.VideoCapture(int(args.cam), cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            raise SystemExit("打不开视频源")
        print(f"视频源 {args.video or ('camera ' + args.cam)}  {cap.getBackendName()}  "
              f"{int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")

    os.makedirs(args.out, exist_ok=True)
    csv_f = None
    if args.csv:
        csv_f = open(args.csv, "w", encoding="utf-8", newline="")
        csv_f.write("frame,x_camera_map,y_camera_map,yaw_camera_map,status,valid,"
                    "marker_count,residual,raw_marks,obs_n,ms\n")
    rate, filt = Rate(30), PoseFilter(args.alpha)
    if args.init:
        try:
            ix, iy, iyaw = [float(s) for s in args.init.split(",")]
            filt.pose = [ix, iy, iyaw % 360.0]
            print(f"初始位姿: x={ix:.2f} m  y={iy:.2f} m  yaw={iyaw:.1f} deg")
        except Exception:
            print("[warn] --init 格式不对，应为 x,y,yaw（米,米,度）")
    idx, show_mask, lost = 0, False, 0
    mcfg = dict(inlier_thr=args.inlier_thr, min_inliers=args.min_inliers,
                max_residual=args.max_residual, prefer_yaw=args.prefer_yaw)
    # prior：用于消歧的先验位姿，可以是"暂定解"（不报告 valid，仅打破对称歧义）
    prior = tuple(filt.pose) if filt.pose is not None else None
    prior_miss = 0        # 先验连续失效计数，超过阈值才清先验
    sym_warned = False    # 对称场地首次锁定时提示一次

    while True:
        if still is not None:
            frame = still
        else:
            ok, frame = cap.read()
            if not ok:
                break
        idx += 1
        t0 = time.perf_counter()

        # 1) 图像 -> 标志像素点
        marks, mask = detector.detect(frame)      # YOLO 出框 + 框内交点细化

        # 1b) 兜底：用两个 X 交点的地面中点当场地中心 P
        if not any(m["type"] == "P" for m in marks):
            center = derive_center_from_x(marks, cam, circle_r)
            if center is not None:
                marks.append({"type": "P", "u": center["u"], "v": center["v"],
                              "conf": 0.80, "arms": 0, "derived": True,
                              "a": center["a"], "b": center["b"]})

        # 2) 像素 -> 相机局部水平系地面点 (a 前向, b 左向)
        obs = []
        for mk in marks:
            if mk.get("derived"):
                a, b = mk["a"], mk["b"]     # 导出的圆心：地面坐标已知，不再反投影
            else:
                # 细化失败的框只是"线上一段"，不是交点：默认不参与匹配
                if not mk.get("refined", True) and not args.keep_unrefined:
                    continue
                a, b, ok2, why = cam.pixel_to_local_ground(mk["u"], mk["v"], max_range=max_range)
                if not ok2:
                    continue
            if not (0.25 < math.hypot(a, b) <= max_range):
                continue
            obs.append({"type": mk["type"], "xy": np.array([a, b]),
                        "conf": mk["conf"], "mk": mk})
        obs = sorted(obs, key=lambda o: -o["conf"])[:8]

        # 3) 地图匹配 -> (x_camera_map, y_camera_map, yaw_camera_map)
        # 场地点位中心对称，镜像解同分，靠上一帧位姿（时序连续性）消歧
        if prior is not None:
            mcfg["prev_pose"] = prior
        else:
            mcfg.pop("prev_pose", None)
        res = solve_pose(obs, field, mcfg)
        # 歧义但几何自洽时记下暂定解，避免首帧 ambiguous 导致先验无法建立
        if (res["symmetry"] and not sym_warned and not res.get("resolved_by_preference")
                and not res.get("resolved_by_continuity")):
            print("[warn] 场地中心对称，当前解存在绕原点转 180° 的等价解；"
                  "若发现 x、y 同时反号，用 --init 或 --prefer-yaw 固定分支")
            print(f"       候选: " + "  ".join(
                f"({a['x']:.2f},{a['y']:.2f},{a['yaw']:.1f})" for a in (res.get("alts") or [])[:3]))
            sym_warned = True
        if res["valid"]:
            prior = (res["x"], res["y"], res["yaw"]); prior_miss = 0
        elif (res["status"] == "ambiguous" and res["marker_count"] >= 2
              and not math.isnan(res["residual"])
              and res["residual"] <= mcfg["max_residual"]):
            prior = (res["x"], res["y"], res["yaw"]); prior_miss = 0
        else:
            prior_miss += 1
            if prior_miss > 10:          # 连续 10 帧无有效解才彻底丢弃先验
                prior = None
        obs_map = {}
        for o, m, d in res["inliers"]:
            obs_map[id(o["mk"])] = f"{m[2]} d={d:.2f}"

        # 4) 滤波 + 有效性
        if res["valid"]:
            lost = 0
            fx, fy, fyaw = filt.update(res["x"], res["y"], res["yaw"])
            valid, status = True, "ok"
        else:
            lost += 1
            valid, status = False, res["status"]
            if filt.pose is not None and lost <= 5:
                fx, fy, fyaw = filt.pose
            else:
                fx = fy = fyaw = float("nan")
                filt.reset()

        ms = (time.perf_counter() - t0) * 1000.0
        rate.tick()

        if csv_f is not None:
            csv_f.write(
                f"{idx},{fx:.4f},{fy:.4f},{fyaw:.4f},{status},{int(valid)},"
                f"{res['marker_count']},{res['residual']:.4f},{len(marks)},{len(obs)},"
                f"{ms:.2f}\n")

        # ---------- 5) 可视化
        fh, fw = frame.shape[:2]
        if view is None:
            if layout == "split":
                # 右栏宽度自动收紧到"地图 + 信息卡"刚好放得下，余下宽度让给左栏画面
                R = int(round(args.width * args.map_ratio))
                out_h = 0
                for _ in range(8):
                    left_w = max(320, args.width - R)
                    out_h = max(240, int(round(fh * left_w / float(fw))))
                    tmp = MapView(field, w=R, h=out_h, fov_deg=hfov,
                                  orient=args.map_orient)
                    need = int(tmp.map_w + tmp.info_w + 2 * int(16 * tmp.k)) + 4
                    if need >= R - 4:
                        break
                    R = max(need, 320)
                left_w = max(320, args.width - R)
                out_h = max(240, int(round(fh * left_w / float(fw))))
                view = MapView(field, w=R, h=out_h, fov_deg=hfov,
                               orient=args.map_orient)
            else:
                mm = int(min(fw, fh) * 0.42)
                view = MapView(field, w=mm, h=mm, fov_deg=hfov,
                               orient=args.map_orient, info_w=0)
            print(f"布局 {layout}: 画面 {fw}x{fh} -> 面板 {view.w}x{view.h}  "
                  f"地图区 {view.map_w:.0f}x{view.map_h:.0f} px  "
                  f"比例 {view.scale:.1f} px/m")

        matched = [(m[2], d) for _, m, d in res["inliers"]]

        if layout == "split":
            left_w = max(320, args.width - view.w)
            out_h = max(240, int(round(fh * left_w / float(fw))))
            sx, sy = left_w / float(fw), out_h / float(fh)
            left = cv2.resize(frame, (left_w, out_h), interpolation=cv2.INTER_AREA)
            if show_mask:
                ov = left.copy()
                m2 = cv2.resize(mask, (left_w, out_h), interpolation=cv2.INTER_NEAREST)
                ov[m2 > 0] = (0, 220, 0)
                left = cv2.addWeighted(ov, 0.45, left, 0.55, 0)
            draw_markers(left, marks, obs_map, sx, sy)
            st = {"fps": rate.value(), "ms": ms, "idx": idx,
                  "obs_n": len(obs), "mk_n": len(marks), "cnt": res["marker_count"],
                  "res": res["residual"] if not math.isnan(res["residual"]) else 0.0,
                  "x": fx, "y": fy, "yaw": fyaw, "valid": valid, "status": status,
                  "sym": bool(res.get("symmetry"))}
            left = draw_hud(left, st)

            if math.isnan(fx):
                right = view.base.copy()
                cv2.putText(right, "NO VALID POSE", (int(20 * view.k), int(130 * view.k)),
                            FONT, 0.8 * view.k, (90, 90, 90), 2, cv2.LINE_AA)
                cv2.putText(right, f"status = {status}", (int(20 * view.k), int(160 * view.k)),
                            FONT, 0.6 * view.k, (90, 90, 90), 1, cv2.LINE_AA)
            else:
                right = view.draw_pose(fx, fy, fyaw, valid, st["res"], matched)
            canvas = np.hstack([left, right])
        else:
            # minimap：画面保持原始分辨率，地图以正方形叠加在左下角
            canvas = frame.copy()
            if show_mask:
                ov = canvas.copy()
                ov[mask > 0] = (0, 220, 0)
                canvas = cv2.addWeighted(ov, 0.45, canvas, 0.55, 0)
            draw_markers(canvas, marks, obs_map)
            st = {"fps": rate.value(), "ms": ms, "idx": idx,
                  "obs_n": len(obs), "mk_n": len(marks), "cnt": res["marker_count"],
                  "res": res["residual"] if not math.isnan(res["residual"]) else 0.0,
                  "x": fx, "y": fy, "yaw": fyaw, "valid": valid, "status": status,
                  "sym": bool(res.get("symmetry"))}
            canvas = draw_hud(canvas, st)
            mm = view.w
            if math.isnan(fx):
                mini = view.base.copy()
                cv2.putText(mini, "NO POSE", (int(12 * view.k), int(70 * view.k)),
                            FONT, 0.6 * view.k, (90, 90, 90), 2, cv2.LINE_AA)
            else:
                mini = view.draw_pose(fx, fy, fyaw, valid, st["res"], matched)
            y0, x0 = fh - mm, 0
            canvas[y0:fh, x0:x0 + mm] = mini
            cv2.rectangle(canvas, (x0, y0), (x0 + mm - 1, fh - 1), (120, 120, 120), 1)

        if args.dump:
            cv2.imwrite(args.dump, canvas)
            print(f"[dump] {args.dump}\n"
                  f"  pose  x={res['x']:.3f}  y={res['y']:.3f}  yaw={res['yaw']:.2f} deg\n"
                  f"  status={status}  valid={valid}  marker_count={res['marker_count']}  "
                  f"residual={res['residual']:.3f} m")
            break

        if not args.headless:
            cv2.imshow("opencv field localization", canvas)
            if idx == 1:
                cv2.resizeWindow("opencv field localization", canvas.shape[1], canvas.shape[0])

        key = 255 if args.headless else (cv2.waitKey(1) & 0xFF)
        if key in (ord("q"), 27):
            break
        if key == ord("s"):
            p = os.path.join(args.out, f"loc_{idx:06d}.png")
            cv2.imwrite(p, canvas)
            print("[snap]", p)
        if key == ord("d"):
            show_mask = not show_mask
        if key == ord("r"):
            filt.reset()
        if key == ord("m"):
            layout = "minimap" if layout == "split" else "split"
            view = None                       # 下一帧按新布局重建地图视图
            print("[layout] ->", layout)

        if args.headless and args.max_frames and idx >= args.max_frames:
            break
        if idx % 15 == 0:
            print(f"#{idx:5d} FPS {rate.value():5.1f} proc {ms:5.1f}ms  raw {len(marks)} obs {len(obs)} "
                  f"matched {res['marker_count']} res {res['residual']:.3f} "
                  f"pose=({fx:.2f},{fy:.2f},{fyaw:.1f}deg) {status}")

    if cap is not None:
        cap.release()
    if csv_f is not None:
        csv_f.close()
        print(f"[csv] -> {args.csv}")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
