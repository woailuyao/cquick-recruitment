"""任务二主程序：单目场地自定位（左 RGB + 右地图）

运行前需要三个文件（前两个要自己标定）：
    calibration/out/camera_intrinsics.yaml   内参（已有）
    calibration/camera_extrinsics.yaml   外参（相机高度/俯仰）
    config/field.yaml                        场地地图（已有）

用法：
    python localize_app.py                        # 相机 + 几何检测器
    python localize_app.py --detector yolo --weights best.pt   # 换成训练好的 YOLO
    python localize_app.py --video xxx.mp4        # 用视频文件（录屏/调试）
    python localize_app.py --click                # 鼠标点标志（验证几何/匹配用）
    python localize_app.py --ros                  # 同时发 ROS2 话题
    python localize_app.py --headless --video x.mp4 --save out.mp4   # 不开窗口

界面：
    左边 = 相机实时画面 + 标志检测 + 投到地面的结果
    右边 = 场地俯视图 + 相机位置/箭头 + x/y/yaw/残差/状态/FPS
"""

import argparse
import math
import os
import sys
import time

import cv2
import numpy as np

def _repo_root():
    """向上找到包含 config/field.yaml 的目录，作为仓库根。"""
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(4):
        if os.path.exists(os.path.join(d, "config", "field.yaml")):
            return d
        d = os.path.dirname(d)
    return os.getcwd()

ROOT = _repo_root()
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from vision2.console import setup_console  # noqa: E402
from vision2.field_match import FieldMap, bounds_from_geometry, match  # noqa: E402
from vision2.geometry import GroundProjector, angles_from_rotation  # noqa: E402
from vision2.io_utils import imread, imwrite, list_images  # noqa: E402
from vision2.yaml_mini import load_yaml  # noqa: E402

setup_console()

CAL = os.path.join(ROOT, "calibration")


def _first(*paths):
    for p in paths:
        if os.path.exists(p):
            return p
    return paths[0]


INTR = _first(os.path.join(CAL, "out", "camera_intrinsics.yaml"),
              os.path.join(CAL, "camera_intrinsics.yaml"))
EXTR = _first(os.path.join(CAL, "out", "camera_extrinsics.yaml"),
              os.path.join(CAL, "camera_extrinsics.yaml"))
FIELD = os.path.join(ROOT, "config", "field.yaml")
GEOM = os.path.join(ROOT, "config", "field_geometry.yaml")

TYPE_COLOR = {"L": (0, 200, 255), "T": (80, 220, 80), "X": (255, 140, 60), "P": (0, 0, 255), "C": (255, 255, 0)}


# ------------------------------------------------------------------ 检测器

def make_detector(args):
    if args.detector == "yolo":
        return YoloDetector(args.weights, args.conf, refine=not args.no_refine, imgsz=args.imgsz)
    if args.detector == "click":
        return ClickDetector()
    if args.detector == "thick":
        return ThickDetector(args.conf)
    return ClassicalDetector(args.conf)


class ThickDetector:
    """粗线几何检测器（近景/线很粗时用这个）：RANSAC 拟合粗线 + 四条胳膊判定。"""

    name = "thick"

    def __init__(self, conf=0.45):
        from vision2 import markers_thick
        self.m = markers_thick
        self.conf = conf

    def detect(self, frame):
        dets = [d for d in self.m.detect(frame) if d["conf"] >= self.conf]
        if dets:
            return dets
        # 兜底：粗线拟合没找到交点时，换端点法再试一次（两套方法互补）
        from vision2 import markers as endpoint
        return [d for d in endpoint.detect(frame, min_conf=self.conf) if d["conf"] >= self.conf]


class ClassicalDetector:
    """白线几何检测器：光照正常时可用，强光过曝时基本失效。"""

    name = "classical"

    def __init__(self, conf=0.45):
        from vision2 import markers
        self.markers = markers
        self.conf = conf

    def detect(self, frame):
        return self.markers.detect(frame, min_conf=self.conf)


class YoloDetector:
    """YOLO 检测器 + 几何细化。

    支持两种权重：
      * 单类 `marker`：模型只负责"这里有标志"，L/T/X 由 vision2/refine.py 判；
      * 多类 `LCross/TCross/XCross`：直接用模型给的类别，仍做一次几何细化取精确交点。
    """

    name = "yolo"
    MAP = {"lcross": "L", "tcross": "T", "xcross": "X",
           "penaltypoint": "P", "penalitypoint": "P", "marker": None}

    def __init__(self, weights, conf=0.35, refine=True, imgsz=480):
        from ultralytics import YOLO

        if not weights or not os.path.exists(weights):
            raise SystemExit("[错误] 找不到 YOLO 权重：%s\n  先训练，再用 --weights best.pt" % weights)
        self.model = YOLO(weights)
        self.conf = conf
        self.refine = refine
        self.single_class = len(self.model.names) == 1
        self.imgsz = imgsz

    def detect(self, frame):
        res = self.model.predict(frame, conf=self.conf, imgsz=self.imgsz, verbose=False)[0]
        raw = []
        for b in res.boxes:
            cls = int(b.cls.item())
            name = str(self.model.names[cls]).lower()
            if name not in self.MAP:
                continue
            t = self.MAP[name]
            x1, y1, x2, y2 = [float(v) for v in b.xyxy[0].tolist()]
            raw.append({"type": t, "u": (x1 + x2) / 2.0, "v": (y1 + y2) / 2.0,
                        "conf": float(b.conf.item()), "bbox": (x1, y1, x2, y2)})

        if not self.refine:
            return [d for d in raw if d["type"]]

        from vision2 import refine as refiner

        out = []
        for d in raw:
            r = refiner.refine(frame, d["u"], d["v"], box=d["bbox"])
            if r is None:
                # 细化失败：多类模型还能用它的类别，单类模型就丢掉
                if d["type"]:
                    out.append(d)
                continue
            typ, x, y, cf = r
            out.append({"type": d["type"] or typ, "u": x, "v": y,
                        "conf": min(1.0, 0.5 * d["conf"] + 0.5 * cf), "bbox": d["bbox"]})
        return out


class ClickDetector:
    """鼠标点选：用来自测几何和匹配（不依赖任何模型）。左键点一个点，按 1/2/3/4 选类别。"""

    name = "click"

    def __init__(self):
        self.points = []
        self.cur_type = "T"
        print("[click] 点一下标志，然后按 1=L 2=T 3=X 4=P 切换类别；r 清空，空格确认一帧。")

    def detect(self, frame):
        return list(self.points)


# ------------------------------------------------------------------ 主程序

class Localizer:
    def __init__(self, args):
        self.args = args
        self.intr = load_yaml(INTR)
        self.extr = load_yaml(EXTR)
        self.geom = load_yaml(GEOM)
        self.fmap = FieldMap.load(FIELD)
        self.bounds = bounds_from_geometry(self.geom)
        self.K = np.asarray(self.intr["camera_matrix"], float).reshape(3, 3)
        self.dist = np.asarray(self.intr.get("dist_coeffs") or self.intr["distortion_coefficients"], float).reshape(-1, 1)
        self.W = int(self.intr.get("image_width", 1280))
        self.H = int(self.intr.get("image_height", 720))
        self.proj = GroundProjector(
            self.K, self.dist,
            np.asarray(self.extr["R_gc"], float).reshape(3, 3),
            np.asarray(self.extr["t_gc"], float).reshape(3),
            width=self.W, height=self.H,
        )
        self.smooth = None
        self.last_valid = None
        self.source_label = "video replay" if getattr(args, "video", "") else "live camera"

    # -------------------------------------------------- 一帧
    def process(self, frame, dets):
        obs, used = [], []
        for d in dets:
            g = self.proj.pixel_to_ground(d["u"], d["v"])
            if g is None:
                continue
            item = {"type": d["type"], "x": float(g[0]), "y": float(g[1]), "conf": d.get("conf", 1.0)}
            obs.append(item)
            used.append((d, g))

        res = match(obs, self.fmap, gating=0.8, bounds=self.bounds)
        res = self._temporal(res)
        return res, obs, used

    def _temporal(self, res):
        """时序滤波 + 跳变门限：上一帧和这一帧差太多时先不认，避免突然跳到地图另一头。"""
        if not res["valid"]:
            self.smooth = None
            return res
        cur = np.array([res["x"], res["y"], res["yaw_deg"]])
        if self.smooth is None:
            self.smooth = cur
        else:
            dpos = math.hypot(cur[0] - self.smooth[0], cur[1] - self.smooth[1])
            dyaw = abs((cur[2] - self.smooth[2] + 180) % 360 - 180)
            if dpos > 1.5 or dyaw > 45:
                # 突变：可能是误匹配，先标记为可疑
                res["valid"] = False
                res["status"] = "jump_rejected"
                return res
            a = 0.35
            self.smooth = self.smooth + a * np.array([cur[0] - self.smooth[0],
                                                      cur[1] - self.smooth[1],
                                                      (cur[2] - self.smooth[2] + 180) % 360 - 180])
            self.smooth[2] = (self.smooth[2] + 180) % 360 - 180
        res["x_f"], res["y_f"], res["yaw_f"] = float(self.smooth[0]), float(self.smooth[1]), float(self.smooth[2])
        return res

    # -------------------------------------------------- 画面
    def render(self, frame, dets, res, obs, used):
        left = self._render_left(frame, dets, used, res)
        right = self._render_map(res)
        h = max(left.shape[0], right.shape[0])
        if left.shape[0] < h:
            left = cv2.copyMakeBorder(left, 0, h - left.shape[0], 0, 0, cv2.BORDER_CONSTANT, value=(20, 20, 20))
        if right.shape[0] < h:
            right = cv2.copyMakeBorder(right, 0, h - right.shape[0], 0, 0, cv2.BORDER_CONSTANT, value=(20, 20, 20))
        return np.hstack([left, right])

    def _render_left(self, frame, dets, used, res):
        vis = frame.copy()
        for d in dets:
            c = TYPE_COLOR.get(d["type"], (255, 255, 255))
            if "bbox" in d:
                x1, y1, x2, y2 = [int(v) for v in d["bbox"]]
                cv2.rectangle(vis, (x1, y1), (x2, y2), c, 2)
            p = (int(round(d["u"])), int(round(d["v"])))
            cv2.circle(vis, p, 7, c, 2, cv2.LINE_AA)
            cv2.putText(vis, "%s %.2f" % (d["type"], d.get("conf", 1.0)), (p[0] + 9, p[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 1, cv2.LINE_AA)
        for d, g in used:
            cv2.putText(vis, "%.2f,%.2f" % (g[0], g[1]), (int(d["u"]) + 9, int(d["v"]) + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)

        lines = [
            "SOURCE: %s" % self.source_label,
            "res %dx%d  FPS %5.1f" % (self.W, self.H, getattr(self, "fps", 0.0)),
            "det %d  ground %d" % (len(dets), len(used)),
        ]
        if res.get("x") is not None:
            lines.append("raw  x=%6.2f y=%6.2f yaw=%7.1f" % (res["x"], res["y"], res["yaw_deg"]))
        if "x_f" in res:
            lines.append("flt  x=%6.2f y=%6.2f yaw=%7.1f" % (res["x_f"], res["y_f"], res["yaw_f"]))
        lines.append("markers %d  resid %s" % (res.get("marker_count", 0),
                                               "-" if res.get("residual") is None else "%.2f m" % res["residual"]))
        lines.append("valid %s  [%s]" % ("YES" if res.get("valid") else "NO", res.get("status", "-")))
        y = 22
        for s in lines:
            cv2.putText(vis, s, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 255, 0) if ("valid YES" in s) else (240, 240, 240), 1, cv2.LINE_AA)
            y += 22
        return cv2.resize(vis, (self.W // 2, self.H // 2))

    def _render_map(self, res):
        depth = float(self.geom["field"]["depth_m"])
        width = float(self.geom["field"]["width_m"])
        panel_w, panel_h = self.W // 2, self.H // 2
        margin = 28
        sx = (panel_w - 2 * margin) / depth
        sy = (panel_h - 2 * margin) / width
        s = min(sx, sy)
        ox = (panel_w - depth * s) / 2.0
        oy = (panel_h - width * s) / 2.0

        def mp(x, y):
            return (int(ox + x * s), int(oy + (width / 2.0 - y) * s))

        img = np.full((panel_h, panel_w, 3), 32, np.uint8)
        cv2.rectangle(img, mp(0, -width / 2), mp(depth, width / 2), (28, 88, 42), -1)
        for a, b in (((0, -width / 2), (depth, -width / 2)), ((0, width / 2), (depth, width / 2)),
                     ((0, -width / 2), (0, width / 2)), ((depth, -width / 2), (depth, width / 2))):
            cv2.line(img, mp(*a), mp(*b), (220, 220, 220), 1, cv2.LINE_AA)
        r = float(self.geom["field"]["center_circle_radius_m"])
        cv2.ellipse(img, mp(0, 0), (int(r * s), int(r * s)), 0, -90, 90, (220, 220, 220), 1, cv2.LINE_AA)

        for f in self.fmap.features:
            p = mp(f.x, f.y)
            c = TYPE_COLOR.get(f.type, (200, 200, 200))
            cv2.circle(img, p, 3, c, -1, cv2.LINE_AA)

        if res.get("x") is not None:
            p = mp(res["x"], res["y"])
            color = (0, 255, 0) if res.get("valid") else (0, 165, 255)
            cv2.drawMarker(img, p, color, cv2.MARKER_TRIANGLE_UP, 14, 2)
            yaw = math.radians(res["yaw_deg"])
            d = 0.7 * s
            q = (int(p[0] + d * math.cos(yaw)), int(p[1] - d * math.sin(yaw)))
            cv2.arrowedLine(img, p, q, color, 2, tipLength=0.25)
        cv2.putText(img, "field map  %.1fx%.1f m" % (depth, width), (8, 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(img, "T/L/X/P dots  green=camera", (8, panel_h - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (160, 160, 160), 1, cv2.LINE_AA)
        return img


def open_source(args):
    if args.video:
        return cv2.VideoCapture(args.video), None
    cap = cv2.VideoCapture(args.index, cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    return cap, None


def main():
    ap = argparse.ArgumentParser(description="单目场地自定位")
    ap.add_argument("--detector", default="thick", choices=["thick", "classical", "yolo", "click"])
    ap.add_argument("--weights", default="", help="YOLO 权重路径")
    ap.add_argument("--conf", type=float, default=0.45, help="检测置信度阈值")
    ap.add_argument("--video", default="", help="用视频文件代替相机")
    ap.add_argument("--index", type=int, default=0, help="摄像头序号")
    ap.add_argument("--click", action="store_true", help="等价 --detector click")
    ap.add_argument("--ros", action="store_true", help="发布 ROS2 话题")
    ap.add_argument("--headless", action="store_true", help="不开窗口（配合 --save 用）")
    ap.add_argument("--save", default="", help="把左右分屏结果写成视频")
    ap.add_argument("--max-frames", type=int, default=0, help="最多处理多少帧（0=全部）")
    ap.add_argument("--display-scale", type=float, default=0.0, help="显示放大倍数（0=自动：开窗口时 1.5 倍）")
    ap.add_argument("--no-refine", action="store_true", help="YOLO 后不做几何细化（更快，但没类型）")
    ap.add_argument("--imgsz", type=int, default=480, help="YOLO 推理尺寸")
    args = ap.parse_args()
    if args.click:
        args.detector = "click"

    for p in (INTR, EXTR, FIELD, GEOM):
        if not os.path.exists(p):
            print("[错误] 缺少文件：%s" % p)
            if p == EXTR:
                print("       外参还没标：相机装好后双击 双击我_标定外参.bat（1 分钟）")
            return 1

    loc = Localizer(args)
    if str(loc.extr.get("method", "")).upper().startswith("PLACEHOLDER"):
        print("[警告] 现在用的外参是占位值（未实测），定位结果不可信！")
        print("       相机装好后双击 双击我_标定外参.bat 重新标一次。")
    det = make_detector(args)
    if args.detector != "click":
        print("[信息] 检测器：%s" % det.name)

    ros = None
    if args.ros:
        try:
            import rclpy
            from geometry_msgs.msg import Pose2D
            from std_msgs.msg import String

            rclpy.init()
            node = rclpy.create_node("field_localization")
            pub = node.create_publisher(Pose2D, "/localization/field_pose", 10)
            pub_s = node.create_publisher(String, "/localization/field_status", 10)
            ros = (rclpy, node, pub, pub_s, Pose2D, String)
            print("[信息] ROS2 已开启：/localization/field_pose")
        except Exception as exc:
            print("[警告] ROS2 不可用（%s），只在本机显示" % exc)

    cap, _ = open_source(args)
    if not cap.isOpened():
        print("[错误] 打不开视频/相机")
        return 1

    writer = None
    click_state = {"pts": [], "type": "T"}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            click_state["pts"].append({"type": click_state["type"], "u": float(x), "v": float(y), "conf": 1.0})
            print("[click] 加点 %s (%.0f, %.0f)，共 %d 个" % (click_state["type"], x, y, len(click_state["pts"])))

    if args.detector == "click" and not args.headless:
        cv2.namedWindow("field localization")
        cv2.setMouseCallback("field localization", on_mouse)

    t0 = time.time()
    frames = 0
    last_show = 0.0
    print("[信息] 开始。按 q 退出。")
    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            break
        frame = cv2.resize(frame, (loc.W, loc.H))

        if args.detector == "click":
            dets = list(click_state["pts"])
        else:
            dets = det.detect(frame)

        res, obs, used = loc.process(frame, dets)
        frames += 1
        dt = time.time() - t0
        loc.fps = frames / dt if dt > 0 else 0.0

        canvas = loc.render(frame, dets, res, obs, used)
        dscale = args.display_scale or (1.0 if (args.headless or args.save) else 1.5)
        if dscale != 1.0:
            canvas = cv2.resize(canvas, None, fx=dscale, fy=dscale, interpolation=cv2.INTER_LINEAR)

        if writer is None and args.save:
            writer = cv2.VideoWriter(args.save, cv2.VideoWriter_fourcc(*"mp4v"), 15.0,
                                     (canvas.shape[1], canvas.shape[0]))
        if writer is not None:
            writer.write(canvas)

        if ros is not None and res.get("x") is not None:
            rclpy, node, pub, pub_s, Pose2D, String = ros
            m = Pose2D()
            m.x = float(res.get("x_f", res["x"]))
            m.y = float(res.get("y_f", res["y"]))
            m.theta = math.radians(float(res.get("yaw_f", res["yaw_deg"])))
            pub.publish(m)
            s = String()
            s.data = "%s|valid=%s|markers=%d|resid=%s" % (
                res.get("status", "-"), res.get("valid"), res.get("marker_count", 0),
                "-" if res.get("residual") is None else "%.3f" % res["residual"])
            pub_s.publish(s)

        if not args.headless:
            cv2.imshow("field localization", canvas)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if args.detector == "click":
                if key in (ord("1"), ord("2"), ord("3"), ord("4")):
                    click_state["type"] = "LTXP"[key - ord("1")]
                    print("[click] 当前类别：%s" % click_state["type"])
                elif key == ord("r"):
                    click_state["pts"] = []
                    print("[click] 已清空")
        if args.max_frames and frames >= args.max_frames:
            break

    cap.release()
    if writer is not None:
        writer.release()
    cv2.destroyAllWindows()
    if ros is not None:
        try:
            ros[1].destroy_node()
            ros[0].shutdown()
        except Exception:
            pass
    print("[信息] 结束，共 %d 帧，平均 %.1f FPS" % (frames, frames / max(time.time() - t0, 1e-6)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
