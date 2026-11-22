"""Task 2 main program: left RGB detections + right field map, camera pose output.

    python scripts/run_localize.py --camera 0
    python scripts/run_localize.py --image debug_frames/cam0.jpg --save out.jpg
    python scripts/run_localize.py --camera 0 --roi-scale 2.0    # digital zoom
    python scripts/run_localize.py --camera 0 --init-pose 0.0 -3.0 0

Right panel shows:
    x_camera_map / y_camera_map / yaw_camera_map
    valid / status / marker_count / residual
    fps and pose update rate
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_ROOT))


from field_localize.field_map import FieldMap, sha256_of                      # noqa: E402
from field_localize.ground_projection import GroundProjector, R_to_euler       # noqa: E402
from field_localize.marker_geometry import refine_marker_point                 # noqa: E402
from field_localize.pose_match import Locator, Observation                     # noqa: E402
from field_localize.viz import MapPanel, compose, draw_hud, TYPE_COLORS        # noqa: E402


# ----------------------------------------------------------------------
class Smoother:
    """Exponential smoothing with a jump gate; drops the pose when lost."""

    def __init__(self, alpha=0.45, jump_pos=1.0, jump_yaw_deg=25.0, lost_after=5):
        self.alpha = alpha
        self.jump_pos = jump_pos
        self.jump_yaw = math.radians(jump_yaw_deg)
        self.lost_after = lost_after
        self.reset()

    def reset(self):
        self.have = False
        self.x = self.y = 0.0
        self.yaw = 0.0
        self.lost = 0
        self.jumps = 0

    def update(self, res):
        if not res.valid:
            self.lost += 1
            if self.lost >= self.lost_after:
                self.have = False
            return None
        self.lost = 0
        if not self.have:
            self.x, self.y, self.yaw = res.x, res.y, math.radians(res.yaw_deg)
            self.have = True
            return (self.x, self.y, math.degrees(self.yaw))
        dx, dy = res.x - self.x, res.y - self.y
        dyaw = (math.radians(res.yaw_deg) - self.yaw + math.pi) % (2 * math.pi) - math.pi
        if math.hypot(dx, dy) > self.jump_pos or abs(dyaw) > self.jump_yaw:
            self.jumps += 1          # big jump: follow it instead of smoothing
            self.x, self.y, self.yaw = res.x, res.y, math.radians(res.yaw_deg)
            return (self.x, self.y, math.degrees(self.yaw))
        a = self.alpha
        self.x += a * dx
        self.y += a * dy
        self.yaw += a * dyaw
        return (self.x, self.y, math.degrees(self.yaw))


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def to_camera(carrier_deg, Ox, Oy, yaw_off):
    """carrier pose -> camera optical centre pose (degrees)."""
    x, y, yaw = carrier_deg
    th = math.radians(yaw)
    xc = x + math.cos(th) * Ox - math.sin(th) * Oy
    yc = y + math.sin(th) * Ox + math.cos(th) * Oy
    return (xc, yc, wrap_deg(yaw + yaw_off))


# ----------------------------------------------------------------------
def resolve_model(path):
    """If --model does not exist, fall back to the newest runs/**/weights/best.pt."""
    p = Path(path)
    if p.exists():
        return str(p)
    cands = sorted(ROOT.glob("runs/**/weights/best.pt"),
                   key=lambda q: q.stat().st_mtime, reverse=True)
    if cands:
        print(f"[model] {path} not found, using newest: {cands[0]}")
        return str(cands[0])
    raise SystemExit(f"[model] not found: {path} (and no runs/**/weights/best.pt)")


def build(args):
    fmap = FieldMap.load(args.field)
    proj = GroundProjector.from_yaml(args.intrinsics, args.extrinsics,
                                     max_range=args.max_range)
    locator = Locator(fmap,
                      pair_len_tol=args.pair_tol,
                      inlier_thresh=args.inlier_thresh,
                      max_rms=args.max_rms,
                      min_inliers=args.min_inliers,
                      ambiguity_score_ratio=args.amb_ratio,
                      prior_gate_m=args.prior_gate,
                      prior_gate_yaw_deg=args.prior_gate_yaw)
    return fmap, proj, locator


def _predict_boxes(model, img, args, offset=(0, 0)):
    r = model.predict(img, conf=args.conf, imgsz=args.imgsz,
                      device=args.device, verbose=False)[0]
    out = []
    if r.boxes is None:
        return out
    ox, oy = offset
    for b in r.boxes:
        x1, y1, x2, y2 = [float(v) for v in b.xyxy[0].tolist()]
        out.append((x1 + ox, y1 + oy, x2 + ox, y2 + oy,
                    float(b.conf[0]), int(b.cls[0])))
    return out


def _nms(boxes, iou_th=0.45):
    boxes = sorted(boxes, key=lambda d: -d[4])
    keep = []
    for d in boxes:
        drop = False
        for k in keep:
            ix1, iy1 = max(d[0], k[0]), max(d[1], k[1])
            ix2, iy2 = min(d[2], k[2]), min(d[3], k[3])
            iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
            inter = iw * ih
            a1 = (d[2] - d[0]) * (d[3] - d[1])
            a2 = (k[2] - k[0]) * (k[3] - k[1])
            if inter / max(1e-6, a1 + a2 - inter) > iou_th:
                drop = True
                break
        if not drop:
            keep.append(d)
    return keep


def detect_and_observations(frame, model, proj, args):
    """整图 + 切片（tiling）推理。

    远处标志在 1280x720 里只有 60~110 px，而模型是在 250 px 的框上训的，
    整图直接推理基本检不出。把画面切成 tile x tile 个互相重叠的小块，
    每块单独放大到 imgsz 再送模型，标志在输入里就放大了 ~2 倍。
    """
    H, W = frame.shape[:2]
    boxes = _predict_boxes(model, frame, args)

    tile = int(getattr(args, "tile", 1) or 1)
    if tile > 1:
        for ty in range(tile):
            for tx in range(tile):
                tw, th = W / tile, H / tile
                pad_x, pad_y = tw * 0.12, th * 0.12
                x0 = max(0, int(tx * tw - pad_x))
                y0 = max(0, int(ty * th - pad_y))
                x1 = min(W, int((tx + 1) * tw + pad_x))
                y1 = min(H, int((ty + 1) * th + pad_y))
                if x1 - x0 < 32 or y1 - y0 < 32:
                    continue
                boxes += _predict_boxes(model, frame[y0:y1, x0:x1], args, (x0, y0))
        boxes = _nms(boxes)

    dets, obs, n_fallback = [], [], 0
    for x1, y1, x2, y2, conf, cls in boxes:
        name = model.names.get(cls, str(cls))
        ref = refine_marker_point(frame, (x1, y1, x2, y2), name,
                                  allow_bbox_fallback=args.bbox_fallback)
        if ref is None:
            continue
        if ref["method"] == "bbox_center":
            n_fallback += 1
        g = (proj.project(ref["u"], ref["v"])
             if (ref["ok"] or args.use_fallback_point) else None)
        dets.append({"box": (x1, y1, x2, y2), "type": name, "conf": conf,
                     "ref": ref, "ground": g})
        if g is not None:
            obs.append(Observation(type=name, x=g["x"], y=g["y"], conf=conf,
                                   range_m=g["range_m"],
                                   pixel=(ref["u"], ref["v"])))
    return dets, obs, n_fallback


# ----------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--video", default=None,
                    help="用视频文件代替相机跑（离线自检 / 回放录屏用）")
    ap.add_argument("--image", default=None, help="single image mode")
    ap.add_argument("--save", default=None)
    ap.add_argument("--model", default=str(ROOT / "weights" / "markers_v7_best.pt"))
    ap.add_argument("--intrinsics", default=str(ROOT / "calibration" / "camera_intrinsics.yaml"))
    ap.add_argument("--extrinsics", default=str(ROOT / "calibration" / "camera_extrinsics.yaml"))
    ap.add_argument("--field", default=str(ROOT / "config" / "field.yaml"))
    ap.add_argument("--geometry", default=str(ROOT / "config" / "field_geometry.yaml"))
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--tile", type=int, default=1,
                    help="切片推理：1 = 整图（v6 已能整图检出，默认关）；"
                         "远处仍检不到时可设 2，把画面切 2x2 重叠块分别推理")
    ap.add_argument("--imgsz", type=int, default=1280,
                    help="1280 = 不缩小原图（1280x720 正好），远处标志更清楚；960 会把图缩到 0.75 倍")
    ap.add_argument("--exposure", type=float, default=None,
                    help="手动曝光（DirectShow，通常是负值越小越暗）；不给则用自动曝光")
    ap.add_argument("--gain", type=float, default=None, help="手动增益")
    ap.add_argument("--roi-scale", type=float, default=1.0,
                    help=">1: feed only the central 1/roi of the frame to the model"
                         " (digital zoom). Helps distant markers, narrows the view.")
    ap.add_argument("--device", default="0")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--max-range", type=float, default=15.0)
    ap.add_argument("--pair-tol", type=float, default=0.35)
    ap.add_argument("--inlier-thresh", type=float, default=0.30)
    ap.add_argument("--max-rms", type=float, default=0.35)
    ap.add_argument("--min-inliers", type=int, default=3,
                    help="至少多少个内点才算定位成功（2 = 允许两个标志）")
    ap.add_argument("--amb-ratio", type=float, default=0.75,
                    help="歧义判据：次优解分数/最优解 超过这个比例就算歧义（越大越宽松）")
    ap.add_argument("--prior-gate", type=float, default=1.5,
                    help="上一帧先验的搜索半径（米）；扛着支架走得远就调大")
    ap.add_argument("--prior-gate-yaw", type=float, default=25.0,
                    help="上一帧先验的朝向门限（度）")
    ap.add_argument("--relaxed", action="store_true",
                    help="放宽模式：先验门限 2.5 m/50°，max_rms 0.6，允许 2 个内点，"
                         "歧义比例 0.93 —— 更容易出位姿，但误差上限也更大")
    ap.add_argument("--bbox-fallback", action="store_true", default=True,
                    help="fall back to bbox centre when line fitting fails")
    ap.add_argument("--use-fallback-point", action="store_true", default=True)
    ap.add_argument("--no-smooth", action="store_true")
    ap.add_argument("--init-pose", nargs=3, type=float, default=None,
                    metavar=("X", "Y", "YAW"), help="initial pose prior (m / deg)")
    ap.add_argument("--bounds-margin", type=float, default=0.6,
                    help="how far outside the map a pose may go before it is rejected")
    ap.add_argument("--no-bounds", action="store_true")
    ap.add_argument("--save-video", default=None)
    ap.add_argument("--csv", default=None)
    ap.add_argument("--no-show", action="store_true")
    args = ap.parse_args()

    if args.relaxed:
        args.prior_gate = max(args.prior_gate, 2.5)
        args.prior_gate_yaw = max(args.prior_gate_yaw, 30.0)
        args.max_rms = max(args.max_rms, 0.5)
        args.amb_ratio = max(args.amb_ratio, 0.93)
        print("[relaxed] prior_gate=%.1fm/%.0fdeg max_rms=%.2f amb=%.2f min_inliers=%d"
              % (args.prior_gate, args.prior_gate_yaw, args.max_rms,
                 args.amb_ratio, args.min_inliers))

    fmap, proj, locator = build(args)
    if not args.no_bounds:
        x0, y0, x1, y1 = fmap.bbox(pad=args.bounds_margin)
        locator.bounds = (x0, y0, x1, y1)
    model = __import__("ultralytics").YOLO(resolve_model(args.model))

    geom = {}
    if Path(args.geometry).exists():
        with open(args.geometry, "r", encoding="utf-8") as f:
            geom = yaml.safe_load(f) or {}

    print("=" * 70)
    print(fmap.summary())
    print(f"SHA256   : {sha256_of(args.field)}")
    print("-" * 70)
    print(proj.describe())
    print(f"model    : {args.model}")
    print(f"classes  : {model.names}")
    print(f"params   : conf={args.conf} roi={args.roi_scale} pair_tol={args.pair_tol} "
          f"inlier={args.inlier_thresh} max_rms={args.max_rms}")
    print("=" * 70)

    cam_fwd = proj.R @ np.array([0.0, 0.0, 1.0])
    yaw_off = math.degrees(math.atan2(cam_fwd[1], cam_fwd[0]))
    Ox, Oy, Oz = proj.O

    panel = MapPanel(fmap, size=(640, 720),
                     field_size=geom.get("size") or fmap.size)

    # ---------------- single image ----------------
    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            raise SystemExit(f"cannot read image: {args.image}")
        dets, obs, nfb = detect_and_observations(frame, model, proj, args)
        init = tuple(args.init_pose) if args.init_pose else None
        res = locator.locate(obs, prior_pose=init)
        print(f"detected {len(dets)} markers, usable ground obs {len(obs)}, "
              f"bbox fallback {nfb}")
        print("pose:", res.as_dict())
        for inf in res.inliers:
            print(f"   obs#{inf['obs_index']} {obs[inf['obs_index']].type} "
                  f"-> {inf['map_id']}  err={inf['dist']:.3f} m")
        left = draw_left(frame.copy(), dets, res, "offline", None, None)
        cam_pose = (to_camera((res.x, res.y, res.yaw_deg), Ox, Oy, yaw_off)
                    if res.valid else None)
        pose = ((cam_pose[0], cam_pose[1], math.radians(cam_pose[2]))
                if cam_pose else None)
        right = panel.render(pose=pose,
                             obs_map=[{"x": o.x, "y": o.y} for o in obs],
                             title=f"field map ({fmap.version})",
                             status_lines=status_lines(res, cam_pose, None, None, fmap))
        out = compose(left, right)
        if args.save:
            cv2.imwrite(args.save, out)
            print("saved:", args.save)
        if not args.no_show:
            cv2.imshow("localize", out)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        return

    # ---------------- live camera ----------------
    cap = cv2.VideoCapture(args.video) if args.video else \
        cv2.VideoCapture(args.camera, cv2.CAP_DSHOW)
    if not cap.isOpened():
        raise SystemExit(f"cannot open camera index={args.camera}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, 30)
    if args.exposure is not None:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)   # manual
        cap.set(cv2.CAP_PROP_EXPOSURE, args.exposure)
    if args.gain is not None:
        cap.set(cv2.CAP_PROP_GAIN, args.gain)
    print(f"exposure={cap.get(cv2.CAP_PROP_EXPOSURE):.1f} "
          f"gain={cap.get(cv2.CAP_PROP_GAIN):.1f}")
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"camera index={args.camera}, {W}x{H}, {cap.get(cv2.CAP_PROP_FPS):.1f} fps")

    writer = None
    if args.save_video:
        writer = cv2.VideoWriter(args.save_video, cv2.VideoWriter_fourcc(*"mp4v"), 20,
                                 (1280 + panel.W, panel.H))
    fcsv = fwriter = None
    if args.csv:
        fcsv = open(args.csv, "w", newline="", encoding="utf-8")
        fwriter = csv.writer(fcsv)
        fwriter.writerow(["frame", "t", "x_carrier", "y_carrier", "yaw_carrier",
                          "x_camera", "y_camera", "yaw_camera", "valid", "status",
                          "markers", "residual", "n_obs", "fps"])

    smooth = None if args.no_smooth else Smoother()
    prior = tuple(args.init_pose) if args.init_pose else None
    prev_pose = None
    frame_no = 0
    fps = 0.0
    last = time.time()
    upd_t = []
    while True:
        ok, frame = cap.read()
        if not ok:
            print("read frame failed")
            break
        frame_no += 1
        t0 = time.perf_counter()
        dets, obs, nfb = detect_and_observations(frame, model, proj, args)
        res = locator.locate(obs, prior_pose=prior)
        infer_ms = (time.perf_counter() - t0) * 1000

        carrier = (res.x, res.y, res.yaw_deg) if res.valid else None
        smooth_pose = smooth.update(res) if smooth else carrier
        # 位姿突变告警：这块场地是 180 度对称的，出现"跳几米"通常是镜像错解
        if smooth_pose and prev_pose:
            jump = math.hypot(smooth_pose[0] - prev_pose[0],
                              smooth_pose[1] - prev_pose[1])
            if jump > 2.0:
                print(f"[warn] frame={frame_no} 位姿突跳 {jump:.1f} m "
                      f"({prev_pose[0]:.1f},{prev_pose[1]:.1f}) -> "
                      f"({smooth_pose[0]:.1f},{smooth_pose[1]:.1f})  "
                      f"— 可能是镜像错解，慢一点走 / 转个角度重新锁定")
        if smooth_pose:
            prev_pose = smooth_pose
        cam_pose = to_camera(smooth_pose, Ox, Oy, yaw_off) if smooth_pose else None
        pose = ((cam_pose[0], cam_pose[1], math.radians(cam_pose[2]))
                if cam_pose else None)
        if cam_pose is not None:
            prior = cam_pose          # prior for the next frame

        now = time.time()
        dt = max(1e-6, now - last)
        last = now
        fps = (1.0 / dt) if fps == 0 else 0.9 * fps + 0.1 * (1.0 / dt)
        if pose is not None:
            upd_t.append(now)
        while upd_t and now - upd_t[0] > 2.0:
            upd_t.pop(0)
        hz = len(upd_t) / 2.0

        left = draw_left(frame, dets, res, f"frame={frame_no}", fps, infer_ms)
        right = panel.render(pose=pose,
                             obs_map=[{"x": o.x, "y": o.y} for o in obs],
                             title=f"field map ({fmap.version})",
                             status_lines=status_lines(res, cam_pose, fps, hz, fmap))
        canvas = compose(left, right)

        if fwriter:
            cp = cam_pose or (0, 0, 0)
            fwriter.writerow([frame_no, f"{now:.3f}",
                              f"{res.x:.3f}", f"{res.y:.3f}", f"{res.yaw_deg:.2f}",
                              f"{cp[0]:.3f}", f"{cp[1]:.3f}", f"{cp[2]:.2f}",
                              int(res.valid), res.status, res.marker_count,
                              f"{res.residual:.3f}", len(obs), f"{fps:.1f}"])
        if writer:
            writer.write(canvas)
        if frame_no % 15 == 0:
            print(f"frame={frame_no} fps={fps:.1f} infer={infer_ms:.1f}ms det={len(dets)} "
                  f"obs={len(obs)} {res.status} markers={res.marker_count} "
                  f"res={res.residual:.2f} pose=({res.x:.2f},{res.y:.2f},{res.yaw_deg:.1f})")
        if not args.no_show:
            cv2.imshow("localize : left=RGB  right=field map", canvas)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if key == ord("p"):        # 按 p 记录当前位姿（每个测试点按一次）
                sp = smooth_pose or (res.x, res.y, res.yaw_deg)
                snap = ROOT / "pose_snapshots.csv"
                new = not snap.exists()
                with open(snap, "a", newline="", encoding="utf-8") as f:
                    w = csv.writer(f)
                    if new:
                        w.writerow(["t", "frame", "x", "y", "yaw", "status",
                                    "markers", "residual", "n_obs"])
                    w.writerow([f"{now:.1f}", frame_no, f"{sp[0]:.3f}", f"{sp[1]:.3f}",
                                f"{wrap_deg(sp[2]):.2f}", res.status, res.marker_count,
                                f"{res.residual:.3f}", len(obs)])
                print(f"[snapshot] frame={frame_no}  x={sp[0]:.3f} y={sp[1]:.3f} "
                      f"yaw={wrap_deg(sp[2]):.2f}  {res.status}  -> {snap}")

    cap.release()
    if writer:
        writer.release()
    if fcsv:
        fcsv.close()
    cv2.destroyAllWindows()


# ----------------------------------------------------------------------
def draw_left(frame, dets, res, tag, fps, infer_ms):
    img = frame.copy()
    fps_s = "--" if fps is None else f"{fps:.1f}"
    inf_s = "--" if infer_ms is None else f"{infer_ms:.1f}"
    by_idx = {inf["obs_index"]: inf for inf in res.inliers}
    oi = 0
    for d in dets:
        x1, y1, x2, y2 = [int(round(v)) for v in d["box"]]
        color = TYPE_COLORS.get(d["type"], (0, 255, 255))
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        ref = d["ref"]
        cu, cv_ = int((x1 + x2) / 2), int((y1 + y2) / 2)
        cv2.drawMarker(img, (cu, cv_), (0, 0, 255), cv2.MARKER_CROSS, 8, 1)
        if ref["ok"]:
            p = (int(ref["u"]), int(ref["v"]))
            cv2.circle(img, p, 4, (0, 255, 255), -1)
            cv2.line(img, (cu, cv_), p, (0, 255, 255), 1)
        g = d["ground"]
        lab = f"{d['type']} {d['conf']:.2f}"
        if g is not None:
            lab += f" ({g['x']:.2f},{g['y']:.2f})m"
            inf = by_idx.get(oi)
            if inf:
                lab += f" ->{inf['map_id']}"
            oi += 1
        cv2.putText(img, lab, (max(2, x1), max(16, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    draw_hud(img, [f"{tag}  FPS={fps_s}  infer={inf_s}ms  conf-th",
                   f"det={len(dets)}  " + res.status],
             origin=(8, 22), scale=0.5)
    return img


def status_lines(res, cam_pose, fps, hz, fmap):
    sp = cam_pose or (res.x, res.y, res.yaw_deg)
    flag = "" if res.valid else "!"
    fps_s = "--" if fps is None else f"{fps:.1f}"
    hz_s = "--" if hz is None else f"{hz:.1f}"
    return [
        f"{flag}x_camera_map  = {sp[0]:7.2f} m",
        f"{flag}y_camera_map  = {sp[1]:7.2f} m",
        f"{flag}yaw_camera_map= {wrap_deg(sp[2]):7.1f} deg",
        f"{flag}valid={res.valid}  status={res.status}",
        f"{flag}markers={res.marker_count}  residual={res.residual:.2f} m",
        f"fps={fps_s}  update={hz_s} Hz  map={fmap.version}",
    ]


if __name__ == "__main__":
    main()