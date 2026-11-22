# 任务2 主程序：检测白线角点，解算相机在场地坐标系的位姿
import os
import sys
import time

import cv2
import numpy as np
import yaml
from ultralytics import YOLO

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "detect"))
from stream_server import StreamServer

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
CFG_PATH = os.path.normpath(os.path.join(HERE, "..", "..", "config", "markers_c0.yaml"))
if not os.path.exists(CFG_PATH):
    raise FileNotFoundError(f"找不到配置文件 {CFG_PATH}")

CFG = yaml.safe_load(open(CFG_PATH, encoding="utf-8"))
CORNERS = CFG["corners"]
FIELD = CFG["field"]

_ARGS = [a for a in sys.argv[1:] if not a.startswith("--")]
SRC = _ARGS[0] if _ARGS else "http://localhost:8000/stream"
SAVE = "--save" in sys.argv
MODEL = os.environ.get("K1_MODEL") or os.path.join(
    ROOT, "runs", "detect", "corner11", "weights", "best.pt")
PORT = 8002
OUT_VIDEO = os.path.join(ROOT, "outputs", "localize.mp4")

CONF = 0.40
MIN_MARKERS = 2
RESIDUAL_MAX = 8.0
LOST_AFTER = 5
JUMP_MAX = 2.5
OWNER = "chenjialiang"
REFINE = True
REPLAY = 1
SMOOTH = 0.35
DUP_DIST = 60.0

PANEL_W = 380
PANEL_H = 720
DISP = 0.75
OUT_W, OUT_H = int((1280 + PANEL_W) * DISP), int(PANEL_H * DISP)

CAM_HEIGHT = CFG.get("camera", {}).get("height_m", 1.07)
PITCH_DEG = CFG.get("camera", {}).get("pitch_deg", 20.0)
MAP_VER = "field.yaml v2026-11-08"
DIST_K1 = 0.282308

# 内参按 1280x720 标定，改分辨率会全部失效
K = np.array([
    [1653.274, 0.0, 646.380],
    [0.0, 1642.710, 307.337],
    [0.0, 0.0, 1.0],
], dtype=np.float64)
DIST = np.array([0.282308, -2.161663, -0.004933, 0.003390, 3.708067])

def solve_pose(pts_3d, pts_2d):
    pts_2d = np.asarray(pts_2d, dtype=np.float64).reshape(-1, 1, 2)
    try:
        n, rvecs, tvecs, errors = cv2.solvePnPGeneric(
            np.asarray(pts_3d, dtype=np.float64), pts_2d, K, DIST,
            flags=cv2.SOLVEPNP_IPPE)
    except cv2.error:
        return None, None, 999.0

    best = None
    for i in range(n):
        R, _ = cv2.Rodrigues(rvecs[i])
        C = (-R.T @ tvecs[i]).flatten()
        if C[2] <= 0:
            continue
        err = float(errors[i]) if errors is not None else 0.0
        if best is None or err < best[2]:
            best = (rvecs[i], tvecs[i], err)

    if best is None:
        return None, None, 999.0
    return best

def pose_to_xy_yaw(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    C = (-R.T @ tvec).flatten()
    axis = (R.T @ np.array([[0.0], [0.0], [1.0]])).flatten()
    yaw = np.degrees(np.arctan2(axis[1], axis[0]))
    return float(C[0]), float(C[1]), float(yaw)

def pixel_to_body(u, v, pitch, h):
    x = (u - K[0, 2]) / K[0, 0]
    y = (v - K[1, 2]) / K[1, 1]
    d_cam = np.array([x, y, 1.0])
    f = np.array([np.cos(pitch), 0.0, -np.sin(pitch)])
    r = np.array([0.0, -1.0, 0.0])
    dn = np.cross(f, r)
    R = np.vstack([r, dn, f])
    d_world = R.T @ d_cam
    if d_world[2] >= -1e-6:
        return None
    lam = -h / d_world[2]
    p = lam * d_world
    return np.array([p[0], p[1]])

def pixel_depth(u, v, pitch, h):
    p = pixel_to_body(u, v, pitch, h)
    return None if p is None else float(p[1])

def solve_pose_height(pts_3d, pts_2d, h, pitch_deg):
    # 2 点残差恒为 0，不能靠残差判对错
    P = np.asarray(pts_3d, dtype=np.float64)[:, :2]
    pts_2d = np.asarray(pts_2d, dtype=np.float64)
    pitch = np.radians(pitch_deg)

    body = []
    for u, v in pts_2d:
        p = pixel_to_body(u, v, pitch, h)
        if p is None:
            return None, None, 999.0
        body.append(p)
    B = np.array(body)

    bc = B - B.mean(axis=0)
    wc = P - P.mean(axis=0)
    num = np.sum(bc[:, 0] * wc[:, 1] - bc[:, 1] * wc[:, 0])
    den = np.sum(bc[:, 0] * wc[:, 0] + bc[:, 1] * wc[:, 1])
    yaw = np.arctan2(num, den)
    c, s = np.cos(yaw), np.sin(yaw)
    R2 = np.array([[c, -s], [s, c]])
    C = P.mean(axis=0) - R2 @ B.mean(axis=0)
    err = np.sqrt(np.mean(np.sum((R2 @ B.T).T + C - P, axis=1) ** 2))
    return (float(C[0]), float(C[1])), float(np.degrees(yaw)), float(err)

def corner_xy(name, front):
    if name in ("goalarea_far_left", "goalarea_far_right",
                "penalty_far_left", "penalty_far_right"):
        is_left = name.endswith("left")
        gx = CORNERS[name][0] * (1 if front else -1)
        gy = abs(CORNERS[name][1]) * (1 if is_left else -1) * (1 if front else -1)
        return gx, gy
    return CORNERS[name][0], CORNERS[name][1]

def refine_pixel(gray, x1, y1, x2, y2, cx, cy):
    pad = 4
    h, w = gray.shape[:2]
    xa, ya = max(0, x1 - pad), max(0, y1 - pad)
    xb, yb = min(w, x2 + pad), min(h, y2 + pad)
    roi = gray[ya:yb, xa:xb]
    if roi.size < 400:
        return cx, cy, False
    pts = cv2.goodFeaturesToTrack(roi, maxCorners=12, qualityLevel=0.05,
                                  minDistance=6)
    if pts is None or len(pts) == 0:
        return cx, cy, False
    pts = cv2.cornerSubPix(roi, pts, (5, 5), (-1, -1),
                           (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                            20, 0.05))
    pts = pts.reshape(-1, 2)
    pts[:, 0] += xa
    pts[:, 1] += ya
    d = np.hypot(pts[:, 0] - cx, pts[:, 1] - cy)
    i = int(np.argmin(d))
    return float(pts[i, 0]), float(pts[i, 1]), True

def draw_field_panel(w, h, cam_xy=None, yaw=None, info="", stale=False):
    panel = np.full((h, w, 3), 28, np.uint8)
    TOP = 232
    usable_h = h - TOP - 20
    usable_w = w - 40
    scale = min(usable_h / FIELD["length_x"], usable_w / FIELD["width_y"])
    cx = w // 2
    cy = TOP + usable_h // 2

    def to_px(x, y):
        return int(cx - y * scale), int(cy - x * scale)

    lx = FIELD["length_x"] / 2
    ly = FIELD["width_y"] / 2
    white = (200, 200, 200)

    cv2.rectangle(panel, to_px(-lx, ly), to_px(lx, -ly), white, 2)
    cv2.line(panel, to_px(0, ly), to_px(0, -ly), white, 1)
    cv2.circle(panel, to_px(0, 0), max(3, int(FIELD["center_circle_radius"] * scale)), white, 1)
    for sx in (-1, 1):
        px = FIELD["penalty_far_x"] * sx
        hy = FIELD["penalty_half_y"]
        cv2.rectangle(panel, to_px(px, hy), to_px(lx * sx, -hy), white, 1)
        gx = FIELD["goalarea_far_x"] * sx
        gy = FIELD["goalarea_half_y"]
        cv2.rectangle(panel, to_px(gx, gy), to_px(lx * sx, -gy), white, 1)

    gp = to_px(lx, FIELD["width_y"] / 2 * 0.45)
    gp2 = to_px(lx, -FIELD["width_y"] / 2 * 0.45)
    cv2.line(panel, gp, gp2, (90, 190, 250), 4)

    for name, (fx, fy) in CORNERS.items():
        cv2.circle(panel, to_px(fx, fy), 2, (110, 110, 110), -1)

    if cam_xy is None:
        cv2.putText(panel, "NO POSE", (w // 2 - 46, cy + 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (60, 60, 230), 2)
    if cam_xy is not None:
        p = to_px(cam_xy[0], cam_xy[1])
        dot_col = (0, 140, 255) if stale else (0, 220, 255)
        cv2.circle(panel, p, 7, dot_col, -1)
        if stale:
            cv2.putText(panel, "HOLD", (p[0] + 12, p[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, dot_col, 2)
        if yaw is not None:
            rad = np.deg2rad(yaw)
            tip = (int(p[0] - 34 * np.sin(rad)), int(p[1] - 34 * np.cos(rad)))
            cv2.arrowedLine(panel, p, tip, (0, 220, 255), 2, tipLength=0.3)

    ax, ay = 34, TOP + 34
    cv2.arrowedLine(panel, (ax, ay), (ax, ay - 26), (120, 200, 120), 2, tipLength=0.35)
    cv2.putText(panel, "+X", (ax - 8, ay - 32), cv2.FONT_HERSHEY_SIMPLEX,
                0.42, (120, 200, 120), 1)
    cv2.arrowedLine(panel, (ax, ay), (ax - 26, ay), (120, 200, 120), 2, tipLength=0.35)
    cv2.putText(panel, "+Y", (ax - 46, ay + 6), cv2.FONT_HERSHEY_SIMPLEX,
                0.42, (120, 200, 120), 1)

    y0 = 26
    for line in info.split("\n"):
        cv2.putText(panel, line, (10, y0), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (220, 220, 220), 1)
        y0 += 22
    return panel

def main():
    model = YOLO(MODEL) if os.path.exists(MODEL) else None
    if model is None:
        print(f"[警告] 没找到模型 {MODEL}，先跑训练。当前只画画面不定位。")

    cap = cv2.VideoCapture(SRC)
    if not cap.isOpened():
        print(f"打不开输入源 {SRC}")
        sys.exit(1)

    srv = StreamServer(port=PORT).start()

    writer = None
    log = None
    if SAVE:
        os.makedirs(os.path.dirname(OUT_VIDEO), exist_ok=True)
        writer = cv2.VideoWriter(OUT_VIDEO, cv2.VideoWriter_fourcc(*"mp4v"),
                                 20, (OUT_W, OUT_H))
        print(f"录制到 {OUT_VIDEO}")
        tag = os.path.splitext(os.path.basename(SRC))[0] or "stream"
        log_path = os.path.join(ROOT, "outputs", f"localize_log_{tag}.csv")
        log = open(log_path, "w", encoding="utf-8")
        log.write("frame,x,y,yaw,markers,residual,valid,status\n")
        print(f"逐帧结果 -> {log_path}")

    fps = 0.0
    frame_id = 0
    miss_count = 0
    stale = False
    t_prev = time.time()
    last_pose = None

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                if SRC.startswith("http"):
                    continue
                if REPLAY == 1:
                    print("\n视频已跑完一遍，退出")
                    break
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue

            if frame.shape[1] != 1280 or frame.shape[0] != 720:
                frame = cv2.resize(frame, (1280, 720))
            left = frame.copy()

            dets = []
            if model is not None:
                res = model(frame, conf=CONF, verbose=False)[0]
                raw = []
                for box in res.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    name = model.names[int(box.cls[0])]
                    raw.append([name, (x1 + x2) / 2.0, (y1 + y2) / 2.0,
                                float(box.conf[0]), x1, y1, x2, y2])
                raw.sort(key=lambda r: -r[3])
                for r in raw:
                    dup = False
                    for q in dets:
                        if r[0] == q[0] and (r[1] - q[1]) ** 2 + (r[2] - q[2]) ** 2 < DUP_DIST ** 2:
                            dup = True
                            break
                    if not dup:
                        dets.append(r)
                goal_hint = any(d[0] == "goal" for d in dets)
                dets = [d for d in dets if d[0] in CORNERS]

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if REFINE else None
                for r in dets:
                    if REFINE and r[0] in CORNERS:
                        nx, ny, ok_ref = refine_pixel(gray, r[4], r[5], r[6], r[7], r[1], r[2])
                        if ok_ref:
                            r[1], r[2] = nx, ny

                for name, cx, cy, conf, x1, y1, x2, y2 in dets:
                    cv2.rectangle(left, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.drawMarker(left, (int(cx), int(cy)), (0, 0, 255),
                                   cv2.MARKER_CROSS, 10, 2)
                    cv2.putText(left, f"{name} {conf:.2f}",
                                (x1, max(y1 - 6, 14)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    mx, my = CORNERS[name]
                    cv2.putText(left, f"px({int(cx)},{int(cy)}) -> map({mx:.2f},{my:.2f})",
                                (x1, min(y2 + 16, 706)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 120), 1)

            status = "insufficient_markers"
            stale = False
            marker_count = len(dets)
            residual = -1.0
            cam_xy, yaw = None, None

            known_all = [d for d in dets if d[0] in CORNERS]

            FAM_ORDER = {"goalarea_far": 0, "penalty_far": 1,
                         "corner_front": 2, "corner_back": 3}

            # 混入同侧两点会让构型不正定，只取一个家族
            def pick_pair(items):
                fams = {}
                for d in items:
                    nm = d[0]
                    side = "L" if nm.endswith("_left") else "R"
                    fam = nm[:-5] if nm.endswith("_left") else nm[:-6]
                    fams.setdefault(fam, {})[side] = d

                best, best_key = [], (-1, -1)
                for fam, sides in fams.items():
                    has2 = 1 if ("L" in sides and "R" in sides) else 0
                    key = (has2, len(sides))
                    if key > best_key:
                        best_key = key
                        best = list(sides.values())
                return best

            known = pick_pair(known_all)
            goal_seen = goal_hint

            if len(known) >= MIN_MARKERS:
                cands = []
                for front in (True, False):
                    p3h = [[*corner_xy(d[0], front), 0.0] for d in known]
                    p2h = [[d[1], d[2]] for d in known]
                    if len(known) >= 4:
                        rv, tv, rs = solve_pose(p3h, p2h)
                        if rv is None:
                            continue
                        hx, hy, hyaw = pose_to_xy_yaw(rv, tv)
                    else:
                        hxy, hyaw, rs = solve_pose_height(p3h, p2h, CAM_HEIGHT, PITCH_DEG)
                        if hxy is None:
                            continue
                        hx, hy = hxy
                    mwh = np.mean(np.asarray(p3h, dtype=np.float64), axis=0)[:2]
                    bearing_h = np.degrees(np.arctan2(mwh[1] - hy, mwh[0] - hx))
                    if abs((hyaw - bearing_h + 180.0) % 360.0 - 180.0) > 30.0:
                        continue
                    cands.append((front, (hx, hy), hyaw, rs))

                hl, hw = FIELD["length_x"] / 2, FIELD["width_y"] / 2

                def rank(c):
                    fr, xy_h, _, _ = c
                    inside = 0 if (abs(xy_h[0]) <= hl and abs(xy_h[1]) <= hw) else 1
                    dist = 0.0 if last_pose is None else (
                        (xy_h[0] - last_pose[0][0]) ** 2 + (xy_h[1] - last_pose[0][1]) ** 2)
                    return (inside, dist)

                if cands and last_pose is None and goal_seen:
                    cf = [c for c in cands if c[0]]
                    if cf:
                        cands = cf

                if cands:
                    best = min(cands, key=rank)
                    used_front, cam_xy, yaw, residual = best
                    p3 = [[*corner_xy(d[0], used_front), 0.0] for d in known]

                    if len(known) == 2 and last_pose is not None:
                        gap = np.hypot(cam_xy[0] - last_pose[0][0],
                                       cam_xy[1] - last_pose[0][1])
                        if gap > 1.5:
                            status = "jump_rejected"

                    if p3 is not None:
                        if len(known) >= 4:
                            status = "ok" if residual <= RESIDUAL_MAX else "high_residual"
                        else:
                            status = "ok" if residual <= 1.0 else "high_residual"
                else:
                    p3 = None
                    status = "no_valid_hypothesis"

            if cam_xy is not None:
                lx = FIELD["length_x"] / 2 + 1.0
                ly = FIELD["width_y"] / 2 + 1.0
                if abs(cam_xy[0]) > lx or abs(cam_xy[1]) > ly:
                    status = "out_of_field"
                    cam_xy, yaw = None, None

            if cam_xy is not None and last_pose is not None:
                jx, jy = cam_xy
                px_, py_ = last_pose[0]
                if np.hypot(jx - px_, jy - py_) > JUMP_MAX:
                    status = "jump_rejected"
                    cam_xy, yaw = None, None
                else:
                    a = SMOOTH
                    cam_xy = (a * jx + (1 - a) * px_, a * jy + (1 - a) * py_)
                    dy = (yaw - last_pose[1] + 180.0) % 360.0 - 180.0
                    yaw = (last_pose[1] + a * dy + 180.0) % 360.0 - 180.0

            if status == "ok":
                miss_count = 0
                last_pose = (cam_xy, yaw)
            else:
                miss_count += 1
                if last_pose is not None and miss_count <= LOST_AFTER:
                    cam_xy, yaw = last_pose
                    stale = True
                    status = status + f" (hold {miss_count})"
                else:
                    last_pose = None
                    cam_xy, yaw = None, None

            now = time.time()
            dt = now - t_prev
            if dt > 0:
                fps = 0.9 * fps + 0.1 / dt
            t_prev = now

            valid = 1 if (status == "ok" and cam_xy is not None) else 0
            head = (f"map {MAP_VER}\n"
                    f"fx {K[0, 0]:.0f} fy {K[1, 1]:.0f} k1 {DIST_K1:+.2f}\n"
                    f"h {CAM_HEIGHT:.2f} m  pitch {PITCH_DEG:.1f} deg\n")
            if cam_xy:
                info = (head +
                        f"x = {cam_xy[0]:+.2f} m\n"
                        f"y = {cam_xy[1]:+.2f} m\n"
                        f"yaw = {yaw:+.1f} deg\n"
                        f"markers = {marker_count}\n"
                        f"residual = {residual:.2f}\n"
                        f"valid = {valid}\n"
                        f"status = {status}")
            else:
                info = (head +
                        "x = --\n"
                        "y = --\n"
                        "yaw = --\n"
                        f"markers = {marker_count}\n"
                        f"residual = {residual:.2f}\n"
                        f"valid = {valid}\n"
                        f"status = {status}")

            right = draw_field_panel(PANEL_W, PANEL_H, cam_xy, yaw, info, stale)

            canvas = np.hstack([left, right])
            cv2.putText(canvas, f"FPS {fps:.1f}", (10, 700),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            hud = f"{OWNER}   {time.strftime('%Y-%m-%d %H:%M:%S')}"
            (hw_, _), _ = cv2.getTextSize(hud, cv2.FONT_HERSHEY_SIMPLEX, 0.62, 2)
            cv2.putText(canvas, hud, (canvas.shape[1] - hw_ - 14, canvas.shape[0] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 255), 2)
            if DISP != 1.0:
                canvas = cv2.resize(canvas, (OUT_W, OUT_H))

            srv.push(canvas)
            if writer is not None:
                writer.write(canvas)
            if log is not None:
                if cam_xy is not None:
                    log.write(f"{frame_id},{cam_xy[0]:.4f},{cam_xy[1]:.4f},"
                              f"{yaw:.2f},{marker_count},"
                              f"{residual:.3f},{valid},{status}\n")
                else:
                    log.write(f"{frame_id},,,,{marker_count},"
                              f"{residual:.3f},{valid},{status}\n")
                frame_id += 1

    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if writer is not None:
            writer.release()
            print(f"录制完成 {OUT_VIDEO}")
        if log is not None:
            log.close()
        srv.stop()

if __name__ == "__main__":
    main()
