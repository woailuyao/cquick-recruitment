#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
localize.py —— 足球场标志 单目自定位 (v2)
链路: YOLO检测 -> 几何细化(框中心->交点) -> 去畸变 -> 射线打地面 -> SE(2)匹配 -> (x,y,yaw)
      + 左画面/右俯视图 分屏可视化

用法:
  /root/yolo_env/bin/python localize.py --sim                       # 合成自测(先跑)
  /root/yolo_env/bin/python localize.py --img 图片.jpg               # 单张(生成 _loc.jpg 分屏图)
  /root/yolo_env/bin/python localize.py --video 视频.mp4 --save out.avi   # 逐帧(可选存分屏视频)

坐标约定: 地图系原点=球场中心, +X 指向球门, +Y=面向+X时的左侧; yaw 绕+Z逆时针为正。
"""
import argparse, math, os, re, sys
import numpy as np
import cv2
import yaml

# ---------------- 实测外参 ----------------
H_CM      = 0.728     # 相机光心离地高度(米)
PITCH_DEG = 15.0      # 俯角: 光轴在水平线"下方"15°
ROLL_DEG  = 5.0       # 横滚(现场若朝向反了就改成 -5.0)

INTRINSICS = os.path.expanduser("~/soccer_ws/camera_intrinsics.yaml")
MAP_FILE   = os.path.expanduser("~/soccer_ws/field_new.yaml")

_FIX3 = os.path.expanduser("~/field_train/markers_fix3/weights/best.pt")
_FIX4 = os.path.expanduser("~/field_train/markers_fix4/weights/best.pt")
_FIX5 = os.path.expanduser("~/field_train/markers_fix5/weights/best.pt")
# 默认优先用最新训的 fix5; 想指定就: export FIELD_MODEL=/path/to/best.pt
MODEL = os.environ.get("FIELD_MODEL",
                       _FIX5 if os.path.exists(_FIX5)
                       else (_FIX4 if os.path.exists(_FIX4) else _FIX3))

CLASSES = ["LCross", "TCross", "XCross", "Point"]   # 数据集 0/1/2/3
COLOR   = {"LCross": (0, 0, 255), "TCross": (0, 255, 0),
           "XCross": (255, 255, 0), "Point": (255, 0, 0)}   # BGR(与审计图一致的配色)


# ================= 内参 / 地图 / 外参 =================
def load_intrinsics(path=None):
    d = yaml.safe_load(open(path or INTRINSICS))
    W, H = int(d["image_width"]), int(d["image_height"])
    K = np.array([[d["fx"], 0, d["cx"]], [0, d["fy"], d["cy"]], [0, 0, 1]], dtype=np.float64)
    dist = np.array([d["k1"], d["k2"], d["p1"], d["p2"], d["k3"]], dtype=np.float64)
    return K, dist, W, H


def key_class(k):
    k = "Point" if "penal" in k.lower() else k
    return re.match(r"[A-Za-z]+", k).group(0)


def load_map(path=None):
    raw = yaml.safe_load(open(path or MAP_FILE))
    mp = {}
    for k, v in raw.items():
        mp.setdefault(key_class(k), []).append((float(v[0]), float(v[1])))
    return mp


def make_rotation(pitch_deg, roll_deg):
    a, r = math.radians(pitch_deg), math.radians(roll_deg)
    ca, sa = math.cos(a), math.sin(a)
    cr, sr = math.cos(r), math.sin(r)
    Rbc_noroll = np.array([[0, -sa, ca], [-1, 0, 0], [0, -ca, -sa]])
    Rz = np.array([[cr, -sr, 0], [sr, cr, 0], [0, 0, 1]])
    return Rbc_noroll @ Rz


R_BC = make_rotation(PITCH_DEG, ROLL_DEG)
R_CB = R_BC.T


# ================= 核心几何 =================
def undistort_direction(u, v, K, dist):
    out = cv2.undistortPoints(np.array([[[u, v]]], dtype=np.float32), K, dist, P=None)
    return np.array([out[0, 0, 0], out[0, 0, 1], 1.0])


def pixel_to_ground(px, py, K, dist):
    dc = undistort_direction(px, py, K, dist)
    db = R_BC @ dc
    if db[2] >= 0:
        return None
    t = -H_CM / db[2]
    return np.array([t * db[0], t * db[1]])


def ground_to_pixel(gx, gy, K, dist):
    Xc = R_CB @ (np.array([gx, gy, 0.0]) - np.array([0.0, 0.0, H_CM]))
    if Xc[2] <= 0:
        return None
    img, _ = cv2.projectPoints(np.array([Xc], dtype=np.float32).reshape(1, 1, 3),
                               np.zeros(3), np.zeros(3), K, dist)
    return float(img[0, 0, 0]), float(img[0, 0, 1])


# ================= 几何细化: 框中心 -> 真实交点 =================
def refine_intersection(gray, box, name):
    """在检测框附近用 Harris 角点细化到标志交点。返回 (u,v)。"""
    x1, y1, x2, y2 = [int(v) for v in box]
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    if name == "Point":              # 点球点是圆点, 框中心即圆心
        return cx, cy

    w, h = max(1, x2 - x1), max(1, y2 - y1)
    m = 0.30
    ww, hh = int(w * (1 + 2 * m)), int(h * (1 + 2 * m))
    x0 = max(0, min(int(round(cx - ww / 2)), gray.shape[1] - ww - 1))
    y0 = max(0, min(int(round(cy - hh / 2)), gray.shape[0] - hh - 1))
    roi = gray[y0:y0 + hh, x0:x0 + ww]
    corners = cv2.goodFeaturesToTrack(roi, maxCorners=4, qualityLevel=0.10, minDistance=4)
    if corners is None or len(corners) == 0:
        return cx, cy
    pts = corners.reshape(-1, 2)
    if name == "XCross":             # X 的交点 = 四周凹角点的几何中心
        p = pts[:4].mean(0)
    else:                            # L 的拐角 / T 的三通 = 离框中心最近的角点
        cc = np.array([ww / 2.0, hh / 2.0])
        p = pts[np.argmin(((pts - cc) ** 2).sum(1))]
    return x0 + float(p[0]), y0 + float(p[1])


# ================= SE(2) 匹配 =================
def rot2(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])


def se2_from_pairs(o1, o2, m1, m2):
    a = math.atan2(m2[1] - m1[1], m2[0] - m1[0]) - math.atan2(o2[1] - o1[1], o2[0] - o1[0])
    R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    return R, m1 - R @ o1


def fit_se2(src, dst):
    src, dst = np.asarray(src, float), np.asarray(dst, float)
    ms, md = src.mean(0), dst.mean(0)
    U, _, Vt = np.linalg.svd((src - ms).T @ (dst - md))
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1; R = Vt.T @ U.T
    return R, md - R @ ms


def match(obs, map_by_class, tol=0.20, iters=300, seed=1):
    obs = [o for o in obs if map_by_class.get(o[0])]
    if len(obs) < 2:
        return dict(x=0, y=0, yaw=0, marker_count=0, residual=1e9,
                    valid=False, status="insufficient", inliers=[])

    rng = np.random.default_rng(seed)
    best = None
    for _ in range(iters):
        i1, i2 = rng.choice(len(obs), size=2, replace=False)
        c1, c2 = map_by_class[obs[i1][0]], map_by_class[obs[i2][0]]
        m1 = c1[rng.integers(len(c1))]; m2 = c2[rng.integers(len(c2))]
        R, t = se2_from_pairs(np.array(obs[i1][1]), np.array(obs[i2][1]),
                              np.array(m1), np.array(m2))
        cnt, res, pairs = 0, 0.0, []
        for name, p in obs:
            bc = min(map_by_class[name], key=lambda m: float(np.linalg.norm(R @ p + t - m)))
            d = float(np.linalg.norm(R @ p + t - bc))
            if d < tol:
                cnt += 1; res += d; pairs.append((np.array(p), np.array(bc)))
        if best is None or cnt > best["cnt"] or (cnt == best["cnt"] and res < best["res"]):
            best = dict(cnt=cnt, res=res, pairs=pairs, R=R, t=t)

    if best["pairs"]:
        R, t = fit_se2([p[0] for p in best["pairs"]], [p[1] for p in best["pairs"]])
    else:
        R, t = best["R"], best["t"]

    res, pairs = 0.0, []
    for name, p in obs:
        bc = min(map_by_class[name], key=lambda m: float(np.linalg.norm(R @ p + t - m)))
        d = float(np.linalg.norm(R @ p + t - bc))
        if d < tol:
            res += d; pairs.append((name, tuple(bc)))
    n = len(pairs)
    if n: res /= n
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    if n < 2:
        valid, status = False, "insufficient"
    elif n < 3:
        valid, status = False, "ambiguous"
    elif res < tol:
        valid, status = True, "ok"
    else:
        valid, status = False, "poor"
    return dict(x=float(t[0]), y=float(t[1]), yaw=yaw, marker_count=n, residual=res,
                valid=valid, status=status, inliers=pairs, R=R, t=t)


# ================= YOLO 检测 =================
_model = None
def detect(image_bgr, conf=0.25, imgsz=960):
    global _model
    from ultralytics import YOLO
    if _model is None:
        _model = YOLO(MODEL)
    r = _model.predict(image_bgr, imgsz=imgsz, conf=conf, device="cpu", verbose=False)[0]
    return [dict(name=CLASSES[int(b.cls[0])], u=float((b.xyxy[0][0] + b.xyxy[0][2]) / 2),
                 v=float((b.xyxy[0][1] + b.xyxy[0][3]) / 2), conf=float(b.conf[0]),
                 box=tuple(b.xyxy[0].tolist())) for b in r.boxes]


def localize_image(img, K, dist, mp, W, H, conf=0.25, tol=0.20, refine=True, imgsz=960):
    if img.shape[0] != H or img.shape[1] != W:      # 保证和内参分辨率一致
        img = cv2.resize(img, (W, H))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    dets = detect(img, conf, imgsz=imgsz)
    obs = []
    for d in dets:
        u, v = d["u"], d["v"]
        if refine:
            u, v = refine_intersection(gray, d["box"], d["name"])
        g = pixel_to_ground(u, v, K, dist)
        d["pu"], d["pv"] = u, v
        if g is not None:
            d["ground"] = (float(g[0]), float(g[1]))
            obs.append((d["name"], d["ground"], float(d["conf"])))

    # —— 同类去重 + 按地图真实数量限流 ——
    # 1) 同一标志被重复框多次(地面点相距<0.20m)时只留置信度最高的一个;
    #    地图上同类标志最近间距 0.495m, 0.20m 阈值不会把两个真标志误并成一个
    # 2) 每类观测数不得超过地图上该类标志总数(场上只有1个Point, 检出4个必3个是假)
    limit = {k: len(v) for k, v in mp.items()}
    obs.sort(key=lambda t: -t[2])
    deduped, used = [], {}
    for name, g, c in obs:
        if used.get(name, 0) >= limit.get(name, 0):
            continue                          # 超出场上真实数量 -> 误检
        if any(n2 == name and math.hypot(g[0] - g2[0], g[1] - g2[1]) < 0.20
               for n2, g2, _ in deduped):
            continue                          # 与更高置信度的同类观测重合 -> 重复框
        deduped.append((name, g, c))
        used[name] = used.get(name, 0) + 1
    kept = [(n, g) for n, g, _ in deduped]

    sol = match(kept, mp, tol=tol)
    sol["dropped"] = len(obs) - len(kept)     # 被限流+去重丢掉的观测数
    # 边界校验(细则2.7): 解出的相机位置落在合理场地范围之外 -> 判无效, 不伪造 ok
    if sol["status"] != "insufficient":
        if not (FIELD["xmin"] <= sol["x"] <= FIELD["xmax"]
                and FIELD["ymin"] <= sol["y"] <= FIELD["ymax"]):
            sol["status"] = "out_of_field"
            sol["valid"] = False
    return sol, dets, kept


# ================= 分屏可视化 =================
FIELD = dict(xmin=-0.8, xmax=4.8, ymin=-3.6, ymax=3.6, size=720)

def _m2p(x, y):
    sx = FIELD["size"] / (FIELD["xmax"] - FIELD["xmin"])
    sy = FIELD["size"] / (FIELD["ymax"] - FIELD["ymin"])
    return int(round((x - FIELD["xmin"]) * sx)), int(round((FIELD["ymax"] - y) * sy))


def draw_camera_view(img, dets, sol):
    out = img.copy()
    for d in dets:
        x1, y1, x2, y2 = [int(v) for v in d["box"]]
        cv2.rectangle(out, (x1, y1), (x2, y2), COLOR[d["name"]], 2)
        cv2.circle(out, (int(round(d["pu"])), int(round(d["pv"]))), 3, (0, 0, 255), -1)
        cv2.putText(out, f"{d['name']} {d['conf']:.2f}", (x1, y1 - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, COLOR[d["name"]], 2)
    cv2.putText(out, f"x={sol['x']:.2f} y={sol['y']:.2f} yaw={sol['yaw']:.1f} [{sol['status']}]",
                (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    return out


def draw_map_view(mp, sol, obs):
    S = FIELD["size"]
    canvas = np.full((S, S, 3), (42, 42, 48), np.uint8)
    cv2.rectangle(canvas, _m2p(0, 2.995), _m2p(3.962, -2.995), (200, 200, 200), 1)  # 场地边框
    for name, pts in mp.items():
        for m in pts:
            cv2.circle(canvas, _m2p(*m), 6, COLOR[name], -1)
            cv2.putText(canvas, name[0], (_m2p(*m)[0] + 8, _m2p(*m)[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR[name], 1)
    if sol["status"] in ("insufficient", "out_of_field"):   # 无解/出界时不画假箭头(诚实显示)
        cv2.putText(canvas, f"[{sol['status']}] marker_count={sol['marker_count']}",
                    (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        return canvas
    R = rot2(sol["yaw"]); t = np.array([sol["x"], sol["y"]])
    for name, g in obs:                                    # 检测点(机器人系)转到地图系
        p = R @ np.array(g) + t
        cv2.circle(canvas, _m2p(*p), 4, (255, 255, 255), 1)
    px, py = _m2p(sol["x"], sol["y"])                     # 机器人箭头
    hx, hy = _m2p(sol["x"] + 0.9 * math.cos(math.radians(sol["yaw"])),
                  sol["y"] + 0.9 * math.sin(math.radians(sol["yaw"])))
    cv2.arrowedLine(canvas, (px, py), (hx, hy), (0, 0, 255), 3, tipLength=0.3)
    cv2.putText(canvas, f"x={sol['x']:.2f} y={sol['y']:.2f} yaw={sol['yaw']:.1f}\n"
                        f"match={sol['marker_count']} res={sol['residual']:.2f} {sol['status']}",
                (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return canvas


def combine(cam_view, map_view):
    s = map_view.shape[0]
    scale = s / cam_view.shape[0]
    cam = cv2.resize(cam_view, (int(cam_view.shape[1] * scale), s))
    pad = 12
    bar = np.full((s, pad, 3), (255, 255, 255), np.uint8)
    return np.hstack([cam, bar, map_view])


# ================= 合成自测 =================
def sim(K, dist, W, H, mp):
    print(f"[几何] 光轴打到地面的水平距离 = {H_CM/math.tan(math.radians(PITCH_DEG)):.2f} m\n")

    print("[测试1] 几何往返(地面点->像素->反投影, 只测画面内点)")
    worst, n_ok = 0.0, 0
    for (px0, py0, pw0) in [(2.0, -1.5, 0.0), (2.5, 0.0, 15.0), (2.0, 0.5, -10.0)]:
        t0 = np.array([px0, py0]); R0 = rot2(pw0)
        for name, pts in mp.items():
            for m in pts:
                pr = R0.T @ (np.array(m) - t0)
                q = ground_to_pixel(pr[0], pr[1], K, dist)
                if q is None: continue
                u, v = q
                if not (0 <= u < W and 0 <= v < H): continue
                g = pixel_to_ground(u, v, K, dist)
                if g is None: continue
                worst = max(worst, float(np.linalg.norm(g - pr))); n_ok += 1
    print(f"  画面内 {n_ok} 个点, 最大往返误差 = {worst*1000:.2f} mm  "
          f"{'✅' if worst < 0.02 else '❌'}\n")

    print("[测试2] 位姿恢复(源码观测加噪声+丢2点)")
    for (x0, y0, yaw0) in [(2.0, -1.5, 30.0), (1.5, -1.0, -60.0), (2.0, 0.0, 10.0)]:
        t0 = np.array([x0, y0]); R0 = rot2(yaw0)
        obs = []
        for name, pts in mp.items():
            for m in pts:
                pr = R0.T @ (np.array(m) - t0)
                obs.append((name, (float(pr[0]), float(pr[1]))))
        rng = np.random.default_rng(3)
        rng.shuffle(obs); obs = obs[:-2]
        obs = [(n, (p[0] + rng.normal(0, 0.02), p[1] + rng.normal(0, 0.02))) for n, p in obs]
        sol = match(obs, mp)
        ex, ey = sol["x"] - x0, sol["y"] - y0
        eyaw = (sol["yaw"] - yaw0 + 180) % 360 - 180
        good = abs(ex) < 0.05 and abs(ey) < 0.05 and abs(eyaw) < 2.0
        print(f"  真值({x0},{y0},{yaw0:+.0f}°) -> ({sol['x']:.2f},{sol['y']:.2f},{sol['yaw']:+.1f}°) "
              f"误差 x={ex:+.3f} y={ey:+.3f} yaw={eyaw:+.2f}° {'✅' if good else '❌'}")
    print("\n自测通过即可用真实图/视频验证。")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--img"); ap.add_argument("--video"); ap.add_argument("--save")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--tol", type=float, default=0.20)
    ap.add_argument("--norefine", action="store_true", help="关掉几何细化, 用框中心")
    args = ap.parse_args()

    K, dist, W, H = load_intrinsics()
    mp = load_map()

    if args.sim:
        print("地图特征:", {k: len(v) for k, v in mp.items()})
        sim(K, dist, W, H, mp); return
    if args.img:
        img = cv2.imread(args.img)
        if img is None:
            print("读图失败:", args.img); sys.exit(1)
        sol, dets, obs = localize_image(img, K, dist, mp, W, H,
                                        conf=args.conf, tol=args.tol,
                                        refine=not args.norefine)
        print(f"检测 {len(dets)} 个, 投地面 {len(obs)} 个 -> x={sol['x']:.3f} y={sol['y']:.3f} "
              f"yaw={sol['yaw']:.1f}° 匹配{sol['marker_count']} 残差{sol['residual']:.3f} "
              f"valid={sol['valid']} [{sol['status']}]")
        cam = draw_camera_view(img, dets, sol)
        out = combine(cam, draw_map_view(mp, sol, obs))
        op = os.path.splitext(args.img)[0] + "_loc.jpg"
        cv2.imwrite(op, out); print("分屏图已保存:", op); return

    if args.video:
        cap = cv2.VideoCapture(args.video)
        writer = None; fidx = 0
        while True:
            ok, img = cap.read()
            if not ok: break
            fidx += 1
            sol, dets, obs = localize_image(img, K, dist, mp, W, H,
                                            conf=args.conf, tol=args.tol,
                                            refine=not args.norefine)
            print(f"frame {fidx:4d}: x={sol['x']:.3f} y={sol['y']:.3f} yaw={sol['yaw']:.1f} "
                  f"n={sol['marker_count']} res={sol['residual']:.3f} "
                  f"valid={sol['valid']} [{sol['status']}]")
            if args.save:
                cam = draw_camera_view(img, dets, sol)
                fr = combine(cam, draw_map_view(mp, sol, obs))
                if writer is None:
                    writer = cv2.VideoWriter(args.save, cv2.VideoWriter_fourcc(*"XVID"),
                                             15.0, (fr.shape[1], fr.shape[0]))
                writer.write(fr)
        cap.release()
        if writer: writer.release()
        print("完成"); return

    ap.print_help()


if __name__ == "__main__":
    main()