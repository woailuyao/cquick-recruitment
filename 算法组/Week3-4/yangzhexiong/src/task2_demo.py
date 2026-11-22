"""任务 2 端到端演示（点击式）：像素 → 地面 → 地图匹配 → 位姿。

不依赖训练好的标志检测模型：由操作者手动点击画面里的场地标志并选择类型，
后续的去畸变、射线求交、地图匹配、位姿求解全部真实运行。

用法：
    python3 task2_demo.py --height 0.63 --pitch 20 --intr calibration/camera_intrinsics.yaml \
        --field config/field.yaml

操作：
    1/2/3/4   选择要标注的类型：1=LCross  2=TCross  3=XCross  4=PenaltyPoint
    鼠标左键  在画面里点击该标志的几何中心
    f         求解位姿（也可自动求解）
    c         清空所有点击
    g         开关地面网格
    s         存快照
    q         退出

右侧面板显示 field.yaml 俯视图、已匹配点位、相机位置与朝向。
"""
import argparse
import os

import cv2
import numpy as np
import yaml

import camera
import field_map as fm_mod
import pose_solver
from ground_projection import build_extrinsics, pixel_to_ground, project_to_image

TYPE_KEYS = {ord("1"): "LCross", ord("2"): "TCross", ord("3"): "XCross", ord("4"): "PenaltyPoint"}
COLORS = {"LCross": (0, 200, 255), "TCross": (255, 160, 0),
          "XCross": (0, 100, 255), "PenaltyPoint": (200, 0, 255)}


def render_map(fm, pose, obs, size=(640, 640)):
    W, H = size
    img = np.full((H, W, 3), 35, np.uint8)
    b = fm.field.get("boundary") or {}
    x_min, x_max = b.get("x_min", -4.0), b.get("x_max", 4.0)
    y_min, y_max = b.get("y_min", -3.0), b.get("y_max", 3.0)
    span_x = max(1e-6, x_max - x_min)
    span_y = max(1e-6, y_max - y_min)
    pad = 30
    sx = (W - 2 * pad) / span_y     # +Y 画到屏幕横向（向左）
    sy = (H - 2 * pad) / span_x     # +X 画到屏幕纵向（向上）

    def to_px(x, y):
        return (int(pad + (y_max - y) * sx), int(pad + (x_max - x) * sy))

    cv2.rectangle(img, to_px(x_min, y_max), to_px(x_max, y_min), (0, 140, 0), 2)
    for m in fm.markers:
        c = COLORS.get(m["type"], (200, 200, 200))
        p = to_px(m["x"], m["y"])
        cv2.drawMarker(img, p, c, cv2.MARKER_TILTED_CROSS, 10, 2)

    matched_ids = set()
    if pose.get("matches"):
        for mid in pose["matches"]:
            if mid:
                matched_ids.add(mid)
    for m in fm.markers:
        if m["id"] in matched_ids:
            p = to_px(m["x"], m["y"])
            cv2.circle(img, p, 7, (0, 255, 0), 2)

    if pose.get("valid"):
        p = to_px(pose["x"], pose["y"])
        th = np.radians(pose["yaw"])
        d = 34
        tip = (int(p[0] - d * np.sin(th)), int(p[1] - d * np.cos(th)))
        cv2.arrowedLine(img, p, tip, (0, 0, 255), 3, tipLength=0.35)
        cv2.circle(img, p, 5, (0, 0, 255), -1)

    cv2.putText(img, "map (top view)", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--height", type=float, required=True)
    ap.add_argument("--pitch", type=float, default=0.0)
    ap.add_argument("--roll", type=float, default=0.0)
    ap.add_argument("--yaw", type=float, default=0.0)
    ap.add_argument("--intr", default="calibration/camera_intrinsics.yaml")
    ap.add_argument("--field", default="config/field.yaml")
    args = ap.parse_args()

    for p in (args.intr, args.field):
        if not os.path.exists(p):
            print(f"找不到 {p}")
            return 1

    with open(args.intr, "r", encoding="utf-8") as f:
        cal = yaml.safe_load(f)
    cm, dc = cal["camera_matrix"], cal["dist_coeffs"]
    K = np.array([[cm["fx"], 0, cm["cx"]], [0, cm["fy"], cm["cy"]], [0, 0, 1]], float)
    dist = np.array([dc["k1"], dc["k2"], dc["p1"], dc["p2"], dc["k3"]], float)

    fm = fm_mod.load(args.field)
    problems, warns, _ = fm_mod.validate(fm)
    if problems:
        print("地图有错误，先用 field_map.py 修：")
        for p in problems:
            print("  ✗", p)
        return 1
    map_markers = [{"id": m["id"], "type": m["type"], "x": m["x"], "y": m["y"]} for m in fm.markers]

    R_bc, O_b = build_extrinsics(args.height, args.pitch, args.roll, args.yaw)
    cap, frame, info = camera.auto_open(verbose=False)
    h, w = frame.shape[:2]

    state = {"type": "LCross", "obs": [], "pixels": [], "pose": {}, "grid": False}

    def on_mouse(event, x, y, flags, _):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        res = pixel_to_ground(x, y, K, dist, R_bc, O_b)
        if res is None:
            print("  该像素没有有效地面交点")
            return
        X, Y, lam = res
        state["obs"].append({"type": state["type"], "x": X, "y": Y, "conf": 1.0})
        state["pixels"].append((x, y))
        print(f"  点击 ({x},{y}) → {state['type']} 地面坐标 X={X:.3f} Y={Y:.3f} (距离 {lam:.2f} m)")
        solve_now()

    def solve_now():
        res = pose_solver.solve(state["obs"], map_markers)
        state["pose"] = res
        if res["valid"]:
            print(f"  位姿: x={res['x']:.3f} y={res['y']:.3f} yaw={res['yaw']:.1f}°  "
                  f"markers={res['marker_count']} residual={res['residual']:.3f}")
        elif state["obs"]:
            print(f"  位姿无效: {res['status']} (可用标志 {res['marker_count']} 个)")

    win = "task2 demo  [1=L 2=T 3=X 4=P] click marker  c=clear  g=grid  s=snap  q=quit"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(win, on_mouse)

    while True:
        ok, frame = cap.read()
        if not ok:
            print("取帧失败")
            break
        canvas = frame.copy()

        if state["grid"]:
            for X in range(-4, 5):
                prev = None
                for Y in np.arange(-3.0, 3.01, 0.25):
                    p = project_to_image(float(X), float(Y), K, R_bc, O_b)
                    if p and 0 <= p[0] < w and 0 <= p[1] < h:
                        if prev:
                            cv2.line(canvas, prev, p, (80, 80, 80), 1)
                        prev = p
                    else:
                        prev = None

        for (px, py), o in zip(state["pixels"], state["obs"]):
            c = COLORS.get(o["type"], (255, 255, 255))
            cv2.drawMarker(canvas, (px, py), c, cv2.MARKER_CROSS, 16, 2)
            cv2.putText(canvas, f"{o['type'][0]}({o['x']:.2f},{o['y']:.2f})",
                        (px + 8, py - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 2)

        pose = state["pose"]
        hud1 = f"pick={state['type']}   points={len(state['obs'])}   h={args.height}m pitch={args.pitch}deg"
        cv2.rectangle(canvas, (0, 0), (w, 30), (0, 0, 0), -1)
        cv2.putText(canvas, hud1, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        if pose.get("valid"):
            hud2 = (f"POS x={pose['x']:.2f}m y={pose['y']:.2f}m yaw={pose['yaw']:.1f}deg "
                    f"markers={pose['marker_count']} res={pose['residual']:.3f}")
            color = (0, 255, 0)
        elif state["obs"]:
            hud2 = f"INVALID: {pose.get('status', '-')}  markers={pose.get('marker_count', 0)}"
            color = (0, 0, 255)
        else:
            hud2 = "点击场地标志开始（先按数字键选类型）"
            color = (200, 200, 200)
        cv2.rectangle(canvas, (0, 32), (w, 62), (0, 0, 0), -1)
        cv2.putText(canvas, hud2, (8, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        split = cv2.hconcat([canvas, render_map(fm, pose, state["obs"], (h, h))])
        cv2.imshow(win, split)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key in TYPE_KEYS:
            state["type"] = TYPE_KEYS[key]
            print(f"当前类型：{state['type']}")
        if key == ord("c"):
            state["obs"].clear()
            state["pixels"].clear()
            state["pose"] = {}
            print("已清空")
        if key == ord("g"):
            state["grid"] = not state["grid"]
        if key == ord("f"):
            solve_now()
        if key == ord("s"):
            os.makedirs("outputs", exist_ok=True)
            cv2.imwrite("outputs/task2_demo_snapshot.jpg", split)
            print("快照：outputs/task2_demo_snapshot.jpg")

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
