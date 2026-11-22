"""地面投影工具：在画面上点一个地面点，实时输出它在载体坐标系下的米制坐标。

这是任务 2 里「像素 → 地面」那一环的可视化验证工具。

用法：
    python3 ground_projection.py --height 0.45 --pitch 12
    python3 ground_projection.py --height 0.45 --pitch 12 --roll 0 --yaw 0 --grid

参数：
    --height  相机光心离地高度，米（卷尺量）
    --pitch   相机俯仰角，度（镜头朝下为正）
    --roll    滚转角，度（默认 0）
    --yaw     航向角，度（相对载体正前方，默认 0）
    --intr    内参文件，默认 calibration/camera_intrinsics.yaml

操作：
    鼠标左键点击画面 → 打印该点的地面坐标 (X 前, Y 左)，单位米
    g 切换地面网格     s 存快照     q 退出
"""
import argparse
import os

import cv2
import numpy as np
import yaml

import camera


def build_extrinsics(height, pitch_deg, roll_deg, yaw_deg):
    """相机坐标(右下前) → 载体坐标(前左上) 的旋转矩阵 R_bc，以及光心位置。"""
    t = np.radians(pitch_deg)
    r = np.radians(roll_deg)
    y = np.radians(yaw_deg)

    # 零姿态基准：body_X = z_c, body_Y = -x_c, body_Z = -y_c
    R0 = np.array([[0, 0, 1],
                   [-1, 0, 0],
                   [0, -1, 0]], dtype=float)

    def Ry(a):
        c, s = np.cos(a), np.sin(a)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

    def Rx(a):
        c, s = np.cos(a), np.sin(a)
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

    def Rz(a):
        c, s = np.cos(a), np.sin(a)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    R_bc = Rz(y) @ Rx(r) @ Ry(t) @ R0
    O_b = np.array([0.0, 0.0, height])  # 相机光心在载体坐标（车体原点正上方）
    return R_bc, O_b


def pixel_to_ground(u, v, K, dist, R_bc, O_b):
    """像素点 → 载体坐标系地面点。无效返回 None。"""
    pts = np.array([[[u, v]]], dtype=np.float32)
    und = cv2.undistortPoints(pts, K, dist)[0, 0]

    d_c = np.array([und[0], und[1], 1.0], dtype=float)
    d_b = R_bc @ d_c
    if abs(d_b[2]) < 1e-6:
        return None
    lam = -O_b[2] / d_b[2]
    if lam <= 0:
        return None
    P = O_b + lam * d_b
    return P[0], P[1], lam


def project_to_image(X, Y, K, R_bc, O_b):
    """载体坐标地面点 → 像素，用于画网格。"""
    P = np.array([X, Y, 0.0])
    p_c = R_bc.T @ (P - O_b)
    if p_c[2] <= 1e-6:
        return None
    uv = K @ (p_c / p_c[2])
    return int(round(uv[0])), int(round(uv[1]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--height", type=float, required=True, help="相机光心离地高度，米")
    ap.add_argument("--pitch", type=float, default=0.0, help="俯仰角（朝下为正），度")
    ap.add_argument("--roll", type=float, default=0.0)
    ap.add_argument("--yaw", type=float, default=0.0)
    ap.add_argument("--intr", default="calibration/camera_intrinsics.yaml")
    ap.add_argument("--grid", action="store_true", help="启动时显示地面网格")
    args = ap.parse_args()

    if not os.path.exists(args.intr):
        print(f"找不到内参文件 {args.intr}，先跑 calibrate.py")
        return 1
    with open(args.intr, "r", encoding="utf-8") as f:
        cal = yaml.safe_load(f)
    cm, dc = cal["camera_matrix"], cal["dist_coeffs"]
    K = np.array([[cm["fx"], 0, cm["cx"]],
                  [0, cm["fy"], cm["cy"]],
                  [0, 0, 1]], dtype=float)
    dist = np.array([dc["k1"], dc["k2"], dc["p1"], dc["p2"], dc["k3"]], dtype=float)

    R_bc, O_b = build_extrinsics(args.height, args.pitch, args.roll, args.yaw)
    print(f"外参：高度 {args.height} m, pitch {args.pitch}°, roll {args.roll}°, yaw {args.yaw}°")

    cap, frame, info = camera.auto_open(verbose=False)
    h, w = frame.shape[:2]
    print(f"相机 {w}x{h}；鼠标点击画面上的地面点，输出米制坐标，q 退出")

    state = {"pts": [], "grid": args.grid, "dirty": True}

    def on_mouse(event, x, y, flags, _):
        if event == cv2.EVENT_LBUTTONDOWN:
            res = pixel_to_ground(x, y, K, dist, R_bc, O_b)
            if res is None:
                print(f"  像素 ({x},{y}) → 无有效地面交点")
                return
            X, Y, lam = res
            print(f"  像素 ({x},{y}) → 地面 X={X:.3f} m（前）, Y={Y:.3f} m（左）, 距离={lam:.3f} m")
            state["pts"].append((x, y, X, Y))
            state["dirty"] = True

    win = "ground projection"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(win, on_mouse)

    while True:
        ok, frame = cap.read()
        if not ok:
            print("取帧失败")
            break
        canvas = frame

        if state["grid"]:
            for X in range(1, 7):
                prev = None
                for Y in np.arange(-3.0, 3.01, 0.2):
                    p = project_to_image(X, Y, K, R_bc, O_b)
                    if p and 0 <= p[0] < w and 0 <= p[1] < h:
                        if prev:
                            cv2.line(canvas, prev, p, (90, 90, 90), 1)
                        prev = p
                    else:
                        prev = None
                p = project_to_image(X, -3.0, K, R_bc, O_b)
                if p:
                    cv2.putText(canvas, f"{X}m", (p[0], p[1] - 4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120, 200, 255), 1)

        for (x, y, X, Y) in state["pts"]:
            cv2.drawMarker(canvas, (x, y), (0, 0, 255), cv2.MARKER_CROSS, 18, 2)
            cv2.putText(canvas, f"({X:.2f},{Y:.2f})", (x + 8, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        cv2.putText(canvas, f"h={args.height}m pitch={args.pitch}deg  click=project  g=grid  s=snap  q=quit",
                    (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        cv2.imshow(win, canvas)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("g"):
            state["grid"] = not state["grid"]
        if key == ord("s"):
            os.makedirs("outputs", exist_ok=True)
            cv2.imwrite("outputs/ground_projection_snapshot.jpg", canvas)
            print("  快照：outputs/ground_projection_snapshot.jpg")

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
