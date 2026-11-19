# -*- coding: utf-8 -*-
"""
外参标定：棋盘格平放在地面上，反解相机相对地面的高度 / 俯仰 / 滚转
用法：
    1) 采集：  python extrinsics_calib.py capture
    2) 计算：  python extrinsics_calib.py
前提：calibration.npz 已存在（内参先标好）；棋盘格平放在地面，相机在最终安装位姿
输出：camera_extrinsics.yaml（同时打印每张图的独立解，用于看一致性）
"""
import cv2
import os
import sys
import glob
import numpy as np

BOARD = (9, 6)
SQUARE_MM = 20.0
IMG_DIR = "extr_imgs"
# 棋盘所在平面离地高度（mm）。棋盘直接放地面填 0；
# 若放在椅子/桌子上，量出该平面离地高度填这里，解算结果会自动加回
PLANE_HEIGHT_MM = 0.0
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((BOARD[0] * BOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:BOARD[0], 0:BOARD[1]].T.reshape(-1, 2) * SQUARE_MM


def imread_safe(p):
    d = np.fromfile(p, np.uint8)
    return cv2.imdecode(d, cv2.IMREAD_COLOR) if d.size else None


def imwrite_safe(p, img):
    ok, buf = cv2.imencode(os.path.splitext(p)[1], img)
    if ok:
        buf.tofile(p)
    return ok


def capture():
    d_dir = get_dir()
    os.makedirs(d_dir, exist_ok=True)
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("相机打不开")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    for _ in range(10):
        cap.read()
    print("棋盘格平放地面，相机保持最终安装位姿。绿字 OK 时按空格拍，按 q 结束")
    print("建议顺序：正前 0.5/0.7/1.0/1.3/1.6 m 各一张，再左右各偏 20 cm 各一张")
    n = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        vis = frame.copy()
        found, corners = cv2.findChessboardCorners(
            cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), BOARD, None)
        if found:
            cv2.drawChessboardCorners(vis, BOARD, corners, found)
            cv2.putText(vis, "OK  SPACE", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            cv2.putText(vis, "no chessboard", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.putText(vis, f"saved: {n}", (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        cv2.imshow("extrinsics capture", vis)
        k = cv2.waitKey(1) & 0xFF
        if k == ord(' ') and found:
            p = os.path.join(d_dir, f"extr_{n:02d}.jpg")
            imwrite_safe(p, frame)
            n += 1
            print(f"已保存 {p}（当前 {n} 张）")
        elif k == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
    print(f"共 {n} 张，跑 python extrinsics_calib.py 计算")


def get_arg(name, default=None):
    """取命令行数值参数，例如 --plane 460 / --height 0.45"""
    for i, a in enumerate(sys.argv):
        if a == name and i + 1 < len(sys.argv):
            try:
                return float(sys.argv[i + 1])
            except ValueError:
                pass
    return default


def get_plane_mm():
    """命令行 --plane 450 表示棋盘所在平面离地 450 mm；不传则用文件里的 PLANE_HEIGHT_MM"""
    return get_arg("--plane", PLANE_HEIGHT_MM)


def get_dir():
    """命令行 --dir extr_imgs_new 指定图片目录；不传则用 extr_imgs"""
    for i, a in enumerate(sys.argv):
        if a == "--dir" and i + 1 < len(sys.argv):
            return sys.argv[i + 1]
    return IMG_DIR


def compute():
    plane_mm = get_plane_mm()
    if not os.path.exists("calibration.npz"):
        print("找不到 calibration.npz，先把内参标定做完")
        return
    if plane_mm != 0:
        print(f"棋盘所在平面离地 {plane_mm:.0f} mm，解算结果会自动加回这个高度")
    d = np.load("calibration.npz")
    K, dist = d["K"], d["dist"]
    files = sorted(glob.glob(os.path.join(get_dir(), "*.jpg")))
    if not files:
        print(f"{get_dir()}/ 里没有图片，先跑 capture")
        return

    results = []
    for f in files:
        img = imread_safe(f)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ok, c = cv2.findChessboardCorners(gray, BOARD, None)
        if not ok:
            print(f"跳过（未检出角点）：{os.path.basename(f)}")
            continue
        c = cv2.cornerSubPix(gray, c, (11, 11), (-1, -1), criteria)
        ok, rvec, tvec = cv2.solvePnP(objp, c, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            print(f"跳过（PnP 失败）：{os.path.basename(f)}")
            continue
        R, _ = cv2.Rodrigues(rvec)          # 棋盘系 -> 相机系
        C = -R.T @ tvec                      # 相机光心在棋盘系中的位置（mm）
        height_m = abs(float(C.flatten()[2])) / 1000.0 + plane_mm / 1000.0
        # 地面法线在相机系中的方向。棋盘可能被摆了 180°，法线方向会反，
        # 用「法线必须背向相机（指向上方）」这个物理约束来定符号
        n_c = R[:, 2].reshape(3).astype(float)
        if float(n_c @ tvec.reshape(3)) > 0:
            n_c = -n_c
        n_c /= np.linalg.norm(n_c)
        # 光轴与地面夹角（俯角为正）
        pitch = np.degrees(np.arcsin(np.clip(-n_c[2], -1, 1)))
        # 相机 x 轴相对地面的倾斜 = 滚转
        roll = np.degrees(np.arcsin(np.clip(n_c[0], -1, 1)))
        dist_m = float(np.linalg.norm(tvec)) / 1000.0
        # 地面系两根轴在相机系中的方向（x 轴取相机 x 轴在地面上的投影，yaw 定义为 0）
        x_c = np.array([1.0, 0, 0])
        x_g = x_c - (x_c @ n_c) * n_c
        nx = np.linalg.norm(x_g)
        x_g = x_g / nx if nx > 1e-6 else np.array([0.0, 1.0, 0])
        y_g = np.cross(n_c, x_g)
        results.append((height_m, pitch, roll, dist_m, os.path.basename(f), n_c.copy(), x_g.copy()))
        print(f"{os.path.basename(f):18s} 高度 {height_m:6.3f} m  俯仰 {pitch:6.2f}°  "
              f"滚转 {roll:6.2f}°  棋盘距离 {dist_m:5.2f} m")

    if not results:
        print("没有可用样本")
        return

    h = np.array([r[0] for r in results])
    p = np.array([r[1] for r in results])
    rl = np.array([r[2] for r in results])

    # 高度允许被卷尺实测值覆盖（板面起伏/云台微沉时，解算不如卷尺可靠）
    h_override = get_arg("--height", None)
    h_out = h_override if h_override is not None else float(np.median(h))
    if h_override is not None:
        print(f"\n注意：输出高度改用卷尺实测值 {h_override:.3f} m（解算中位数 {np.median(h):.4f} m，"
              f"相对偏差 {(h_override - np.median(h)) / np.median(h) * 100:+.1f}%）")

    print("\n" + "=" * 56)
    print(f"样本数 {len(results)}")
    print(f"高度  中位数 {np.median(h):.4f} m   标准差 {h.std()*1000:.1f} mm   极差 {(h.max()-h.min())*1000:.0f} mm")
    print(f"俯仰  中位数 {np.median(p):.2f}°    标准差 {p.std():.2f}°")
    print(f"滚转  中位数 {np.median(rl):.2f}°    标准差 {rl.std():.2f}°")
    if h.std() * 1000 > 30:
        print("⚠️ 高度标准差偏大（>30mm）：检查棋盘是否平整、是否平贴地面")
    if p.std() > 3:
        print("⚠️ 俯仰标准差偏大（>3°）：检查相机拍摄中是否移动")
    print("=" * 56)

    with open("camera_extrinsics.yaml", "w", encoding="utf-8") as f:
        h_src = "卷尺实测（--height 覆盖）" if h_override is not None else "几何解算中位数"
        i_best = int(np.argmin(np.abs(p - np.median(p))))
        n_c, x_g = results[i_best][5], results[i_best][6]
        y_g = np.cross(n_c, x_g)
        R_gc = np.stack([x_g, y_g, n_c], axis=1)   # 列 = 地面轴在相机系中的方向
        f.write(f"""# 由 extrinsics_calib.py 自动生成于 {len(results)} 张地面棋盘样本
reference_frame: rig_frame   # 相机装在测试架上请改成 rig_frame

camera_height_m: {h_out:.4f}          # {h_src}
pitch_deg: {np.median(p):.2f}      # 光轴向下俯角，正=朝下看
roll_deg: {np.median(rl):.2f}      # 相机 x 轴相对地平面的倾角
yaw_deg: 0.0                       # 地面系 X 轴按相机 x 轴的投影定义，故为 0
offset_x_m: 0.0
offset_y_m: 0.0
plane_height_mm_used: {plane_mm:.0f}   # 棋盘所在平面离地高度（已加进 camera_height_m）

# 一致性统计（写进 README 更有说服力）
num_samples: {len(results)}
height_std_mm: {h.std()*1000:.1f}
pitch_std_deg: {p.std():.2f}
roll_std_deg: {rl.std():.2f}

# 地面系 -> 相机系 旋转矩阵（行优先）。地面系 Z 向上，原点取相机光心的地面投影点
rotation_ground_to_camera:
  rows: 3
  cols: 3
  data: [{", ".join(f"{v:.6f}" for v in R_gc.ravel())}]
""")
    print("已保存 camera_extrinsics.yaml")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "capture":
        capture()
    else:
        compute()
