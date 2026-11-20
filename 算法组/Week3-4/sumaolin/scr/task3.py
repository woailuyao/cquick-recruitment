"""Task3：左右分屏的实时场地定位（默认 PnP）。

左半屏：相机画面 + 足球检测 + L/T/X 检测 + 像素坐标。
右半屏：field.yaml 简易俯视图，实时估计相机光心在场地地图中的位置
         (x_camera_map, y_camera_map) 和偏航角 yaw_camera_map
         （朝向与地图 +X 轴的夹角，逆时针为正）。

定位链路（--pose-mode pnp，默认）：
    标志像素 -> 去畸变 -> 与 field.yaml 的 3D 点 (X,Y,0) 枚举 2D-3D 对应
             -> 4 点 PnP -> 重投影内点筛选 + 迭代细化
             -> 相机在地图中的 x/y/yaw
    PnP 不需要相机安装外参（高度/俯角/横滚），只需要内参和畸变。
    平面点 PnP 需要至少 4 个标志；只有 4 个时存在镜像歧义，会标记 ambiguous。
    PnP 求解代码已内联在本文件，不再依赖单独的 pnp_pose.py。

    --pose-mode plane 保留原来的“地面射线求交 + 2D-2D 匹配 + SE(2)”方案，
    该模式才需要相机安装外参。

相机参数：
    内参已于 2026-11-15 用棋盘格视频标定（1280x720）并写入下方 CAMERA；
    PnP 只使用内参/畸变，安装外参仅 plane 模式使用。

使用（在 scr/ 目录下执行）：
    python task3.py
    python task3.py --pose-mode plane
按 q 或 Esc 退出。
"""

import argparse
import itertools
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml
from ultralytics import YOLO

BASE_DIR = Path(__file__).resolve().parent
DEFAULT_MARKER_MODEL = BASE_DIR / "marker.pt"
DEFAULT_BALL_MODEL = BASE_DIR / "ball.pt"
DEFAULT_FIELD = BASE_DIR.parent / "config" / "field.yaml"
DEFAULT_CALIB = BASE_DIR.parent / "config" / "camera_intrinsics.yaml"
OPERATOR_NAME = "sumaolin"

# ---------------- 相机参数：内参已标定，安装外参仍为占位值 ----------------
CAMERA = {
    "calib_width": 1280,
    "calib_height": 720,
    "fx": 1563.6194,           # 2026-11-15 棋盘格标定
    "fy": 1632.7168,
    "cx": 640.0,               # 标定时固定到图像中心
    "cy": 360.0,
    "dist": (0.252780, -1.094336, -0.032371, 0.004958, 0.0),
    "max_range": 8.0,
}

# 布局：左相机面板 + 右地图面板
LEFT_W, LEFT_H = 1200, 720
MAP_W, MAP_H = 600, 720

TYPE_MAP = {
    "LCross": "L",
    "TCross": "T",
    "XCross": "X",
    "L": "L",
    "T": "T",
    "X": "X",
}

TYPE_COLORS = {
    "L": (0, 220, 0),
    "T": (255, 220, 0),
    "X": (255, 0, 255),
}

MATCH_GATE = 0.65       # 观测点与地图点的最大匹配距离（m）
MAX_RESIDUAL = 0.50     # 定位残差阈值（m）
MIN_INLIERS = 3         # 有效定位所需的最少匹配标志数


def parse_args():
    parser = argparse.ArgumentParser(description="左右分屏实时场地定位（task3）")
    parser.add_argument("--marker-model", default=str(DEFAULT_MARKER_MODEL), help="场地标志模型，默认 scr/marker.pt")
    parser.add_argument("--ball-model", default=str(DEFAULT_BALL_MODEL), help="足球模型，默认 scr/ball.pt")
    parser.add_argument("--field", default=str(DEFAULT_FIELD), help="场地标志坐标 yaml，默认 config/field.yaml")
    parser.add_argument("--source", default="auto",
                        help="auto/usb/builtin、摄像头编号(0/1/...)、视频文件或 RTSP/HTTP 流地址")
    parser.add_argument("--camera-backend", choices=("auto", "dshow", "msmf"), default="auto",
                        help="Windows 摄像头后端，默认 auto（优先 DirectShow）")
    parser.add_argument("--list-cameras", action="store_true", help="列出可用摄像头后退出")
    parser.add_argument("--ball-conf", type=float, default=0.25, help="足球置信度阈值")
    parser.add_argument("--marker-conf", type=float, default=0.2, help="标志置信度阈值")
    parser.add_argument("--pose-mode", choices=("pnp", "plane"), default="pnp",
                        help="pnp：像素-地图点 direct PnP；plane：原地面投影+2D匹配")
    parser.add_argument("--imgsz", type=int, default=640, help="推理分辨率")
    parser.add_argument("--device", default="", help="推理设备：'0' 用 GPU，'cpu' 用 CPU，留空自动选择")
    parser.add_argument("--calib", default=str(DEFAULT_CALIB) if DEFAULT_CALIB.is_file() else "",
                        help="相机标定 yaml 路径（默认加载 config/camera_intrinsics.yaml）")
    parser.add_argument("--mount-height", type=float, default=None, help="覆盖光心离地高度（m）")
    parser.add_argument("--pitch", type=float, default=None, help="覆盖光轴向下俯角（deg）")
    parser.add_argument("--roll", type=float, default=None, help="覆盖横滚角（deg）")
    parser.add_argument("--yaw", type=float, default=None, help="覆盖偏航角（deg）")
    return parser.parse_args()


def build_camera(args, require_mount=False):
    cam = dict(CAMERA)
    if args.calib:
        path = Path(args.calib)
        if not path.is_file():
            raise SystemExit(f"标定文件不存在: {path}")
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        if not isinstance(data, dict):
            raise SystemExit(f"标定文件格式错误: {path}")
        for key in cam:
            if key in data and data[key] is not None:
                cam[key] = data[key]
        cam["dist"] = tuple(float(v) for v in cam["dist"])
        print(f"已加载相机标定: {path}")
    else:
        print("[提示] 未指定 --calib，使用脚本内置的占位相机参数，定位结果仅为近似值。")

    overrides = {
        "mount_height": args.mount_height,
        "pitch_deg": args.pitch,
        "roll_deg": args.roll,
        "yaw_deg": args.yaw,
    }
    for key, value in overrides.items():
        if value is not None:
            cam[key] = value

    if cam["fx"] <= 0 or cam["fy"] <= 0:
        raise SystemExit("相机内参 fx/fy 必须大于 0")

    # 只有 plane 模式需要安装外参；PnP 只用内参和畸变。
    if require_mount:
        required = ("mount_height", "mount_x", "mount_y",
                    "pitch_deg", "roll_deg", "yaw_deg")
        missing = [key for key in required if key not in cam]
        if missing:
            raise SystemExit(
                "plane 模式需要相机安装外参，但当前配置缺少: "
                + ", ".join(missing)
                + "。请改用 --pose-mode pnp，或补回外参。"
            )
        if cam["mount_height"] <= 0:
            raise SystemExit("plane 模式要求相机安装高度 mount_height > 0")
    return cam


CAMERA_SCAN_MAX = 6


def _camera_backends(name):
    """返回按优先级排列的 (名称, OpenCV backend) 列表。"""
    if name == "dshow":
        return [("dshow", cv2.CAP_DSHOW)]
    if name == "msmf":
        return [("msmf", cv2.CAP_MSMF)]
    if sys.platform.startswith("win"):
        # 默认后端兼容性最好；失败再试 MSMF / DirectShow
        return [("default", 0), ("msmf", cv2.CAP_MSMF), ("dshow", cv2.CAP_DSHOW)]
    return [("default", 0)]


def _probe_camera(index, backend="auto"):
    for label, flag in _camera_backends(backend):
        cap = cv2.VideoCapture(index, flag)
        if not cap.isOpened():
            cap.release()
            continue
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        ok, frame = cap.read()
        if not ok or frame is None:
            cap.release()
            cap = cv2.VideoCapture(index, flag)
            if not cap.isOpened():
                cap.release()
                continue
            ok, frame = cap.read()
        if not ok or frame is None:
            cap.release()
            continue
        h, w = frame.shape[:2]
        return cap, (int(w), int(h)), label
    return None


def list_cameras(backend="auto", max_index=CAMERA_SCAN_MAX):
    found = []
    for i in range(max_index):
        item = _probe_camera(i, backend)
        if item is not None:
            cap, size, label = item
            found.append((i, size, label))
            cap.release()
    if not found:
        print("未发现可用摄像头")
    for i, size, label in found:
        print(f"camera {i}: {size[0]}x{size[1]} ({label})")
    return found


def open_capture(source, backend="auto"):
    """打开 auto/usb/builtin、摄像头编号、视频文件或流地址。"""
    source = str(source).strip()
    key = source.lower()

    if key in ("auto", "usb", "builtin"):
        found = []
        caps = {}
        for i in range(CAMERA_SCAN_MAX):
            item = _probe_camera(i, backend)
            if item is not None:
                cap, size, label = item
                found.append((i, size, label))
                caps[i] = cap
        if not found:
            raise SystemExit("未发现可用摄像头")
        listing = ", ".join(f"{i}={w}x{h}" for i, (w, h), _ in found)
        print(f"检测到摄像头: {listing}")
        index = found[0][0] if key == "builtin" else found[-1][0]
        for i, cap in caps.items():
            if i != index:
                cap.release()
        print(f"使用 camera {index}（USB 通常排在后面；如不对，用 --source 0/1 指定）")
        return caps[index]

    if key.isdigit():
        item = _probe_camera(int(key), backend)
        if item is None:
            raise SystemExit(f"无法打开摄像头: {source}")
        return item[0]

    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise SystemExit(f"无法打开视频源: {source}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def scaled_intrinsics(cam, frame_w, frame_h):
    calib_w = max(float(cam["calib_width"]), 1.0)
    calib_h = max(float(cam["calib_height"]), 1.0)
    sx = frame_w / calib_w
    sy = frame_h / calib_h
    k_matrix = np.array([
        [cam["fx"] * sx, 0.0, cam["cx"] * sx],
        [0.0, cam["fy"] * sy, cam["cy"] * sy],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)
    dist = np.array(cam["dist"], dtype=np.float64).reshape(1, -1)
    return k_matrix, dist


def carrier_from_camera(pitch_deg, roll_deg, yaw_deg):
    """相机光学坐标系 -> 载体坐标系（x 前 / y 左 / z 上）。"""
    pitch = np.radians(pitch_deg)
    roll = np.radians(roll_deg)
    yaw = np.radians(yaw_deg)

    right = np.array([0.0, -1.0, 0.0])
    down = np.array([-np.sin(pitch), 0.0, -np.cos(pitch)])
    forward = np.array([np.cos(pitch), 0.0, -np.sin(pitch)])

    right_r = right * np.cos(roll) - down * np.sin(roll)
    down_r = right * np.sin(roll) + down * np.cos(roll)

    rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0.0],
        [np.sin(yaw), np.cos(yaw), 0.0],
        [0.0, 0.0, 1.0],
    ])
    return np.column_stack([rz @ right_r, rz @ down_r, rz @ forward])


def ray_to_ground(nx, ny, r_bc, origin, max_range):
    direction = r_bc @ np.array([nx, ny, 1.0])
    if direction[2] >= -1e-6:
        return None
    lam = -origin[2] / direction[2]
    if lam <= 0:
        return None
    x = origin[0] + lam * direction[0]
    y = origin[1] + lam * direction[1]
    if x < -1.0 or x > max_range or abs(y) > max_range:
        return None
    return float(x), float(y)


def short_type(name):
    key = str(name).strip()
    if key in TYPE_MAP:
        return TYPE_MAP[key]
    key_lower = key.lower()
    for full, short in TYPE_MAP.items():
        if full.lower() == key_lower:
            return short
    return None


def put_text(frame, text, org, scale, color, thickness=1):
    cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale,
                (0, 0, 0), thickness + 2, cv2.LINE_AA)
    cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale,
                color, thickness, cv2.LINE_AA)


def draw_box_label(frame, text, x, y, color, scale=0.55, thickness=2):
    (tw, th), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
    y = max(y, th + baseline + 2)
    cv2.rectangle(frame, (x, y - th - baseline), (x + tw + 8, y), color, -1)
    cv2.putText(frame, text, (x + 4, y - baseline), cv2.FONT_HERSHEY_SIMPLEX,
                scale, (0, 0, 0), thickness, cv2.LINE_AA)


def draw_json_panel(frame, observed):
    """左下角列出观测标志的 JSON（载体坐标系，单位 m）。"""
    if not observed:
        return
    lines = [json.dumps(item, separators=(",", ":")) for item in observed]
    scale = 0.5
    (_, text_h), baseline = cv2.getTextSize("Ag", cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    line_h = text_h + baseline + 6
    text_w = max(cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)[0][0] for line in lines)

    panel_w = min(text_w + 20, frame.shape[1] - 20)
    panel_h = line_h * len(lines) + 10
    x = 10
    y = frame.shape[0] - panel_h - 10

    overlay = frame.copy()
    cv2.rectangle(overlay, (x, y), (x + panel_w, y + panel_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)
    for i, line in enumerate(lines):
        put_text(frame, line, (x + 10, y + 8 + text_h + i * line_h), scale, (0, 255, 255), 1)


def draw_ball(frame, box, conf):
    x1, y1, x2, y2 = [int(round(v)) for v in box]
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    draw_box_label(frame, f"ball {conf:.2f}", x1, y1, (0, 255, 0))
    u, v = (x1 + x2) // 2, (y1 + y2) // 2
    cv2.circle(frame, (u, v), 4, (0, 0, 255), -1)
    cv2.line(frame, (u - 10, v), (u + 10, v), (0, 0, 255), 1)
    cv2.line(frame, (u, v - 10), (u, v + 10), (0, 0, 255), 1)
    put_text(frame, f"({u}, {v})", (u + 12, v - 10), 0.5, (0, 255, 0), 1)


def draw_marker(frame, box, short, conf, u, v, ground, coord_text=None):
    color = TYPE_COLORS.get(short, (255, 255, 255))
    x1, y1, x2, y2 = [int(round(v_)) for v_ in box]
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    draw_box_label(frame, f"{short} {conf:.2f}", x1, y1, color)

    cv2.circle(frame, (u, v), 4, (0, 0, 255), -1)
    cv2.line(frame, (u - 10, v), (u + 10, v), (0, 0, 255), 1)
    cv2.line(frame, (u, v - 10), (u, v + 10), (0, 0, 255), 1)

    if coord_text is not None:
        put_text(frame, coord_text, (u + 12, v + 20), 0.55, color, 1)
    elif ground is None:
        put_text(frame, "no ground", (u + 12, v + 20), 0.5, (0, 0, 255), 1)
    else:
        put_text(frame, f"({ground[0]:.2f}, {ground[1]:.2f}) m", (u + 12, v + 20), 0.55, color, 1)


def load_field(path):
    if not Path(path).is_file():
        raise SystemExit(f"场地文件不存在: {path}")
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise SystemExit(f"场地文件格式错误: {path}")

    points = []
    for name, xy in data.items():
        key = str(name)
        short = None
        if key.startswith("LCross"):
            short = "L"
        elif key.startswith("TCross"):
            short = "T"
        elif key.startswith("XCross"):
            short = "X"
        if short is None:
            print(f"[提示] 跳过无法识别的场地键: {name}")
            continue
        if not isinstance(xy, (list, tuple)) or len(xy) < 2:
            print(f"[提示] 跳过坐标格式异常的场地键: {name}")
            continue
        digits = "".join(ch for ch in key if ch.isdigit())
        points.append({
            "name": key,
            "label": f"{short}{digits}",
            "type": short,
            "xy": np.array([float(xy[0]), float(xy[1])], dtype=np.float64),
        })

    if len(points) < 2:
        raise SystemExit(f"场地文件中有效标志不足: {path}")
    return points


def build_field_model(points):
    by_type = {}
    for point in points:
        by_type.setdefault(point["type"], []).append(point)
    arrays = {t: np.array([p["xy"] for p in pts], dtype=np.float64) for t, pts in by_type.items()}
    return {"points": points, "by_type": by_type, "arrays": arrays}


def wrap_deg(deg):
    return (deg + 180.0) % 360.0 - 180.0


def match_points(transformed, obs_types, arrays, gate):
    """按距离贪心做一对一匹配，返回 [(观测序号, 地图点序号, 距离), ...]。"""
    candidates = []
    for i, type_ in enumerate(obs_types):
        pts = arrays.get(type_)
        if pts is None:
            continue
        dists = np.linalg.norm(pts - transformed[i], axis=1)
        for j in np.nonzero(dists <= gate)[0]:
            candidates.append((float(dists[j]), i, int(j)))
    candidates.sort(key=lambda item: item[0])

    used_obs = set()
    used_map = set()
    matched = []
    for dist, i, j in candidates:
        key = (obs_types[i], j)
        if i in used_obs or key in used_map:
            continue
        used_obs.add(i)
        used_map.add(key)
        matched.append((i, j, dist))
    return matched


def fit_rigid(src, dst):
    """Kabsch：求 R, t 使 R @ src + t 尽量逼近 dst。"""
    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    src_c = src - src_mean
    dst_c = dst - dst_mean
    h = src_c.T @ dst_c
    u, _, vt = np.linalg.svd(h)
    rot = vt.T @ u.T
    if np.linalg.det(rot) < 0:
        vt[-1, :] *= -1
        rot = vt.T @ u.T
    trans = dst_mean - rot @ src_mean
    return rot, trans


def refine_pose(obs_xy, obs_types, rot, trans, field, gate, iterations=3):
    arrays = field["arrays"]
    by_type = field["by_type"]
    matched = []
    for _ in range(iterations):
        transformed = obs_xy @ rot.T + trans
        matched = match_points(transformed, obs_types, arrays, gate)
        if len(matched) < 2:
            break
        src = np.array([obs_xy[i] for i, _, _ in matched], dtype=np.float64)
        dst = np.array([by_type[obs_types[i]][j]["xy"] for i, j, _ in matched], dtype=np.float64)
        rot, trans = fit_rigid(src, dst)

    transformed = obs_xy @ rot.T + trans
    matched = match_points(transformed, obs_types, arrays, gate)
    if matched:
        rms = math.sqrt(sum(d * d for _, _, d in matched) / len(matched))
    else:
        rms = float("inf")
    return rot, trans, matched, rms


def invalid_pose(status, marker_count=0):
    return {
        "valid": False,
        "status": status,
        "marker_count": int(marker_count),
        "residual": None,
        "x_camera_map": None,
        "y_camera_map": None,
        "yaw_camera_map": None,
        "x_carrier_map": None,
        "y_carrier_map": None,
        "yaw_carrier_map": None,
    }


def check_ambiguous(hypotheses, best):
    """存在得分相同但位姿明显不同的假设时，判为歧义。"""
    best_theta = math.atan2(best["rot"][1, 0], best["rot"][0, 0])
    for hyp in hypotheses:
        if hyp is best:
            continue
        if hyp["inliers"] < best["inliers"]:
            continue
        if hyp["rms"] > best["rms"] + 0.05:
            continue
        theta = math.atan2(hyp["rot"][1, 0], hyp["rot"][0, 0])
        d_theta = abs(math.degrees((theta - best_theta + math.pi) % (2.0 * math.pi) - math.pi))
        d_trans = float(np.linalg.norm(hyp["trans"] - best["trans"]))
        if d_trans > 0.5 or d_theta > 10.0:
            return True
    return False


def solve_pose(observations, field, cam):
    """观测标志（载体坐标系）与 field.yaml 匹配，求相机在地图中的位姿。"""
    if len(observations) < 2:
        return invalid_pose("insufficient_markers", len(observations))

    obs_xy = np.array([[o["x"], o["y"]] for o in observations], dtype=np.float64)
    obs_types = [o["type"] for o in observations]
    arrays = field["arrays"]

    hypotheses = []
    n = len(observations)
    for i in range(n):
        for j in range(i + 1, n):
            vi = obs_xy[j] - obs_xy[i]
            len_i = float(np.linalg.norm(vi))
            if len_i < 0.5:          # 两点太近，姿态解病态
                continue
            pts_i = arrays.get(obs_types[i])
            pts_j = arrays.get(obs_types[j])
            if pts_i is None or pts_j is None:
                continue
            for mi in range(len(pts_i)):
                for mj in range(len(pts_j)):
                    if obs_types[i] == obs_types[j] and mi == mj:
                        continue
                    vm = pts_j[mj] - pts_i[mi]
                    len_m = float(np.linalg.norm(vm))
                    if len_m < 0.5 or abs(len_i - len_m) > 0.6:
                        continue
                    theta = math.atan2(vm[1], vm[0]) - math.atan2(vi[1], vi[0])
                    c, s = math.cos(theta), math.sin(theta)
                    rot = np.array([[c, -s], [s, c]])
                    trans = pts_i[mi] - rot @ obs_xy[i]

                    transformed = obs_xy @ rot.T + trans
                    matched = match_points(transformed, obs_types, arrays, MATCH_GATE)
                    if len(matched) < 2:
                        continue
                    rms = math.sqrt(sum(d * d for _, _, d in matched) / len(matched))
                    hypotheses.append({
                        "inliers": len(matched),
                        "rms": rms,
                        "rot": rot,
                        "trans": trans,
                    })

    if not hypotheses:
        return invalid_pose("insufficient_markers", 0)

    best = max(hypotheses, key=lambda h: (h["inliers"], -h["rms"]))
    if best["inliers"] < MIN_INLIERS:
        return invalid_pose("insufficient_markers", best["inliers"])

    ambiguous = check_ambiguous(hypotheses, best)

    rot, trans, matched, rms = refine_pose(obs_xy, obs_types, best["rot"], best["trans"], field, MATCH_GATE)
    inliers = len(matched)
    if inliers < MIN_INLIERS:
        return invalid_pose("insufficient_markers", inliers)
    if rms > MAX_RESIDUAL:
        return invalid_pose("high_residual", inliers)
    if ambiguous:
        return invalid_pose("ambiguous", inliers)

    theta = math.atan2(rot[1, 0], rot[0, 0])
    cam_offset = np.array([cam["mount_x"], cam["mount_y"]], dtype=np.float64)
    cam_xy = trans + rot @ cam_offset

    return {
        "valid": True,
        "status": "ok",
        "marker_count": int(inliers),
        "residual": float(rms),
        "x_carrier_map": float(trans[0]),
        "y_carrier_map": float(trans[1]),
        "yaw_carrier_map": wrap_deg(math.degrees(theta)),
        "x_camera_map": float(cam_xy[0]),
        "y_camera_map": float(cam_xy[1]),
        "yaw_camera_map": wrap_deg(math.degrees(theta) + cam["yaw_deg"]),
    }


# ---------------------------------------------------------------------------
# PnP 位姿：像素 + field.yaml 三维点直接解相机位姿，不依赖安装外参
# ---------------------------------------------------------------------------
PNP_MIN_INLIERS = 4
PNP_MAX_RMS_PX = 8.0
PNP_GATE_PX = 14.0
PNP_MAX_COMBOS = 6000
# 时序消歧：上一帧有效位姿与当前多解中某一解的位置/偏航差在这个范围内，
# 就选该解；否则保持 ambiguous，不强行输出错误位姿。
PNP_TEMPORAL_POS_M = 0.45
PNP_TEMPORAL_YAW_DEG = 15.0
PNP_TEMPORAL_MAX_GAP = 8


def _pnp_match_all(rvec, tvec, det_uv, det_types, field, k_matrix, dist, gate):
    """把当前位姿下所有地图点投到图像，与检测点按类型做一对一匹配。"""
    proj_by_type = {}
    for type_, pts in field["by_type"].items():
        obj = np.array([[p["xy"][0], p["xy"][1], 0.0] for p in pts], dtype=np.float64)
        projected, _ = cv2.projectPoints(obj, rvec, tvec, k_matrix, dist)
        proj_by_type[type_] = (pts, projected.reshape(-1, 2))

    candidates = []
    for i, (uv, type_) in enumerate(zip(det_uv, det_types)):
        item = proj_by_type.get(type_)
        if item is None:
            continue
        _, projected = item
        distances = np.linalg.norm(projected - np.asarray(uv), axis=1)
        for j in np.nonzero(distances <= gate)[0]:
            candidates.append((float(distances[j]), i, type_, int(j)))
    candidates.sort(key=lambda c: c[0])

    used_det = set()
    used_map = set()
    matched = []
    for d, i, type_, j in candidates:
        if i in used_det or (type_, j) in used_map:
            continue
        used_det.add(i)
        used_map.add((type_, j))
        matched.append((i, type_, j, d))
    return matched


def _pnp_refine(matched, det_uv, field, k_matrix, dist):
    obj = []
    img = []
    for i, type_, j, _ in matched:
        point = field["by_type"][type_][j]
        obj.append([point["xy"][0], point["xy"][1], 0.0])
        img.append(det_uv[i])
    obj = np.asarray(obj, dtype=np.float64)
    img = np.asarray(img, dtype=np.float64)
    if len(obj) < 4:
        return None
    ok, rvec, tvec = cv2.solvePnP(obj, img, k_matrix, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    return rvec, tvec


def _pnp_camera_pose(rvec, tvec):
    rot, _ = cv2.Rodrigues(rvec)
    center = (-rot.T @ tvec).reshape(3)
    axis = rot.T @ np.array([0.0, 0.0, 1.0])  # 光轴在地图系中的方向
    yaw = math.degrees(math.atan2(axis[1], axis[0]))
    yaw = (yaw + 180.0) % 360.0 - 180.0
    return center, yaw


def _pnp_ambiguous(solutions, best):
    for item in solutions:
        if item is best:
            continue
        if item["inliers"] < best["inliers"]:
            continue
        if item["rms"] > best["rms"] * 1.5 + 1.0:
            continue
        d_xy = float(np.linalg.norm(item["center"][:2] - best["center"][:2]))
        d_yaw = abs((item["yaw"] - best["yaw"] + 180.0) % 360.0 - 180.0)
        if d_xy > 0.6 or d_yaw > 12.0:
            return True
    return False


def _pnp_temporal_pick(solutions, prev_pose):
    """在多个 4 点解里挑与上一帧位姿最接近的一个。"""
    if not prev_pose:
        return None
    px = prev_pose.get("x_camera_map")
    py = prev_pose.get("y_camera_map")
    pyaw = prev_pose.get("yaw_camera_map")
    if px is None or py is None or pyaw is None:
        return None
    best = None
    best_score = None
    for item in solutions:
        if item["inliers"] < PNP_MIN_INLIERS:
            continue
        if item["rms"] > PNP_MAX_RMS_PX:
            continue
        d_xy = float(np.linalg.norm(item["center"][:2] - np.array([px, py], dtype=np.float64)))
        d_yaw = abs((item["yaw"] - float(pyaw) + 180.0) % 360.0 - 180.0)
        if d_xy > PNP_TEMPORAL_POS_M or d_yaw > PNP_TEMPORAL_YAW_DEG:
            continue
        score = d_xy / PNP_TEMPORAL_POS_M + d_yaw / PNP_TEMPORAL_YAW_DEG
        if best_score is None or score < best_score:
            best_score = score
            best = item
    return best


def solve_pose_pnp(detections, field, k_matrix, dist, prev_pose=None):
    """detections: [{'type','uv','confidence'}]；返回相机在地图中的位姿。

    prev_pose 为上一帧有效位姿时，若当前 4 点解有歧义，优先选与上一帧
    位置/偏航接近的解；没有接近解则仍返回 valid=False。
    """
    detections = sorted(detections, key=lambda d: -d["confidence"])[:6]
    if len(detections) < 4:
        return {
            "valid": False, "status": "insufficient_markers",
            "marker_count": len(detections), "residual": None, "residual_unit": "px",
            "x_camera_map": None, "y_camera_map": None, "yaw_camera_map": None,
            "x_carrier_map": None, "y_carrier_map": None, "yaw_carrier_map": None,
            "matched": [],
        }

    det_uv = [d["uv"] for d in detections]
    det_types = [d["type"] for d in detections]
    # 先把像素去畸变到理想针孔模型；PnP 内部统一用零畸变
    raw = np.asarray(det_uv, dtype=np.float64).reshape(-1, 1, 2)
    undistorted = cv2.undistortPoints(raw, k_matrix, dist, P=k_matrix).reshape(-1, 2)
    det_uv = [tuple(map(float, uv)) for uv in undistorted]
    dist = np.zeros(5, dtype=np.float64)

    candidate_lists = []
    for type_ in det_types:
        pts = field["by_type"].get(type_, [])
        candidate_lists.append(list(range(len(pts))))
    if any(len(c) == 0 for c in candidate_lists):
        return {
            "valid": False, "status": "insufficient_markers",
            "marker_count": 0, "residual": None, "residual_unit": "px",
            "x_camera_map": None, "y_camera_map": None, "yaw_camera_map": None,
            "x_carrier_map": None, "y_carrier_map": None, "yaw_carrier_map": None,
            "matched": [],
        }

    total = 1
    for c in candidate_lists:
        total *= len(c)
    if total > PNP_MAX_COMBOS:
        return {
            "valid": False, "status": "ambiguous",
            "marker_count": 0, "residual": None, "residual_unit": "px",
            "x_camera_map": None, "y_camera_map": None, "yaw_camera_map": None,
            "x_carrier_map": None, "y_carrier_map": None, "yaw_carrier_map": None,
            "matched": [],
        }

    solutions = []
    all_points = field.get("points")
    if not all_points:
        all_points = [p for pts in field["by_type"].values() for p in pts]
    xs = [p["xy"][0] for p in all_points]
    ys = [p["xy"][1] for p in all_points]
    min_x, max_x = min(xs) - 2.0, max(xs) + 2.0
    min_y, max_y = min(ys) - 2.0, max(ys) + 2.0

    def add_solution(rvec, tvec, matched):
        rms = math.sqrt(sum(item[3] * item[3] for item in matched) / len(matched))
        center, yaw = _pnp_camera_pose(rvec, tvec)
        if center[2] < 0.05 or center[2] > 3.0:
            return
        if not (min_x <= center[0] <= max_x and min_y <= center[1] <= max_y):
            return
        solutions.append({
            "inliers": len(matched),
            "rms": rms,
            "center": center,
            "yaw": yaw,
            "rvec": rvec,
            "tvec": tvec,
            "matched": matched,
        })

    # 4 点 PnP 是闭式解，枚举 4 个检测 × 4 个地图点的组合；
    # 5/6 个检测时再对每个 4 点子集分别求，避免 6 点组合爆炸。
    for det_subset in itertools.combinations(range(len(det_types)), 4):
        sub_types = [det_types[i] for i in det_subset]
        sub_uv = [det_uv[i] for i in det_subset]
        sub_candidates = [candidate_lists[i] for i in det_subset]
        for combo in itertools.product(*sub_candidates):
            keys = [(sub_types[k], j) for k, j in enumerate(combo)]
            if len(set(keys)) < len(keys):
                continue
            obj = []
            img = []
            for k, j in enumerate(combo):
                point = field["by_type"][sub_types[k]][j]
                obj.append([point["xy"][0], point["xy"][1], 0.0])
                img.append(sub_uv[k])
            obj = np.asarray(obj, dtype=np.float64)
            img = np.asarray(img, dtype=np.float64)
            try:
                ok, rvec, tvec = cv2.solvePnP(
                    obj, img, k_matrix, dist, flags=cv2.SOLVEPNP_ITERATIVE)
            except cv2.error:
                continue
            if not ok:
                continue
            center, yaw = _pnp_camera_pose(rvec, tvec)
            if center[2] < 0.05 or center[2] > 3.0:
                continue
            if not (min_x <= center[0] <= max_x and min_y <= center[1] <= max_y):
                continue

            matched = _pnp_match_all(rvec, tvec, det_uv, det_types, field,
                                     k_matrix, dist, PNP_GATE_PX)
            if len(matched) < PNP_MIN_INLIERS:
                continue
            add_solution(rvec, tvec, matched)

            refined = _pnp_refine(matched, det_uv, field, k_matrix, dist)
            if refined is None:
                continue
            rvec2, tvec2 = refined
            matched2 = _pnp_match_all(rvec2, tvec2, det_uv, det_types, field,
                                      k_matrix, dist, PNP_GATE_PX)
            if len(matched2) < PNP_MIN_INLIERS:
                continue
            add_solution(rvec2, tvec2, matched2)

    if not solutions:
        return {
            "valid": False, "status": "not_converged",
            "marker_count": 0, "residual": None, "residual_unit": "px",
            "x_camera_map": None, "y_camera_map": None, "yaw_camera_map": None,
            "x_carrier_map": None, "y_carrier_map": None, "yaw_carrier_map": None,
            "matched": [],
        }

    best = max(solutions, key=lambda s: (s["inliers"], -s["rms"]))
    temporal = False
    if best["rms"] > PNP_MAX_RMS_PX:
        status = "high_residual"
    elif _pnp_ambiguous(solutions, best):
        temporal_pick = _pnp_temporal_pick(solutions, prev_pose) if prev_pose else None
        if temporal_pick is not None:
            best = temporal_pick
            temporal = True
            status = "ok"
        else:
            status = "ambiguous"
    else:
        status = "ok"

    matched_names = []
    for i, type_, j, _ in best["matched"]:
        matched_names.append({
            "det_index": int(i),
            "type": type_,
            "map_index": int(j),
            "name": field["by_type"][type_][j]["name"],
        })

    return {
        "valid": status == "ok",
        "status": status,
        "temporal": temporal,
        "marker_count": int(best["inliers"]),
        "residual": float(best["rms"]),
        "residual_unit": "px",
        "rvec": np.asarray(best["rvec"], dtype=np.float64).reshape(3),
        "tvec": np.asarray(best["tvec"], dtype=np.float64).reshape(3),
        "x_camera_map": float(best["center"][0]),
        "y_camera_map": float(best["center"][1]),
        "yaw_camera_map": float(best["yaw"]),
        "x_carrier_map": float(best["center"][0]),
        "y_carrier_map": float(best["center"][1]),
        "yaw_carrier_map": float(best["yaw"]),
        "matched": matched_names,
    }


def draw_map(field, pose, fps, infer_ms):
    """右半屏：field.yaml 俯视图 + 相机位姿。"""
    panel = np.full((MAP_H, MAP_W, 3), (30, 30, 34), dtype=np.uint8)
    put_text(panel, "field map (field.yaml)", (16, 30), 0.62, (255, 255, 255), 1)

    xs = [p["xy"][0] for p in field["points"]]
    ys = [p["xy"][1] for p in field["points"]]
    margin = 0.8
    min_x, max_x = min(xs) - margin, max(xs) + margin
    min_y, max_y = min(ys) - margin, max(ys) + margin

    plot_x, plot_y = 20, 50
    plot_w, plot_h = MAP_W - 40, 460
    scale = min(plot_w / (max_x - min_x), plot_h / (max_y - min_y))
    off_x = plot_x + (plot_w - (max_x - min_x) * scale) / 2.0
    off_y = plot_y + (plot_h - (max_y - min_y) * scale) / 2.0

    def to_px(x, y):
        return (int(round(off_x + (x - min_x) * scale)),
                int(round(off_y + (max_y - y) * scale)))

    # 1 m 网格
    for gx in range(int(math.ceil(min_x)), int(math.floor(max_x)) + 1):
        px, _ = to_px(gx, 0)
        cv2.line(panel, (px, plot_y), (px, plot_y + plot_h), (48, 48, 54), 1)
    for gy in range(int(math.ceil(min_y)), int(math.floor(max_y)) + 1):
        _, py = to_px(0, gy)
        cv2.line(panel, (plot_x, py), (plot_x + plot_w, py), (48, 48, 54), 1)
    cv2.rectangle(panel, (plot_x, plot_y), (plot_x + plot_w, plot_y + plot_h), (70, 70, 78), 1)

    # 地图坐标轴（原点在场地内时绘制）
    ox, oy = to_px(0.0, 0.0)
    if plot_x <= ox <= plot_x + plot_w and plot_y <= oy <= plot_y + plot_h:
        cv2.arrowedLine(panel, (ox, oy), (ox + 45, oy), (120, 120, 130), 1, tipLength=0.25)
        put_text(panel, "+X", (ox + 50, oy + 5), 0.45, (150, 150, 160), 1)
        cv2.arrowedLine(panel, (ox, oy), (ox, oy - 45), (120, 120, 130), 1, tipLength=0.25)
        put_text(panel, "+Y", (ox + 6, oy - 50), 0.45, (150, 150, 160), 1)
        put_text(panel, "O", (ox - 15, oy + 14), 0.4, (150, 150, 160), 1)

    # 地图标志
    for point in field["points"]:
        px, py = to_px(point["xy"][0], point["xy"][1])
        color = TYPE_COLORS.get(point["type"], (255, 255, 255))
        cv2.circle(panel, (px, py), 4, color, -1)
        put_text(panel, point["label"], (px + 6, py - 6), 0.42, color, 1)

    # 相机位置与朝向
    if pose["valid"]:
        cx, cy = to_px(pose["x_camera_map"], pose["y_camera_map"])
        heading = math.radians(pose["yaw_camera_map"])
        dx, dy = math.cos(heading), -math.sin(heading)   # 像素坐标 y 向下
        perp_x, perp_y = -dy, dx
        tip = (int(round(cx + 22 * dx)), int(round(cy + 22 * dy)))
        base_l = (int(round(cx - 12 * dx + 10 * perp_x)), int(round(cy - 12 * dy + 10 * perp_y)))
        base_r = (int(round(cx - 12 * dx - 10 * perp_x)), int(round(cy - 12 * dy - 10 * perp_y)))
        triangle = np.array([tip, base_l, base_r], dtype=np.int32)
        cv2.fillPoly(panel, [triangle], (0, 255, 255))
        cv2.polylines(panel, [triangle], True, (0, 0, 0), 1)
        cv2.circle(panel, (cx, cy), 3, (0, 0, 0), -1)

    # 读数
    hud_x, hud_y, line_h = 20, 545, 24
    if pose["valid"]:
        status_color = (0, 255, 0)
        unit = pose.get("residual_unit", "m")
        lines = [
            (f"x_camera_map   = {pose['x_camera_map']:6.2f} m", (255, 255, 255)),
            (f"y_camera_map   = {pose['y_camera_map']:6.2f} m", (255, 255, 255)),
            (f"yaw_camera_map = {pose['yaw_camera_map']:6.1f} deg", (255, 255, 255)),
            (f"valid = true    status = {pose['status']}", status_color),
            (f"marker_count = {pose['marker_count']}   residual = {pose['residual']:.2f} {unit}", (255, 255, 255)),
        ]
    else:
        status_color = (0, 165, 255)
        lines = [
            ("x_camera_map   =   --", (200, 200, 200)),
            ("y_camera_map   =   --", (200, 200, 200)),
            ("yaw_camera_map =   --", (200, 200, 200)),
            (f"valid = false   status = {pose['status']}", status_color),
            (f"marker_count = {pose['marker_count']}   residual = --", (255, 255, 255)),
        ]

    for i, (text, color) in enumerate(lines):
        put_text(panel, text, (hud_x, hud_y + i * line_h), 0.55, color, 1)
    put_text(panel, f"FPS {fps:4.1f}   infer {infer_ms:4.1f} ms", (hud_x, hud_y + 5 * line_h), 0.55, (255, 255, 255), 1)

    return panel


def fit_panel(frame, width, height):
    """按比例缩放并居中，保持相机画面不变形。"""
    h, w = frame.shape[:2]
    scale = min(width / w, height / h)
    new_w, new_h = int(round(w * scale)), int(round(h * scale))
    interp = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    resized = cv2.resize(frame, (new_w, new_h), interpolation=interp)
    panel = np.zeros((height, width, 3), dtype=np.uint8)
    x0 = (width - new_w) // 2
    y0 = (height - new_h) // 2
    panel[y0:y0 + new_h, x0:x0 + new_w] = resized
    return panel


def marker_json(item):
    if "u" in item:
        return {
            "type": item["type"],
            "u": int(item["u"]),
            "v": int(item["v"]),
            "confidence": round(item["confidence"], 2),
        }
    return {
        "type": item["type"],
        "x": round(item["x"], 2),
        "y": round(item["y"], 2),
        "confidence": round(item["confidence"], 2),
    }


def main():
    args = parse_args()

    if args.list_cameras:
        list_cameras(args.camera_backend)
        return

    for model_path in (args.ball_model, args.marker_model):
        if not Path(model_path).is_file():
            raise SystemExit(f"模型文件不存在: {model_path}")

    cam = build_camera(args, require_mount=(args.pose_mode == "plane"))
    field_points = load_field(Path(args.field))
    field = build_field_model(field_points)
    print(f"场地标志: {len(field_points)} 个, 各类型 {[(t, len(field['by_type'][t])) for t in sorted(field['by_type'])]}")

    ball_model = YOLO(args.ball_model)
    marker_model = YOLO(args.marker_model)

    if args.pose_mode == "plane":
        r_bc = carrier_from_camera(cam["pitch_deg"], cam["roll_deg"], cam["yaw_deg"])
        origin = np.array([cam["mount_x"], cam["mount_y"], cam["mount_height"]], dtype=np.float64)
    else:
        r_bc = None
        origin = None
    max_range = float(cam.get("max_range", 8.0))

    print(f"相机: {cam['calib_width']}x{cam['calib_height']} "
          f"fx={cam['fx']:.1f} fy={cam['fy']:.1f} cx={cam['cx']:.1f} cy={cam['cy']:.1f}")
    if args.pose_mode == "plane":
        print(f"安装: height={cam['mount_height']:.2f} m pitch={cam['pitch_deg']:.1f} deg")
    else:
        print("安装外参: 不使用（PnP 只用内参/畸变）")
    print("坐标: 左图为检测与像素坐标；右图为场地地图，yaw 逆时针为正")
    print(f"位姿模式: {args.pose_mode}" + ("（像素-地图点直接 PnP，不依赖安装外参）"
                                         if args.pose_mode == "pnp" else "（地面投影 + 2D 匹配）"))

    cap = open_capture(args.source, args.camera_backend)
    cv2.namedWindow("task3 localization", cv2.WINDOW_NORMAL)
    cv2.setWindowTitle("task3 localization", f"task3 localization - {OPERATOR_NAME}")

    fps = 0.0
    last_time = time.perf_counter()
    last_log = 0.0
    size_printed = False
    frame_idx = 0
    last_pose = None
    last_pose_frame = -10 ** 9

    print("按 q 或 Esc 退出")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("读取视频帧失败，退出")
                break
            frame_idx += 1

            h, w = frame.shape[:2]
            if not size_printed:
                print(f"实际画面分辨率: {w}x{h}")
                size_printed = True
            k_matrix, dist = scaled_intrinsics(cam, w, h)

            # 场地标志
            t0 = time.perf_counter()
            marker_result = marker_model.predict(
                frame, conf=args.marker_conf, imgsz=args.imgsz,
                device=args.device or None, verbose=False,
            )[0]
            marker_ms = (time.perf_counter() - t0) * 1000.0

            # 足球
            t1 = time.perf_counter()
            ball_result = ball_model.predict(
                frame, conf=args.ball_conf, imgsz=args.imgsz,
                device=args.device or None, verbose=False,
            )[0]
            ball_ms = (time.perf_counter() - t1) * 1000.0

            # 解析标志检测，计算载体坐标系地面坐标
            m_boxes = marker_result.boxes
            m_xyxy = m_boxes.xyxy.cpu().numpy()
            m_conf = m_boxes.conf.cpu().numpy()
            m_cls = m_boxes.cls.cpu().numpy().astype(int)
            m_names = marker_result.names

            detections = []
            for i in range(len(m_xyxy)):
                name = (m_names.get(int(m_cls[i]), str(m_cls[i]))
                        if isinstance(m_names, dict) else m_names[int(m_cls[i])])
                short = short_type(name)
                if short is None:
                    continue
                x1, y1, x2, y2 = m_xyxy[i]
                detections.append({
                    "short": short,
                    "conf": float(m_conf[i]),
                    "box": (x1, y1, x2, y2),
                    "uv": (float((x1 + x2) * 0.5), float((y1 + y2) * 0.5)),
                })

            if detections:
                uv = np.array([d["uv"] for d in detections], dtype=np.float64).reshape(-1, 1, 2)
                normalized = cv2.undistortPoints(uv, k_matrix, dist).reshape(-1, 2)
            else:
                normalized = np.empty((0, 2), dtype=np.float64)

            observations = []
            pnp_detections = []
            for detection, (nx, ny) in zip(detections, normalized):
                u, v = detection["uv"]
                if args.pose_mode == "pnp":
                    draw_marker(frame, detection["box"], detection["short"], detection["conf"],
                                int(round(u)), int(round(v)), None,
                                coord_text=f"({u:.0f}, {v:.0f})")
                    pnp_detections.append({
                        "type": detection["short"],
                        "uv": (u, v),
                        "confidence": detection["conf"],
                    })
                else:
                    ground = ray_to_ground(nx, ny, r_bc, origin, max_range)
                    draw_marker(frame, detection["box"], detection["short"], detection["conf"],
                                int(round(u)), int(round(v)), ground)
                    if ground is not None:
                        observations.append({
                            "type": detection["short"],
                            "x": float(ground[0]),
                            "y": float(ground[1]),
                            "confidence": detection["conf"],
                        })

            # 解析足球检测
            b_boxes = ball_result.boxes
            b_xyxy = b_boxes.xyxy.cpu().numpy()
            b_conf = b_boxes.conf.cpu().numpy()
            balls = []
            for i in range(len(b_xyxy)):
                draw_ball(frame, b_xyxy[i], float(b_conf[i]))
                x1, y1, x2, y2 = b_xyxy[i]
                balls.append({
                    "confidence": float(b_conf[i]),
                    "center_u": int(round((x1 + x2) * 0.5)),
                    "center_v": int(round((y1 + y2) * 0.5)),
                    "bbox": [float(x1), float(y1), float(x2), float(y2)],
                })

            # 地图匹配与位姿
            if args.pose_mode == "pnp":
                prev_pose = None
                if (last_pose is not None
                        and frame_idx - last_pose_frame <= PNP_TEMPORAL_MAX_GAP):
                    prev_pose = last_pose
                pose = solve_pose_pnp(pnp_detections, field, k_matrix, dist,
                                      prev_pose=prev_pose)
                if pose["valid"]:
                    last_pose = pose
                    last_pose_frame = frame_idx
                display_markers = [
                    {"type": d["type"], "u": d["uv"][0], "v": d["uv"][1],
                     "confidence": d["confidence"]}
                    for d in pnp_detections
                ]
            else:
                pose = solve_pose(observations, field, cam)
                display_markers = observations

            # 帧率
            now = time.perf_counter()
            dt = now - last_time
            last_time = now
            if dt > 0:
                inst_fps = 1.0 / dt
                fps = inst_fps if fps == 0.0 else 0.9 * fps + 0.1 * inst_fps

            # 操作者姓名（拼音）与逐帧刷新的时间戳
            stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            marker_info = (f"pnp {pose['marker_count']}" if args.pose_mode == "pnp"
                           else f"ground {len(observations)}")
            put_text(frame,
                     f"FPS {fps:4.1f} | marker {marker_ms:5.1f} ms | ball {ball_ms:5.1f} ms | "
                     f"balls {len(balls)} | markers {len(detections)} ({marker_info})",
                     (10, 60), 0.62, (0, 255, 0), 2)

            draw_json_panel(frame, [marker_json(item) for item in display_markers])

            left_panel = fit_panel(frame, LEFT_W, LEFT_H)
            map_panel = draw_map(field, pose, fps, marker_ms + ball_ms)
            canvas = np.hstack([left_panel, map_panel])
            cv2.line(canvas, (LEFT_W, 0), (LEFT_W, LEFT_H - 1), (200, 200, 200), 2)

            # 姓名/时间戳画在整个分屏画面上，左右两屏都能看到
            canvas_w = canvas.shape[1]
            put_text(canvas, OPERATOR_NAME, (10, 30), 0.9, (0, 255, 255), 2)
            tw = cv2.getTextSize(stamp, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)[0][0]
            put_text(canvas, stamp, (canvas_w - tw - 10, 30), 0.6, (0, 255, 255), 1)

            if now - last_log >= 1.0:
                payload = {
                    "camera_pose_map": {
                        "x_camera_map": None if pose["x_camera_map"] is None else round(pose["x_camera_map"], 2),
                        "y_camera_map": None if pose["y_camera_map"] is None else round(pose["y_camera_map"], 2),
                        "yaw_camera_map": None if pose["yaw_camera_map"] is None else round(pose["yaw_camera_map"], 1),
                        "valid": pose["valid"],
                        "status": pose["status"],
                        "temporal": bool(pose.get("temporal", False)),
                        "marker_count": pose["marker_count"],
                        "residual": None if pose["residual"] is None else round(pose["residual"], 2),
                        "residual_unit": pose.get("residual_unit", "m"),
                    },
                    "markers": [marker_json(item) for item in display_markers],
                    "balls": [
                        {
                            "confidence": round(ball["confidence"], 2),
                            "center_u": ball["center_u"],
                            "center_v": ball["center_v"],
                        }
                        for ball in balls
                    ],
                }
                print(json.dumps(payload, ensure_ascii=False), flush=True)
                last_log = now

            cv2.imshow("task3 localization", canvas)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
