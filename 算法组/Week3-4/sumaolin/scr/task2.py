"""Task2：用 marker.pt 实时检测场地标志，并算出标志在载体坐标系下的地面坐标。

每帧输出（终端与画面中同步显示）类似：
    [{"type": "T", "x": 2.15, "y": 0.43, "confidence": 0.92},
     {"type": "L", "x": 3.20, "y": -0.62, "confidence": 0.87}]

坐标约定：
    载体坐标系 x 向前、y 向左、z 向上，地面平面 z=0，单位 m。
    标志像素点先去畸变、反投影成相机射线，再用相机安装外参转到载体坐标系，
    与 z=0 平面求交，得到特征点的地面坐标 (x, y)。
    当前特征点取检测框中心；L/T/X 的精确交点可在检测框内做线段细化。

相机参数：
    内参已于 2026-11-15 用棋盘格视频标定（1280x720），并写入下方 CAMERA；
    安装外参（光心高度/俯角/横滚/偏航/偏移）仍是占位值，必须实测后替换：
      calib_width / calib_height  内参标定时的图像分辨率
      fx / fy / cx / cy           相机内参
      dist                        畸变系数 [k1, k2, p1, p2, k3]
      mount_x / mount_y           光心相对载体原点的平移，m
      mount_height                光心离地高度，m
      pitch_deg                   光轴向下俯角，deg
      roll_deg / yaw_deg          绕光轴横滚 / 绕载体 z 轴偏航，deg
      max_range                   地面交点有效距离上限，m
    也可以用 --calib 加载 config/camera_intrinsics.yaml；安装参数还可用
    --mount-height / --pitch / --roll / --yaw 临时覆盖。

用法（在 scr/ 目录下执行）：
    python task2.py
    python task2.py --calib ../config/camera_intrinsics.yaml
    python task2.py --mount-height 0.45 --pitch 30
按 q 或 Esc 退出。
"""

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml
from ultralytics import YOLO

BASE_DIR = Path(__file__).resolve().parent
DEFAULT_MODEL = BASE_DIR / "marker.pt"
DEFAULT_CALIB = BASE_DIR.parent / "config" / "camera_intrinsics.yaml"
OPERATOR_NAME = "sumaolin"  # 画面左上角显示的操作者姓名（拼音）

# ---------------- 相机参数：内参已标定，安装外参仍为占位值 ----------------
CAMERA = {
    "calib_width": 1280,       # 内参标定时的图像宽度
    "calib_height": 720,       # 内参标定时的图像高度
    "fx": 1563.6194,           # 水平焦距（像素），2026-11-15 棋盘格标定
    "fy": 1632.7168,           # 垂直焦距（像素）
    "cx": 640.0,               # 主点 u（标定时固定到图像中心）
    "cy": 360.0,               # 主点 v
    "dist": (0.252780, -1.094336, -0.032371, 0.004958, 0.0),  # k1, k2, p1, p2, k3
    "mount_x": 0.0,            # 光心相对载体原点的 x（m，待实测）
    "mount_y": 0.0,            # 光心相对载体原点的 y（m，待实测）
    "mount_height": 0.60,      # 光心离地高度（m，占位值）
    "pitch_deg": 20.0,         # 光轴向下俯角（deg，占位值）
    "roll_deg": 0.0,           # 绕光轴横滚（deg，待实测）
    "yaw_deg": 0.0,            # 绕载体 z 轴偏航（deg，待实测）
    "max_range": 8.0,          # 地面交点有效距离上限（m）
}

# YOLO 类别名 -> 输出里的短类型
TYPE_MAP = {
    "LCross": "L",
    "TCross": "T",
    "XCross": "X",
    "L": "L",
    "T": "T",
    "X": "X",
}

TYPE_COLORS = {
    "L": (0, 220, 0),     # 绿
    "T": (255, 220, 0),   # 青
    "X": (255, 0, 255),   # 品红
}


def parse_args():
    parser = argparse.ArgumentParser(description="marker.pt 实时场地标志地面坐标（task2）")
    parser.add_argument("--model", default=str(DEFAULT_MODEL), help="模型路径，默认 scr/marker.pt")
    parser.add_argument("--source", default="auto",
                        help="auto/usb/builtin、摄像头编号(0/1/...)、视频文件或 RTSP/HTTP 流地址")
    parser.add_argument("--camera-backend", choices=("auto", "dshow", "msmf"), default="auto",
                        help="Windows 摄像头后端，默认 auto（优先 DirectShow）")
    parser.add_argument("--list-cameras", action="store_true", help="列出可用摄像头后退出")
    parser.add_argument("--conf", type=float, default=0.2, help="置信度阈值，标志点可适当调低")
    parser.add_argument("--imgsz", type=int, default=640, help="推理分辨率")
    parser.add_argument("--device", default="", help="推理设备：'0' 用 GPU，'cpu' 用 CPU，留空自动选择")
    parser.add_argument("--calib", default=str(DEFAULT_CALIB) if DEFAULT_CALIB.is_file() else "",
                        help="相机标定 yaml 路径（默认加载 config/camera_intrinsics.yaml）")
    parser.add_argument("--mount-height", type=float, default=None, help="覆盖光心离地高度（m）")
    parser.add_argument("--pitch", type=float, default=None, help="覆盖光轴向下俯角（deg）")
    parser.add_argument("--roll", type=float, default=None, help="覆盖横滚角（deg）")
    parser.add_argument("--yaw", type=float, default=None, help="覆盖偏航角（deg）")
    return parser.parse_args()


def build_camera(args):
    """内置参数 + 标定 yaml + 命令行覆盖，合成最终相机配置。"""
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
        print("[提示] 未指定 --calib，使用脚本内置的占位相机参数，x/y 仅为近似值。")

    overrides = {
        "mount_height": args.mount_height,
        "pitch_deg": args.pitch,
        "roll_deg": args.roll,
        "yaw_deg": args.yaw,
    }
    for key, value in overrides.items():
        if value is not None:
            cam[key] = value

    if cam["mount_height"] <= 0:
        raise SystemExit("相机安装高度 mount_height 必须大于 0")
    if cam["fx"] <= 0 or cam["fy"] <= 0:
        raise SystemExit("相机内参 fx/fy 必须大于 0")
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
    return cap


def scaled_intrinsics(cam, frame_w, frame_h):
    """实际画面分辨率与标定分辨率不同时，按比例缩放内参。"""
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
    """相机光学坐标系 -> 载体坐标系的旋转矩阵（列 = 相机三轴在载体下的方向）。

    相机光学坐标系：x 向右、y 向下、z 向前（光轴）。
    载体坐标系：x 向前、y 向左、z 向上。
    """
    pitch = np.radians(pitch_deg)
    roll = np.radians(roll_deg)
    yaw = np.radians(yaw_deg)

    right = np.array([0.0, -1.0, 0.0])                    # 相机 x
    down = np.array([-np.sin(pitch), 0.0, -np.cos(pitch)])  # 相机 y
    forward = np.array([np.cos(pitch), 0.0, -np.sin(pitch)])  # 相机 z

    # 绕光轴横滚
    right_r = right * np.cos(roll) - down * np.sin(roll)
    down_r = right * np.sin(roll) + down * np.cos(roll)

    # 绕载体 z 轴偏航
    rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0.0],
        [np.sin(yaw), np.cos(yaw), 0.0],
        [0.0, 0.0, 1.0],
    ])
    return np.column_stack([rz @ right_r, rz @ down_r, rz @ forward])


def ray_to_ground(nx, ny, r_bc, origin, max_range):
    """归一化像素坐标 -> 载体坐标系地面点；射线朝上或交点越界时返回 None。"""
    direction = r_bc @ np.array([nx, ny, 1.0])
    if direction[2] >= -1e-6:          # 射线水平或朝上，打不到地面
        return None
    lam = -origin[2] / direction[2]
    if lam <= 0:                       # 交点不在相机前方
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
    """带黑色描边的文字，避免在明亮或杂乱背景上看不清。"""
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


def draw_marker(frame, box, short, conf, u, v, ground, color):
    x1, y1, x2, y2 = [int(round(v_)) for v_ in box]
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    draw_box_label(frame, f"{short} {conf:.2f}", x1, y1, color)

    # 特征点（当前为检测框中心）
    cv2.circle(frame, (u, v), 4, (0, 0, 255), -1)
    cv2.line(frame, (u - 10, v), (u + 10, v), (0, 0, 255), 1)
    cv2.line(frame, (u, v - 10), (u, v + 10), (0, 0, 255), 1)

    if ground is None:
        put_text(frame, "no ground", (u + 12, v + 20), 0.5, (0, 0, 255), 1)
    else:
        gx, gy = ground
        put_text(frame, f"({gx:.2f}, {gy:.2f}) m", (u + 12, v + 20), 0.55, color, 1)


def draw_json_panel(frame, observed):
    """画面左下角用半透明底板列出与终端一致的 JSON 数据。"""
    if not observed:
        return
    lines = [json.dumps(item, ensure_ascii=False) for item in observed]
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


def main():
    args = parse_args()

    if args.list_cameras:
        list_cameras(args.camera_backend)
        return

    if not Path(args.model).is_file():
        raise SystemExit(f"模型文件不存在: {args.model}")

    cam = build_camera(args)
    model = YOLO(args.model)
    r_bc = carrier_from_camera(cam["pitch_deg"], cam["roll_deg"], cam["yaw_deg"])
    origin = np.array([cam["mount_x"], cam["mount_y"], cam["mount_height"]], dtype=np.float64)
    max_range = float(cam["max_range"])

    print(f"模型类别: {model.names}")
    print("类型映射: LCross->L, TCross->T, XCross->X")
    print(f"相机: {cam['calib_width']}x{cam['calib_height']} "
          f"fx={cam['fx']:.1f} fy={cam['fy']:.1f} cx={cam['cx']:.1f} cy={cam['cy']:.1f} | "
          f"height={cam['mount_height']:.2f} m pitch={cam['pitch_deg']:.1f} deg")
    print("坐标: 载体坐标系 x 向前 / y 向左, 地面 z=0, 单位 m")

    cap = open_capture(args.source, args.camera_backend)

    fps = 0.0
    last_time = time.perf_counter()
    last_log = 0.0
    size_printed = False

    print("按 q 或 Esc 退出")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("读取视频帧失败，退出")
                break

            h, w = frame.shape[:2]
            if not size_printed:
                print(f"实际画面分辨率: {w}x{h}")
                size_printed = True
            k_matrix, dist = scaled_intrinsics(cam, w, h)

            t0 = time.perf_counter()
            result = model.predict(
                frame,
                conf=args.conf,
                imgsz=args.imgsz,
                device=args.device or None,
                verbose=False,
            )[0]
            infer_ms = (time.perf_counter() - t0) * 1000.0

            boxes = result.boxes
            xyxy = boxes.xyxy.cpu().numpy()
            confs = boxes.conf.cpu().numpy()
            classes = boxes.cls.cpu().numpy().astype(int)
            names = result.names

            detections = []
            for i in range(len(xyxy)):
                name = names.get(int(classes[i]), str(classes[i])) if isinstance(names, dict) else names[int(classes[i])]
                short = short_type(name)
                if short is None:
                    continue
                x1, y1, x2, y2 = xyxy[i]
                u = float((x1 + x2) * 0.5)
                v = float((y1 + y2) * 0.5)
                detections.append({
                    "short": short,
                    "conf": float(confs[i]),
                    "box": (x1, y1, x2, y2),
                    "uv": (u, v),
                })

            # 一次性完成所有特征点的去畸变/归一化
            if detections:
                uv = np.array([d["uv"] for d in detections], dtype=np.float64).reshape(-1, 1, 2)
                normalized = cv2.undistortPoints(uv, k_matrix, dist).reshape(-1, 2)
            else:
                normalized = np.empty((0, 2), dtype=np.float64)

            observed = []
            for detection, (nx, ny) in zip(detections, normalized):
                ground = ray_to_ground(nx, ny, r_bc, origin, max_range)
                color = TYPE_COLORS.get(detection["short"], (255, 255, 255))
                u, v = detection["uv"]
                draw_marker(frame, detection["box"], detection["short"], detection["conf"],
                            int(round(u)), int(round(v)), ground, color)
                if ground is not None:
                    observed.append({
                        "type": detection["short"],
                        "x": round(ground[0], 2),
                        "y": round(ground[1], 2),
                        "confidence": round(detection["conf"], 2),
                    })

            observed.sort(key=lambda item: (item["x"], item["y"]))

            # 帧率平滑
            now = time.perf_counter()
            dt = now - last_time
            last_time = now
            if dt > 0:
                inst_fps = 1.0 / dt
                fps = inst_fps if fps == 0.0 else 0.9 * fps + 0.1 * inst_fps

            # 操作者姓名（拼音）与逐帧刷新的时间戳
            stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            put_text(frame, OPERATOR_NAME, (10, 30), 0.9, (0, 255, 255), 2)
            tw = cv2.getTextSize(stamp, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)[0][0]
            put_text(frame, stamp, (w - tw - 10, 30), 0.6, (0, 255, 255), 1)

            put_text(frame, f"FPS {fps:5.1f} | infer {infer_ms:5.1f} ms | "
                            f"ground {len(observed)}/{len(detections)}",
                     (10, 60), 0.65, (0, 255, 0), 2)

            draw_json_panel(frame, observed)

            if now - last_log >= 0.5:
                print(json.dumps(observed, ensure_ascii=False), flush=True)
                last_log = now

            cv2.imshow("marker ground position (task2)", frame)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
