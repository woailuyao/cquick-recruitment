"""Task1：用训练好的 ball.pt 对实时摄像头画面做球检测，并实时标出球的位置。

用法（在 scr/ 目录下执行）：
    python task1.py --list-cameras       # 先看有哪些摄像头
    python task1.py                       # 自动选摄像头（优先外接 USB）
    python task1.py --source 1            # 指定 1 号摄像头
    python task1.py --source usb          # 自动选最后一个（通常 USB）
    python task1.py --source rtsp://...   # 网络/RTSP 视频流
    python task1.py --conf 0.3 --device cpu
按 q 或 Esc 退出。

ball.pt 默认从本文件所在目录读取，与运行时的当前工作目录无关。
"""

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
from ultralytics import YOLO

BASE_DIR = Path(__file__).resolve().parent
DEFAULT_MODEL = BASE_DIR / "ball.pt"
OPERATOR_NAME = "sumaolin"  # 画面左上角显示的操作者姓名（拼音）


def parse_args():
    parser = argparse.ArgumentParser(description="实时摄像头球检测（ball.pt）")
    parser.add_argument("--model", default=str(DEFAULT_MODEL), help="模型路径，默认 scr/ball.pt")
    parser.add_argument("--source", default="auto",
                        help="auto/usb/builtin、摄像头编号(0/1/...)、视频文件或 RTSP/HTTP 流地址")
    parser.add_argument("--camera-backend", choices=("auto", "dshow", "msmf"), default="auto",
                        help="Windows 摄像头后端，默认 auto（优先 DirectShow）")
    parser.add_argument("--list-cameras", action="store_true", help="列出可用摄像头后退出")
    parser.add_argument("--conf", type=float, default=0.25, help="置信度阈值")
    parser.add_argument("--imgsz", type=int, default=640, help="推理分辨率")
    parser.add_argument("--device", default="", help="推理设备：'0' 用 GPU，'cpu' 用 CPU，留空自动选择")
    return parser.parse_args()


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


def draw_detection(frame, x1, y1, x2, y2, conf, best=False):
    color = (0, 255, 0) if best else (0, 200, 255)
    thickness = 3 if best else 2
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)

    label = f"ball {conf:.2f}"
    (tw, th), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
    ty = max(y1, th + baseline)
    cv2.rectangle(frame, (x1, ty - th - baseline), (x1 + tw, ty), color, -1)
    cv2.putText(frame, label, (x1, ty - baseline // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2, cv2.LINE_AA)

    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
    cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
    cv2.line(frame, (cx - 12, cy), (cx + 12, cy), (0, 0, 255), 1)
    cv2.line(frame, (cx, cy - 12), (cx, cy + 12), (0, 0, 255), 1)
    cv2.putText(frame, f"({cx}, {cy})", (cx + 15, cy - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
    return cx, cy


def put_text(frame, text, org, scale, color, thickness=1):
    """带黑色描边的文字，避免在明亮或杂乱背景上看不清。"""
    cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale,
                (0, 0, 0), thickness + 2, cv2.LINE_AA)
    cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale,
                color, thickness, cv2.LINE_AA)


def main():
    args = parse_args()

    if args.list_cameras:
        list_cameras(args.camera_backend)
        return

    if not Path(args.model).is_file():
        raise SystemExit(f"模型文件不存在: {args.model}")

    model = YOLO(args.model)
    cap = open_capture(args.source, args.camera_backend)

    fps = 0.0
    last_time = time.perf_counter()

    print("按 q 或 Esc 退出")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("读取视频帧失败，退出")
                break

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

            h, w = frame.shape[:2]
            best_center = None
            if len(xyxy) > 0:
                best_idx = int(confs.argmax())
                for i, ((x1, y1, x2, y2), conf) in enumerate(zip(xyxy, confs)):
                    center = draw_detection(
                        frame, int(x1), int(y1), int(x2), int(y2),
                        float(conf), best=(i == best_idx),
                    )
                    if i == best_idx:
                        best_center = center

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

            put_text(frame, f"FPS {fps:5.1f} | infer {infer_ms:5.1f} ms | balls {len(xyxy)}",
                     (10, 60), 0.7, (0, 255, 0), 2)
            if best_center is not None:
                cx, cy = best_center
                put_text(frame, f"center=({cx}, {cy})  norm=({cx / w:.3f}, {cy / h:.3f})",
                         (10, 90), 0.6, (0, 0, 255), 2)

            cv2.imshow("ball detection (task1)", frame)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
