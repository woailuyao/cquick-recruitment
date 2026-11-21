"""USB 摄像头原始数据采集；R 录像，S 棋盘格照片，P 驱动设置，C 确认返回，Q 退出。

仅接受摄像头编号或 Linux /dev/videoX。视频文件只用于采集后的完整性检查。
依赖：Python 3.10+、OpenCV、NumPy（OpenCV 已依赖 NumPy）。
"""

import argparse
import csv
import hashlib
import json
import math
import platform
import re
import sys
import time
import uuid
from collections import deque
from datetime import datetime
from pathlib import Path

import cv2


NAME = "kuaizhenhua"
KIT_ROOT = Path(__file__).resolve().parents[1]
TIMING_NOTE = (
    "Host receipt timestamps and loop/record throughput are not sensor exposure "
    "timestamps or verified sensor FPS. AVI uses constant storage FPS; use CSV "
    "for actual host receipt intervals. No calibration or focus verification is performed."
)
DRIVER_NOTE = (
    "Driver-reported properties are unverified, may be unsupported, and use backend-specific units. "
    "A successful set() does not guarantee the negotiated capture format. "
    "No exposure or white-balance values are forced by this script."
)


def now_iso():
    """使用操作系统当前本地时区，不写死采集日期或时区。"""
    return datetime.now().astimezone().isoformat(timespec="microseconds")


def unique_suffix():
    return datetime.now().astimezone().strftime("%Y%m%dT%H%M%S_%f") + "_" + uuid.uuid4().hex[:8]


def device_value(value, system=None):
    system = system or platform.system()
    if re.fullmatch(r"[0-9]+", str(value)):
        return int(value)
    if system == "Linux" and re.fullmatch(r"/dev/video[0-9]+", str(value)):
        return str(value)
    raise ValueError("--device must be a nonnegative camera index, or /dev/videoX on Linux; input video files are not accepted")


def backend_value(name, system=None):
    system = system or platform.system()
    supported = {"auto": cv2.CAP_ANY}
    if system == "Windows":
        supported.update(dshow=cv2.CAP_DSHOW, msmf=cv2.CAP_MSMF)
    elif system == "Linux":
        supported.update(v4l2=cv2.CAP_V4L2)
    if name not in supported:
        raise ValueError(f"backend {name!r} is unavailable on {system}; choose {', '.join(supported)}")
    return supported[name]


def parser():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--device", default="0", help="camera index, or Linux /dev/videoX; never an input video")
    p.add_argument("--backend", choices=("auto", "dshow", "msmf", "v4l2"), default="auto")
    p.add_argument("--output", type=Path, default=KIT_ROOT / "local_data")
    p.add_argument("--session", required=True, help="ASCII identifier, e.g. ball_train01 or chessboard01")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=float, default=30.0, help="requested camera FPS; not a guarantee")
    p.add_argument("--fourcc", choices=("MJPG", "YUY2", "default"), default="MJPG",
                   help="requested camera format, not AVI codec; actual driver format may differ")
    p.add_argument("--warmup-frames", type=int, default=30,
                   help="legacy compatibility option (2-600); readiness now uses >=2 seconds and >=2 frames")
    return p


def check_args(args):
    device = device_value(args.device)
    backend = backend_value(args.backend)
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", args.session):
        raise ValueError("--session must be 1-64 ASCII letters/digits/underscores/hyphens, starting with a letter/digit")
    if args.width <= 0 or args.height <= 0:
        raise ValueError("width and height must be positive")
    if not math.isfinite(args.fps) or not 0 < args.fps <= 240:
        raise ValueError("--fps must be finite and in (0, 240]")
    if not 2 <= args.warmup_frames <= 600:
        raise ValueError("--warmup-frames must be between 2 and 600")
    if args.fourcc not in ("MJPG", "YUY2", "default"):
        raise ValueError("--fourcc must be MJPG, YUY2, or default")
    return device, backend


class ReceiptStats:
    """相邻帧到达时间测得的处理吞吐率，不能称为传感器帧率。"""
    def __init__(self):
        self.count = 0
        self.first = None
        self.last = None

    def add(self, tick):
        if self.first is None:
            self.first = tick
        self.last = tick
        self.count += 1

    def summary(self):
        seconds = (self.last - self.first) / 1e9 if self.count >= 2 else 0.0
        return {
            "frame_count": self.count,
            "first_to_last_receipt_seconds": seconds,
            "host_receipt_throughput_fps": (self.count - 1) / seconds if seconds > 0 else None,
            "not_sensor_fps": True,
        }


class RollingReceiptFPS:
    """最近五秒的主机收帧吞吐率，保留窗口边界之前的一帧。"""
    def __init__(self, window_seconds=5.0, minimum_seconds=5.0):
        self.ticks = deque()
        self.window_ns = int(window_seconds * 1e9)
        self.minimum_ns = int(minimum_seconds * 1e9)

    def add(self, tick):
        self.ticks.append(tick)
        cutoff = tick - self.window_ns
        # 保留跨过窗口边界的间隔，不能仅剩突发快速读取的两帧。
        while len(self.ticks) > 2 and self.ticks[1] <= cutoff:
            self.ticks.popleft()
        elapsed = self.ticks[-1] - self.ticks[0]
        if len(self.ticks) < 2 or elapsed < self.minimum_ns or elapsed <= 0:
            return None
        return (len(self.ticks) - 1) * 1e9 / elapsed


def warmup_ready(stats):
    return stats.count >= 2 and stats.summary()["first_to_last_receipt_seconds"] >= 2.0


def choose_storage_fps(warmup_stats, fallback):
    measured = warmup_stats.summary()["host_receipt_throughput_fps"]
    if measured is not None and math.isfinite(measured) and 0 < measured <= 240:
        return measured, "measured_warmup_host_receipt_throughput"
    if not math.isfinite(fallback) or not 0 < fallback <= 240:
        raise ValueError("Storage FPS fallback must be finite and in (0, 240]")
    return fallback, "fallback_requested_fps: warmup measurement missing, nonpositive, nonfinite, or above 240 FPS"


def configure_camera(cap, args):
    # 先请求传输格式，再请求分辨率和帧率；驱动仍可能忽略请求。
    requests = []
    if args.fourcc != "default":
        requests.append(("fourcc", cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc)))
    requests.extend((("width", cv2.CAP_PROP_FRAME_WIDTH, args.width),
                     ("height", cv2.CAP_PROP_FRAME_HEIGHT, args.height),
                     ("fps", cv2.CAP_PROP_FPS, args.fps)))
    if hasattr(cv2, "CAP_PROP_CONVERT_RGB"):
        requests.append(("convert_rgb", cv2.CAP_PROP_CONVERT_RGB, 1))
    if args.fourcc != "default":
        # DSHOW 设置尺寸或 FPS 可能重协商格式，最后再次请求用户选择的格式。
        requests.append(("fourcc_reapply", cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc)))
    accepted = {}
    for key, prop, value in requests:
        try:
            accepted[key] = bool(cap.set(prop, value))
        except cv2.error:
            accepted[key] = False
    return accepted


def camera_properties(cap):
    """仅记录驱动报告值，不推断属性受支持或颜色已经校准。"""
    result = {"backend": cap.getBackendName(), "driver_reported_unverified": True, "note": DRIVER_NOTE}
    names = {
        "width": "FRAME_WIDTH", "height": "FRAME_HEIGHT", "fps": "FPS",
        "fourcc_code": "FOURCC", "format": "FORMAT", "convert_rgb": "CONVERT_RGB",
        "exposure": "EXPOSURE", "auto_exposure": "AUTO_EXPOSURE", "auto_wb": "AUTO_WB",
        "wb_temperature": "WB_TEMPERATURE", "brightness": "BRIGHTNESS",
        "gain": "GAIN", "saturation": "SATURATION", "contrast": "CONTRAST",
        "hue": "HUE", "gamma": "GAMMA",
    }
    for key, name in names.items():
        prop = getattr(cv2, "CAP_PROP_" + name, None)
        try:
            result[key] = finite_or_none(cap.get(prop)) if prop is not None else None
        except cv2.error:
            result[key] = None
    code = result["fourcc_code"]
    result["fourcc"] = None
    if code is not None and code > 0:
        raw = int(code).to_bytes(4, "little", signed=False) if code <= 0xFFFFFFFF else b""
        result["fourcc"] = raw.decode("ascii", errors="replace").rstrip("\x00") or None
    return result


def verify_video(path, expected_count, size):
    """只重读首尾帧及容器帧数，避免结束长录像时再完整读一遍。"""
    cap = cv2.VideoCapture(str(path))
    result = {"method": "container_frame_count_and_decoded_first_last", "expected_frames": expected_count}
    try:
        result["opened"] = bool(cap.isOpened())
        if not result["opened"]:
            result["ok"] = False
            return result
        reported = cap.get(cv2.CAP_PROP_FRAME_COUNT)
        count = int(round(reported)) if math.isfinite(reported) else -1
        result["reported_frames"] = count
        result["reported_storage_fps"] = finite_or_none(cap.get(cv2.CAP_PROP_FPS))
        first_ok, first = cap.read()
        first_ok = bool(first_ok and first is not None and first.shape[:2] == (size[1], size[0]))
        last_ok = first_ok
        if count > 1:
            seek_ok = cap.set(cv2.CAP_PROP_POS_FRAMES, count - 1)
            last_ok, last = cap.read()
            last_ok = bool(seek_ok and last_ok and last is not None and last.shape[:2] == (size[1], size[0]))
        result.update(first_frame_decoded=first_ok, last_frame_decoded=last_ok)
        result["ok"] = bool(expected_count > 0 and count == expected_count and first_ok and last_ok)
        return result
    finally:
        cap.release()


def finite_or_none(value):
    return float(value) if math.isfinite(value) else None


class Clip:
    def __init__(self, folder, number, size, storage_fps, fps_source, session):
        self.path = folder / f"clip_{number:03d}_{unique_suffix()}.avi"
        self.stats = ReceiptStats()
        self.size = size
        self.closed = False
        self.info = {
            "file": self.path.name, "session": session, "started_at": now_iso(),
            "codec": "MJPG", "container": "AVI", "raw_unannotated_frames": True,
            "compression": "lossy MJPEG; raw means no overlay/resize, not lossless encoding",
            "storage_fps": storage_fps, "storage_fps_source": fps_source,
        }
        self.writer = cv2.VideoWriter(str(self.path), cv2.VideoWriter_fourcc(*"MJPG"), storage_fps, size)
        if not self.writer.isOpened():
            self.writer.release()
            raise RuntimeError(f"Cannot open MJPG AVI writer: {self.path}. Try a writable ASCII-only --output path if this OpenCV build cannot write Unicode paths.")

    def write(self, frame, tick, received_at):
        self.writer.write(frame)
        self.stats.add(tick)
        if self.stats.count == 1:
            self.info["first_frame_received_at"] = received_at
        self.info["last_frame_received_at"] = received_at
        return self.stats.count

    def close(self):
        if self.closed:
            return self.info
        self.closed = True
        self.writer.release()
        self.info.update(ended_at=now_iso(), timing=self.stats.summary())
        self.info["nominal_playback_seconds"] = self.stats.count / self.info["storage_fps"]
        self.info["validation"] = verify_video(self.path, self.stats.count, self.size)
        return self.info


def save_png(folder, frame, received_at, frame_index, session):
    path = folder / f"chessboard_{unique_suffix()}.png"
    ok, encoded = cv2.imencode(".png", frame)
    if not ok:
        raise RuntimeError("PNG encoding failed")
    # imwrite 在部分 Windows 构建中不支持中文路径，tofile 使用 Python 路径。
    encoded.tofile(str(path))
    return {
        "file": path.name, "session": session, "frame_index": frame_index,
        "received_at": received_at, "saved_at": now_iso(),
        "raw_unannotated_frame": True,
        "raw_bgr_pixel_sha256": hashlib.sha256(frame.tobytes()).hexdigest(),
    }


def make_preview(frame, received_at, loop_fps, recording, read_ms=None, ready=True, settings_pending=False):
    # 只在副本上叠字，传给 VideoWriter / PNG 编码器的原帧不变。
    preview = frame.copy()
    height, width = frame.shape[:2]
    fps_text = f"{loop_fps:.1f}" if loop_fps is not None else "n/a"
    read_text = f"{read_ms:.1f}" if read_ms is not None else "n/a"
    lines = [
        NAME,
        received_at,
        f"{width}x{height}  host FPS (~5s): {fps_text}  read: {read_text} ms",
        "Host receipt rate, NOT sensor FPS",
        f"REC: {'ON' if recording else 'OFF'}  {'READY' if ready else 'WARMUP (>=2s, >=2 frames)'}",
        "R: record/stop  S: raw PNG  P: driver settings  Q: quit",
    ]
    if settings_pending:
        lines[-2:] = ["REC: OFF  R/S/P paused", "Press C after closing driver settings  Q: quit"]
    for number, line in enumerate(lines):
        cv2.putText(preview, line, (14, 28 + number * 28), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(preview, line, (14, 28 + number * 28), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (0, 255, 0), 1, cv2.LINE_AA)
    return preview


def run(args):
    device, backend = check_args(args)  # 先拒绝视频输入，再创建文件／打开设备。
    root = args.output.expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    folder = root / f"{args.session}_{unique_suffix()}"
    folder.mkdir(exist_ok=False)
    meta_path = folder / "session.json"
    started_at = now_iso()
    meta = {
        "schema_version": 1, "session": args.session, "run_folder": folder.name,
        "name": NAME, "started_at": started_at, "capture_dates_local": [],
        "platform": platform.platform(), "python": platform.python_version(),
        "opencv": cv2.__version__, "device": device,
        "requested": {"width": args.width, "height": args.height, "fps": args.fps,
                      "backend": args.backend, "fourcc": args.fourcc, "convert_rgb": 1},
        "status": "starting", "timing_note": TIMING_NOTE,
        "calibration_performed": False, "focus_verified": False,
        "frames_csv": "frames.csv", "clips": [], "photos": [], "errors": [],
        "driver_property_note": DRIVER_NOTE, "driver_settings_events": [],
        "driver_settings_return_pending": False,
        "preview_fps_window_seconds": 5.0, "preview_fps_minimum_seconds": 5.0,
        "warmup_minimum_seconds": 2.0,
    }
    cap = None
    clip = None
    csv_file = None
    all_stats, warmup_stats, preview_stats = ReceiptStats(), ReceiptStats(), ReceiptStats()
    rolling_fps = RollingReceiptFPS()
    settings_event = None
    capture_dates = set()
    window_name = "Lab capture"

    def persist():
        # 临时文件替换，避免正常写入中断留下一半 JSON。
        temporary = meta_path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(meta, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
        temporary.replace(meta_path)

    def error(stage, exc):
        meta["errors"].append({"at": now_iso(), "stage": stage, "message": str(exc)})

    def finish_clip():
        nonlocal clip
        if clip is None:
            return
        active = clip
        clip = None
        try:
            info = active.close()
        except Exception as exc:
            active.info["validation"] = {"ok": False, "error": str(exc)}
            meta["clips"].append(active.info)
            raise
        meta["clips"].append(info)
        persist()
        if not info["validation"]["ok"]:
            raise RuntimeError(f"Saved clip failed validation: {active.path.name}; see session.json")

    try:
        persist()
        csv_file = (folder / "frames.csv").open("w", newline="", encoding="utf-8-sig")
        rows = csv.writer(csv_file)
        rows.writerow(["frame_index", "receipt_iso_local", "receipt_perf_counter_ns", "phase", "clip_file", "clip_frame_index"])
        cap = cv2.VideoCapture(device, backend)
        if not cap.isOpened():
            raise RuntimeError(f"Cannot open camera {device!r} with backend {args.backend}")
        meta["property_set_accepted"] = configure_camera(cap, args)
        meta["reported"] = camera_properties(cap)
        meta["status"] = "warming_up"
        persist()
        print(f"Session folder: {folder}")
        print("Preview opens on first frame; R: record after >=2s / >=2 frames; S: raw PNG; P: DSHOW settings; C: confirm return; Q: quit.")
        print(f"Requested capture FOURCC: {args.fourcc}; driver reports: {meta['reported']['fourcc']!r} (unverified).")
        storage_fps = None
        fps_source = None
        last_flush_tick = time.perf_counter_ns()
        while True:
            read_started = time.perf_counter_ns()
            ok, frame = cap.read()
            tick = time.perf_counter_ns()
            read_ms = (tick - read_started) / 1e6
            received_at = now_iso()
            if not ok or frame is None:
                raise RuntimeError("Camera read failed; check USB connection and camera ownership")
            if frame.ndim != 3 or frame.shape[2] != 3 or frame.dtype.name != "uint8":
                raise RuntimeError("Camera must return uint8 BGR frames with three channels")
            actual_size = [int(frame.shape[1]), int(frame.shape[0])]
            meta["actual_frame_size"] = actual_size
            if actual_size != [args.width, args.height]:
                raise RuntimeError(f"Actual frame size {actual_size[0]}x{actual_size[1]} differs from requested {args.width}x{args.height}; frames are not resized")
            all_stats.add(tick)
            capture_dates.add(received_at[:10])
            meta["capture_dates_local"] = sorted(capture_dates)
            phase = "warmup" if storage_fps is None else "preview"
            if settings_event is not None:
                phase = "driver_settings"
            clip_file, clip_index = "", ""
            if clip is not None:
                clip_index = clip.write(frame, tick, received_at)
                clip_file = clip.path.name
                phase = "recording"
            rows.writerow([all_stats.count, received_at, tick, phase, clip_file, clip_index])
            if tick - last_flush_tick >= 1_000_000_000:
                csv_file.flush()
                last_flush_tick = tick
            if phase == "warmup":
                warmup_stats.add(tick)
                if warmup_ready(warmup_stats):
                    storage_fps, fps_source = choose_storage_fps(warmup_stats, args.fps)
                    meta["storage_fps"] = storage_fps
                    meta["storage_fps_source"] = fps_source
                    meta["status"] = "previewing"
                    persist()
                    print(f"Storage FPS: {storage_fps:.3f}; source: {fps_source} (not sensor FPS).")
            preview_stats.add(tick)
            loop_fps = rolling_fps.add(tick) if settings_event is None else None
            cv2.imshow(window_name, make_preview(frame, received_at, loop_fps, clip is not None,
                                                 read_ms, ready=storage_fps is not None,
                                                 settings_pending=settings_event is not None))
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q")):
                break
            if settings_event is not None:
                if key in (ord("c"), ord("C")):
                    settings_event.update(user_confirmed_return_at=now_iso(), after=camera_properties(cap))
                    settings_event = None
                    meta["driver_settings_return_pending"] = False
                    # 用户关闭属性窗并确认后，才重新测量收帧速度。
                    rolling_fps, warmup_stats = RollingReceiptFPS(), ReceiptStats()
                    storage_fps, fps_source = None, None
                    meta["status"] = "warming_up"
                    persist()
                    print("Return confirmed; after snapshot saved. Measuring again for >=2 seconds.")
                elif key in (ord("r"), ord("R"), ord("s"), ord("S"), ord("p"), ord("P")):
                    print("R/S/P paused. Close driver settings, then press C in the preview.")
            elif key in (ord("r"), ord("R")):
                if storage_fps is None:
                    print("Still warming up: recording requires >=2 seconds and >=2 received frames.")
                elif clip is None:
                    clip = Clip(folder, len(meta["clips"]) + 1, (args.width, args.height), storage_fps, fps_source, args.session)
                    print(f"Recording: {clip.path.name}")
                else:
                    finish_clip()
                    print("Recording stopped and validated.")
            elif key in (ord("s"), ord("S")):
                photo = save_png(folder, frame, received_at, all_stats.count, args.session)
                meta["photos"].append(photo)
                persist()
                print(f"Saved: {photo['file']}")
            elif key in (ord("p"), ord("P")):
                if clip is not None:
                    print("Stop recording before opening driver settings.")
                elif platform.system() != "Windows" or meta["reported"]["backend"].upper() != "DSHOW":
                    print("Driver settings are available only with Windows DirectShow (DSHOW).")
                else:
                    event = {"settings_requested_at": now_iso(), "before": camera_properties(cap)}
                    meta["driver_settings_events"].append(event)
                    persist()
                    try:
                        event["set_accepted"] = bool(cap.set(cv2.CAP_PROP_SETTINGS, 1))
                    except cv2.error as exc:
                        event["error"] = str(exc)
                    # 驱动可能异步打开属性窗，set 返回不代表用户已关闭窗口。
                    event["immediate_snapshot"] = camera_properties(cap)
                    settings_event = event
                    storage_fps, fps_source = None, None
                    meta["driver_settings_return_pending"] = True
                    meta["status"] = "awaiting_driver_settings_return"
                    persist()
                    print("Driver settings requested. Close the settings window, then press C in the preview; R/S/P are paused.")
            if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        meta["interrupted"] = True
    except Exception as exc:
        error("capture", exc)
        print(f"Capture error: {exc}", file=sys.stderr)
    finally:
        try:
            finish_clip()
        except Exception as exc:
            error("finalize_clip", exc)
        if cap is not None:
            cap.release()
        if csv_file is not None:
            csv_file.close()
        try:
            cv2.destroyAllWindows()
        except cv2.error as exc:
            error("close_preview", exc)
        meta.update(ended_at=now_iso(), all_frame_timing=all_stats.summary(),
                    warmup_timing=warmup_stats.summary(), preview_loop_timing=preview_stats.summary())
        meta["status"] = "failed" if meta["errors"] else "completed"
        persist()
    print(f"Status: {meta['status']}; metadata: {meta_path}")
    return 1 if meta["errors"] else 0


def main(argv=None):
    p = parser()
    args = p.parse_args(argv)
    try:
        check_args(args)
    except ValueError as exc:
        p.error(str(exc))
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
