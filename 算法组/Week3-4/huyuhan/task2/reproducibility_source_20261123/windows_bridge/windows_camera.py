"""Run with Windows Python; keep USB camera detached from usbipd/WSL."""
import argparse
import json
import os
import time
import uuid
from datetime import datetime
from pathlib import Path

from bridge_transport import ClockResponder, atomic_write, encode_packet


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--camera", type=int, default=1)
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=float, default=30)
    p.add_argument("--backend", choices=["dshow", "msmf"], default="dshow")
    p.add_argument("--shared", type=Path, default=Path(__file__).resolve().parent / "shared")
    a = p.parse_args()
    if os.name != "nt":
        raise SystemExit("Run windows_camera.py with Windows Python, not Ubuntu Python.")
    import cv2
    a.shared.mkdir(parents=True, exist_ok=True)
    backend = cv2.CAP_DSHOW if a.backend == "dshow" else cv2.CAP_MSMF

    def open_camera(index):
        print("Opening camera index:", index, "backend:", a.backend, flush=True)
        c = cv2.VideoCapture(index, backend)
        if c.isOpened():
            c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            c.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
            c.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
            c.set(cv2.CAP_PROP_FPS, a.fps)
        return c

    index = a.camera
    cap = open_camera(index)
    session = uuid.uuid4().hex
    clock = ClockResponder(a.shared, session)
    transmitting = False
    sequence = published = 0
    started = None
    cv2.namedWindow("Windows camera bridge", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Windows camera bridge", 960, 540)
    print("Bridge protocol v2: monotonic round-trip clock calibration.")
    print("Select EXTERNAL camera: N = next camera, S = start sending, Q = quit.")
    print("Shared folder:", a.shared)
    try:
        while True:
            clock.poll()
            ok, frame = cap.read() if cap.isOpened() else (False, None)
            capture_monotonic_ns = time.perf_counter_ns()
            capture_ns = time.time_ns()
            if not ok:
                if transmitting:
                    print("CAMERA READ FAILED. Sending stopped; WSL must show STALE.", flush=True)
                    break
                import numpy as np
                frame = np.zeros((540, 960, 3), dtype=np.uint8)
                cv2.putText(frame, "No image: N = next camera; Q = quit", (20, 100), 0, .7, (0, 0, 255), 2)
            capture_fps = 0.0
            if ok and transmitting:
                if started is None:
                    started = time.perf_counter()
                sequence += 1
                capture_fps = max(0, sequence-1) / max(time.perf_counter()-started, 1e-9)
                success, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
                if not success:
                    raise RuntimeError("JPEG encode failed")
                meta = dict(source="windows_live_camera", session=session, sequence=sequence,
                            protocol_version=2, capture_monotonic_ns=capture_monotonic_ns,
                            capture_unix_ns=capture_ns, width=frame.shape[1], height=frame.shape[0],
                            camera_index=index, backend=a.backend, capture_loop_fps=capture_fps,
                            published_before_this_frame=published,
                            timestamp_kind="host_timestamp_after_camera_read_not_sensor_exposure")
                if atomic_write(a.shared / "latest_frame.bin", encode_packet(meta, encoded.tobytes())):
                    published += 1
            view = frame.copy()
            state = "SENDING v2" if transmitting else "SELECT CAMERA v2: N next | S start"
            cv2.putText(view, f"{state} | camera={index} | Q quit", (15, 30), 0, .65, (0, 220, 0), 2)
            if transmitting:
                cv2.putText(view, f"capture loop={capture_fps:.1f} FPS | seq={sequence}", (15, 58), 0, .6, (0, 220, 0), 2)
            cv2.imshow("Windows camera bridge", view)
            key = cv2.waitKey(1 if ok else 50) & 255
            if key == ord("q"):
                break
            if key == ord("n"):
                if transmitting:
                    print("Restart this program to change camera. Calibration must match the selected device.")
                else:
                    cap.release()
                    index = (index + 1) % 4
                    cap = open_camera(index)
            if key == ord("s") and ok and not transmitting:
                transmitting = True
                print("LIVE camera sending started:", index, frame.shape, "session:", session, flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
        report = dict(session=session, camera_index=index, backend=a.backend,
                      ended=datetime.now().astimezone().isoformat(), read_frames=sequence,
                      published_frames=published, elapsed_s=time.perf_counter()-started if started else 0,
                      note="Live capture bridge only. Not detector FPS, model evaluation or acceptance recording.")
        (a.shared / ("capture_"+session+".json")).write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
