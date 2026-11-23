"""快速相机体检：三组关键组合，各存一张图，输出极简结果。

用法（普通 PowerShell，venv 已激活）：
  python tools\\quick_cam_check.py
  python tools\\quick_cam_check.py --index 2
"""
import argparse
import time
from pathlib import Path

import cv2

# 静音 OpenCV 的警告输出（不同版本 API 位置不同，兼容处理）
try:
    cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_SILENT)
except Exception:
    for _fn in ("setLogLevel",):
        if hasattr(cv2, _fn):
            try:
                getattr(cv2, _fn)(0)
            except Exception:
                pass

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "logs"


def run(index, api, fourcc, w, h, n=25):
    if fourcc:
        cap = cv2.VideoCapture(index, api)
        if not cap.isOpened():
            cap.release()
            return None
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    else:
        cap = cv2.VideoCapture(index, api)
        if not cap.isOpened():
            cap.release()
            return None
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, 30)
    for _ in range(8):
        cap.read()
    frames, frame = 0, None
    t0 = time.time()
    while frames < n and time.time() - t0 < 5:
        ok, f = cap.read()
        if ok:
            frames += 1
            frame = f
    dt = time.time() - t0
    aw, ah = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cap.release()
    fps = frames / dt if dt > 0 else 0
    mean = mx = -1
    if frame is not None:
        g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        mean, mx = float(g.mean()), int(g.max())
    return dict(fps=fps, aw=aw, ah=ah, mean=mean, mx=mx, frame=frame)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--index", type=int, default=None)
    args = ap.parse_args()

    OUT.mkdir(parents=True, exist_ok=True)
    tmp = OUT / "_tmp.jpg"
    indices = [args.index] if args.index is not None else list(range(5))

    print(f"OpenCV {cv2.__version__} | {cv2.__file__}")
    usable = []
    for idx in indices:
        probe = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
        ok = probe.isOpened()
        probe.release()
        if not ok:
            continue
        print(f"\n--- index={idx} ---")
        cases = [("默认 1280x720", cv2.CAP_DSHOW, None, 1280, 720),
                 ("MJPG 1280x720", cv2.CAP_DSHOW, "MJPG", 1280, 720),
                 ("MJPG  640x480", cv2.CAP_DSHOW, "MJPG", 640, 480)]
        for label, api, fourcc, cw, ch in cases:
            r = run(idx, api, fourcc, cw, ch)
            if r is None:
                print(f"  {label:<15} 打不开")
                continue
            alive = r["mx"] > 20
            print(f"  {label:<15} 实际 {r['aw']}x{r['ah']:<5} {r['fps']:>5.1f} fps "
                  f"亮度 {r['mean']:>6.1f} max {r['mx']:>3}  {'✅有画面' if alive else '❌黑帧'}")
            if r["frame"] is not None:
                fn = OUT / f"quick_{idx}_{label.split()[0]}_{cw}x{ch}.jpg"
                cv2.imwrite(str(fn), r["frame"])
                print(f"                  存图 -> {fn.relative_to(ROOT)}")
            if alive and r["fps"] >= 20:
                usable.append((idx, label, r["fps"]))

    print("\n" + "=" * 60)
    if usable:
        print("可用组合：")
        for idx, label, fps in usable:
            print(f"  index={idx}  {label}  {fps:.1f} fps")
    else:
        print("仍然没有可用组合（全部黑帧或帧率不足）")
    print("请把上图输出贴给我，并把 logs 下生成的 quick_*.jpg 一起发我。")


if __name__ == "__main__":
    main()
