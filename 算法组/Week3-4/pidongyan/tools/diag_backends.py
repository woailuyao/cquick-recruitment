"""后端/格式矩阵测试：找出能跑满 720p30 且画面正常的组合。

针对症状：Camera 应用有画面，但 OpenCV DSHOW 只有 1fps 且纯黑。

测试矩阵：
  后端   : DSHOW / MSMF / ANY
  格式   : 默认 / MJPG / YUY2
  分辨率 : 640x480 / 1280x720

每组跑 15 帧，输出实测 FPS、画面亮度、最大像素值（判断是否黑帧），
并把每组第一帧存到 logs/backend_matrix/ 便于肉眼确认。

用法（在普通 PowerShell 里跑）：
  python tools\\diag_backends.py
  python tools\\diag_backends.py --index 2
"""
import argparse
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "logs" / "backend_matrix"

BACKENDS = [
    ("DSHOW", cv2.CAP_DSHOW),
    ("MSMF", cv2.CAP_MSMF),
    ("ANY", cv2.CAP_ANY),
]
FORMATS = [("默认", None), ("MJPG", "MJPG"), ("YUY2", "YUY2")]
SIZES = [(640, 480), (1280, 720)]


def trial(index, api, fourcc, w, h, nframes=15):
    cap = cv2.VideoCapture(index, api)
    if not cap.isOpened():
        cap.release()
        return None
    if fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, 30)

    aw, ah = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fc = int(cap.get(cv2.CAP_PROP_FOURCC))
    tag = "".join(chr((fc >> 8 * i) & 0xFF) for i in range(4))

    for _ in range(5):
        cap.read()

    frames, frame = 0, None
    t0 = time.time()
    while frames < nframes and time.time() - t0 < 6:
        ok, f = cap.read()
        if ok:
            frames += 1
            frame = f
    elapsed = time.time() - t0
    cap.release()

    fps = frames / elapsed if elapsed > 0 else 0.0
    mean = mx = -1
    if frame is not None:
        g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        mean, mx = float(g.mean()), int(g.max())
    return dict(aw=aw, ah=ah, tag=tag, fps=fps, mean=mean, mx=mx, frame=frame)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--index", type=int, default=None, help="只测指定索引")
    ap.add_argument("--max-index", type=int, default=4)
    args = ap.parse_args()

    OUT.mkdir(parents=True, exist_ok=True)
    indices = [args.index] if args.index is not None else list(range(args.max_index))

    print(f"OpenCV {cv2.__version__}\n")
    header = f"{'后端':<6} {'格式':<5} {'请求':<10} {'实际':<10} {'fourcc':<8} {'FPS':>6} {'亮度':>7} {'max':>4}  判定"
    print(header)
    print("-" * len(header))

    best = None
    for idx in indices:
        # 先确认这个索引能不能开
        probe = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
        opened = probe.isOpened()
        probe.release()
        if not opened and args.index is None:
            continue

        for bname, api in BACKENDS:
            for fname, fourcc in FORMATS:
                for (w, h) in SIZES:
                    r = trial(idx, api, fourcc, w, h)
                    if r is None:
                        print(f"{bname:<6} {fname:<5} {w}x{h:<6} {'打不开':<10}")
                        continue
                    ok_res = (r["aw"], r["ah"]) == (w, h)
                    alive = r["mx"] > 20
                    good = r["fps"] >= 25 and alive and ok_res
                    verdict = "✅ 可用" if good else (
                        "黑帧" if not alive else
                        ("分辨率没生效 " if not ok_res else "帧率不足"))
                    print(f"{bname:<6} {fname:<5} {w}x{h:<6} "
                          f"{r['aw']}x{r['ah']:<5} {r['tag']:<8} {r['fps']:>6.1f} "
                          f"{r['mean']:>7.1f} {r['mx']:>4}  {verdict}")

                    if r["frame"] is not None:
                        fn = OUT / f"idx{idx}_{bname}_{fname}_{w}x{h}.jpg"
                        cv2.imwrite(str(fn), r["frame"])

                    if good and (best is None or r["fps"] > best[2]["fps"]):
                        best = (idx, (bname, fname, w, h), r)
        print()

    print("=" * 70)
    if best:
        idx, cfg, r = best
        print(f"最佳组合：index={idx}  后端={cfg[0]}  格式={cfg[1]}  "
              f"{cfg[2]}x{cfg[3]}  {r['fps']:.1f} fps")
        print(f"样例帧：logs/backend_matrix/idx{idx}_{cfg[0]}_{cfg[1]}_{cfg[2]}x{cfg[3]}.jpg")
    else:
        print("没有任何组合同时满足：分辨率正确 + 画面非黑 + ≥25fps")
        print("下一步建议：降级到 opencv-python 4.11 再测（见回复中的命令）")


if __name__ == "__main__":
    main()
