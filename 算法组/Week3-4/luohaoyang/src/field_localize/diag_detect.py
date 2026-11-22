"""一键诊断：到底是"模型没调对"还是"画面里根本没有可检出的标志"。

    python scripts/diag_detect.py                 # 用相机
    python scripts/diag_detect.py --image a.jpg   # 用图片

它会打印：
    1) 实际加载的模型路径、类别、文件大小
    2) 不同 conf / imgsz / 预处理下，检出的框数、最高置信度、框大小
    3) 保存一张带框的预览图
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_ROOT))


def grab(camera, save):
    cap = cv2.VideoCapture(camera, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    for _ in range(15):
        cap.read()
    ok, frame = cap.read()
    cap.release()
    if not ok:
        raise SystemExit("cannot grab frame")
    cv2.imwrite(save, frame)
    return frame


def clahe(im):
    lab = cv2.cvtColor(im, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    l = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8)).apply(l)
    return cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--image", default=None)
    ap.add_argument("--model", default=str(ROOT / "runs" / "markers_v5" / "weights" / "best.pt"))
    ap.add_argument("--save", default=str(ROOT / "debug_frames" / "diag_frame.jpg"))
    ap.add_argument("--device", default="0")
    args = ap.parse_args()

    import torchvision  # noqa: F401  (must come before ultralytics on this box)
    from ultralytics import YOLO

    mp = Path(args.model)
    print("=" * 70)
    print(f"model path : {mp}")
    print(f"exists     : {mp.exists()}  size={mp.stat().st_size if mp.exists() else 0}")
    model = YOLO(str(mp))
    print(f"classes    : {model.names}")
    print("=" * 70)

    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            raise SystemExit(f"cannot read {args.image}")
        Path(args.save).parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(args.save, frame)
    else:
        frame = grab(args.camera, args.save)
    print(f"frame      : {frame.shape}  saved -> {args.save}")

    variants = [("raw", frame), ("clahe", clahe(frame))]
    best_conf = 0.0
    for tag, img in variants:
        for imgsz in (960, 1280):
            for conf in (0.25, 0.10, 0.03):
                r = model.predict(img, conf=conf, imgsz=imgsz,
                                  device=args.device, verbose=False)[0]
                boxes = [] if r.boxes is None else r.boxes
                n = len(boxes)
                top = max([float(b.conf[0]) for b in boxes], default=0.0)
                best_conf = max(best_conf, top)
                sizes = [f"{int(b.xyxy[0][2]-b.xyxy[0][0])}x{int(b.xyxy[0][3]-b.xyxy[0][1])}"
                         for b in boxes]
                cls = [model.names[int(b.cls[0])] for b in boxes]
                print(f"  {tag:<5} imgsz={imgsz:<5} conf={conf:<5} -> {n} 个  "
                      f"最高={top:.3f}  {list(zip(cls, sizes))[:4]}")

    print("=" * 70)
    if best_conf < 0.05:
        print("结论: 画面里一个候选都没有 —— 不是模型调错，是拍摄条件问题")
        print("      常见原因: 标志太远(<100px)、过曝、失焦、镜头太脏")
    elif best_conf < 0.2:
        print(f"结论: 有候选但置信度很低 ({best_conf:.2f}) —— 画面条件勉强")
        print("      建议: 靠近一点 / 降低曝光 / 擦镜头")
    else:
        print(f"结论: 模型正常，最高置信度 {best_conf:.2f}")

    # 存一张 conf=0.10 的预览
    r = model.predict(frame, conf=0.10, imgsz=960, device=args.device, verbose=False)[0]
    img = frame.copy()
    if r.boxes is not None:
        for b in r.boxes:
            x1, y1, x2, y2 = [int(v) for v in b.xyxy[0].tolist()]
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(img, f"{model.names[int(b.cls[0])]} {float(b.conf[0]):.2f}",
                        (x1, max(14, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    out = str(ROOT / "debug_frames" / "diag_preview.jpg")
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(out, img)
    print(f"preview    : {out}")


if __name__ == "__main__":
    main()