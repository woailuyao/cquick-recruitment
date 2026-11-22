# -*- coding: utf-8 -*-
"""从真机原图里导出「误检案例」：测试集上 P=1.0 没有误检，但开放场景会误报。

背景：测试集 49 张与验证集 54 张都是 0 误检（`fp_*.png` 一张也导不出来），
可细则 1.8 要求「至少分析一个误检和一个漏检案例」。真机画面里恰好有：
地球仪、人头、护栏立柱都会被当成 ball（conf 0.3–0.7）。这些是真实的误检，
比在测试集里硬凑一张更诚实，也更能说明模型的边界。

本脚本对给定原图跑推理，把「最大的且接近方形」的那一框当真球（绿），
其余框标成误检（红 + FP 置信度），导出到 outputs/cases_ball/fp_*.png。

用法（项目根目录）：
    .\\run.bat tools\\export_live_fp.py
    .\\run.bat tools\\export_live_fp.py --images .tmp\\now_0.png .tmp\\cam1_check2.png
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.paths import OUTPUT_DIR, WEIGHTS_DIR, rel          # noqa: E402


def pick_ball(boxes):
    """真球判据：面积最大、且宽高比接近 1（球在画面里总是圆的）。"""
    best, best_key = None, -1.0
    for i, (x1, y1, x2, y2, cf) in enumerate(boxes):
        w, h = max(x2 - x1, 1), max(y2 - y1, 1)
        ar = w / h
        if not (0.7 <= ar <= 1.4):
            continue
        key = cf * (w * h) ** 0.5
        if key > best_key:
            best, best_key = i, key
    return best


def main() -> int:
    ap = argparse.ArgumentParser(description="导出真机误检案例")
    ap.add_argument("--images", nargs="*", default=[], help="原图路径（默认取 .tmp 里的两帧）")
    ap.add_argument("--conf", type=float, default=0.4)
    ap.add_argument("--weights", default=str(WEIGHTS_DIR / "best_ball.pt"))
    ap.add_argument("--out-dir", default=str(OUTPUT_DIR / "cases_ball"))
    args = ap.parse_args()

    from ultralytics import YOLO

    images = [Path(p) for p in args.images] or [ROOT / ".tmp" / "now_0.png",
                                                ROOT / ".tmp" / "cam1_check2.png"]
    images = [p for p in images if p.exists()]
    if not images:
        print("没有可用原图，请用 --images 指定")
        return 1

    model = YOLO(args.weights)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    n = 0
    for src in images:
        img = cv2.imread(str(src))
        if img is None:
            print(f"读不到 {src}")
            continue
        r = model.predict(img, conf=args.conf, verbose=False, device="0")[0]
        boxes = []
        for b in r.boxes:
            x1, y1, x2, y2 = [float(v) for v in b.xyxy[0].tolist()]
            boxes.append((x1, y1, x2, y2, float(b.conf[0])))
        if len(boxes) < 2:
            print(f"{src.name}: 只检出 {len(boxes)} 个，没有误检可导出")
            continue
        ball_i = pick_ball(boxes)
        view = img.copy()
        fps = []
        for i, (x1, y1, x2, y2, cf) in enumerate(boxes):
            if i == ball_i:
                cv2.rectangle(view, (int(x1), int(y1)), (int(x2), int(y2)), (0, 200, 0), 2)
                cv2.putText(view, f"TP {cf:.2f}", (int(x1), max(int(y1) - 6, 14)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0), 2)
            else:
                cv2.rectangle(view, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
                cv2.putText(view, f"FP {cf:.2f}", (int(x1), max(int(y1) - 6, 14)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                fps.append(cf)
        cv2.putText(view, "false positives: globe / head / railing", (10, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        n += 1
        dst = out_dir / f"fp_{n:02d}_live_{src.stem}.png"
        cv2.imwrite(str(dst), view)
        print(f"{rel(dst)}  真球 conf {boxes[ball_i][4]:.3f}，误检 {len(fps)} 个 "
              f"(conf {', '.join(f'{c:.2f}' for c in sorted(fps, reverse=True))})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
