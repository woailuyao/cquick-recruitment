# -*- coding: utf-8 -*-
"""把标志标签的「框尺寸」按实际内容重估，中心与类别保持不变。

为什么需要：
    tools/prelabel_marker_lowconf.py 在低机位给的是**固定尺寸**种子框（0.05×0.09），
    因为掠射角下没法从分支长度估尺度。人工复核只改类别、不动尺寸，于是新段的框尺寸
    几乎恒定（实测中位 64×65 px、10–90 分位 59–94），而旧高机位段是 93×85 px、
    55–220 px。尺寸分布不一致会有两个后果：
      · 训练：模型学到「这个视角下框总是 64 px」，近处大交点学不会；
      · 评估：IoU 0.5 下，真实 150 px 的交点配一个 64 px 的框必然算漏检，指标失真。

做法：
    用同一张图以极低阈值跑一次模型，取每个标签中心最近的检测框，把它的宽高搬过来；
    中心与类别一个字都不动。找不到对应检测时保留原值并计数，便于人工兜底。

用法（项目根目录）：
    run.bat tools\\resize_marker_labels.py --dirs dataset\\raw_marker\\low_01 dataset\\raw_marker\\low_02
    run.bat tools\\resize_marker_labels.py --dirs dataset\\raw_marker\\low_01 --dry-run
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.paths import WEIGHTS_DIR                                    # noqa: E402

EXTS = (".png", ".jpg", ".jpeg", ".bmp")


def quantiles(vals):
    a = sorted(vals)
    if not a:
        return (0, 0, 0)
    def q(p):
        return a[min(len(a) - 1, int(len(a) * p))]
    return (q(0.5), q(0.1), q(0.9))


def main() -> int:
    ap = argparse.ArgumentParser(description="按实际检测框重估标志标签的框尺寸")
    ap.add_argument("--dirs", nargs="+", required=True)
    ap.add_argument("--weights", default=str(WEIGHTS_DIR / "best_marker_v9.pt"))
    ap.add_argument("--conf", type=float, default=0.02)
    ap.add_argument("--match-px", type=float, default=45.0,
                    help="标签中心与检测中心的配对半径（像素）")
    ap.add_argument("--scale", type=float, default=1.0, help="整体缩放修正（默认 1.0）")
    ap.add_argument("--device", default="0")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    from ultralytics import YOLO
    model = YOLO(args.weights)

    total_changed = total_unmatched = total_filled = 0
    for d in args.dirs:
        img_dir = Path(d) if Path(d).is_absolute() else ROOT / d
        imgs = sorted(p for p in img_dir.rglob("*") if p.suffix.lower() in EXTS)

        # 第一遍：逐框找最近的检测框，先把「配上了」的尺寸记下来
        jobs = []            # [(txt_path, lines, [(cls,cx,cy,w_old,h_old,v_px,size_px|None)])]
        for p in imgs:
            txt = p.with_suffix(".txt")
            if not txt.exists():
                continue
            lines = [l for l in txt.read_text(encoding="utf-8").splitlines() if l.strip()]
            if not lines:
                continue
            img = cv2.imread(str(p))
            if img is None:
                continue
            h, w = img.shape[:2]
            res = model.predict(img, conf=args.conf, imgsz=640, device=args.device,
                                verbose=False)[0]
            dets = []
            for b in res.boxes:
                x1, y1, x2, y2 = [float(v) for v in b.xyxy[0]]
                dets.append(((x1 + x2) / 2.0, (y1 + y2) / 2.0, x2 - x1, y2 - y1))
            items = []
            for l in lines:
                f = l.split()
                if len(f) != 5:
                    items.append(None)
                    continue
                cls, cx, cy, bw, bh = int(float(f[0])), float(f[1]), float(f[2]), \
                    float(f[3]), float(f[4])
                pu, pv = cx * w, cy * h
                best, best_d = None, 1e9
                for (du, dv, dw, dh) in dets:
                    dist = float(np.hypot(du - pu, dv - pv))
                    if dist < best_d:
                        best, best_d = (dw, dh), dist
                size = best if (best is not None and best_d <= args.match_px) else None
                items.append([cls, cx, cy, bw * w, bh * h, pv, size])
            jobs.append((txt, lines, items, w, h))

        # 用「配上了」的样本拟合 尺寸 ~ 纵向位置，给没配上的外推
        vw = [(it[5], it[6][0]) for _t, _l, items, _w, _h in jobs for it in items
              if it and it[6]]
        vh = [(it[5], it[6][1]) for _t, _l, items, _w, _h in jobs for it in items
              if it and it[6]]

        def fit(pairs):
            if len(pairs) < 12:
                return None
            xs = np.array([p[0] for p in pairs], float)
            ys = np.array([p[1] for p in pairs], float)
            A = np.vstack([xs, np.ones_like(xs)]).T
            coef, *_ = np.linalg.lstsq(A, ys, rcond=None)
            return coef

        fw, fh = fit(vw), fit(vh)

        changed = unmatched = filled = 0
        before_w, before_h, after_w, after_h = [], [], [], []
        for txt, lines, items, w, h in jobs:
            out = []
            for l, it in zip(lines, items):
                if it is None:
                    out.append(l)
                    continue
                cls, cx, cy, ow, oh, pv, size = it
                before_w.append(ow); before_h.append(oh)
                if size is not None:
                    nw, nh = max(8.0, size[0] * args.scale), max(8.0, size[1] * args.scale)
                    changed += 1
                elif fw is not None and fh is not None:
                    nw = max(8.0, float(fw[0] * pv + fw[1]) * args.scale)
                    nh = max(8.0, float(fh[0] * pv + fh[1]) * args.scale)
                    filled += 1
                else:
                    nw, nh = ow, oh
                    unmatched += 1
                after_w.append(nw); after_h.append(nh)
                out.append(f"{cls} {cx:.6f} {cy:.6f} {nw / w:.6f} {nh / h:.6f}")
            if not args.dry_run:
                txt.write_text("\n".join(out) + "\n", encoding="utf-8")

        bw_ = quantiles(before_w); aw_ = quantiles(after_w)
        bh_ = quantiles(before_h); ah_ = quantiles(after_h)
        print(f"{img_dir.name}: {len(imgs)} 图｜检测配对 {changed}｜回归外推 {filled}｜"
              f"保留原值 {unmatched}{'（dry-run，未写盘）' if args.dry_run else ''}")
        print(f"    宽 px  改前 中位{bw_[0]:.0f} [{bw_[1]:.0f}~{bw_[2]:.0f}]"
              f"  →  改后 中位{aw_[0]:.0f} [{aw_[1]:.0f}~{aw_[2]:.0f}]")
        print(f"    高 px  改前 中位{bh_[0]:.0f} [{bh_[1]:.0f}~{bh_[2]:.0f}]"
              f"  →  改后 中位{ah_[0]:.0f} [{ah_[1]:.0f}~{ah_[2]:.0f}]")
        total_changed += changed
        total_unmatched += unmatched
        total_filled += filled

    print("-" * 70)
    print(f"合计：检测配对 {total_changed}｜回归外推 {total_filled}｜保留原值 {total_unmatched}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
