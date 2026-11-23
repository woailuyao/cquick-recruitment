"""任务一评估案例导出：在测试集上跑模型，把「误检 / 漏检」案例可视化。

考核要求训练报告里给出案例分析（至少 1 个误检 + 1 个漏检）。本脚本：
  1. 用训练好的模型预测 test 集每一张图；
  2. 与真值框按 IoU 匹配，划分：
       误检 FP = 有预测框但没有真值框与之匹配
       漏检 FN = 有真值框但没有预测框与之匹配
       正确 TP = 匹配成功
  3. 把每张含 FP/FN 的图加上标注（黄=误检，红框=漏检，绿=正确）存到
     check_out/eval_cases/；
  4. 输出统计到日志，并按「困难程度」排序（漏检优先、框小的优先、
     FP 置信度高的优先）——排前面的就是要写进报告的案例。

用法：
  python tools/eval_cases.py --model runs/detect/train/weights/best.pt
  python tools/eval_cases.py --model best.pt --data config/data.yaml --conf 0.25
"""
import argparse
import csv
import datetime as dt
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def load_gt(label_path, img_w, img_h):
    """YOLO txt -> [(cls, x1,y1,x2,y2)] 像素坐标。空文件返回 []。"""
    boxes = []
    text = label_path.read_text(encoding="utf-8").strip() if label_path.exists() else ""
    for line in text.splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        c, cx, cy, w, h = (float(x) for x in parts[:5])
        x1, y1 = (cx - w / 2) * img_w, (cy - h / 2) * img_h
        x2, y2 = (cx + w / 2) * img_w, (cy + h / 2) * img_h
        boxes.append((int(c), x1, y1, x2, y2))
    return boxes


def iou(a, b):
    ax1, ay1, ax2, ay2 = a[1:5]
    bx1, by1, bx2, by2 = b[1:5]
    ix = max(0, min(ax2, bx2) - max(ax1, bx1))
    iy = max(0, min(ay2, by2) - max(ay1, by1))
    inter = ix * iy
    ua = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - inter
    return inter / ua if ua > 0 else 0.0


def greedy_match(preds, gts, iou_thr):
    """按 IoU 贪心匹配。返回 (tp_pred_idx, matched_gt_idx, fp_pred_idx, fn_gt_idx)。"""
    pairs = []
    for i, p in enumerate(preds):
        for j, g in enumerate(gts):
            if p[0] != g[0]:
                continue
            v = iou(p, g)
            if v >= iou_thr:
                pairs.append((v, i, j))
    pairs.sort(reverse=True)
    used_p, used_g = set(), set()
    for v, i, j in pairs:
        if i not in used_p and j not in used_g:
            used_p.add(i)
            used_g.add(j)
    fp = [i for i in range(len(preds)) if i not in used_p]
    fn = [j for j in range(len(gts)) if j not in used_g]
    return used_p, used_g, fp, fn


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="runs/detect/train/weights/best.pt")
    ap.add_argument("--data", default=str(ROOT / "config" / "data.yaml"))
    ap.add_argument("--split", default="test")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--iou", type=float, default=0.5)
    ap.add_argument("--max-save", type=int, default=40,
                    help="最多保存多少张案例图")
    args = ap.parse_args()

    import yaml
    from ultralytics import YOLO

    cfg = yaml.safe_load(Path(args.data).read_text(encoding="utf-8"))
    base = Path(cfg["path"])
    if not base.is_absolute():
        base = (Path(args.data).parent / base).resolve()
    img_dir = base / cfg[args.split]
    lbl_dir = base / ("labels/" + args.split)
    names = cfg["names"]
    names = names if isinstance(names, list) else list(names.values())
    color = lambda c: names[int(c)]

    imgs = sorted(p for p in img_dir.iterdir() if p.suffix.lower() in IMG_EXTS)
    if not imgs:
        raise SystemExit(f"{img_dir} 没有图片")

    model = YOLO(str(ROOT / args.model) if not Path(args.model).is_absolute() else args.model)
    out_dir = ROOT / "check_out" / "eval_cases"
    out_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    n_img = n_fp = n_fn = n_tp = 0

    for img_path in imgs:
        img = cv2.imread(str(img_path))
        if img is None:
            continue
        H, W = img.shape[:2]
        gts = load_gt(lbl_dir / (img_path.stem + ".txt"), W, H)
        r = model(img, verbose=False, conf=args.conf)[0]
        preds = [(int(b.cls), *b.xyxy[0].tolist(), float(b.conf))
                 for b in r.boxes] if r.boxes is not None else []
        matched_p, matched_g, fp_idx, fn_idx = greedy_match(preds, gts, args.iou)

        n_img += 1
        n_fp += len(fp_idx)
        n_fn += len(fn_idx)
        n_tp += len(matched_p)
        if not fp_idx and not fn_idx:
            continue

        # ---- 可视化 ----
        vis = img.copy()
        for i in matched_p:
            _, x1, y1, x2, y2, cf = preds[i]
            cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), (80, 200, 80), 2)
        for i in fp_idx:
            _, x1, y1, x2, y2, cf = preds[i]
            cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), (60, 220, 230), 3)
            cv2.putText(vis, f"FP {cf:.2f}", (int(x1), max(15, int(y1) - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (60, 220, 230), 2)
        for j in fn_idx:
            _, x1, y1, x2, y2 = gts[j]
            cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), (60, 60, 230), 3)
            cv2.putText(vis, "FN(漏检)", (int(x1), min(H - 6, int(y2) + 18)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (60, 60, 230), 2)

        # 案例困难度：漏检优先，漏检的框越小越值得看；误检置信度越高越值得看
        fn_area = sum((gts[j][3]-gts[j][1])*(gts[j][4]-gts[j][2]) for j in fn_idx) or 1e9
        fp_conf = max([preds[i][5] for i in fp_idx], default=0)
        hardness = (len(fn_idx) > 0, -min(fn_area, 1e9), fp_conf)
        rows.append((hardness, img_path.name, len(fp_idx), len(fn_idx), vis))

    # 按困难度排序保存前 max_save 张
    # 按困难度排序：有漏检的排前、漏检目标越小越前、误检置信度越高越前
    rows.sort(key=lambda r: (not r[0][0], r[0][1], -r[0][2]))
    saved = 0
    summary_path = out_dir / "cases_summary.csv"
    with open(summary_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["image", "n_FP", "n_FN"])
        for _, name, nfp, nfn, vis in rows[:args.max_save]:
            cv2.imwrite(str(out_dir / f"case_{name}"), vis)
            w.writerow([name, nfp, nfn])
            saved += 1

    log = ROOT / "logs" / f"eval_cases_{dt.datetime.now():%Y%m%d_%H%M%S}.txt"
    log.write_text(
        f"模型: {args.model}\n数据: {args.data} split={args.split} conf={args.conf} iou={args.iou}\n"
        f"图片数: {n_img}  TP: {n_tp}  误检FP: {n_fp}  漏检FN: {n_fn}\n"
        f"案例图已保存: {saved} 张 -> {out_dir}\n清单: {summary_path}\n",
        encoding="utf-8")

    print(f"图片 {n_img} 张 | 正确 {n_tp} | 误检 {n_fp} | 漏检 {n_fn}")
    print(f"案例图 {saved} 张 -> {out_dir}（文件名 case_ 开头）")
    print(f"统计 -> {log}")
    if n_fn:
        print("⚠ 有漏检案例，建议优先写进训练报告（尤其小目标）")


if __name__ == "__main__":
    main()
