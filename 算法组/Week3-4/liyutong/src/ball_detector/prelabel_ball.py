# -*- coding: utf-8 -*-
"""
用 COCO 预训练的 yolov8n 自动预标注「足球」，生成 YOLO 格式 .txt，
让人工只需要「改」而不是「从零画所有框」。

用法：
    conda activate calib
    python prelabel_ball.py              # 默认 conf=0.10
    python prelabel_ball.py --conf 0.05  # 更宽松，召回高但误检多

输出：
    prelabels/<视频段>/<同名>.txt   —— YOLO 格式，class 0 = ball
    prelabels_viz/<视频段>/<同名>.jpg —— 画好框的预览图
    prelabel_report.txt             —— 统计报告，可直接摘进 README

注意：预标注只是"草稿"，人工必须逐张确认。误检/漏检都靠 labelImg 里改。
"""
import argparse
import os
import shutil

import cv2
import numpy as np

RAW = "raw"
PRE = "prelabels"
VIZ = "prelabels_viz"
REPORT = "prelabel_report.txt"

SPORTS_BALL_CLASS = 32   # COCO 里 sports ball 的类别 id
MIN_BOX_PX = 6           # 小于这个像素尺寸的框丢掉（太小，标了也没意义）
MAX_AR_PER_BOX = 2.0     # 宽高比偏离 1 太多的不当球
MERGE_IOU = 0.35         # IoU 高于此值视为同一个球，只留置信度最高的

# 静态杂物误检过滤（针对本场地实测调参）：
# 画面左上角场地外有一团白色杂物（器材/塑料袋），COCO 模型会把它当球，
# 置信度 0.05~0.24。实测它固定出现在 cx≈0.056、cy≈0.166，宽 19~36 px，
# 在 video2/video3/video6 里反复出现 15 次——同一个像素位置，显然是死物不是球。
# 真球就算再远也不会精确地每次都停在同一坐标，所以这条规则风险很低。
CLUTTER_CX = 0.25        # 归一化中心 x 上限
CLUTTER_CY = 0.22        # 归一化中心 y 上限
CLUTTER_SIZE = 40        # 像素边长上限


def is_static_clutter(cx, cy, bw, bh):
    return (cx < CLUTTER_CX and cy < CLUTTER_CY
            and max(bw, bh) < CLUTTER_SIZE)


def iou(a, b):
    ax1, ay1, ax2, ay2 = a[:4]
    bx1, by1, bx2, by2 = b[:4]
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    ua = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - inter
    return inter / ua if ua > 0 else 0.0


def merge_boxes(recs, thr=MERGE_IOU):
    """按置信度贪心合并同一个球上的多个重叠框"""
    kept = []
    for r in sorted(recs, key=lambda x: -x[4]):
        if all(iou(r, k) <= thr for k in kept):
            kept.append(r)
    return kept


def imwrite_safe(path, img):
    ok, buf = cv2.imencode(os.path.splitext(path)[1], img)
    if ok:
        buf.tofile(path)
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--conf", type=float, default=0.10)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--weights", default="yolov8n.pt")
    ap.add_argument("--max-per-img", type=int, default=1,
                    help="每张最多保留几个框。场景里只有一颗球，默认 1 个，"
                         "可以顺带干掉远处的白色杂物误检（实测杂物 conf 只有 0.05~0.24）")
    args = ap.parse_args()

    from ultralytics import YOLO
    model = YOLO(args.weights)
    print(f"模型 {args.weights} / conf={args.conf} / imgsz={args.imgsz}")

    # 不删除已有文件，直接就地覆盖同名 .txt / .jpg
    for d in (PRE, VIZ):
        os.makedirs(d, exist_ok=True)

    segs = sorted(x for x in os.listdir(RAW) if os.path.isdir(os.path.join(RAW, x)))
    lines = []
    tot_img = tot_hit = tot_box = 0
    all_sizes = []
    noball = []
    confs = []

    for seg in segs:
        src_dir = os.path.join(RAW, seg)
        files = sorted(f for f in os.listdir(src_dir) if f.lower().endswith(".jpg"))
        os.makedirs(os.path.join(PRE, seg), exist_ok=True)
        os.makedirs(os.path.join(VIZ, seg), exist_ok=True)

        hits = boxes_cnt = 0
        for f in files:
            p = os.path.join(src_dir, f)
            img = cv2.imread(p)
            if img is None:
                continue
            h, w = img.shape[:2]
            res = model.predict(img, imgsz=args.imgsz, conf=args.conf,
                                verbose=False, classes=[SPORTS_BALL_CLASS])[0]

            # 去掉过小/形状不对的，再按 IoU 合并同一个球上的重复框
            recs = []
            for b in res.boxes:
                x1, y1, x2, y2 = [float(v) for v in b.xyxy[0].tolist()]
                bw, bh = x2 - x1, y2 - y1
                recs.append((x1, y1, x2, y2, float(b.conf), bw, bh))
            recs = [r for r in recs if r[5] >= MIN_BOX_PX and r[6] >= MIN_BOX_PX
                    and max(r[5] / r[6], r[6] / r[5]) <= MAX_AR_PER_BOX]
            # 去掉固定位置的静态杂物误检（见文件顶部注释）
            recs = [r for r in recs
                    if not is_static_clutter((r[0] + r[2]) / 2 / w,
                                             (r[1] + r[3]) / 2 / h,
                                             r[5], r[6])]
            recs = merge_boxes(recs)
            recs = recs[:args.max_per_img]
            r_conf = recs[0][4] if recs else 0.0

            txt = os.path.join(PRE, seg, os.path.splitext(f)[0] + ".txt")
            with open(txt, "w", encoding="utf-8") as fp:
                for x1, y1, x2, y2, c, bw, bh in recs:
                    cx = (x1 + x2) / 2 / w
                    cy = (y1 + y2) / 2 / h
                    nw, nh = bw / w, bh / h
                    fp.write(f"0 {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}\n")
                    all_sizes.append(max(bw, bh))
            if recs:
                hits += 1
                boxes_cnt += len(recs)
                confs.append((r_conf, f"{seg}/{f}"))
            else:
                noball.append(f"{seg}/{f}")

            vis = img.copy()
            for x1, y1, x2, y2, c, _, _ in recs:
                cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(vis, f"{c:.2f}", (int(x1), max(12, int(y1) - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
            imwrite_safe(os.path.join(VIZ, seg, f), vis)

        tot_img += len(files)
        tot_hit += hits
        tot_box += boxes_cnt
        line = (f"{seg:>10}: {len(files):3d} 张中有 {hits:3d} 张检出球"
                f"（{hits / max(1, len(files)) * 100:5.1f}%），共 {boxes_cnt} 个框")
        print("  " + line)
        lines.append(line)

    # 尺寸分布
    sizes = np.array(all_sizes) if all_sizes else np.array([0])
    header = [
        "=== 预标注统计（COCO yolov8n / sports ball）===",
        f"总图片 {tot_img} 张，检出 {tot_hit} 张（{tot_hit / max(1, tot_img) * 100:.1f}%），框 {tot_box} 个",
        f"框边长（像素）：中位数 {np.median(sizes):.0f} / 均值 {sizes.mean():.1f} / "
        f"最小 {sizes.min():.0f} / 最大 {sizes.max():.0f}",
        "分档：" + "  ".join(
            f"<32px:{np.sum(sizes < 32)}  "
            f"32-64px:{np.sum((sizes >= 32) & (sizes < 64))}  "
            f"64-128px:{np.sum((sizes >= 64) & (sizes < 128))}  "
            f">=128px:{np.sum(sizes >= 128)}".split("  ")),
        "",
    ]
    with open(REPORT, "w", encoding="utf-8") as fp:
        fp.write("\n".join(header + lines) + "\n")
    with open("prelabel_noball.txt", "w", encoding="utf-8") as fp:
        fp.write("\n".join(noball) + ("\n" if noball else ""))
    # 按置信度升序写一份"优先复核清单"，低置信的框最可能是错的
    confs.sort()
    with open("prelabel_低置信清单.txt", "w", encoding="utf-8") as fp:
        fp.write("# 按置信度升序；低于 0.40 的请重点复核（可能框错位置或漏球）\n")
        for c, name in confs:
            if c < 0.40:
                fp.write(f"{c:.3f}  {name}\n")
    low = sum(1 for c, _ in confs if c < 0.40)
    print("\n" + "\n".join(header))
    print(f"预标注报告写入 {REPORT}")
    print(f"无球帧清单（{len(noball)} 张）写入 prelabel_noball.txt")
    print(f"低置信待复核（{low} 张）写入 prelabel_低置信清单.txt")


if __name__ == "__main__":
    main()
