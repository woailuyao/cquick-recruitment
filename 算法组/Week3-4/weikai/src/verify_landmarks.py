# -*- coding: utf-8 -*-
"""verify_landmarks.py —— 场地标志标注质量校验（两类别：L=0 / T=1）

检查项：
  1. 每张图是否有同名 .txt（缺标签单独统计，不直接判错）
  2. 每行是否 5 个数；class_id 只能是 0(L) 或 1(T)，出现其他值说明类别表被污染
  3. 归一化坐标合法性：0 <= cx,cy <= 1；0 < w,h <= 1；框不能越出图像边界
  4. 框尺寸合理性：面积过小（<8 像素）或过大（>图面积 60%）→ 疑漏标/误框
  5. 长宽比 > 6 的细长框 → 框没贴住标志，把整段线框进去了
  6. 同帧内近重复框（IoU > 0.9）→ 同一标志被标了两次
  7. 空 txt（无目标的背景帧）单独统计，训练时可用作负样本
  8. 按视频来源统计帧数/框数，检查有没有整段视频漏标

输出：
  终端汇总 + <out>/report.md + <out>/grid.png（随机抽帧把框画回去，肉眼核对）

用法：
  python verify_landmarks.py
  python verify_landmarks.py --data-dir D:\\captures\\_anno\\labelimg_work --n 24
"""
import argparse
import os
import random
import statistics
from collections import Counter, defaultdict

import cv2
import numpy as np

IMG_EXT = (".jpg", ".jpeg", ".png")
CLASS_NAMES = {0: "L", 1: "T"}


def read_label(path):
    """返回 [(行号, [class, cx, cy, w, h])]，解析失败的行单独返回。"""
    rows, bad = [], []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for ln, line in enumerate(f, 1):
            s = line.strip()
            if not s:
                continue
            parts = s.split()
            if len(parts) != 5:
                bad.append((ln, "列数=%d（应为 5）" % len(parts)))
                continue
            try:
                cls = int(float(parts[0]))
                cx, cy, w, h = [float(x) for x in parts[1:]]
            except ValueError:
                bad.append((ln, "数值解析失败: %s" % s))
                continue
            rows.append((ln, [cls, cx, cy, w, h]))
    return rows, bad


def iou(a, b):
    ax1, ay1 = a[1] - a[3] / 2, a[2] - a[4] / 2
    ax2, ay2 = a[1] + a[3] / 2, a[2] + a[4] / 2
    bx1, by1 = b[1] - b[3] / 2, b[2] - b[4] / 2
    bx2, by2 = b[1] + b[3] / 2, b[2] + b[4] / 2
    ix = max(0.0, min(ax2, bx2) - max(ax1, bx1))
    iy = max(0.0, min(ay2, by2) - max(ay1, by1))
    inter = ix * iy
    union = a[3] * a[4] + b[3] * b[4] - inter
    return inter / union if union > 0 else 0.0


def draw(img, boxes):
    vis = img.copy()
    h, w = vis.shape[:2]
    for cls, cx, cy, bw, bh in boxes:
        col = (0, 220, 0) if cls == 0 else (0, 165, 255)
        x1, y1 = int((cx - bw / 2) * w), int((cy - bh / 2) * h)
        x2, y2 = int((cx + bw / 2) * w), int((cy + bh / 2) * h)
        cv2.rectangle(vis, (x1, y1), (x2, y2), col, 2)
        cv2.putText(vis, CLASS_NAMES.get(cls, str(cls)), (x1, max(12, y1 - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
    return vis


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", default=r"D:\captures\_anno\labelimg_work")
    ap.add_argument("--out", default=r"D:\captures\_anno\check")
    ap.add_argument("--n", type=int, default=24, help="抽查可视化帧数")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    imgs = sorted(f for f in os.listdir(args.data_dir)
                  if f.lower().endswith(IMG_EXT))
    if not imgs:
        print("目录里没有图片:", args.data_dir)
        return 1

    problems = []                       # (级别, 文件名, 说明)
    per_video = defaultdict(lambda: {"frames": 0, "boxes": 0,
                                     "empty": 0, "missing": 0})
    per_video_cls = defaultdict(Counter)
    cls_total = Counter()
    boxes_per_frame = Counter()
    empty_frames, missing_labels = [], []
    size_cache = {}

    for name in imgs:
        stem = os.path.splitext(name)[0]
        vid = stem.rsplit("_", 1)[0]
        per_video[vid]["frames"] += 1

        img = cv2.imread(os.path.join(args.data_dir, name))
        if img is None:
            problems.append(("错误", name, "图片读不出来"))
            continue
        h, w = img.shape[:2]
        size_cache[name] = (w, h)

        txt = os.path.join(args.data_dir, stem + ".txt")
        if not os.path.exists(txt):
            per_video[vid]["missing"] += 1
            per_video[vid]["empty"] += 1
            empty_frames.append(stem)
            boxes_per_frame[0] += 1
            continue

        rows, bad = read_label(txt)
        for ln, why in bad:
            problems.append(("错误", name, "第%d行 %s" % (ln, why)))
        if not rows:
            if not bad:
                per_video[vid]["empty"] += 1
                empty_frames.append(stem)
                boxes_per_frame[0] += 1
            continue

        per_video[vid]["boxes"] += len(rows)
        boxes_per_frame[len(rows)] += 1

        ok_boxes = []
        for ln, (cls, cx, cy, bw, bh) in rows:
            if cls not in CLASS_NAMES:
                problems.append(("错误", name,
                                 "第%d行 class_id=%d 越界（只允许 0=L / 1=T）"
                                 % (ln, cls)))
                continue
            cls_total[cls] += 1
            per_video_cls[vid][cls] += 1

            if not (0.0 <= cx <= 1.0 and 0.0 <= cy <= 1.0):
                problems.append(("错误", name,
                                 "第%d行 中心点越界 cx=%.3f cy=%.3f"
                                 % (ln, cx, cy)))
                continue
            if not (0.0 < bw <= 1.0 and 0.0 < bh <= 1.0):
                problems.append(("错误", name,
                                 "第%d行 宽高非法 w=%.3f h=%.3f" % (ln, bw, bh)))
                continue

            x1, x2 = cx - bw / 2, cx + bw / 2
            y1, y2 = cy - bh / 2, cy + bh / 2
            if x1 < -0.005 or y1 < -0.005 or x2 > 1.005 or y2 > 1.005:
                problems.append(("警告", name,
                                 "第%d行 框越出图像边界 (%.3f,%.3f)-(%.3f,%.3f)"
                                 % (ln, x1, y1, x2, y2)))

            px_area = (bw * w) * (bh * h)
            if px_area < 8:
                problems.append(("警告", name,
                                 "第%d行 框太小 %.0f 像素²，可能是误框" % (ln, px_area)))
            if bw * bh > 0.6:
                problems.append(("警告", name,
                                 "第%d行 框占图 %.0f%%，太大" % (ln, bw * bh * 100)))
            aspect = max(bw * w, bh * h) / max(1e-6, min(bw * w, bh * h))
            if aspect > 6:
                problems.append(("警告", name,
                                 "第%d行 长宽比 %.1f，框可能包住了整段线" % (ln, aspect)))

            ok_boxes.append([cls, cx, cy, bw, bh])

        # 同帧近重复框
        for i in range(len(ok_boxes)):
            for j in range(i + 1, len(ok_boxes)):
                v = iou(ok_boxes[i], ok_boxes[j])
                if v > 0.9:
                    problems.append(("警告", name,
                                     "两个框几乎重合 (IoU=%.2f)，同一标志标了两次" % v))

    # ---------- 汇总 ----------
    total_boxes = sum(cls_total.values())
    lines = []
    lines.append("# 标注校验报告")
    lines.append("")
    lines.append("- 目录：`%s`" % args.data_dir)
    lines.append("- 图片 %d 张，框 %d 个（L %d / T %d）"
                 % (len(imgs), total_boxes, cls_total[0], cls_total[1]))
    lines.append("- 空标签帧 %d 张（训练时可作背景负样本）" % len(empty_frames))
    lines.append("- 缺标签文件 %d 张" % sum(v["missing"] for v in per_video.values()))
    lines.append("")
    lines.append("## 按视频来源")
    lines.append("")
    lines.append("| 视频 | 帧数 | 框数 | L | T | 空标签帧 |")
    lines.append("|---|---|---|---|---|---|")
    for vid in sorted(per_video):
        d = per_video[vid]
        c = per_video_cls[vid]
        lines.append("| %s | %d | %d | %d | %d | %d |"
                     % (vid, d["frames"], d["boxes"], c[0], c[1], d["empty"]))
    lines.append("")
    lines.append("## 问题清单")
    lines.append("")
    if problems:
        err = sum(1 for p in problems if p[0] == "错误")
        lines.append("共 %d 条（错误 %d / 警告 %d）" % (len(problems), err,
                                                    len(problems) - err))
        lines.append("")
        lines.append("| 级别 | 文件 | 说明 |")
        lines.append("|---|---|---|")
        for lvl, fn, why in problems[:400]:
            lines.append("| %s | %s | %s |" % (lvl, fn, why))
        if len(problems) > 400:
            lines.append("| ... | ... | 其余 %d 条省略 |" % (len(problems) - 400))
    else:
        lines.append("未发现问题，全部标注合法。")

    # ---------- 抽查可视化 ----------
    random.seed(args.seed)
    pick = random.sample(imgs, min(args.n, len(imgs)))
    tiles = []
    for name in pick:
        img = cv2.imread(os.path.join(args.data_dir, name))
        stem = os.path.splitext(name)[0]
        txt = os.path.join(args.data_dir, stem + ".txt")
        boxes = []
        if os.path.exists(txt):
            rows, _ = read_label(txt)
            boxes = [r for _, r in rows]
        vis = draw(img, boxes)
        vis = cv2.resize(vis, (480, int(480 * vis.shape[0] / vis.shape[1])))
        cv2.putText(vis, stem, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 2, cv2.LINE_AA)
        tiles.append(vis)
    if tiles:
        cols = 4
        rows_n = (len(tiles) + cols - 1) // cols
        th, tw = tiles[0].shape[:2]
        grid = 255 * np.ones((rows_n * th, cols * tw, 3),
                                              dtype="uint8")
        for k, t in enumerate(tiles):
            r, c = divmod(k, cols)
            grid[r * th:(r + 1) * th, c * tw:(c + 1) * tw] = t
        cv2.imwrite(os.path.join(args.out, "grid.png"), grid)

    report = os.path.join(args.out, "report.md")
    with open(report, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    # ---------- 终端输出 ----------
    print("图片 %d 张，框 %d 个（L %d / T %d）"
          % (len(imgs), total_boxes, cls_total[0], cls_total[1]))
    print("空标签帧 %d，缺标签文件 %d"
          % (len(empty_frames), sum(v["missing"] for v in per_video.values())))
    counts = list(boxes_per_frame.elements())
    counts.sort()
    print("每帧框数：最少 %d，中位 %.1f，最多 %d"
          % (min(counts), statistics.median(counts), max(counts)))
    if problems:
        err = sum(1 for p in problems if p[0] == "错误")
        print("问题 %d 条（错误 %d / 警告 %d）→ 详情见 %s"
              % (len(problems), err, len(problems) - err, report))
        for lvl, fn, why in problems[:12]:
            print("  [%s] %s: %s" % (lvl, fn, why))
    else:
        print("未发现问题")
    print("报告：%s" % report)
    print("抽查图：%s" % os.path.join(args.out, "grid.png"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
