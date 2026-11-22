# -*- coding: utf-8 -*-
"""make_anno_crops.py —— 批量扫描场地视频，自动生成 YOLO 标注素材

产出（默认 D:\\captures\\_anno\\）：
  images\\    干净原帧（JPEG），YOLO 训练用
  labels\\    与 images 同名的 YOLO txt（class cx cy w h，已归一化）
  classes.txt L / T（顺序即类别号 0/1）
  crops\\L|T\\     交点周边方图（去重，用于快速核对类别是否正确）
  sheets\\    L_sheet.png / T_sheet.png  拼版大图，一眼扫完
  preview\\   画好框的 640 宽小图（只用于检查，不要拿去训练）
  review.html 预览画廊（浏览器打开就能翻看全部自动标注结果）
  report.md   统计报告

用法：
  python make_anno_crops.py
  python make_anno_crops.py --videos M1 M2 M3 --stride 0.5
  python make_anno_crops.py --with-v        # 额外纳入 V1-V5 足球视频
  python make_anno_crops.py --min-votes 3   # 更保守，只留高置信交点
"""
import argparse
import html
import os
import sys
import time

import cv2
import numpy as np

from detect_landmarks import detect_voted

CLASSES = ["L", "T"]
DEFAULT_VIDEOS = ["M1", "M2", "M3", "MC1", "MC2", "MC4", "MC5"]
EXTRA_VIDEOS = ["V1", "V2", "V3", "V4", "V5"]


# ---------- 工具 ----------
def dhash(bgr, size=8):
    """差值哈希，用于剔除重复裁剪图。"""
    g = cv2.resize(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), (size + 1, size))
    d = g[:, 1:] > g[:, :-1]
    v = 0
    for b in d.flatten():
        v = (v << 1) | int(b)
    return v


def hamming(a, b):
    return bin(a ^ b).count("1")


def box_half_at(dt, x, y, mult=2.0):
    """用距离变换实测交点附近的线宽，框半边长 = mult × 线宽。
    近景粗线（50px+）与远景细线（4px）都能框得准。"""
    xi, yi = int(x), int(y)
    r = 25
    x0, x1 = max(0, xi - r), min(dt.shape[1], xi + r + 1)
    y0, y1 = max(0, yi - r), min(dt.shape[0], yi + r + 1)
    win = dt[y0:y1, x0:x1]
    width_est = 2.0 * float(np.percentile(win, 90)) if win.size else 6.0
    width_est = min(width_est, 55.0)
    return float(np.clip(mult * width_est, 14.0, 100.0))


def clamp_box(cx, cy, half, w, h):
    """把正方形框平移进画面内，返回 (x0, y0, x1, y1)。"""
    x0 = max(0.0, cx - half)
    x1 = min(float(w), cx + half)
    y0 = max(0.0, cy - half)
    y1 = min(float(h), cy + half)
    if x1 - x0 < 2 or y1 - y0 < 2:
        return None
    return x0, y0, x1, y1


def draw_boxes(bgr, pts, halfs, scale=0.5):
    vis = cv2.resize(bgr, None, fx=scale, fy=scale,
                     interpolation=cv2.INTER_AREA)
    for (x, y, typ, votes), half in zip(pts, halfs):
        col = {"L": (0, 220, 0), "T": (0, 165, 255)}[typ]
        half = half * scale
        p1 = (int(x * scale - half), int(y * scale - half))
        p2 = (int(x * scale + half), int(y * scale + half))
        cv2.rectangle(vis, p1, p2, col, 1)
        cv2.putText(vis, "%s%d" % (typ, votes), (p1[0], max(9, p1[1] - 3)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, col, 1, cv2.LINE_AA)
    return vis


# ---------- 主流程 ----------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video-dir", default=r"D:\captures")
    ap.add_argument("--out-dir", default=r"D:\captures\_anno")
    ap.add_argument("--videos", nargs="*", default=None,
                    help="指定视频名前缀，如 M1 M2 MC1")
    ap.add_argument("--with-v", action="store_true",
                    help="额外纳入 V1-V5 足球视频")
    ap.add_argument("--stride", type=float, default=0.5,
                    help="抽帧时间间隔（秒）")
    ap.add_argument("--min-votes", type=int, default=2,
                    help="交点最少投票数，越大越保守")
    ap.add_argument("--box-mult", type=float, default=2.0,
                    help="标注框半边长 = box_mult × 距离变换实测线宽")
    ap.add_argument("--max-per-video", type=int, default=40)
    ap.add_argument("--static-thresh", type=float, default=5.0,
                    help="与上一保留帧的灰度差小于此值视为静止画面")
    ap.add_argument("--force-interval", type=float, default=4.0,
                    help="静止画面也至少每 N 秒保留一帧")
    ap.add_argument("--max-crops-per-class", type=int, default=160)
    ap.add_argument("--max-pts-per-frame", type=int, default=20,
                    help="单帧交点数超过此值视为纹理噪声帧，整帧丢弃")
    args = ap.parse_args()

    names = args.videos if args.videos else list(DEFAULT_VIDEOS)
    if args.with_v and not args.videos:
        names += EXTRA_VIDEOS

    paths = []
    for n in names:
        p = os.path.join(args.video_dir, n + ".mp4")
        if os.path.exists(p):
            paths.append((n, p))
        else:
            print("[跳过] 找不到 %s" % p, flush=True)
    if not paths:
        print("没有可用视频")
        return 1

    dirs = {k: os.path.join(args.out_dir, k)
            for k in ("images", "labels", "crops", "sheets", "preview")}
    for d in dirs.values():
        os.makedirs(d, exist_ok=True)
    for c in CLASSES:
        os.makedirs(os.path.join(dirs["crops"], c), exist_ok=True)

    with open(os.path.join(args.out_dir, "classes.txt"), "w",
              encoding="utf-8") as f:
        f.write("\n".join(CLASSES) + "\n")

    # 收集裁剪图（用于拼版核对）
    crops_by_cls = {c: [] for c in CLASSES}          # [(bgr, 来源)]
    hashes_by_cls = {c: [] for c in CLASSES}
    gallery = []                                     # [(preview 相对路径, 说明)]
    stats = []
    t_start = time.time()

    for name, path in paths:
        cap = cv2.VideoCapture(path)
        if not cap.isOpened():
            print("[跳过] 打不开 %s" % path, flush=True)
            continue
        fps = cap.get(cv2.CAP_PROP_FPS) or 21.0
        total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 0
        step = max(1, int(round(fps * args.stride)))
        dur = total / fps if fps > 0 else 0.0
        print("\n=== %s  时长 %.1fs  %.1ffps  每 %d 帧取一帧 ===" %
              (name, dur, fps, step), flush=True)

        n_keep = 0
        n_box = {c: 0 for c in CLASSES}
        last_small = None
        last_t = -1e9
        idx = 0
        t0 = time.time()
        while True:
            ok = cap.grab()
            if not ok:
                break
            if idx % step != 0:
                idx += 1
                continue
            ok, frame = cap.retrieve()
            idx += 1
            if not ok or frame is None:
                continue

            t_sec = idx / fps
            small = cv2.resize(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY),
                               (64, 36)).astype(np.int16)
            if last_small is not None:
                diff = float(np.abs(small - last_small).mean())
                if diff < args.static_thresh and \
                        (t_sec - last_t) < args.force_interval:
                    continue
            if n_keep >= args.max_per_video:
                break

            pts, mask = detect_voted(frame, min_votes=args.min_votes,
                                     return_mask=True)
            if not pts:
                continue
            if len(pts) > args.max_pts_per_frame:
                print("  [丢弃] 帧%d %d 个交点，疑似纹理噪声帧"
                      % (idx, len(pts)), flush=True)
                continue
            dt = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
            halfs = [box_half_at(dt, x, y, args.box_mult)
                     for x, y, _, _ in pts]

            H, W = frame.shape[:2]
            lines = []
            for (x, y, typ, votes), half in zip(pts, halfs):
                bb = clamp_box(x, y, half, W, H)
                if bb is None:
                    continue
                x0, y0, x1, y1 = bb
                cx = (x0 + x1) / 2 / W
                cy = (y0 + y1) / 2 / H
                bw = (x1 - x0) / W
                bh = (y1 - y0) / H
                if bw <= 0.004 or bh <= 0.004:
                    continue
                lines.append("%d %.6f %.6f %.6f %.6f" %
                             (CLASSES.index(typ), cx, cy, bw, bh))
                n_box[typ] += 1

                # 裁剪（扩 25% 边距，便于看清标志形态）
                mx = (x1 - x0) * 0.25
                cx0 = int(max(0, x0 - mx))
                cy0 = int(max(0, y0 - mx))
                cx1 = int(min(W, x1 + mx))
                cy1 = int(min(H, y1 + mx))
                patch = frame[cy0:cy1, cx0:cx1]
                if patch.size == 0 or min(patch.shape[:2]) < 16:
                    continue
                if len(crops_by_cls[typ]) < args.max_crops_per_class:
                    hh = dhash(patch)
                    if all(hamming(hh, o) > 4 for o in hashes_by_cls[typ]):
                        hashes_by_cls[typ].append(hh)
                        crops_by_cls[typ].append((patch, "%s@%.1fs" % (name, t_sec)))

            if not lines:
                continue

            base = "%s_%05d" % (name, idx)
            cv2.imwrite(os.path.join(dirs["images"], base + ".jpg"), frame,
                        [cv2.IMWRITE_JPEG_QUALITY, 92])
            with open(os.path.join(dirs["labels"], base + ".txt"), "w",
                      encoding="utf-8") as f:
                f.write("\n".join(lines) + "\n")
            pv = draw_boxes(frame, pts, halfs, scale=0.5)
            pv_name = base + "_preview.jpg"
            cv2.imwrite(os.path.join(dirs["preview"], pv_name), pv,
                        [cv2.IMWRITE_JPEG_QUALITY, 85])
            gallery.append((pv_name, base, len(lines)))
            n_keep += 1
            last_small = small
            last_t = t_sec

            print("  [%3d] 帧%-6d %5.1fs  %d 框 (%s)" % (
                n_keep, idx, t_sec, len(lines),
                " ".join("%s%d" % (c, n_box[c]) for c in CLASSES if n_box[c])),
                flush=True)

        cap.release()
        stats.append((name, n_keep, n_box, dur, time.time() - t0))

    # ---------- 裁剪图落盘 + 拼版 ----------
    for c in CLASSES:
        items = crops_by_cls[c]
        for k, (patch, src) in enumerate(items):
            cv2.imwrite(os.path.join(dirs["crops"], c,
                                     "%s_%03d.png" % (c, k)), patch)
        if not items:
            print("[提示] 类别 %s 无裁剪图" % c, flush=True)
            continue
        tile = 96
        cols = min(10, len(items))
        rows = int(np.ceil(len(items) / cols))
        sheet = np.full((rows * (tile + 14), cols * tile, 3), 24, np.uint8)
        for k, (patch, _) in enumerate(items):
            r, cc = divmod(k, cols)
            thumb = cv2.resize(patch, (tile, tile))
            y0 = r * (tile + 14)
            sheet[y0:y0 + tile, cc * tile:(cc + 1) * tile] = thumb
            cv2.putText(sheet, str(k), (cc * tile + 2, y0 + tile + 11),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.34, (190, 190, 190), 1)
        cv2.imwrite(os.path.join(dirs["sheets"], "%s_sheet.png" % c), sheet)
        print("拼版 %s: %d 张" % (c, len(items)), flush=True)

    # ---------- 预览画廊 ----------
    rows_html = []
    for pv_name, base, n in gallery:
        rows_html.append(
            '<figure><img loading="lazy" src="preview/%s" alt="%s">'
            '<figcaption>%s · %d 框</figcaption></figure>'
            % (html.escape(pv_name), base, base, n))
    doc = """<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>自动标注检查画廊</title>
<style>
body{background:#15171a;color:#d8dce1;font:14px/1.5 system-ui,"Microsoft YaHei";
     margin:0;padding:20px}
h1{font-size:18px;font-weight:600;margin:0 0 4px}
p.hint{color:#8b949e;margin:0 0 18px}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(300px,1fr));gap:12px}
figure{margin:0;background:#1d2026;border:1px solid #2a2e35;border-radius:6px;
       overflow:hidden}
img{width:100%%;display:block}
figcaption{padding:6px 8px;color:#9aa4b0;font-size:12px}
</style></head><body>
<h1>自动标注检查画廊（共 %d 帧）</h1>
<p class="hint">绿=L  橙=T  红=X，数字是该交点的投票数（越大越可信）。
框位置/类别有问题的帧，在标注工具里改正或删除对应 txt 即可。</p>
<div class="grid">%s</div>
</body></html>""" % (len(gallery), "\n".join(rows_html))
    with open(os.path.join(args.out_dir, "review.html"), "w",
              encoding="utf-8") as f:
        f.write(doc)

    # ---------- 报告 ----------
    tot_frames = sum(s[1] for s in stats)
    tot_box = {c: sum(s[2][c] for s in stats) for c in CLASSES}
    rep = ["# 自动标注素材报告", "",
           "生成时间：%s" % time.strftime("%Y-%m-%d %H:%M"),
           "抽帧间隔 %.2fs，最少投票数 %d，标注框半边长 %.1f×估计线宽" %
           (args.stride, args.min_votes, args.box_mult), "",
           "| 视频 | 时长(s) | 保留帧 | L | T | 耗时(s) |",
           "|---|---|---|---|---|---|"]
    for name, n_keep, n_box, dur, cost in stats:
        rep.append("| %s | %.1f | %d | %d | %d | %.0f |" %
                   (name, dur, n_keep, n_box["L"], n_box["T"], cost))
    rep += ["", "合计：%d 帧，L %d / T %d 个框" %
            (tot_frames, tot_box["L"], tot_box["T"]), "",
            "## 下一步", "",
            "1. 用 X-AnyLabeling 打开 `images` 目录，格式选 YOLO，自动载入 `labels`",
            "2. 先看 `sheets/*.png` 确认两类的形态是否分对（L 两条线都终止、T 一条线终止在另一条线上）",
            "3. 逐帧修正：漏检补框、误检删框、类别错的改类别",
            "4. 标注完成后按 YOLO 目录结构拆分 train/val 训练"]
    with open(os.path.join(args.out_dir, "report.md"), "w",
              encoding="utf-8") as f:
        f.write("\n".join(rep) + "\n")

    print("\n完成，用时 %.0fs" % (time.time() - t_start), flush=True)
    print("帧 %d，框 L%d T%d" % (tot_frames, tot_box["L"],
                                tot_box["T"]), flush=True)
    print("输出目录：%s" % args.out_dir, flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
