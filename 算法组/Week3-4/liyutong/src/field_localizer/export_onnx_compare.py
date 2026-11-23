# -*- coding: utf-8 -*-
"""
细则加分项 +3：导出 ONNX，并在同一测试片段上做 PyTorch / ONNXRuntime 推理对比。

产出 onnx_compare.md，含三张表：
  1. 模型文件体积与导出参数
  2. 同一批测试图上的耗时（预热后取 20 次均值/中位/P95）
  3. 检测结果一致性（PyTorch vs ONNXRuntime，按 IoU≥0.5 配对）

用法
    python export_onnx_compare.py            # 两个模型都做
    python export_onnx_compare.py --ball-only
"""
import argparse
import glob
import os
import statistics
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

JOBS = [
    {
        "name": "任务 1 · 足球检测",
        "pt": os.path.join(HERE, "runs_ball", "baseline", "weights", "best.pt"),
        "onnx": os.path.join(HERE, "runs_ball", "baseline", "weights", "best.onnx"),
        "imgs": os.path.join(HERE, "soccer_dataset", "images", "test"),
        "conf": 0.25,
    },
    {
        "name": "任务 2 · 场地标志",
        "pt": os.path.join(HERE, "runs_marker", "baseline", "weights", "best.pt"),
        "onnx": os.path.join(HERE, "runs_marker", "baseline", "weights", "best.onnx"),
        "imgs": os.path.join(HERE, "marker_dataset", "images", "test"),
        "conf": 0.25,
    },
]


def export(job):
    from ultralytics import YOLO

    if os.path.exists(job["onnx"]):
        print(f"[export] 已存在，跳过 {job['onnx']}")
        return job["onnx"]
    print(f"[export] {job['pt']} -> onnx (opset 12, imgsz 640)")
    m = YOLO(job["pt"])
    out = m.export(format="onnx", imgsz=640, opset=12, simplify=False, device="cpu")
    print(f"[export] 完成：{out}  {os.path.getsize(out) / 1e6:.2f} MB")
    return out


def pick_images(dirpath, n=20):
    exts = ("*.jpg", "*.jpeg", "*.png", "*.bmp")
    files = []
    for e in exts:
        files += glob.glob(os.path.join(dirpath, e))
    files.sort()
    if not files:
        raise SystemExit(f"测试图目录为空：{dirpath}")
    if len(files) > n:
        idx = np.linspace(0, len(files) - 1, n).round().astype(int)
        files = [files[i] for i in idx]
    return files


def run_one(weights, imgs, conf, warmup=3):
    """返回 (每次推理耗时列表, 每张图的检测结果列表)"""
    from ultralytics import YOLO

    m = YOLO(weights)
    ts, dets = [], []
    for i, f in enumerate(imgs):
        t0 = time.perf_counter()
        r = m.predict(f, conf=conf, imgsz=640, verbose=False)[0]
        dt = (time.perf_counter() - t0) * 1000
        if i >= warmup:
            ts.append(dt)
        boxes = []
        for b in r.boxes:
            x1, y1, x2, y2 = (float(v) for v in b.xyxy[0])
            boxes.append((int(b.cls[0]), x1, y1, x2, y2, float(b.conf[0])))
        dets.append(boxes)
    return ts, dets


def iou(a, b):
    ix1, iy1 = max(a[1], b[1]), max(a[2], b[2])
    ix2, iy2 = min(a[3], b[3]), min(a[4], b[4])
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    ua = (a[3] - a[1]) * (a[4] - a[2]) + (b[3] - b[1]) * (b[4] - b[2]) - inter
    return inter / ua if ua > 0 else 0.0


def compare(dets_a, dets_b, thr=0.5):
    n_a = sum(len(d) for d in dets_a)
    n_b = sum(len(d) for d in dets_b)
    matched, conf_diff = 0, []
    for da, db in zip(dets_a, dets_b):
        used = set()
        for a in da:
            best, bj = 0.0, -1
            for j, b in enumerate(db):
                if j in used or a[0] != b[0]:
                    continue
                v = iou(a, b)
                if v > best:
                    best, bj = v, j
            if best >= thr and bj >= 0:
                used.add(bj)
                matched += 1
                conf_diff.append(abs(a[5] - db[bj][5]))
    return n_a, n_b, matched, (statistics.mean(conf_diff) if conf_diff else float("nan"))


def fmt_stat(ts):
    ts = sorted(ts)
    p95 = ts[min(len(ts) - 1, int(round(0.95 * (len(ts) - 1))))]
    return statistics.mean(ts), statistics.median(ts), p95


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=20, help="每类取多少张测试图")
    ap.add_argument("--ball-only", action="store_true")
    args = ap.parse_args()

    jobs = JOBS[:1] if args.ball_only else JOBS
    md = ["# ONNX 导出与 PyTorch / ONNXRuntime 推理对比", "",
          f"> 生成时间：{time.strftime('%Y-%m-%d %H:%M')}　"
          "> 设备：CPU（torch 2.2.2+cpu / onnxruntime CPUExecutionProvider），"
          "输入 640×640，conf 0.25，预热 3 张后统计。", ""]

    for job in jobs:
        if not os.path.exists(job["pt"]):
            print(f"[skip] 缺 {job['pt']}")
            continue
        onnx_path = export(job)
        imgs = pick_images(job["imgs"], args.n)
        print(f"[{job['name']}] 测试图 {len(imgs)} 张：{job['imgs']}")

        t_pt, d_pt = run_one(job["pt"], imgs, job["conf"])
        t_ox, d_ox = run_one(onnx_path, imgs, job["conf"])

        m1, md1, p1 = fmt_stat(t_pt)
        m2, md2, p2 = fmt_stat(t_ox)
        n_a, n_b, matched, cdiff = compare(d_pt, d_ox)

        md += [f"## {job['name']}", "",
               "| 项 | PyTorch (.pt) | ONNXRuntime (.onnx) |",
               "| --- | ---: | ---: |",
               f"| 文件体积 | {os.path.getsize(job['pt']) / 1e6:.2f} MB "
               f"| {os.path.getsize(onnx_path) / 1e6:.2f} MB |",
               f"| 单图耗时 均值 | {m1:.1f} ms | {m2:.1f} ms |",
               f"| 单图耗时 中位 | {md1:.1f} ms | {md2:.1f} ms |",
               f"| 单图耗时 P95 | {p1:.1f} ms | {p2:.1f} ms |",
               f"| 等效 FPS | {1000 / m1:.1f} | {1000 / m2:.1f} |",
               "",
               f"检测结果一致性（IoU≥0.5 配对，{len(imgs)} 张图）："
               f"PyTorch 共 {n_a} 框 / ONNX 共 {n_b} 框 / 配对成功 {matched} 框"
               f"（{100.0 * matched / max(1, n_a):.1f}%），"
               f"配对框置信度平均差 {cdiff:.4f}", ""]
        print(f"  pt {m1:.1f} ms | onnx {m2:.1f} ms | 框 {n_a}/{n_b} 配对 {matched}")

    out = os.path.join(HERE, "onnx_compare.md")
    with open(out, "w", encoding="utf-8") as f:
        f.write("\n".join(md) + "\n")
    print(f"\n已写 {out}")


if __name__ == "__main__":
    main()
