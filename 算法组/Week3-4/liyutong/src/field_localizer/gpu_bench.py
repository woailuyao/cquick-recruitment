# -*- coding: utf-8 -*-
"""ONNX 推理的 CPU 与 GPU（DirectML）对比

背景
    ONNX Runtime 有多个执行提供器（Execution Provider）。同一个 .onnx 文件，
    换成 DmlExecutionProvider 就能把卷积跑在 DX12 显卡上。
    本机是 RTX 5060 Laptop（8 GB），PyTorch 装的是 CPU 版（`pip install` 的
    `torch 2.2.2+cpu`），CUDA 版在新版 Windows 上被"智能应用控制"拦截，
    所以训练走 CPU；但 ONNX 这条路不依赖 torch 的 CUDA 构建，
    装 `onnxruntime-directml` 就能直接用上显卡。

用法
    pip install onnxruntime-directml        # 一次性
    python gpu_bench.py                     # 终端打印
    python gpu_bench.py --write             # 同时更新 onnx_compare.md 的 GPU 一节

口径
    纯前向（session.run），不含预处理、后处理与 NMS；输入 1×3×640×640；
    预热 5 次后统计 30 次，取中位。用随机张量测耗时（避免磁盘 IO 干扰），
    另用真实测试图验证 CPU 与 GPU 的输出数值是否一致。
"""
import argparse
import glob
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

# 允许把 onnxruntime-directml 装到独立目录后临时挂进来，不动主环境
for _cand in ("_dml",):
    _p = os.path.join(HERE, _cand)
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

import onnxruntime as ort  # noqa: E402

MODELS = [
    ("任务 1 · 足球 ball", os.path.join(HERE, "runs_ball", "baseline", "weights", "best.onnx")),
    ("任务 2 · 标志 L/T/X", os.path.join(HERE, "runs_marker", "baseline", "weights", "best.onnx")),
]

N_WARM = 5
N_RUN = 30
SIZE = 640


def pick_images(n=8):
    d = os.path.join(HERE, "soccer_dataset", "images", "test")
    files = sorted(glob.glob(os.path.join(d, "*.jpg")))
    if len(files) > n:
        idx = np.linspace(0, len(files) - 1, n).round().astype(int)
        files = [files[i] for i in idx]
    return files


def letterbox(path, size=SIZE):
    """等比缩放到 size×size，右下补 114（YOLOv8 的默认填充色）

    注意：目录名含中文时 cv2.imread 会返回 None（它走 ANSI 路径 API），
    所以这里用 np.fromfile + imdecode 读。
    """
    import cv2

    buf = np.fromfile(path, dtype=np.uint8)
    img = cv2.imdecode(buf, cv2.IMREAD_COLOR) if buf.size else None
    if img is None:
        return None
    h, w = img.shape[:2]
    r = min(size / h, size / w)
    nh, nw = int(round(h * r)), int(round(w * r))
    img = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    out = np.full((size, size, 3), 114, dtype=np.uint8)
    top, left = (size - nh) // 2, (size - nw) // 2
    out[top:top + nh, left:left + nw] = img
    out = cv2.cvtColor(out, cv2.COLOR_BGR2RGB)
    x = out.transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    return np.ascontiguousarray(x)


def session(path, providers):
    so = ort.SessionOptions()
    so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    so.log_severity_level = 3
    return ort.InferenceSession(path, so, providers=providers)


def bench(sess, x):
    name = sess.get_inputs()[0].name
    for _ in range(N_WARM):
        sess.run(None, {name: x})
    ts = []
    for _ in range(N_RUN):
        t0 = time.perf_counter()
        sess.run(None, {name: x})
        ts.append((time.perf_counter() - t0) * 1000)
    ts = np.array(ts)
    return float(np.median(ts)), float(ts.mean()), float(np.percentile(ts, 95))


def consistency(sa, sb, imgs):
    """逐图比较 CPU 与 GPU 的原始输出张量"""
    na, nb = sa.get_inputs()[0].name, sb.get_inputs()[0].name
    maxs, means = [], []
    for f in imgs:
        x = letterbox(f)
        if x is None:
            continue
        ya = sa.run(None, {na: x})[0]
        yb = sb.run(None, {nb: x})[0]
        d = np.abs(ya - yb)
        maxs.append(float(d.max()))
        means.append(float(d.mean()))
    if not maxs:
        return None
    return max(maxs), float(np.mean(means)), len(maxs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true", help="把 GPU 一节写进 onnx_compare.md")
    args = ap.parse_args()

    print("onnxruntime :", ort.__version__)
    provs = ort.get_available_providers()
    print("providers   :", provs)
    if "DmlExecutionProvider" not in provs:
        print("\n当前 onnxruntime 没有 DmlExecutionProvider。")
        print("先执行： pip install onnxruntime-directml")
        return 1

    imgs = pick_images()
    x = np.random.rand(1, 3, SIZE, SIZE).astype(np.float32)

    rows = []
    for name, path in MODELS:
        if not os.path.exists(path):
            print(f"[跳过] 缺模型 {path}")
            continue
        sc = session(path, ["CPUExecutionProvider"])
        sd = session(path, ["DmlExecutionProvider", "CPUExecutionProvider"])
        c = bench(sc, x)
        d = bench(sd, x)
        con = consistency(sc, sd, imgs)
        rows.append((name, os.path.getsize(path) / 1e6, c, d, con))
        print(f"\n{name}  ({rows[-1][1]:.2f} MB)")
        print(f"  CPU  中位 {c[0]:6.2f} ms | 均值 {c[1]:6.2f} | P95 {c[2]:6.2f} | {1000/c[0]:5.1f} FPS")
        print(f"  GPU  中位 {d[0]:6.2f} ms | 均值 {d[1]:6.2f} | P95 {d[2]:6.2f} | {1000/d[0]:5.1f} FPS")
        print(f"  加速 {c[0]/d[0]:.2f}×")
        if con:
            print(f"  输出一致性：{con[2]} 张真实测试图，最大绝对差 {con[0]:.2e}，平均 {con[1]:.2e}")

    if not args.write:
        return 0

    md = ["## GPU 推理对比（DirectML，RTX 5060 Laptop 8 GB）", "",
          f"> 生成时间：{time.strftime('%Y-%m-%d %H:%M')}　"
          f"> onnxruntime {ort.__version__} + `onnxruntime-directml`，"
          f"执行提供器 `DmlExecutionProvider`（DX12），输入 1×3×640×640，"
          f"预热 {N_WARM} 次后统计 {N_RUN} 次，取中位。", "",
          "**口径说明**：上面两节是 `ultralytics predict()` 的端到端单图耗时，含读图、letterbox、",
          "前后处理与 NMS；本节只统计 `session.run()` 的前向，所以同一个模型在 CPU 上的数字比上面低。",
          "两节的数字只在本节内部横向比较，不与上面两节交叉比较。", "",
          "| 模型 | 体积 | CPU 前向 | GPU 前向 | CPU P95 | GPU P95 | 加速比 |",
          "| --- | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for name, mb, c, d, con in rows:
        md.append(f"| {name} | {mb:.2f} MB | {c[0]:.2f} ms（{1000/c[0]:.1f} FPS） | "
                  f"**{d[0]:.2f} ms（{1000/d[0]:.1f} FPS）** | {c[2]:.2f} ms | {d[2]:.2f} ms | "
                  f"**{c[0]/d[0]:.1f}×** |")
    md.append("")
    for name, mb, c, d, con in rows:
        if con:
            md.append(f"- {name}：{con[2]} 张真实测试图上 CPU 与 GPU 的原始输出张量"
                      f"最大绝对差 {con[0]:.2e}、平均 {con[1]:.2e}，可视为同一结果。")
    md += ["",
           "这一节用的是 **DirectML** 后端，不是 TensorRT。原因是本机 PyTorch 为 CPU 版，",
           "TensorRT 链路需要 CUDA 版 torch + TensorRT runtime + CUDA Toolkit 三者同时到位，",
           "在这台机器上装不上（新版 PyTorch 的 DLL 被系统策略拦截，见 README 的 GPU 说明）。",
           "DirectML 不依赖 torch 的 CUDA 构建，只吃 ONNX，因此能直接用上这块显卡，",
           "用来验证「同一个模型换后端能快多少」这件事。", ""]

    out = os.path.join(HERE, "onnx_compare.md")
    with open(out, "r", encoding="utf-8") as f:
        base = f.read().rstrip("\n")
    marker = "## GPU 推理对比"
    if marker in base:
        base = base[:base.index(marker)].rstrip("\n")
    with open(out, "w", encoding="utf-8", newline="\n") as f:
        f.write(base + "\n\n" + "\n".join(md))
    print(f"\n已写入 {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
