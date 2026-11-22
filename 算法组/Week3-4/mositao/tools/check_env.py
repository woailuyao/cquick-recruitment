# -*- coding: utf-8 -*-
"""环境自检：版本、GPU、cuDNN、预训练权重、以及一次真实的前向推理耗时。

用途：
1. 第 3-4 周 README 需要记录 Ubuntu/Python/PyTorch/Ultralytics 版本与 GPU 型号；
2. 录屏自证时可作为第一段「环境就绪」画面；
3. 训练前确认 GPU 真的能算（只看 cuda.is_available() 不够，
   旧版 torch 在 Blackwell 上会「报可用、一算就崩」）。

运行（项目根目录）：
    python tools/check_env.py
    python tools/check_env.py --device cpu
"""

from __future__ import annotations

import argparse
import platform
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.config_io import sha256_of          # noqa: E402
from src.common.paths import WEIGHTS_DIR, rel      # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="训练环境自检")
    ap.add_argument("--device", default="0", help="0=第一块 GPU，cpu=CPU")
    ap.add_argument("--weights", default="yolov8n.pt", help="相对 weights/ 的权重文件名")
    ap.add_argument("--repeats", type=int, default=10, help="推理计时重复次数")
    args = ap.parse_args()

    print("=" * 72)
    print("CQuick 第 3-4 周 · 训练环境自检")
    print("=" * 72)
    print(f"操作系统  : {platform.system()} {platform.release()} ({platform.machine()})")
    print(f"Python    : {platform.python_version()}  [{sys.executable.split(chr(92))[-2]}]")

    import numpy as np
    print(f"NumPy     : {np.__version__}")

    import cv2
    print(f"OpenCV    : {cv2.__version__}")

    import torch
    print(f"PyTorch   : {torch.__version__}  (编译 CUDA {torch.version.cuda})")
    print(f"cuDNN     : {torch.backends.cudnn.version() if torch.backends.cudnn.is_available() else '不可用'}")

    device_ok = False
    if torch.cuda.is_available():
        cap = torch.cuda.get_device_capability(0)
        props = torch.cuda.get_device_properties(0)
        print(f"GPU       : {props.name}")
        print(f"算力      : sm_{cap[0]}{cap[1]}  显存 {props.total_memory / 1e9:.2f} GB")
        if args.device != "cpu":
            a = torch.randn(2048, 2048, device="cuda")
            b = torch.randn(2048, 2048, device="cuda")
            for _ in range(5):          # 热身：把 CUDA 上下文初始化与显存分配排除在计时之外
                a @ b
            torch.cuda.synchronize()
            t0 = time.perf_counter()
            for _ in range(20):
                a @ b
            torch.cuda.synchronize()
            dt = (time.perf_counter() - t0) / 20
            gflop = 2 * 2048 ** 3
            print(f"GPU 算力   : 2048³ fp32 matmul {dt * 1000:.2f} ms（{gflop / dt / 1e12:.2f} TFLOPS）")
            device_ok = True
        else:
            print("GPU 算力   : 已按要求跳过（--device cpu）")
    else:
        print("GPU       : 不可用，将退回 CPU")

    import ultralytics
    print(f"Ultralytics: {ultralytics.__version__}")

    wpath = Path(args.weights)
    if not wpath.is_absolute():
        wpath = WEIGHTS_DIR / wpath
    print("-" * 72)
    if not wpath.is_file():
        print(f"权重      : 未找到 {rel(wpath)}（请放入 weights/ 目录后再试）")
        print("=" * 72)
        return 1
    print(f"权重      : {rel(wpath)}  ({wpath.stat().st_size / 1e6:.2f} MB)")
    print(f"SHA256    : {sha256_of(wpath)}")

    from ultralytics import YOLO
    model = YOLO(str(wpath))
    n_param = sum(p.numel() for p in model.model.parameters())
    print(f"模型      : {wpath.name}  参数量 {n_param / 1e6:.3f} M  类别数 {len(model.names)}")

    dev = "cpu" if args.device == "cpu" or not torch.cuda.is_available() else "0"
    dummy = np.random.randint(0, 255, (720, 1280, 3), dtype=np.uint8)
    model.predict(dummy, imgsz=640, device=dev, verbose=False)      # 预热

    t0 = time.perf_counter()
    for _ in range(args.repeats):
        model.predict(dummy, imgsz=640, device=dev, verbose=False)
    dt = (time.perf_counter() - t0) / args.repeats
    print(f"推理      : 1280x720 输入 @ imgsz=640，{dt * 1000:.1f} ms/帧 "
          f"（≈{1 / dt:.1f} FPS），设备 {dev}")
    print("-" * 72)
    print("结论      : " + ("环境就绪，可以训练与实时推理" if (device_ok or dev == "cpu")
                          else "GPU 未验证，请检查驱动与 torch 版本"))
    print("=" * 72)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
