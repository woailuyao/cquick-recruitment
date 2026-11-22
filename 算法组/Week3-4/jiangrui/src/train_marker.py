# -*- coding: utf-8 -*-
"""训练场地标记点检测器 (TCross / LCross)

用法:
    python train_marker.py            # 正式训练
    python train_marker.py --smoke    # 1 epoch 冒烟测试，只验证配置能跑通

注意:
    1) flipud 必须为 0 —— T 型与 L 型标记上下翻转后语义会变；
       水平翻转可以保留（fliplr=0.5），四角 L 本身存在左右两种拐角朝向，
       横翻相当于合法的朝向增强。
    2) 训练采用 0=TCross, 1=LCross，与考核规范的顺序相反；
       权重内 names 与训练一致，运行时按名称映射，不做 remap。
    3) 权重起点使用本地 yolov8n.pt，避免联网下载；路径按实际存放位置修改。
"""

import argparse
import os
import sys
from pathlib import Path

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))

DATA = os.path.join(ROOT, "dataset", "marker_data_all.yaml")
# yolov8n.pt 放在仓库根目录，或用环境变量 YOLOV8N_WEIGHTS 指定其他位置
WEIGHTS = os.environ.get("YOLOV8N_WEIGHTS", os.path.join(ROOT, "yolov8n.pt"))
PROJECT = os.path.join(ROOT, "runs")
NAME = "marker_yolov8n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--smoke", action="store_true", help="1 epoch 冒烟测试")
    ap.add_argument("--epochs", type=int, default=120)
    ap.add_argument("--batch", type=int, default=8)
    ap.add_argument("--imgsz", type=int, default=960)
    args = ap.parse_args()

    for p in (DATA, WEIGHTS):
        if not Path(p).exists():
            sys.exit(f"找不到文件: {p}")

    from ultralytics import YOLO

    model = YOLO(WEIGHTS)

    if args.smoke:
        model.train(
            data=DATA,
            epochs=1,
            imgsz=320,
            batch=4,
            workers=0,
            device=0,
            project=PROJECT,
            name="smoke",
            exist_ok=True,
            fliplr=0.5,
            flipud=0.0,
            plots=False,
            val=True,
            verbose=True,
        )
        return

    model.train(
        data=DATA,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=0,
        workers=2,
        project=PROJECT,
        name=NAME,
        exist_ok=True,
        # ---- 关键: 禁止上下翻转；左右翻转合法（L 角有左右两种朝向）----
        fliplr=0.5,
        flipud=0.0,
        # ---- 适度几何增强，小目标场景不要过猛 ----
        degrees=5.0,
        translate=0.10,
        scale=0.35,
        shear=2.0,
        perspective=0.0,
        mosaic=0.6,
        close_mosaic=15,
        mixup=0.0,
        copy_paste=0.0,
        # ---- 颜色增强可以放开 ----
        hsv_h=0.015,
        hsv_s=0.5,
        hsv_v=0.4,
        # ---- 训练控制 ----
        patience=40,
        optimizer="auto",
        lr0=0.01,
        lrf=0.01,
        warmup_epochs=3.0,
        cos_lr=True,
        cache=False,
        plots=True,
        val=True,
        seed=0,
    )


if __name__ == "__main__":
    main()