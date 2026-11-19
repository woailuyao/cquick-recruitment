# -*- coding: utf-8 -*-
"""
任务 2 标志点检测训练（L / T / X 三类）

用法
    conda activate calib
    python train_marker.py                    # 默认 100 epoch / 640
    python train_marker.py --epochs 60        # 时间紧就少跑点
    python train_marker.py --imgsz 640 --batch 16

为什么是 640
    源视频就是 640x480，imgsz=640 时短边不缩放（超参里 640 指长边），
    等于用满原生分辨率；再往上调只是插值放大，不会多出信息，只会更慢。

数据增强的取舍
    标志点是「白线接头」这种纯几何结构：
    · 左右翻转完全安全（L/T/X 翻转后还是 L/T/X），白赚一倍样本 -> fliplr=0.5
    · 旋转只给 ±5°：真实录制里云台会有几度侧倾，但转多了会出现现实中不存在的视角
    · mosaic 只开到 0.5：mosaic 会把标志点缩得很小，与"标志点本来就只有 20px"的困难叠加
    · 不用上下翻转：场地没有倒过来的情况
"""
import argparse
import os
import sys

from ultralytics import YOLO

CLASSES = ["L", "T", "X"]

# ultralytics 8.4 把相对 project 拼到 runs/detect/ 下，必须传绝对路径，
# 否则权重会落到 runs/detect/runs_marker/baseline/ 而 demo.py 找不到
HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.join(HERE, "runs_marker")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="marker_dataset/data.yaml")
    ap.add_argument("--epochs", type=int, default=100)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=8)
    ap.add_argument("--workers", type=int, default=4,
                    help="dataloader 进程数。本机只剩 3.6 GB 空闲内存，"
                         "顶不住就传 0~2（每进程约 100~200 MB）")
    ap.add_argument("--device", default="cpu", help="有 N 卡写 0")
    ap.add_argument("--name", default="baseline")
    ap.add_argument("--weights", default="yolov8n.pt")
    ap.add_argument("--patience", type=int, default=30)
    ap.add_argument("--resume", default=None,
                    help="断点续训：传入上次的 last.pt，例如 "
                         "runs_marker/baseline/weights/last.pt。传了它以后 "
                         "--epochs/--batch/--data 等参数都以上次存档为准，"
                         "总轮数仍是上次设定的值")
    args = ap.parse_args()

    if args.resume:
        # resume=True 会从 ckpt 里读回原始 args 和优化器状态、学习率调度的进度，
        # 所以这里不要覆盖 epochs/batch，否则等于换了实验
        last = args.resume
        if not os.path.exists(last):
            sys.exit(f"找不到续训权重 {last}")
        print(f"断点续训：{last}")
        model = YOLO(last)
        model.train(resume=True)
    else:
        model = YOLO(args.weights)
        model.train(
            data=args.data,
            epochs=args.epochs,
            imgsz=args.imgsz,
            device=args.device,
            batch=args.batch,
            workers=args.workers,
            seed=42,
            patience=args.patience,
            project=PROJ,
            name=args.name,
            # 见文件头说明
            mosaic=0.5,
            close_mosaic=10,
            fliplr=0.5,
            flipud=0.0,
            degrees=5.0,
            translate=0.1,
            scale=0.3,
            hsv_h=0.015,
            hsv_s=0.5,
            hsv_v=0.4,
        )

    best_path = os.path.join(
        str(getattr(getattr(model, "trainer", None), "save_dir",
                    os.path.join(PROJ, args.name))), "weights", "best.pt")
    print(f"\n训练完成，最佳权重：{best_path}")
    if not os.path.exists(best_path):
        sys.exit(f"找不到 {best_path}，检查 ultralytics 版本的目录策略")

    best = YOLO(best_path)
    m = best.val(data=args.data, split="test",
                 project=os.path.join(PROJ, "val"), name=args.name, verbose=False)
    print("\n===== 测试集指标（写进 README）=====")
    print(f"Precision : {m.box.mp:.4f}")
    print(f"Recall    : {m.box.mr:.4f}")
    print(f"mAP50     : {m.box.map50:.4f}")
    print(f"mAP50-95  : {m.box.map:.4f}")
    # 分类别看，哪一类最弱一目了然
    try:
        names = m.names
        for i, ap50 in enumerate(m.box.ap50):
            print(f"  class {names.get(i, i):<2} AP50 = {ap50:.4f}")
    except Exception:
        pass


if __name__ == "__main__":
    main()
