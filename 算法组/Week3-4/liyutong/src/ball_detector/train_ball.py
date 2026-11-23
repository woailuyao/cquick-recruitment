# -*- coding: utf-8 -*-
"""
任务 1 训练脚本（考核基线：yolov8n / imgsz 640 / 50 epochs）

用法（最常用）：
    python train_ball.py
用法（内存紧张，先试这个）：
    python train_ball.py --batch 4 --workers 2
用法（确认机器扛得住，想跑快一点）：
    python train_ball.py --batch 8 --workers 4

前提：soccer_dataset/data.yaml 已由 split_dataset.py 生成，标注已完成

关于 workers：ultralytics 默认 workers=8，每个 dataloader 子进程常驻
100~200 MB。本机实测剩 3.6 GB 空闲内存（341 个进程占 12.2 GB），
默认 8 会顶到 swap 甚至被系统杀掉。所以这里把默认值降到 4，
出问题就显式传 --workers 0（全部在主线程序里读，最慢最稳）。
"""
import argparse
import os
import sys

from ultralytics import YOLO

# 注意：ultralytics 8.4 会把「相对路径」的 project 拼到 runs/detect/ 下面，
# 结果权重跑到 runs/detect/runs_ball/baseline/ 而不是 runs_ball/baseline/。
# 传绝对路径才能固定在预期位置，否则后面 demo.py / val 全都找不到文件。
HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.join(HERE, "runs_ball")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="soccer_dataset/data.yaml")
    ap.add_argument("--epochs", type=int, default=50, help="考核基线 50")
    ap.add_argument("--imgsz", type=int, default=640, help="考核基线 640")
    ap.add_argument("--batch", type=int, default=8,
                    help="显存/内存不足就降到 4 或 2")
    ap.add_argument("--workers", type=int, default=4,
                    help="dataloader 进程数。内存紧张传 0~2，每进程约 100~200 MB")
    ap.add_argument("--device", default="cpu",
                    help="没有 NVIDIA 显卡就保持 cpu；有的话改成 0")
    ap.add_argument("--name", default="baseline")
    ap.add_argument("--weights", default="yolov8n.pt",
                    help="预训练权重，迁移学习起点（考核允许）")
    args = ap.parse_args()

    model = YOLO(args.weights)
    model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        device=args.device,
        batch=args.batch,
        workers=args.workers,
        seed=42,
        project=PROJ,
        name=args.name,
    )

    # 从 trainer 拿真实保存目录，别自己拼字符串
    save_dir = str(getattr(getattr(model, "trainer", None), "save_dir",
                           os.path.join(PROJ, args.name)))
    best_path = os.path.join(save_dir, "weights", "best.pt")
    print(f"\n训练完成，输出目录 {save_dir}")
    print(f"最佳权重 {best_path}")
    if not os.path.exists(best_path):
        sys.exit(f"找不到 {best_path}，检查 ultralytics 版本的目录策略")

    # 在独立测试集上评估（考核要求：报告 test 指标）
    best = YOLO(best_path)
    metrics = best.val(data=args.data, split="test",
                       project=os.path.join(PROJ, "val"), name=args.name, verbose=False)
    print("\n===== 测试集指标（写进 README）=====")
    print(f"Precision : {metrics.box.mp:.4f}")
    print(f"Recall    : {metrics.box.mr:.4f}")
    print(f"mAP50     : {metrics.box.map50:.4f}")
    print(f"mAP50-95  : {metrics.box.map:.4f}")


if __name__ == "__main__":
    main()
