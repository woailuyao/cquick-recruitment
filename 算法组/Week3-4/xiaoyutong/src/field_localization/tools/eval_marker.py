"""训练完之后：在测试集上评估 + 出可视化对比图。

用法：
    python tools/eval_marker.py
    python tools/eval_marker.py --weights runs/marker/weights/best.pt --conf 0.35
"""

import argparse
import glob
import os
import sys

import cv2
import numpy as np

def _repo_root():
    """向上找到包含 config/field.yaml 的目录，作为仓库根。"""
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(4):
        if os.path.exists(os.path.join(d, "config", "field.yaml")):
            return d
        d = os.path.dirname(d)
    return os.getcwd()

ROOT = _repo_root()
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision2.console import setup_console  # noqa: E402
from vision2.io_utils import imread, imwrite  # noqa: E402

setup_console()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default=os.path.join(ROOT, "runs", "marker", "weights", "best.pt"))
    ap.add_argument("--data", default=os.path.join(ROOT, "dataset", "marker_data.yaml"))
    ap.add_argument("--imgsz", type=int, default=480)
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--out", default=os.path.join(ROOT, "calibration", "out", "eval_grid.jpg"))
    args = ap.parse_args()

    from ultralytics import YOLO

    if not os.path.exists(args.weights):
        print("[错误] 找不到权重：%s" % args.weights)
        return 1

    model = YOLO(args.weights)
    metrics = model.val(data=args.data, imgsz=args.imgsz, conf=args.conf, plots=False, verbose=False)

    print("== 测试集/验证集指标 ==")
    print("  Precision  %.4f" % metrics.box.mp)
    print("  Recall     %.4f" % metrics.box.mr)
    print("  mAP50      %.4f" % metrics.box.map50)
    print("  mAP50-95   %.4f" % metrics.box.map)
    try:
        names = model.names
        for i, c in enumerate(metrics.box.ap_class_index):
            print("    类 %-10s mAP50 %.4f" % (names[int(c)], metrics.box.ap50[i]))
    except Exception:
        pass

    # 可视化：抽 4 张测试图，画预测框
    imgs = sorted(glob.glob(os.path.join(ROOT, "dataset", "marker", "images", "test", "*.jpg")))[:4]
    tiles = []
    for p in imgs:
        im = imread(p)
        if im is None:
            continue
        r = model.predict(im, conf=args.conf, imgsz=args.imgsz, verbose=False)[0]
        for b in r.boxes:
            x1, y1, x2, y2 = [int(v) for v in b.xyxy[0].tolist()]
            cls = model.names[int(b.cls.item())]
            col = {"LCross": (0, 200, 255), "TCross": (80, 220, 80), "XCross": (255, 140, 60)}.get(cls, (255, 255, 255))
            cv2.rectangle(im, (x1, y1), (x2, y2), col, 3)
            cv2.putText(im, "%s %.2f" % (cls, b.conf.item()), (x1 + 4, y1 + 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)
        cv2.putText(im, os.path.basename(p), (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        tiles.append(cv2.resize(im, (440, 248)))

    if tiles:
        while len(tiles) % 2:
            tiles.append(np.zeros((248, 440, 3), np.uint8))
        grid = np.vstack([np.hstack(tiles[i:i + 2]) for i in range(0, len(tiles), 2)])
        imwrite(args.out, grid)
        print("")
        print("预测可视化：%s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
