#!/usr/bin/env python3
"""本地画框标注工具（支持多类别），替代 labelImg / makesense.ai。

鼠标拖一个框 = 一个目标；保存成 YOLO 格式 .txt（每行 class cx cy w h），
和图片同目录同名。

操作：
  数字键 1-9   切换当前类别（顶部黑条会显示当前类别）
  鼠标左键拖拽  画一个框（用当前类别）
  D / 空格     保存当前并下一张
  A            上一张
  U            撤销上一个框
  C            清空当前所有框
  Q / Esc      退出

用法：
  单类（足球）：
    python tools\\annotate.py --dir C:\\...\\frames_sel\\ball_v04 --class-name ball
  多类（场地标志）：
    python tools\\annotate.py --dir C:\\...\\frames_marker\\marker --classes "LCross,TCross,XCross,PenaltyPoint"
"""
from __future__ import annotations

import argparse
import os

import cv2
import numpy as np

IMG_EXT = (".jpg", ".jpeg", ".png", ".bmp")
PALETTE = [(0, 255, 0), (0, 165, 255), (255, 0, 255), (0, 255, 255),
           (255, 128, 0), (128, 255, 0), (255, 0, 0), (0, 0, 255), (200, 200, 0)]


def save_labels(directory: str, stem: str, boxes, w: int, h: int) -> bool:
    """boxes: [(x1,y1,x2,y2,cls)] 像素坐标。没有框就删除旧标签。"""
    path = os.path.join(directory, stem + ".txt")
    if not boxes:
        if os.path.exists(path):
            os.remove(path)
        return False
    lines = []
    for b in boxes:
        x1, y1, x2, y2, cls = b[0], b[1], b[2], b[3], b[4]
        x1, x2 = sorted((max(0.0, min(w, float(x1))), max(0.0, min(w, float(x2)))))
        y1, y2 = sorted((max(0.0, min(h, float(y1))), max(0.0, min(h, float(y2)))))
        bw, bh = x2 - x1, y2 - y1
        if bw < 3 or bh < 3:
            continue
        cx, cy = (x1 + x2) / 2.0 / w, (y1 + y2) / 2.0 / h
        lines.append(f"{int(cls)} {cx:.6f} {cy:.6f} {bw/w:.6f} {bh/h:.6f}")
    if not lines:
        return False
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    return True


def load_labels(directory: str, stem: str, w: int, h: int):
    path = os.path.join(directory, stem + ".txt")
    boxes = []
    if not os.path.exists(path):
        return boxes
    with open(path, "r", encoding="utf-8") as f:
        for ln in f:
            t = ln.split()
            if len(t) < 5:
                continue
            try:
                cls = int(float(t[0]))
                cx, cy, bw, bh = (float(t[1]), float(t[2]), float(t[3]), float(t[4]))
            except ValueError:
                continue
            boxes.append(((cx - bw / 2) * w, (cy - bh / 2) * h,
                          (cx + bw / 2) * w, (cy + bh / 2) * h, cls))
    return boxes


class App:
    def __init__(self, directory: str, classes, start: int = 0):
        self.dir = os.path.abspath(directory)
        self.classes = list(classes)
        self.cur = 0
        self.files = sorted(f for f in os.listdir(self.dir) if f.lower().endswith(IMG_EXT))
        if not self.files:
            raise SystemExit(f"[error] {self.dir} 里没有图片")
        self.i = max(0, min(start, len(self.files) - 1))
        self.boxes = []
        self.drag_start = None
        self.drag_now = None
        self.img = None
        self.win = "annotate  |  drag=box  1-9=class  D=save+next  A=prev  U=undo  C=clear  Q=quit"
        cv2.namedWindow(self.win, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self.win, self.on_mouse)

    def color(self, cls: int):
        return PALETTE[cls % len(PALETTE)]

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drag_start = (x, y)
            self.drag_now = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self.drag_start is not None:
            self.drag_now = (x, y)
        elif event == cv2.EVENT_LBUTTONUP and self.drag_start is not None:
            x0, y0 = self.drag_start
            if abs(x - x0) > 4 and abs(y - y0) > 4:
                self.boxes.append((x0, y0, x, y, self.cur))
            self.drag_start = None
            self.drag_now = None

    def draw(self):
        vis = self.img.copy()
        h, w = vis.shape[:2]
        for b in self.boxes:
            x1, y1, x2, y2, cls = b
            c = self.color(cls)
            cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), c, 2)
            nm = self.classes[cls] if cls < len(self.classes) else f"id{cls}"
            cv2.putText(vis, nm, (int(x1) + 3, max(14, int(y1) - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2, cv2.LINE_AA)
        if self.drag_start and self.drag_now:
            cv2.rectangle(vis, self.drag_start, self.drag_now,
                          self.color(self.cur), 1)

        name = self.files[self.i]
        done = len([f for f in self.files
                    if os.path.exists(os.path.join(
                        self.dir, os.path.splitext(f)[0] + ".txt"))])
        cv2.rectangle(vis, (0, 0), (w, 64), (0, 0, 0), -1)
        cur_name = self.classes[self.cur] if self.cur < len(self.classes) else "?"
        cv2.putText(vis, f"[{self.cur+1}] {cur_name}", (8, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, self.color(self.cur), 2, cv2.LINE_AA)
        menu = "  ".join(f"{i+1}:{n}" for i, n in enumerate(self.classes))
        cv2.putText(vis, menu[:120], (8, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (160, 160, 160), 1, cv2.LINE_AA)
        cv2.putText(vis, f"{self.i+1}/{len(self.files)}  boxes={len(self.boxes)}  已标 {done}",
                    (max(8, w - 430), 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 2, cv2.LINE_AA)
        if not self.boxes:
            cv2.putText(vis, "drag on the target", (w // 2 - 120, h - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow(self.win, vis)

    def load(self):
        path = os.path.join(self.dir, self.files[self.i])
        self.img = cv2.imread(path)
        if self.img is None:
            raise SystemExit(f"[error] 读不了 {path}")
        h, w = self.img.shape[:2]
        stem = os.path.splitext(self.files[self.i])[0]
        self.boxes = load_labels(self.dir, stem, w, h)

    def save(self):
        h, w = self.img.shape[:2]
        stem = os.path.splitext(self.files[self.i])[0]
        return save_labels(self.dir, stem, self.boxes, w, h)

    def run(self):
        self.load()
        print(f"[annotate] {self.dir}")
        print(f"           共 {len(self.files)} 张，类别 {self.classes}")
        print("           数字键切类别；鼠标拖框；D/空格 保存并下一张；Q 退出")
        while True:
            self.draw()
            k = cv2.waitKey(20) & 0xFF
            if ord("1") <= k <= ord("9"):
                idx = k - ord("1")
                if idx < len(self.classes):
                    self.cur = idx
            elif k in (ord("d"), 32, 13):
                self.save()
                if self.i < len(self.files) - 1:
                    self.i += 1
                    self.load()
                else:
                    print("[annotate] 已经是最后一张，按 Q 退出")
            elif k == ord("a"):
                if self.i > 0:
                    self.i -= 1
                    self.load()
            elif k == ord("u"):
                if self.boxes:
                    self.boxes.pop()
            elif k == ord("c"):
                self.boxes.clear()
            elif k in (ord("q"), 27):
                self.save()
                break
        cv2.destroyAllWindows()
        done = len([f for f in self.files
                    if os.path.exists(os.path.join(
                        self.dir, os.path.splitext(f)[0] + ".txt"))])
        print(f"[annotate] 结束：{len(self.files)} 张里有 {done} 张带标签")
        if done == 0:
            print("[warn] 一个标签都没存，检查是不是没按 D")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    ap.add_argument("--class-name", default=None, help="单类别时用这个")
    ap.add_argument("--classes", default=None, help="多类别，逗号分隔，顺序即 class_id")
    ap.add_argument("--start", type=int, default=0, help="从第几张开始（0 起）")
    args = ap.parse_args()
    if not os.path.isdir(args.dir):
        print(f"[error] 目录不存在：{args.dir}")
        return 2
    if args.classes:
        classes = [c.strip() for c in args.classes.split(",") if c.strip()]
    elif args.class_name:
        classes = [args.class_name]
    else:
        classes = ["ball"]
    App(args.dir, classes, args.start).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
