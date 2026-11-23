# -*- coding: utf-8 -*-
"""
标注复核工具（OpenCV 界面，零额外依赖）

定位：**不是从零画框的工具，而是"复核预标注结果"的工具。**
prelabel.py 已经把 90% 的框放好了，这个工具让你快速：
    · 拖动修正框的位置
    · 删掉误检
    · 补上漏检
    · 快速翻页确认

用法：

    # 足球（单类）
    python scripts/label_tool.py --src dataset_raw --task ball

    # 场地标志（4 类，用数字键 1-4 选类别）
    python scripts/label_tool.py --src dataset_marker --task marker

操作：

    鼠标左键拖动   画新框
    左键单击框     选中（再按 x 删除）
    右键单击框     直接删除该框
    ← 或 a         上一张
    → 或 d         下一张
    SPACE 或 s     保存（切换图片时会自动保存，基本不用手动按）
    c              清空本图所有框
    u              撤销最后画的一个框
    1 2 3 4        选择类别（仅 --task marker）
    q 或 ESC       退出

    进度会记在 <src>/review_state.json 里，下次运行自动跳到第一张没复核的。
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

sys.path.insert(0, ROOT)
from src.config import FIELD_MARKER_CLASSES  # noqa: E402
sys.path.insert(0, ROOT)

MARKER_ORDER = FIELD_MARKER_CLASSES          # 现场实际存在的类别，见 src/config.py
COLORS = [(0, 220, 0), (0, 165, 255), (255, 170, 0), (255, 0, 255)]
SEL_COLOR = (0, 0, 255)

MAX_W, MAX_H = 1280, 760


class Labeler:
    def __init__(self, src, task, start_unreviewed=True, only_empty=False):
        self.src = src
        self.task = task
        self.classes = ["ball"] if task == "ball" else MARKER_ORDER
        self.images = sorted(glob.glob(os.path.join(src, "*", "*.jpg"))
                             + glob.glob(os.path.join(src, "*", "*.png")))
        if not self.images:
            raise SystemExit(f"{src} 下没有图片（期望 {src}/<片段>/*.jpg）")

        if only_empty:
            kept = []
            for p in self.images:
                tp = os.path.splitext(p)[0] + ".txt"
                if not os.path.isfile(tp) or os.path.getsize(tp) == 0:
                    kept.append(p)
            print(f"[--only-empty] 只复核空标注图: {len(kept)}/{len(self.images)} 张")
            if not kept:
                raise SystemExit("没有空标注的图了。")
            self.images = kept

        self.state_path = os.path.join(src, "review_state.json")
        self.state = {}
        if os.path.isfile(self.state_path):
            try:
                with open(self.state_path, "r", encoding="utf-8") as f:
                    self.state = json.load(f)
            except Exception:
                self.state = {}

        self.idx = 0
        if start_unreviewed:
            for i, p in enumerate(self.images):
                if not self.state.get(os.path.relpath(p, src), {}).get("reviewed"):
                    self.idx = i
                    break

        self.img = None
        self.disp = None
        self.scale = 1.0
        self.boxes = []          # [[cls, x1,y1,x2,y2], ...] 像素坐标
        self.sel = -1
        self.dragging = False
        self.drag_start = None
        self.drag_now = None
        self.cur_cls = 0
        self.dirty = False

    # ---------------- 文件读写 ----------------
    def txt_path(self, img_path):
        return os.path.splitext(img_path)[0] + ".txt"

    def load(self):
        p = self.images[self.idx]
        self.img = cv2.imread(p)
        if self.img is None:
            raise SystemExit(f"读不了图片: {p}")
        h, w = self.img.shape[:2]
        self.boxes = []
        tp = self.txt_path(p)
        if os.path.isfile(tp):
            with open(tp, "r", encoding="utf-8") as f:
                for line in f:
                    parts = line.split()
                    if len(parts) < 5:
                        continue
                    c = int(float(parts[0]))
                    cx, cy, bw, bh = (float(v) for v in parts[1:5])
                    x1 = (cx - bw / 2) * w
                    y1 = (cy - bh / 2) * h
                    x2 = (cx + bw / 2) * w
                    y2 = (cy + bh / 2) * h
                    self.classes[c] if c < len(self.classes) else "?"
                    self.boxes.append([c, x1, y1, x2, y2])
        self.sel = -1
        self.dirty = False

    def save(self):
        p = self.images[self.idx]
        h, w = self.img.shape[:2]
        lines = []
        for c, x1, y1, x2, y2 in self.boxes:
            x1, x2 = sorted((max(0.0, min(x1, w - 1)), max(0.0, min(x2, w - 1))))
            y1, y2 = sorted((max(0.0, min(y1, h - 1)), max(0.0, min(y2, h - 1))))
            bw, bh = x2 - x1, y2 - y1
            if bw < 2 or bh < 2:
                continue
            lines.append(f"{c} {(x1 + x2) / 2 / w:.6f} {(y1 + y2) / 2 / h:.6f} "
                         f"{bw / w:.6f} {bh / h:.6f}")
        with open(self.txt_path(p), "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + ("\n" if lines else ""))
        self.state[os.path.relpath(p, self.src)] = {"reviewed": True, "boxes": len(lines)}
        self.dirty = False

    def save_state(self):
        with open(self.state_path, "w", encoding="utf-8") as f:
            json.dump(self.state, f, ensure_ascii=False, indent=1)

    # ---------------- 显示 ----------------
    def render(self):
        h, w = self.img.shape[:2]
        self.scale = min(MAX_W / w, MAX_H / h, 1.0)
        dw, dh = int(w * self.scale), int(h * self.scale)
        self.disp = cv2.resize(self.img, (dw, dh), interpolation=cv2.INTER_AREA)

        for i, (c, x1, y1, x2, y2) in enumerate(self.boxes):
            col = SEL_COLOR if i == self.sel else COLORS[c % len(COLORS)]
            p1 = (int(x1 * self.scale), int(y1 * self.scale))
            p2 = (int(x2 * self.scale), int(y2 * self.scale))
            cv2.rectangle(self.disp, p1, p2, col, 3 if i == self.sel else 2)
            name = self.classes[c] if c < len(self.classes) else str(c)
            cv2.putText(self.disp, name, (p1[0] + 2, max(14, p1[1] - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2, cv2.LINE_AA)

        if self.dragging and self.drag_start and self.drag_now:
            cv2.rectangle(self.disp, self.drag_start, self.drag_now, (0, 255, 255), 2)

        # 顶部信息条
        cv2.rectangle(self.disp, (0, 0), (dw, 34), (0, 0, 0), -1)
        rel = os.path.relpath(self.images[self.idx], self.src)
        reviewed = sum(1 for v in self.state.values() if v.get("reviewed"))
        info = (f"[{self.idx + 1}/{len(self.images)}] {rel}   "
                f"boxes={len(self.boxes)}   cls={self.classes[self.cur_cls]}   "
                f"reviewed={reviewed}")
        cv2.putText(self.disp, info, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(self.disp,
                    "drag=box  right-click=del  <-/->=nav  c=clear  u=undo  1-4=class  q=quit",
                    (8, dh - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
        return self.disp

    # ---------------- 鼠标 ----------------
    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            hit = self.hit(x, y)
            if hit >= 0:
                self.sel = hit
            else:
                self.dragging = True
                self.drag_start = (x, y)
                self.drag_now = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging:
            self.drag_now = (x, y)
        elif event == cv2.EVENT_LBUTTONUP and self.dragging:
            self.dragging = False
            if self.drag_start and self.drag_now:
                x1, y1 = self.drag_start
                x2, y2 = self.drag_now
                if abs(x2 - x1) > 5 and abs(y2 - y1) > 5:
                    s = self.scale
                    self.boxes.append([self.cur_cls,
                                       min(x1, x2) / s, min(y1, y2) / s,
                                       max(x1, x2) / s, max(y1, y2) / s])
                    self.sel = len(self.boxes) - 1
                    self.dirty = True
            self.drag_start = self.drag_now = None
        elif event == cv2.EVENT_RBUTTONDOWN:
            hit = self.hit(x, y)
            if hit >= 0:
                self.boxes.pop(hit)
                self.sel = -1
                self.dirty = True

    def hit(self, x, y):
        """点击命中检测：从后往前找，让新画的框优先被选中。"""
        for i in range(len(self.boxes) - 1, -1, -1):
            _, x1, y1, x2, y2 = self.boxes[i]
            if (x1 * self.scale <= x <= x2 * self.scale
                    and y1 * self.scale <= y <= y2 * self.scale):
                return i
        return -1

    # ---------------- 主循环 ----------------
    def run(self):
        win = "label tool"
        cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(win, self.on_mouse)
        self.load()

        while True:
            cv2.imshow(win, self.render())

            # ⚠️ 关键：必须先判断 raw == -1（没按键）。
            #    cv2.waitKey 没按键时返回 -1，而 -1 & 0xFF == 255。
            #    如果直接把 255 当成某个键（比如左方向键），
            #    没人按键时就会每 20 ms 自动翻一张 —— 表现就是"翻得太快看不清"。
            raw = cv2.waitKey(20)
            if raw == -1:
                continue

            # Windows 下方向键的键值（Linux/X11 是 65361/65363，不通用）
            WIN_LEFT, WIN_RIGHT, WIN_UP, WIN_DOWN = 2424832, 2555904, 2490368, 2621440
            X11_LEFT, X11_RIGHT = 65361, 65363

            if raw in (WIN_LEFT, X11_LEFT):
                if self.dirty:
                    self.save()
                self.idx = (self.idx - 1) % len(self.images)
                self.save_state()
                self.load()
                continue
            if raw in (WIN_RIGHT, X11_RIGHT):
                if self.dirty:
                    self.save()
                self.idx = (self.idx + 1) % len(self.images)
                self.save_state()
                self.load()
                continue

            key = raw & 0xFF

            if key in (27, ord("q")):
                if self.dirty:
                    self.save()
                self.save_state()
                break

            nxt = prev = False
            if key == ord("a"):
                prev = True
            if key == ord("d"):
                nxt = True

            if nxt or prev:
                if self.dirty:
                    self.save()
                step = 1 if nxt else -1
                self.idx = (self.idx + step) % len(self.images)
                self.save_state()
                self.load()
                continue

            if key in (13, 10, 32, ord("s")):
                self.save()
                continue
            if key == ord("c"):
                self.boxes = []
                self.sel = -1
                self.dirty = True
                continue
            if key == ord("u"):
                if self.boxes:
                    self.boxes.pop()
                    self.sel = -1
                    self.dirty = True
                continue
            if key >= ord("1") and key <= ord("4"):
                c = key - ord("1")
                if c < len(self.classes):
                    self.cur_cls = c
                continue

        cv2.destroyAllWindows()
        reviewed = sum(1 for v in self.state.values() if v.get("reviewed"))
        print(f"\n已复核 {reviewed}/{len(self.images)} 张")
        print(f"状态: {self.state_path}")
        return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=os.path.join(ROOT, "dataset_raw"))
    ap.add_argument("--task", choices=["ball", "marker"], default="ball")
    ap.add_argument("--restart", action="store_true", help="从头开始，不跳到未复核的那张")
    ap.add_argument("--only-empty", action="store_true",
                    help="只复核空标注图（优先补这些，它们多半是漏检）")
    args = ap.parse_args()

    lab = Labeler(args.src, args.task,
                  start_unreviewed=not args.restart,
                  only_empty=args.only_empty)
    return lab.run()


if __name__ == "__main__":
    sys.exit(main())
