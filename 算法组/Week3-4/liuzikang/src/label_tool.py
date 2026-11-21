#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""极简 YOLO 标注工具（多类别版，只依赖 OpenCV，不会像 labelImg 那样因 Qt 闪退）。"""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

WIN_W, WIN_H = 1280, 760
PALETTE = [(0, 255, 0), (255, 128, 0), (0, 128, 255), (255, 0, 255),
           (0, 255, 255), (128, 0, 255)]
COLOR_NEW = (0, 200, 255)


class Labeler:
    def __init__(self, images: list[Path], label_dir: Path, classes: list[str]):
        self.images = images
        self.label_dir = label_dir
        self.classes = classes
        self.cur_cls = 0
        self.i = 0
        self.zoom = 1.0
        self.cx = None
        self.cy = None
        self.drag_start = None
        self.drag_now = None
        self.panning = False
        self.pan_from = None
        self.pan_origin = None
        self.boxes: list[tuple[float, float, float, float, int]] = []
        self.cur = None

    def _mats(self, shape):
        h, w = shape[:2]
        s = self.zoom
        if self.cx is None:
            self.cx, self.cy = w / 2, h / 2
        x0 = self.cx - WIN_W / (2 * s)
        y0 = self.cy - WIN_H / (2 * s)
        M = np.array([[s, 0, -x0 * s], [0, s, -y0 * s]], np.float32)
        Minv = np.array([[1 / s, 0, x0], [0, 1 / s, y0]], np.float32)
        return M, Minv

    def _to_img(self, pt, Minv):
        p = Minv @ np.array([pt[0], pt[1], 1.0], np.float32)
        return float(p[0]), float(p[1])

    def label_path(self, img_path: Path) -> Path:
        return self.label_dir / (img_path.stem + ".txt")

    def load_boxes(self, img_path: Path, shape):
        h, w = shape[:2]
        self.boxes = []
        p = self.label_path(img_path)
        if not p.exists():
            return
        for line in p.read_text(encoding="utf-8").splitlines():
            parts = line.split()
            if len(parts) < 5:
                continue
            cid = int(float(parts[0]))
            cx, cy, bw, bh = map(float, parts[1:5])
            self.boxes.append(((cx - bw / 2) * w, (cy - bh / 2) * h,
                               (cx + bw / 2) * w, (cy + bh / 2) * h, cid))

    def save(self, img_path: Path, shape) -> None:
        h, w = shape[:2]
        self.label_dir.mkdir(parents=True, exist_ok=True)
        lines = []
        for x1, y1, x2, y2, cid in self.boxes:
            x1, x2 = sorted((max(0.0, x1), min(w - 1.0, x2)))
            y1, y2 = sorted((max(0.0, y1), min(h - 1.0, y2)))
            if x2 - x1 < 2 or y2 - y1 < 2:
                continue
            cx, cy = (x1 + x2) / 2 / w, (y1 + y2) / 2 / h
            bw, bh = (x2 - x1) / w, (y2 - y1) / h
            lines.append(f"{cid} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")
        self.label_path(img_path).write_text("\n".join(lines) + ("\n" if lines else ""),
                                             encoding="utf-8")

    def _group_starts(self) -> list[int]:
        starts, last = [], None
        for idx, p in enumerate(self.images):
            g = p.stem.rsplit("_", 1)[0]
            if g != last:
                starts.append(idx)
                last = g
        return starts

    def _jump(self, direction: int, img: np.ndarray) -> None:
        starts = self._group_starts()
        if direction > 0:
            nxt = [s for s in starts if s > self.i]
            target = nxt[0] if nxt else len(self.images) - 1
        else:
            prv = [s for s in starts if s < self.i]
            target = prv[-1] if prv else 0
        self.save(self.images[self.i], img.shape)
        self.i = target
        self.cx = self.cy = None
        self.zoom = 1.0

    def _color(self, cid: int):
        return PALETTE[cid % len(PALETTE)]

    def render(self, img):
        M, _ = self._mats(img.shape)
        view = cv2.warpAffine(img, M, (WIN_W, WIN_H), flags=cv2.INTER_LINEAR,
                              borderValue=(45, 45, 45))

        def draw(box, color, cid=None):
            p1 = M @ np.array([box[0], box[1], 1.0], np.float32)
            p2 = M @ np.array([box[2], box[3], 1.0], np.float32)
            cv2.rectangle(view, (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), color, 2)
            if cid is not None and cid < len(self.classes):
                cv2.putText(view, self.classes[cid], (int(p1[0]), max(14, int(p1[1]) - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        for b in self.boxes:
            draw(b[:4], self._color(b[4]), b[4])
        if self.drag_start and self.drag_now:
            draw((*self.drag_start, *self.drag_now), COLOR_NEW)

        bar = np.zeros((78, WIN_W, 3), np.uint8)
        name = self.images[self.i].name
        cv2.putText(bar, f"[{self.i + 1}/{len(self.images)}] {name}  boxes={len(self.boxes)}",
                    (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(bar, f"CURRENT: {self.classes[self.cur_cls]}", (10, 52),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, self._color(self.cur_cls), 2, cv2.LINE_AA)
        cls_txt = "  ".join(f"{k + 1}={c}" for k, c in enumerate(self.classes))
        cv2.putText(bar, cls_txt, (330, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (170, 190, 170), 1, cv2.LINE_AA)
        cv2.putText(bar, "drag=box  wheel=zoom  1/2/3=class  D=next  A=prev  N/B=video  C=clear  U=undo  Q=quit",
                    (10, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (150, 200, 150), 1, cv2.LINE_AA)
        return np.vstack([bar, view])

    def run(self) -> None:
        cv2.namedWindow("labeler", cv2.WINDOW_AUTOSIZE)

        def on_mouse(event, x, y, flags, param):
            _, Minv = self._mats(self.cur.shape)
            vy = y - 78
            if event == cv2.EVENT_LBUTTONDOWN and vy >= 0:
                self.drag_start = self._to_img((x, vy), Minv)
                self.drag_now = self.drag_start
            elif event == cv2.EVENT_MOUSEMOVE:
                if self.drag_start is not None and vy >= 0:
                    self.drag_now = self._to_img((x, vy), Minv)
                elif self.panning and self.pan_origin is not None:
                    dx = (x - self.pan_from[0]) / self.zoom
                    dy = (vy - self.pan_from[1]) / self.zoom
                    self.cx = self.pan_origin[0] - dx
                    self.cy = self.pan_origin[1] - dy
            elif event == cv2.EVENT_LBUTTONUP and vy >= 0:
                if self.drag_start and self.drag_now:
                    x1, y1 = self.drag_start
                    x2, y2 = self.drag_now
                    if abs(x2 - x1) > 3 and abs(y2 - y1) > 3:
                        self.boxes.append((x1, y1, x2, y2, self.cur_cls))
                self.drag_start = self.drag_now = None
            elif event == cv2.EVENT_RBUTTONDOWN:
                self.panning = True
                self.pan_from = (x, vy)
                self.pan_origin = (self.cx, self.cy)
            elif event == cv2.EVENT_RBUTTONUP:
                self.panning = False
            elif event == cv2.EVENT_MOUSEWHEEL:
                img_pt = self._to_img((x, max(0, vy)), Minv)
                self.zoom = float(np.clip(self.zoom * (1.15 if flags > 0 else 1 / 1.15), 0.3, 8.0))
                self.cx, self.cy = img_pt
                M2, _ = self._mats(self.cur.shape)
                p = M2 @ np.array([img_pt[0], img_pt[1], 1.0], np.float32)
                self.cx += (p[0] - x) / self.zoom
                self.cy += (p[1] - vy) / self.zoom

        cv2.setMouseCallback("labeler", on_mouse)

        while 0 <= self.i < len(self.images):
            path = self.images[self.i]
            data = np.fromfile(str(path), dtype=np.uint8)
            img = cv2.imdecode(data, cv2.IMREAD_COLOR)
            if img is None:
                print(f"[跳过] 读不到 {path.name}")
                self.i += 1
                continue
            self.cur = img
            self.load_boxes(path, img.shape)
            if self.cx is None:
                self.cx, self.cy = img.shape[1] / 2, img.shape[0] / 2

            while True:
                cv2.imshow("labeler", self.render(img))
                key = cv2.waitKey(20) & 0xFF

                if key in (ord("d"), 83, 3):
                    self.save(path, img.shape)
                    self.i += 1
                    self.cx = self.cy = None
                    self.zoom = 1.0
                    break
                elif key in (ord("a"), 81, 2):
                    self.save(path, img.shape)
                    self.i = max(0, self.i - 1)
                    self.cx = self.cy = None
                    self.zoom = 1.0
                    break
                elif key == ord("n"):
                    self._jump(+1, img)
                    break
                elif key == ord("b"):
                    self._jump(-1, img)
                    break
                elif key == ord("c"):
                    self.boxes = []
                elif key == ord("u"):
                    if self.boxes:
                        self.boxes.pop()
                elif key == ord("s"):
                    self.save(path, img.shape)
                    print(f"[保存] {path.name}  {len(self.boxes)} 个框")
                elif key == ord("r"):
                    self.zoom = 1.0
                    self.cx, self.cy = img.shape[1] / 2, img.shape[0] / 2
                elif ord("1") <= key <= ord("9"):
                    idx = key - ord("1")
                    if idx < len(self.classes):
                        self.cur_cls = idx
                elif key in (ord("q"), 27):
                    self.save(path, img.shape)
                    cv2.destroyAllWindows()
                    return

        cv2.destroyAllWindows()
        print(f"完成，共处理 {len(self.images)} 张")


def main() -> int:
    ap = argparse.ArgumentParser(description="极简 YOLO 标注工具（支持多类别）")
    ap.add_argument("--images", default="dataset/images/all")
    ap.add_argument("--labels", default="dataset/labels/all")
    ap.add_argument("--classes", default="ball", help="类别名，逗号分隔")
    ap.add_argument("--prefix", default="", help="只标文件名以它开头的图")
    ap.add_argument("--skip-done", action="store_true", help="跳过已经有 txt 的图")
    args = ap.parse_args()

    img_dir = Path(args.images)
    if not img_dir.is_dir():
        print(f"图片目录不存在：{img_dir.resolve()}")
        return 1
    classes = [c.strip() for c in args.classes.split(",") if c.strip()]
    images = sorted(img_dir.glob("*.jpg")) + sorted(img_dir.glob("*.png"))
    if args.prefix:
        images = [p for p in images if p.name.startswith(args.prefix)]
    if not images:
        print("目录里没有符合条件的图片")
        return 1

    label_dir = Path(args.labels)
    done = sum(1 for p in images if (label_dir / (p.stem + ".txt")).exists())
    if args.skip_done:
        images = [p for p in images if not (label_dir / (p.stem + ".txt")).exists()]
        if not images:
            print("这一批已经全部标完了")
            return 0

    print(f"共 {len(images)} 张，已标注 {done} 张，类别：{classes}")
    print("操作：1/2/3=切换类别  左键拖=画框  滚轮=缩放  右键拖=平移  "
          "D=下一张  A=上一张  C=清空  U=撤销  Q=退出")
    Labeler(images, label_dir, classes).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
