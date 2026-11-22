#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""零安装 YOLO 标注器（只用现有的 opencv + numpy，不下载、不安装、不改系统）。

为什么有这个工具：X-AnyLabeling 是第三方 .exe 安装包，会写注册表 / 开始菜单，
且默认存 .json 需要额外导出。本项目只需要画矩形框 -> YOLO txt，
用现成的 OpenCV 窗口即可完成，全程只读写 week3 目录内的 .txt 文件。

用法（在 .venv 里）：
  # 任务一：标球（单类 ball）
  python tools/annotate_cv2.py --dir frames_pool

  # 任务二：标标志（四类，顺序必须与 config/marker.yaml 一致）
  python tools/annotate_cv2.py --dir frames_pool_markers --classes L_cross,T_cross,X_cross,penalty_point

  # 复核模式：只过「有框的图」+「缺标签的图」，跳过 89 张已确认空图
  # 用于补标漏掉的三类 / 改错类别，不用重画已有框
  python tools/annotate_cv2.py --dir frames_pool_markers --classes L_cross,T_cross,X_cross,penalty_point --only-labeled

  # 逻辑自检（不弹窗口）
  python tools/annotate_cv2.py --selftest

快捷键（界面为英文，OpenCV 自带字体不支持中文）：
  n / d / Space  保存并下一张        p / a  保存并上一张
  u / z          撤销最后一个框      c      清空当前图所有框
  0              确认"此图无目标"（写空 txt，并自动下一张）
  Tab / v        循环选中已有框（白框高亮）
  x              取消选中
  1..9           设置当前类别；若有选中框，顺手把该框也改成这个类
  f              开关放大镜（找远处小球用）
  s              立即保存            q / Esc 保存并退出
  鼠标左键拖拽   画框                鼠标右键点框内  删除该框

设计约定：
  * 框内部一律存归一化 (cls, cx, cy, w, h)，与图片尺寸无关，换图不会错位。
  * 只有「画了框」或「按 0 确认过」的图才写 txt；没动过的图不写，
    这样 check_labels.py 能把"漏标"和"空场负样本"区分开。
  * empty_field_* 前缀的图自动视为已确认负样本并生成空 txt（可用
    --no-auto-empty 关掉），等价于 make_empty_labels.py。
  * 断点续标：启动自动跳到第一张还没有 txt 的图；--start N 可强制起点。
"""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}
WIN = "week3-annotator"
PALETTE = [
    (0, 255, 255), (255, 140, 0), (0, 200, 255), (180, 80, 255),
    (120, 255, 120), (0, 120, 255), (255, 0, 180), (200, 200, 0),
]


# ---------------------------------------------------------------- I/O
def imread_any(path: Path):
    """cv2.imread 在 Windows 中文路径会失败，用 imdecode 兜底。"""
    data = np.fromfile(str(path), dtype=np.uint8)
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def load_yolo(txt: Path):
    """返回 (boxes, confirmed_empty)。boxes = [(cls, cx, cy, w, h)] 归一化。"""
    if not txt.exists():
        return [], False
    lines = [ln.strip() for ln in txt.read_text(encoding="utf-8").splitlines()
             if ln.strip()]
    if not lines:
        return [], True                      # 空文件 = 已确认无目标
    boxes = []
    for ln in lines:
        p = ln.split()
        if len(p) < 5:
            continue
        try:
            c = int(float(p[0]))
            cx, cy, w, h = float(p[1]), float(p[2]), float(p[3]), float(p[4])
        except ValueError:
            continue
        boxes.append((c, cx, cy, w, h))
    return boxes, False


def save_yolo(txt: Path, boxes, confirmed_empty: bool) -> bool:
    """写 YOLO txt。返回是否真的写了文件。

    boxes 为空且未确认 -> 不写（保留"未标注"状态，让自检能报缺失）。
    """
    if not boxes and not confirmed_empty:
        return False
    if not boxes:
        txt.write_text("", encoding="utf-8")
        return True
    lines = []
    for c, cx, cy, w, h in boxes:
        x1, y1 = max(0.0, cx - w / 2), max(0.0, cy - h / 2)
        x2, y2 = min(1.0, cx + w / 2), min(1.0, cy + h / 2)
        cx, cy, w, h = (x1 + x2) / 2, (y1 + y2) / 2, x2 - x1, y2 - y1
        if w <= 0 or h <= 0:
            continue
        lines.append(f"{c} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")
    txt.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return True


# ---------------------------------------------------------------- GUI
class Annotator:
    def __init__(self, args):
        self.dir = (ROOT / args.dir).resolve()
        self.labels_dir = (ROOT / args.labels).resolve() if args.labels else self.dir
        self.labels_dir.mkdir(parents=True, exist_ok=True)
        self.classes = [s.strip() for s in args.classes.split(",") if s.strip()]
        self.max_w, self.max_h = args.disp_w, args.disp_h
        self.auto_empty = not args.no_auto_empty

        if not self.dir.is_dir():
            raise SystemExit(f"目录不存在：{self.dir}")
        self.paths = sorted(p for p in self.dir.iterdir()
                            if p.suffix.lower() in IMG_EXTS)
        if getattr(args, "only_labeled", False):
            # 复核模式：只保留「已有非空框」或「还没有 txt」的图，
            # 跳过已确认的空负样本，避免把 89 张空图再翻一遍。
            def need_review(p):
                txt = self.labels_dir / (p.stem + ".txt")
                if not txt.exists():
                    return True
                boxes, _ = load_yolo(txt)
                return bool(boxes)
            self.paths = [p for p in self.paths if need_review(p)]
        if not self.paths:
            raise SystemExit(f"目录里没有图片：{self.dir}")

        self.idx = 0
        if args.start is not None:
            self.idx = min(max(0, args.start), len(self.paths) - 1)
        else:
            for i, p in enumerate(self.paths):
                if not (self.labels_dir / (p.stem + ".txt")).exists():
                    self.idx = i
                    break

        self.cur_cls = 0
        self.sel = None                       # 当前选中的框索引（Tab 循环）
        self.zoom = False
        self.mouse = None
        self.drag_start = None
        self.drag_cur = None
        self.boxes = []
        self.confirmed = False
        self.dirty = False
        self.orig = None
        self.dw = self.dh = 1

    def _label_path(self):
        return self.labels_dir / (self.paths[self.idx].stem + ".txt")

    # ---------- 状态 ----------
    def load_current(self):
        p = self.paths[self.idx]
        img = imread_any(p)
        if img is None:
            raise SystemExit(f"读不了图片：{p}")
        self.orig = img
        h, w = img.shape[:2]
        s = min(1.0, self.max_w / w, self.max_h / h)
        self.dw, self.dh = max(1, int(round(w * s))), max(1, int(round(h * s)))

        self.boxes, self.confirmed = load_yolo(self._label_path())
        self.dirty = False
        self.drag_start = self.drag_cur = None
        self.sel = None

        # 空场负样本自动生成空 txt（不覆盖已有的非空标注）
        if (self.auto_empty and p.stem.startswith("empty_field")
                and not self.boxes and not self._label_path().exists()):
            save_yolo(self._label_path(), [], True)
            self.confirmed = True

    def save_current(self):
        if self.dirty or self.confirmed:
            save_yolo(self._label_path(), self.boxes,
                      self.confirmed and not self.boxes)
            self.dirty = False

    def goto(self, delta):
        self.save_current()
        self.idx = min(max(0, self.idx + delta), len(self.paths) - 1)
        self.load_current()

    # ---------- 鼠标 ----------
    def on_mouse(self, event, x, y, flags, param):
        self.mouse = (x, y)
        if y > self.dh:                      # 底部状态栏区域不响应
            return
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drag_start, self.drag_cur = (x, y), (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self.drag_start is not None:
            self.drag_cur = (x, y)
        elif event == cv2.EVENT_LBUTTONUP and self.drag_start is not None:
            x1, y1 = self.drag_start
            x2, y2 = x, y
            x1, x2 = sorted((x1, x2))
            y1, y2 = sorted((y1, y2))
            if x2 - x1 >= 3 and y2 - y1 >= 3:
                self.boxes.append((
                    self.cur_cls,
                    (x1 + x2) / 2 / self.dw,
                    (y1 + y2) / 2 / self.dh,
                    (x2 - x1) / self.dw,
                    (y2 - y1) / self.dh,
                ))
                self.confirmed = False
                self.dirty = True
            self.drag_start = self.drag_cur = None
        elif event == cv2.EVENT_RBUTTONDOWN:
            nx, ny = x / self.dw, y / self.dh
            for i in range(len(self.boxes) - 1, -1, -1):
                _, cx, cy, w, h = self.boxes[i]
                if abs(nx - cx) <= w / 2 and abs(ny - cy) <= h / 2:
                    self.boxes.pop(i)
                    self.sel = None
                    self.dirty = True
                    break

    # ---------- 渲染 ----------
    def render(self):
        img = cv2.resize(self.orig, (self.dw, self.dh),
                         interpolation=cv2.INTER_AREA)
        for i, (c, cx, cy, w, h) in enumerate(self.boxes):
            x1 = int((cx - w / 2) * self.dw)
            y1 = int((cy - h / 2) * self.dh)
            x2 = int((cx + w / 2) * self.dw)
            y2 = int((cy + h / 2) * self.dh)
            col = PALETTE[c % len(PALETTE)]
            is_sel = (i == self.sel)
            cv2.rectangle(img, (x1, y1), (x2, y2),
                          (255, 255, 255) if is_sel else col,
                          3 if is_sel else 2)
            name = self.classes[c] if c < len(self.classes) else str(c)
            cv2.putText(img, ("SEL " if is_sel else "") + name,
                        (x1, max(12, y1 - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (255, 255, 255) if is_sel else col, 1, cv2.LINE_AA)
        if self.drag_start and self.drag_cur:
            cv2.rectangle(img, self.drag_start, self.drag_cur,
                          (255, 255, 255), 1)

        # 放大镜：以鼠标为中心取 80x80 放大到 240x240，贴右上角
        if self.zoom and self.mouse:
            mx, my = self.mouse
            r = 40
            px1, py1 = max(0, mx - r), max(0, my - r)
            px2, py2 = min(self.dw, mx + r), min(self.dh, my + r)
            patch = img[py1:py2, px1:px2]
            if patch.size:
                z = cv2.resize(patch, (240, 240),
                               interpolation=cv2.INTER_NEAREST)
                cv2.rectangle(z, (0, 0), (239, 239), (255, 255, 255), 1)
                img[0:240, self.dw - 240:self.dw] = z

        # 底部状态栏（纯英文，避免 OpenCV 字体中文乱码）
        bar = np.full((48, self.dw, 3), 30, np.uint8)
        p = self.paths[self.idx]
        cls_name = self.classes[self.cur_cls] if self.cur_cls < len(self.classes) else "?"
        state = "EMPTY-CONFIRMED" if (self.confirmed and not self.boxes) else (
            "UNLABELED" if not self.boxes else "")
        line1 = (f"[{self.idx + 1}/{len(self.paths)}] {p.name}  boxes={len(self.boxes)}  "
                 f"cls={self.cur_cls}:{cls_name}  {state}  "
                 f"{'SEL#' + str(self.sel) if self.sel is not None else ''}  "
                 f"{'ZOOM' if self.zoom else ''}")
        line2 = ("[n]ext [p]rev [u]ndo [c]lear [0]none [Tab/v]sel-box [x]unsel "
                 "[1-9]class(=relabel if SEL) [f]zoom [s]ave [q]uit | L-drag=box R-click=del")
        cv2.putText(bar, line1, (8, 19), cv2.FONT_HERSHEY_SIMPLEX,
                    0.48, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(bar, line2, (8, 39), cv2.FONT_HERSHEY_SIMPLEX,
                    0.44, (170, 200, 170), 1, cv2.LINE_AA)
        return np.vstack([img, bar])

    # ---------- 主循环 ----------
    def run(self):
        cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(WIN, self.on_mouse)
        self.load_current()
        print(f"共 {len(self.paths)} 张，从第 {self.idx + 1} 张开始；"
              f"类别 {self.classes}；q 保存退出")
        try:
            while True:
                cv2.imshow(WIN, self.render())
                k = cv2.waitKey(30) & 0xFF
                if k in (ord("q"), 27):
                    self.save_current()
                    break
                elif k in (ord("n"), ord("d"), 32):
                    self.goto(1)
                elif k in (ord("p"), ord("a")):
                    self.goto(-1)
                elif k in (ord("u"), ord("z")):
                    if self.boxes:
                        self.boxes.pop()
                        self.sel = None
                        self.confirmed = False
                        self.dirty = True
                elif k == ord("c"):
                    self.boxes.clear()
                    self.sel = None
                    self.confirmed = False
                    self.dirty = True
                elif k == ord("0"):
                    self.boxes.clear()
                    self.sel = None
                    self.confirmed = True
                    self.dirty = True
                    self.goto(1)
                elif k in (9, ord("v")):              # Tab / v 循环选中已有框
                    if self.boxes:
                        if self.sel is None:
                            self.sel = 0
                        else:
                            self.sel = (self.sel + 1) % len(self.boxes)
                elif k == ord("x"):                   # 取消选中
                    self.sel = None
                elif ord("1") <= k <= ord("9"):
                    new_cls = min(k - ord("1"), len(self.classes) - 1)
                    self.cur_cls = new_cls
                    if self.sel is not None and self.sel < len(self.boxes):
                        # 有选中框 => 改这个框的类别（复核改错类别时用）
                        c, cx, cy, w, h = self.boxes[self.sel]
                        self.boxes[self.sel] = (new_cls, cx, cy, w, h)
                        self.dirty = True
                elif k == ord("f"):
                    self.zoom = not self.zoom
                elif k == ord("s"):
                    self.save_current()
                    print(f"  已保存 {self._label_path().name}")
        finally:
            cv2.destroyAllWindows()
        done = sum(1 for p in self.paths
                   if (self.labels_dir / (p.stem + ".txt")).exists())
        print(f"完成：本轮列表 {done}/{len(self.paths)} 张已有 txt")
        # 全目录口径的四类统计，方便随时确认三类是否补齐
        all_imgs = sorted(p for p in self.dir.iterdir()
                          if p.suffix.lower() in IMG_EXTS)
        cls_cnt = [0] * len(self.classes)
        empty_cnt = 0
        for p in all_imgs:
            boxes, confirmed = load_yolo(self.labels_dir / (p.stem + ".txt"))
            for c in (b[0] for b in boxes):
                if 0 <= c < len(cls_cnt):
                    cls_cnt[c] += 1
            if not boxes and confirmed:
                empty_cnt += 1
        print("全目录四类框数：" + ", ".join(
            f"{n}={cls_cnt[i]}" for i, n in enumerate(self.classes))
            + f" | 已确认空图 {empty_cnt} 张")


# ---------------------------------------------------------------- 自检
def selftest():
    import tempfile
    d = Path(tempfile.mkdtemp(prefix="week3_annotate_"))

    a = d / "a.txt"
    assert save_yolo(a, [], False) is False and not a.exists(), "未标注不该写文件"
    boxes = [(0, 0.5, 0.4, 0.1, 0.2)]
    assert save_yolo(a, boxes, False) is True
    got, emp = load_yolo(a)
    assert len(got) == 1 and not emp
    assert max(abs(x - y) for x, y in zip(got[0], boxes[0])) < 1e-5, "读写往返不一致"

    b = d / "b.txt"
    b.write_text("", encoding="utf-8")
    got2, emp2 = load_yolo(b)
    assert got2 == [] and emp2 is True, "空 txt 应识别为已确认负样本"

    c = d / "c.txt"
    save_yolo(c, [(1, 0.98, 0.5, 0.20, 0.20), (0, 0.02, 0.02, 0.10, 0.10)], False)
    got3, _ = load_yolo(c)
    assert len(got3) == 2
    for _, cx, cy, w, h in got3:
        assert 0 <= cx <= 1 and 0 <= cy <= 1 and 0 <= w <= 1 and 0 <= h <= 1
    assert got3[0][0] == 1, "类别号应保留"

    print("[OK] selftest 通过：未标注不写 / 读写往返 / 空 txt 识别 / 越界裁剪 / 多类别")
    print(f"     临时目录 {d}")
    return 0


def main():
    ap = argparse.ArgumentParser(description="零安装 YOLO 标注器（OpenCV）")
    ap.add_argument("--dir", default="frames_pool", help="图片目录")
    ap.add_argument("--labels", default=None, help="标签目录，默认与图片同目录")
    ap.add_argument("--classes", default="ball", help="类别名，逗号分隔，顺序=YOLO编号")
    ap.add_argument("--disp-w", type=int, default=1180, help="显示最大宽度")
    ap.add_argument("--disp-h", type=int, default=620, help="显示最大高度")
    ap.add_argument("--start", type=int, default=None, help="从第 N 张开始（0 基）")
    ap.add_argument("--no-auto-empty", action="store_true",
                    help="关闭 empty_field_* 自动生成空 txt")
    ap.add_argument("--only-labeled", action="store_true",
                    help="复核模式：只过有框或还没标 txt 的图，跳过已确认空负样本")
    ap.add_argument("--selftest", action="store_true", help="只跑逻辑自检，不弹窗口")
    args = ap.parse_args()

    if args.selftest:
        return selftest()
    Annotator(args).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
