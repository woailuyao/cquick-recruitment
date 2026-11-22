# -*- coding: utf-8 -*-
"""标注器：直接读写 YOLO txt，不依赖 Qt。

两种模式：
  drag   —— 原来的拖拽画框，适合足球等任意目标；
  click  —— 点击式半自动，适合场地标志：在交点上点一下，程序立刻
            按「极坐标分支数」判出 L / T / X 并定好框大小。

为什么标志要用 click 模式：
    纯几何自动出框在本场地误检比真检还多（人造草坪高光 + 远端白墙残块），
    但「人给位置、几何给类别与大小」这个组合几乎不出错。

操作（click 模式）：
    左键点击交点     自动出框并分类
    1 / 2 / 3 / 4    强制指定类别 L / T / X / 点球点（再点即用该类）
    0                取消强制，回到自动判类
    Backspace        撤掉最后一个框
    右键             删除点中的框
    D / 空格 / →     下一张（自动保存）
    A / ←            上一张（自动保存）
    C                清空本张
    S                立即保存
    Q / ESC          退出

用法（项目根目录）：
    .\\run.bat tools\\annotate.py --dir dataset/raw_marker --mode click \
        --classes LCross,TCross,XCross,PenaltyPoint
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.paths import rel          # noqa: E402

try:
    import tools.prelabel_marker as PM
except Exception:                                                  # noqa: BLE001
    PM = None

try:
    cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_SILENT)
except Exception:                                                  # noqa: BLE001
    pass

WIN = "annotate"
MIN_BOX_PX = 6          # 拖拽小于此视为误触
MARKER_CLASSES = ["LCross", "TCross", "XCross", "PenaltyPoint"]
PALETTE = [(60, 200, 255), (90, 240, 90), (255, 150, 60), (200, 120, 255),
           (255, 255, 120), (200, 200, 200)]


# ---------------------------------------------------------------- 纯逻辑（可测）
def load_boxes(txt: Path) -> list[list[float]]:
    """读取 YOLO txt，返回 [[cls, cx, cy, w, h], ...]（归一化）。"""
    out = []
    if not txt.exists():
        return out
    for line in txt.read_text(encoding="utf-8").splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        try:
            cls = int(float(parts[0]))
            cx, cy, w, h = (float(v) for v in parts[1:5])
        except ValueError:
            continue
        out.append([cls, cx, cy, w, h])
    return out


def save_boxes(txt: Path, boxes: list[list[float]]) -> None:
    lines = [f"{int(b[0])} {b[1]:.6f} {b[2]:.6f} {b[3]:.6f} {b[4]:.6f}" for b in boxes]
    txt.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")


def xyxy_to_yolo(x1, y1, x2, y2, w_img, h_img, cls: int = 0) -> list[float]:
    x1, x2 = sorted((x1, x2))
    y1, y2 = sorted((y1, y2))
    return [cls,
            ((x1 + x2) / 2) / w_img,
            ((y1 + y2) / 2) / h_img,
            abs(x2 - x1) / w_img,
            abs(y2 - y1) / h_img]


def yolo_to_xyxy(box, w_img, h_img):
    _, cx, cy, bw, bh = box
    x1 = (cx - bw / 2) * w_img
    y1 = (cy - bh / 2) * h_img
    x2 = (cx + bw / 2) * w_img
    y2 = (cy + bh / 2) * h_img
    return x1, y1, x2, y2


def hit_test(boxes, x, y, w_img, h_img) -> int:
    """返回被点中的框索引（从后往前，优先上层），未命中返回 -1。"""
    for i in range(len(boxes) - 1, -1, -1):
        x1, y1, x2, y2 = yolo_to_xyxy(boxes[i], w_img, h_img)
        if x1 <= x <= x2 and y1 <= y <= y2:
            return i
    return -1


def simple_white_mask(bgr: np.ndarray) -> np.ndarray:
    """只按亮度/饱和度取白，不做「去厚块」。

    点击模式下位置由人给，不需要把墙面白块剔干净；反过来，近景标志的线条本身就有
    30–60 px 宽，用开运算去「厚块」会把真正要标的线一起抹掉（实测 marker_s02 的
    掩码只剩 0.1%）。
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    v, s = hsv[..., 2], hsv[..., 1]
    m = ((v > 165) & (s < 75)).astype(np.uint8) * 255
    return cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))


def build_context(bgr: np.ndarray):
    """预计算白线掩码等，供点击时快速判类。"""
    mask = simple_white_mask(bgr)
    dt = cv2.distanceTransform(mask, cv2.DIST_L2, 3)
    vals = dt[dt > 0]
    width = float(np.clip(np.percentile(vals, 50) * 2.0, 5.0, 40.0)) if vals.size else 12.0
    r_max = int(np.clip(width * 6.0, 30, 120))
    return dict(mask=mask, dt=dt, width=width, r_max=r_max)


def snap_to_line(ctx, x: float, y: float, radius: int = 16):
    """把点击点吸附到最近的白线中心（窗口内距离变换取最大处）。

    远处标线只有几个像素宽，人很难点得那么准；吸附之后就不要求毫厘不差。
    """
    if not ctx:
        return x, y
    dt = ctx["dt"]
    h, w = dt.shape[:2]
    x0, x1 = int(max(0, x - radius)), int(min(w - 1, x + radius))
    y0, y1 = int(max(0, y - radius)), int(min(h - 1, y + radius))
    if x1 <= x0 or y1 <= y0:
        return x, y
    win = dt[y0:y1 + 1, x0:x1 + 1]
    if win.size == 0 or float(win.max()) <= 0.0:
        return x, y
    iy, ix = np.unravel_index(int(np.argmax(win)), win.shape)
    return float(x0 + ix), float(y0 + iy)


def classify_at(ctx, x: float, y: float, min_half=24.0, max_half=110.0):
    """在 (x,y) 判类别并给出框半边长；判不出时返回 (None, 默认半边长)。

    注意：场地角在图像里受透视影响，未必是 90°，因此 L 的角度窗口放宽到 25°–155°。
    点击模式位置由人给，不会误报，放宽窗口只赚不亏。

    框大小按线宽定（约 4 倍线宽），而不是按分支长度——分支可能一直延伸半屏，
    那样框会大得没法训练。
    """
    if not ctx:
        return None, 60.0
    sx, sy = snap_to_line(ctx, x, y)
    # 用点击处的局部线宽，而不是全图统计值：近处标线宽、远处窄，全局值会被大块白色带偏。
    dt_v = float(ctx["dt"][int(sy), int(sx)])
    w_local = float(np.clip(dt_v * 2.0, 5.0, 40.0)) if dt_v > 0 else ctx["width"]
    cls, _arm_len = PM.classify_by_branches(ctx["mask"], ctx["dt"], sx, sy,
                                            w_local, ctx["r_max"],
                                            l_angle=(25.0, 155.0), min_span=6.0)
    half = float(np.clip(w_local * 3.0, 20.0, 110.0))
    return cls, half


# ---------------------------------------------------------------- 主程序
def main() -> int:
    ap = argparse.ArgumentParser(description="YOLO 标注器（拖拽 / 点击半自动）")
    ap.add_argument("--dir", default="dataset/pre")
    ap.add_argument("--start", type=int, default=0, help="从第几张开始（0 基）")
    ap.add_argument("--max-width", type=int, default=1100)
    ap.add_argument("--mode", choices=["drag", "click"], default="drag")
    ap.add_argument("--classes", default="ball", help="逗号分隔的类别名，按 class_id 顺序")
    ap.add_argument("--no-recurse", action="store_true", help="只读顶层，不进子目录")
    args = ap.parse_args()

    classes = [c.strip() for c in args.classes.split(",") if c.strip()]
    auto_cls = (args.mode == "click" and classes == MARKER_CLASSES)

    d = Path(args.dir)
    if not d.is_absolute():
        d = ROOT / d
    if not d.exists():
        print(f"目录不存在：{rel(d)}")
        return 1

    imgs = sorted(d.glob("*.png")) if args.no_recurse else sorted(d.rglob("*.png"))
    imgs += sorted(d.glob("*.jpg")) if args.no_recurse else sorted(d.rglob("*.jpg"))
    imgs = sorted(set(imgs))
    if not imgs:
        print(f"{rel(d)} 里没有图片")
        return 1

    idx = max(0, min(args.start, len(imgs) - 1))
    state = {"boxes": [], "drag": None, "dirty": False, "force": None, "last": None}
    disp = {"scale": 1.0}
    cache = {"name": None, "ctx": None}

    def current_ctx(img):
        name = str(imgs[idx])
        if cache["name"] != name:
            cache["ctx"] = build_context(img) if args.mode == "click" else None
            cache["name"] = name
        return cache["ctx"]

    def refresh() -> np.ndarray:
        img = cv2.imread(str(imgs[idx]))
        if img is None:
            img = np.zeros((720, 1280, 3), np.uint8)
        h, w = img.shape[:2]
        scale = min(1.0, args.max_width / w)
        view = cv2.resize(img, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
        disp["scale"] = scale

        for b in state["boxes"]:
            x1, y1, x2, y2 = yolo_to_xyxy(b, w, h)
            p1 = (int(x1 * scale), int(y1 * scale))
            p2 = (int(x2 * scale), int(y2 * scale))
            cid = int(b[0])
            col = PALETTE[cid % len(PALETTE)]
            cv2.rectangle(view, p1, p2, col, 2)
            label = classes[cid] if 0 <= cid < len(classes) else str(cid)
            cv2.putText(view, label, (p1[0] + 3, max(p1[1] - 5, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)

        if state["drag"]:
            x0, y0, x1, y1 = state["drag"]
            cv2.rectangle(view, (x0, y0), (x1, y1), (0, 200, 255), 1)

        head = f"[{idx + 1}/{len(imgs)}] {imgs[idx].parent.name}/{imgs[idx].name}  boxes={len(state['boxes'])}"
        if args.mode == "click":
            if state["force"] is None:
                head += "   next:AUTO"
            else:
                head += f"   next:FORCED={classes[state['force']]}"
            lt = state["last"]
            if lt:
                an = classes[lt["auto"]] if lt["auto"] is not None else "判不出"
                head += f"   last:auto={an}->{classes[lt['used']]}"
        cv2.putText(view, head, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)
        if args.mode == "click":
            hint = ("click=junction | 1/2/3/4=force next one | R=cycle last | BS=undo | "
                    "R-click=del | D/A=next/prev | C=clear | Q=quit")
        else:
            hint = "drag=L draw | R=delete | D/A=next/prev | C=clear | S=save | Q=quit"
        cv2.putText(view, hint, (10, view.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (210, 210, 210), 1)
        return view

    def save_current() -> None:
        save_boxes(imgs[idx].with_suffix(".txt"), state["boxes"])
        state["dirty"] = False

    def go(step: int) -> None:
        nonlocal idx
        if state["dirty"]:
            save_current()
        idx = max(0, min(idx + step, len(imgs) - 1))
        state["boxes"] = load_boxes(imgs[idx].with_suffix(".txt"))
        state["drag"] = None
        state["force"] = None
        state["last"] = None
        cv2.imshow(WIN, refresh())

    def add_click_box(ix, iy, w_img, h_img) -> None:
        img = cv2.imread(str(imgs[idx]))
        ctx = current_ctx(img)
        sx, sy = snap_to_line(ctx, ix, iy)
        cls, half = classify_at(ctx, sx, sy)
        auto = cls
        forced = state["force"] is not None
        if forced:
            cls = state["force"]
        elif cls is None:
            cls = 0                     # 判不出时先按 LCross 记，标题栏会提示「判不出」
        state["boxes"].append(xyxy_to_yolo(sx - half, sy - half, sx + half, sy + half,
                                           w_img, h_img, cls))
        state["dirty"] = True
        state["last"] = dict(auto=auto, used=cls, half=half, forced=forced)
        state["force"] = None           # 强制只管这一次点击，立即复位，免得类别一路粘下去

    def on_mouse(event, x, y, flags, param):
        w_img = int(param[0])
        h_img = int(param[1])
        s = disp["scale"]
        ix, iy = x / s, y / s
        if event == cv2.EVENT_LBUTTONDOWN:
            state["drag"] = (x, y, x, y)
        elif event == cv2.EVENT_MOUSEMOVE and state["drag"] is not None:
            x0, y0, _, _ = state["drag"]
            state["drag"] = (x0, y0, x, y)
            cv2.imshow(WIN, refresh())
        elif event == cv2.EVENT_LBUTTONUP and state["drag"] is not None:
            x0, y0, _, _ = state["drag"]
            state["drag"] = None
            moved = abs(x - x0) >= MIN_BOX_PX or abs(y - y0) >= MIN_BOX_PX
            if args.mode == "click" and not moved:
                add_click_box(ix, iy, w_img, h_img)
            elif moved:
                # 拖拽画框也要能选类别：先看有没有强制类别，否则用框中心的几何判类。
                cls = state["force"]
                auto = None
                if cls is None:
                    img = cv2.imread(str(imgs[idx]))
                    ctx = current_ctx(img)
                    auto, _ = classify_at(ctx, (x0 / s + ix) / 2, (y0 / s + iy) / 2)
                    cls = auto if auto is not None else 0
                state["boxes"].append(xyxy_to_yolo(x0 / s, y0 / s, x / s, y / s,
                                                   w_img, h_img, cls))
                state["dirty"] = True
                state["last"] = dict(auto=auto, used=cls, half=0.0,
                                     forced=state["force"] is not None)
                state["force"] = None      # 与点击一致：强制只管这一次
            cv2.imshow(WIN, refresh())
        elif event == cv2.EVENT_RBUTTONDOWN:
            i = hit_test(state["boxes"], ix, iy, w_img, h_img)
            if i >= 0:
                state["boxes"].pop(i)
                state["dirty"] = True
                cv2.imshow(WIN, refresh())

    cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE)
    first_img = cv2.imread(str(imgs[idx]))
    if first_img is None:
        print(f"读不了：{rel(imgs[idx])}")
        return 1
    h_img, w_img = first_img.shape[:2]
    state["boxes"] = load_boxes(imgs[idx].with_suffix(".txt"))
    cv2.setMouseCallback(WIN, on_mouse, (w_img, h_img))
    cv2.imshow(WIN, refresh())

    print(f"共 {len(imgs)} 张，从第 {idx + 1} 张开始｜模式 {args.mode}｜类别 {classes}")
    if args.mode == "click" and not auto_cls:
        print("提示：--classes 不是四类标志，自动判类已关闭，只按点击出框。")
    print("窗口内操作，Q 退出。")
    while True:
        key = cv2.waitKey(20) & 0xFF
        if key in (ord("q"), 27):
            if state["dirty"]:
                save_current()
            break
        elif key in (ord("d"), ord(" "), 83, 3):
            go(1)
        elif key in (ord("a"), 81, 2):
            go(-1)
        elif key == ord("c"):
            state["boxes"] = []
            state["dirty"] = True
            cv2.imshow(WIN, refresh())
        elif key == ord("s"):
            save_current()
            cv2.imshow(WIN, refresh())
        elif key == 8:                                  # Backspace 撤销
            if state["boxes"]:
                state["boxes"].pop()
                state["dirty"] = True
                cv2.imshow(WIN, refresh())
        elif key == ord("r") and args.mode == "click":   # 轮换最后一个框的类别
            if state["boxes"]:
                n = max(len(classes), 1)
                state["boxes"][-1][0] = (int(state["boxes"][-1][0]) + 1) % n
                state["dirty"] = True
                cv2.imshow(WIN, refresh())
        elif args.mode == "click" and ord("1") <= key <= ord("4"):
            k = key - ord("1")
            state["force"] = k if k < len(classes) else None
            cv2.imshow(WIN, refresh())
        elif args.mode == "click" and key == ord("0"):
            state["force"] = None
            cv2.imshow(WIN, refresh())

    cv2.destroyAllWindows()
    print(f"已退出。当前第 {idx + 1} 张：{rel(imgs[idx])}")
    print(f"下次接着标：--start {idx}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
