# -*- coding: utf-8 -*-
"""一键标点：把 run/frames 里的图逐张弹出来，鼠标点交点即可。

用法：
    双击 点选标志.bat            （推荐）
    或 python pick_all.py
    或 python pick_all.py --force   （已有点过也重新点）

操作：
    左键点击 = 在当前类型下加一个交点（自动做局部霍夫细化）
    1 / 2 / 3 = 切换当前类型 L / T / X
    u        = 撤销上一个点
    s        = 保存本张并进入下一张
    q        = 跳过本张（不保存）
    ESC      = 全部中止

显示：
    大圆 + 十字线 = 细化后的交点
    灰色小十字   = 你点击的原始位置
    右下角放大镜 = 交点附近 2.5 倍放大，方便判断有没有对在交叉处
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent))
from pick_marker import refine_intersection  # noqa: E402
from imgio import imread, project_root  # noqa: E402

BASE = project_root(__file__)
FRAMES = BASE / "run" / "frames"
OUT = BASE / "clicks"
TYPE_KEYS = {"1": "L", "2": "T", "3": "X"}
TYPE_COLOR = {"L": (0, 0, 255), "T": (255, 0, 255), "X": (0, 255, 255)}


def natural_key(p: Path):
    digits = "".join(ch for ch in p.stem if ch.isdigit())
    return int(digits) if digits else 0


def zoom_inset(img, x, y, half=70, size=240):
    """放大镜：把交点附近 half 像素的区域放大到 size×size。"""
    h, w = img.shape[:2]
    x0, x1 = int(max(0, x - half)), int(min(w, x + half))
    y0, y1 = int(max(0, y - half)), int(min(h, y + half))
    if x1 - x0 < 8 or y1 - y0 < 8:
        return None
    patch = img[y0:y1, x0:x1]
    patch = cv2.resize(patch, (size, size), interpolation=cv2.INTER_NEAREST)
    cv2.line(patch, (size // 2, 0), (size // 2, size - 1), (0, 255, 0), 1)
    cv2.line(patch, (0, size // 2), (size - 1, size // 2), (0, 255, 0), 1)
    cv2.rectangle(patch, (0, 0), (size - 1, size - 1), (0, 255, 0), 2)
    return patch


def click_one(img_path: Path, out_json: Path) -> bool:
    """返回 False 表示用户要求中止。"""
    img = imread(img_path)
    if img is None:
        print(f"  [ERROR] 读不到 {img_path}")
        return True
    h, w = img.shape[:2]
    R = max(18, int(w * 0.022))          # 主圈半径，1280 宽时约 28 px

    state = {"type": "L", "points": [], "abort": False}
    win = f"{img_path.name}   1=L 2=T 3=X   s=save  q=skip  u=undo  ESC=abort"

    def redraw():
        disp = img.copy()
        last = None
        for i, p in enumerate(state["points"], 1):
            x, y = int(round(p["u"])), int(round(p["v"]))
            col = TYPE_COLOR.get(p["type"], (0, 0, 255))
            if "raw" in p:
                rx, ry = int(p["raw"][0]), int(p["raw"][1])
                cv2.line(disp, (rx - 10, ry), (rx + 10, ry), (160, 160, 160), 1)
                cv2.line(disp, (rx, ry - 10), (rx, ry + 10), (160, 160, 160), 1)
            cv2.circle(disp, (x, y), R, col, 3)
            cv2.line(disp, (x - R - 16, y), (x + R + 16, y), col, 1)
            cv2.line(disp, (x, y - R - 16), (x, y + R + 16), col, 1)
            cv2.putText(disp, f"{i}{p['type']}", (x + R + 6, y - R - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, col, 2)
            last = p

        cv2.putText(disp, f"current: {state['type']}    points: {len(state['points'])}",
                    (16, 42), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.putText(disp, "click junction | 1/2/3 type | s save&next | q skip | u undo",
                    (16, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)

        if last is not None:
            inset = zoom_inset(disp, last["u"], last["v"])
            if inset is not None:
                disp[h - 252:h - 12, w - 252:w - 12] = inset
                cv2.putText(disp, "zoom x2", (w - 250, h - 260),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        cv2.imshow(win, disp)

    def on_mouse(event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        xi, yi, how = refine_intersection(img, x, y)
        state["points"].append({"type": state["type"], "u": xi, "v": yi,
                                "how": how, "raw": (x, y)})
        print(f"    + {state['type']}  click=({x},{y})  refined=({xi:.1f},{yi:.1f})  [{how}]")
        redraw()

    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1280, 720)
    cv2.setMouseCallback(win, on_mouse)
    redraw()

    while True:
        k = cv2.waitKey(30) & 0xFF
        if k in (ord(c) for c in TYPE_KEYS):
            state["type"] = TYPE_KEYS[chr(k)]
            redraw()
        elif k == ord("u") and state["points"]:
            state["points"].pop()
            redraw()
        elif k == ord("s"):
            break
        elif k == ord("q"):
            state["points"] = []
            break
        elif k == 27:
            state["abort"] = True
            break

    cv2.destroyWindow(win)

    if state["abort"]:
        return False
    if state["points"]:
        out_json.parent.mkdir(parents=True, exist_ok=True)
        with open(out_json, "w", encoding="utf-8") as f:
            json.dump({"image": str(img_path), "points": state["points"]}, f,
                      ensure_ascii=False, indent=2)
        print(f"    [SAVE] {out_json.name}  共 {len(state['points'])} 个点")
    else:
        print("    [SKIP] 未保存")
    return True


def main():
    force = "--force" in sys.argv
    OUT.mkdir(parents=True, exist_ok=True)
    files = sorted(FRAMES.glob("*.png"), key=natural_key)
    if not files:
        print(f"[ERROR] 找不到图片：{FRAMES}")
        return

    todo = [p for p in files if force or not (OUT / (p.stem + ".json")).exists()]
    print(f"共 {len(files)} 张，待点 {len(todo)} 张")
    print(f"输出目录：{OUT}\n")

    for i, p in enumerate(todo, 1):
        print(f"[{i}/{len(todo)}] {p.name}")
        if not click_one(p, OUT / (p.stem + ".json")):
            print("已中止")
            return

    print("\n全部完成，结果在：", OUT)
    print(f"已有 {len(list(OUT.glob('*.json')))} 个 json")


if __name__ == "__main__":
    main()
