"""
足球包围框标注工具（替代 labelImg）
用法：python label_tool.py train      （train / val / test 三选一）
操作：
  鼠标左键拖拽 -> 画一个框
  鼠标右键点击 -> 删除该框
  s -> 保存并到下一张
  d -> 下一张（自动保存）
  a -> 上一张（自动保存）
  u -> 撤销当前这张最后画的框
  c -> 清空当前这张的框
  q 或 Esc -> 退出
"""
import os
import sys
import glob
import cv2

ROOT = r"D:\W34_dataset\ball"
# ===== 在这里选要标注的划分：train / val / test =====
# 直接改下面这个字符串即可；也可以在 PyCharm 运行配置的 Parameters 里传同名参数覆盖
SPLIT = "test"
split = sys.argv[1] if len(sys.argv) > 1 else SPLIT
IMG_DIR = os.path.join(ROOT, "images", split)
LBL_DIR = os.path.join(ROOT, "labels", split)

images = sorted(glob.glob(os.path.join(IMG_DIR, "*.jpg")))
if not images:
    print("没有找到图片:", IMG_DIR)
    sys.exit()

idx = 0
boxes = []
drawing = False
p1 = p2 = None
W = H = 0
WIN = "Ball Label Tool - drag:box  rightclick:delete  s:save+next  d:next  a:prev  u:undo  c:clear  q:quit"


def lbl_path(i):
    stem = os.path.splitext(os.path.basename(images[i]))[0]
    return os.path.join(LBL_DIR, stem + ".txt")


def load_boxes(i):
    res = []
    p = lbl_path(i)
    if not os.path.exists(p):
        return res
    with open(p, "r", encoding="utf-8") as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) != 5:
                continue
            cx, cy, bw, bh = [float(v) for v in parts[1:]]
            res.append([(cx - bw / 2) * W, (cy - bh / 2) * H,
                        (cx + bw / 2) * W, (cy + bh / 2) * H])
    return res


def save_boxes(i):
    p = lbl_path(i)
    with open(p, "w", encoding="utf-8") as f:
        for b in boxes:
            cx = (b[0] + b[2]) / 2.0 / W
            cy = (b[1] + b[3]) / 2.0 / H
            bw = (b[2] - b[0]) / W
            bh = (b[3] - b[1]) / H
            f.write(f"0 {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n")


def mouse_cb(event, x, y, flags, param):
    global drawing, p1, p2, boxes
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        p1 = (x, y)
        p2 = (x, y)
    elif event == cv2.EVENT_MOUSEMOVE and drawing:
        p2 = (x, y)
    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        if p1 is None:
            return
        x1, y1 = min(p1[0], x), min(p1[1], y)
        x2, y2 = max(p1[0], x), max(p1[1], y)
        if x2 - x1 > 4 and y2 - y1 > 4:
            boxes.append([x1, y1, x2, y2])
        p2 = None
    elif event == cv2.EVENT_RBUTTONDOWN:
        for i in range(len(boxes) - 1, -1, -1):
            b = boxes[i]
            if b[0] <= x <= b[2] and b[1] <= y <= b[3]:
                boxes.pop(i)
                break


def show(i):
    global boxes, W, H
    img = cv2.imread(images[i])
    if img is None:
        print("读不出图片:", images[i])
        return False
    H, W = img.shape[:2]
    boxes = load_boxes(i)
    cv2.imshow(WIN, img)
    return True


cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE)
cv2.setMouseCallback(WIN, mouse_cb)
cv2.moveWindow(WIN, 80, 40)

if not show(idx):
    sys.exit()

print("标注窗口已打开，标题：", WIN)
print("如果没看到窗口，按 Alt+Tab 切换，或在任务栏找 OpenCV 窗口")

try:
    while True:
        img = cv2.imread(images[idx])
        if img is None:
            break
        disp = img.copy()
        for b in boxes:
            cv2.rectangle(disp, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 255, 0), 2)
            cv2.putText(disp, "ball", (int(b[0]), max(int(b[1]) - 6, 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        if drawing and p1 and p2:
            cv2.rectangle(disp, p1, p2, (0, 220, 255), 2)
        bar = f"[{idx + 1}/{len(images)}]  boxes={len(boxes)}  {os.path.basename(images[idx])}"
        cv2.rectangle(disp, (0, 0), (disp.shape[1], 34), (0, 0, 0), -1)
        cv2.putText(disp, bar, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1)
        cv2.imshow(WIN, disp)
        key = cv2.waitKey(30) & 0xFF

        if key == ord("q") or key == 27:
            save_boxes(idx)
            break
        elif key == ord("s"):
            save_boxes(idx)
            if idx < len(images) - 1:
                idx += 1
                show(idx)
        elif key == ord("d"):
            save_boxes(idx)
            if idx < len(images) - 1:
                idx += 1
                show(idx)
        elif key == ord("a"):
            save_boxes(idx)
            if idx > 0:
                idx -= 1
                show(idx)
        elif key == ord("u"):
            if boxes:
                boxes.pop()
        elif key == ord("c"):
            boxes = []
except KeyboardInterrupt:
    save_boxes(idx)
    print("被中断，当前张已保存")
finally:
    cv2.destroyAllWindows()
print("已退出，标注保存在:", LBL_DIR)
