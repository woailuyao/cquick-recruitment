"""
任务 2 场地标志 · 多类标注工具（label_tool_markers）
=====================================================
三类标志（编号与 markers_data.yaml 一致）：
  0 LCross  L 型交点/角点   绿色框
  1 TCross  T 型交点        蓝色框
  2 XCross  X 型交点        红色框

用法（Python 3.10，在 PyCharm 里 Run，或命令行）：
    python label_tool_markers.py train     # train / val / test 三选一

操作：
  鼠标左键拖拽      画一个框（类别 = 顶部当前选中的类别）
  右键点击框        删除该框
  数字键 1 / 2 / 3  切换当前类别（对应 LCross / TCross / XCross）
  s                 保存当前张并到下一张
  d                 下一张（自动保存当前张）
  a                 上一张（自动保存当前张）
  u                 撤销当前张最后画的框
  c                 清空当前张所有框
  q 或 Esc          保存并退出

输出：与图片同名的 .txt，YOLO 归一化格式
      每行：类别号 中心x 中心y 框宽 框高（0~1 归一化）
  空帧（没有任何标志）不生成 txt 文件。
"""
import os
import sys
import glob
import cv2

# ===== 数据集根目录 =====
ROOT = r"D:\W34_dataset\markers"
# 命令行传划分；不传默认 train
split = sys.argv[1] if len(sys.argv) > 1 else "train"
if split not in ("train", "val", "test"):
    print("划分只能是 train / val / test，当前:", split)
    sys.exit(1)

IMG_DIR = os.path.join(ROOT, "images", split)
LBL_DIR = os.path.join(ROOT, "labels", split)
os.makedirs(LBL_DIR, exist_ok=True)

# 类别名与 BGR 颜色（顺序必须和 markers_data.yaml 的 names 一致）
CLASSES = ["LCross", "TCross", "XCross"]
COLORS = [(0, 200, 0), (255, 128, 0), (0, 0, 255)]  # 绿 / 蓝 / 红

images = sorted(glob.glob(os.path.join(IMG_DIR, "*.jpg")))
if not images:
    print("没有找到图片:", IMG_DIR)
    sys.exit()

idx = 0
boxes = []          # 当前张框：[x1, y1, x2, y2, cls]
current_cls = 0     # 当前选中类别编号
drawing = False
p1 = p2 = None
W = H = 0
WIN = "Marker Label  1/2/3:class  drag:box  rightclick:del  s:next  a:prev  u:undo  c:clear  q:quit"


def lbl_path(i):
    """第 i 张图对应的标签 txt 路径（同名换后缀）。"""
    stem = os.path.splitext(os.path.basename(images[i]))[0]
    return os.path.join(LBL_DIR, stem + ".txt")


def load_boxes(i):
    """读取已有标签，还原成像素框；没有标签就返回空列表。"""
    res = []
    p = lbl_path(i)
    if not os.path.exists(p):
        return res
    with open(p, "r", encoding="utf-8") as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) != 5:
                continue
            cls = int(parts[0])
            cx, cy, bw, bh = [float(v) for v in parts[1:]]
            x1 = (cx - bw / 2) * W
            y1 = (cy - bh / 2) * H
            x2 = (cx + bw / 2) * W
            y2 = (cy + bh / 2) * H
            res.append([x1, y1, x2, y2, cls])
    return res


def save_boxes(i):
    """把当前张框写成 YOLO 归一化 txt；空帧则删除旧 txt（保持空帧无标签）。"""
    p = lbl_path(i)
    if not boxes:
        if os.path.exists(p):
            os.remove(p)
        return
    with open(p, "w", encoding="utf-8") as f:
        for x1, y1, x2, y2, cls in boxes:
            cx = (x1 + x2) / 2.0 / W
            cy = (y1 + y2) / 2.0 / H
            bw = (x2 - x1) / W
            bh = (y2 - y1) / H
            f.write(f"{cls} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n")


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
        # 框太小（小于 5 像素）视为误触，不保存
        if x2 - x1 > 5 and y2 - y1 > 5:
            boxes.append([x1, y1, x2, y2, current_cls])
        p2 = None
    elif event == cv2.EVENT_RBUTTONDOWN:
        # 从后往前找，删掉右键点中的第一个框
        for k in range(len(boxes) - 1, -1, -1):
            b = boxes[k]
            if b[0] <= x <= b[2] and b[1] <= y <= b[3]:
                boxes.pop(k)
                break


def show(i):
    """读取第 i 张图并载入已有框，返回是否成功。"""
    global boxes, W, H
    img = cv2.imread(images[i])
    if img is None:
        print("读不出图片:", images[i])
        return False
    H, W = img.shape[:2]
    boxes = load_boxes(i)
    return True


cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE)
cv2.setMouseCallback(WIN, mouse_cb)
cv2.moveWindow(WIN, 60, 30)

if not show(idx):
    sys.exit()

print("标注窗口已打开；若没看到，按 Alt+Tab 找 OpenCV 窗口。")
print(f"当前划分: {split}，共 {len(images)} 张；当前类别: {CLASSES[current_cls]}")

try:
    while True:
        img = cv2.imread(images[idx])
        if img is None:
            break
        disp = img.copy()

        # 画已有框：按类别取颜色，标注类别名
        for x1, y1, x2, y2, cls in boxes:
            color = COLORS[cls]
            cv2.rectangle(disp, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            cv2.putText(disp, CLASSES[cls], (int(x1), max(int(y1) - 6, 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

        # 正在拖拽的临时框，用当前类别颜色
        if drawing and p1 and p2:
            cv2.rectangle(disp, p1, p2, COLORS[current_cls], 2)

        # 顶部状态栏：进度 + 当前类别 + 各类已标数量
        n_by_cls = [sum(1 for b in boxes if b[4] == c) for c in range(3)]
        bar = (f"[{idx + 1}/{len(images)}]  current: {CLASSES[current_cls]}  "
               f"L={n_by_cls[0]} T={n_by_cls[1]} X={n_by_cls[2]}  {os.path.basename(images[idx])}")
        cv2.rectangle(disp, (0, 0), (disp.shape[1], 34), (0, 0, 0), -1)
        cv2.putText(disp, bar, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.imshow(WIN, disp)

        key = cv2.waitKey(30) & 0xFF

        if key == ord("q") or key == 27:
            save_boxes(idx)
            break
        elif key == ord("1"):
            current_cls = 0
        elif key == ord("2"):
            current_cls = 1
        elif key == ord("3"):
            current_cls = 2
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

print("已退出，标签保存在:", LBL_DIR)
