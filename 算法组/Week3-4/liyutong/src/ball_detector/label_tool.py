# -*- coding: utf-8 -*-
"""
极简标注工具（OpenCV 窗口，不依赖 Qt / PyQt5）

为什么要有这个：labelImg 在这台机器上被「智能应用控制」拦了 exe 和 sip.pyd，
绕过之后又在画框时 Qt 底层崩溃。OpenCV 的窗口在本项目里已验证可用。

两种模式
    足球模式（默认，单类、每张一个框）
        python label_tool.py video2
        python label_tool.py video3 --empty          # 只翻还没有框的
    标志模式（L / T / X 三类，每张可以有多个框）
        python label_tool.py L_3   --cls auto        # 按目录名前缀定类别
        python label_tool.py X_1   --cls X           # 强制指定类别

标志类别号（与 marker_dataset/data.yaml 一致）：L=0  T=1  X=2
足球类别号：0 = ball

操作
    鼠标左键拖拽      画一个框（标志模式是「追加」，足球模式是「替换」）
    鼠标右键点框内    删掉这个框（标志模式用）
    数字键 1 / 2 / 3  标志模式下切换「下一个框」的类别（1=L 2=T 3=X）
    D / → / 空格      下一张（自动保存）
    A / ←             上一张（自动保存）
    X / Delete        清空这张的所有框
    Q / Esc           退出（自动保存）
"""
import argparse
import os
import sys

import cv2
import numpy as np

RAW = "raw"
CLASS_ID = 0                     # 足球模式固定 0

# 标志类别：名字 -> 类别号（顺序就是 data.yaml 里 names 的顺序）
MARKER_CLASSES = {"L": 0, "T": 1, "X": 2}
MARKER_COLORS = {"L": (0, 165, 255), "T": (0, 200, 0), "X": (255, 0, 255)}
COLOR_BALL = (60, 60, 255)


# ---------------- 标注文件读写 ----------------

def read_boxes(txt_path, w, h):
    """读 YOLO txt，返回 [(cls, x1, y1, x2, y2), ...]（像素坐标）。没有则空列表。"""
    out = []
    if not os.path.isfile(txt_path) or os.path.getsize(txt_path) == 0:
        return out
    try:
        with open(txt_path, encoding="utf-8") as f:
            for line in f:
                p = line.split()
                if len(p) < 5:
                    continue
                c = int(float(p[0]))
                cx, cy, bw, bh = map(float, p[1:5])
                out.append((c,
                            int(round((cx - bw / 2) * w)), int(round((cy - bh / 2) * h)),
                            int(round((cx + bw / 2) * w)), int(round((cy + bh / 2) * h))))
    except Exception as e:
        print("读标注失败", txt_path, e)
    return out


def read_box(txt_path, w, h):
    """兼容旧接口：只返回第一个框的像素坐标，没有则 None"""
    b = read_boxes(txt_path, w, h)
    return b[0][1:] if b else None


def write_boxes(txt_path, boxes, w, h):
    """把框列表写成 YOLO txt。空列表时写空文件（训练脚本会跳过）。"""
    if not boxes:
        open(txt_path, "w").close()
        return
    lines = []
    for (c, x1, y1, x2, y2) in boxes:
        x1, x2 = sorted((max(0, min(x1, w - 1)), max(0, min(x2, w - 1))))
        y1, y2 = sorted((max(0, min(y1, h - 1)), max(0, min(y2, h - 1))))
        cx = (x1 + x2) / 2 / w
        cy = (y1 + y2) / 2 / h
        bw = max(1, x2 - x1) / w
        bh = max(1, y2 - y1) / h
        lines.append(f"{c} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")
    with open(txt_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def write_box(txt_path, box, w, h):
    """兼容旧接口：单个框（class 0）"""
    write_boxes(txt_path, [] if box is None else [(CLASS_ID,) + tuple(box)], w, h)


def list_empty(seg_dir):
    """返回还没有标注框的图片文件名（txt 不存在或为 0 字节）"""
    out = []
    for f in sorted(os.listdir(seg_dir)):
        if not f.lower().endswith(".jpg"):
            continue
        t = os.path.join(seg_dir, os.path.splitext(f)[0] + ".txt")
        if (not os.path.exists(t)) or os.path.getsize(t) == 0:
            out.append(f)
    return out


# ---------------- 主程序 ----------------

class Tool:
    def __init__(self, seg_dir, scale, files=None, cls_id=None, multi=False, cls_name="",
                 inherit=True, inherit_max_diff=15.0):
        self.scale = scale
        self.files = (list(files) if files is not None
                      else sorted(f for f in os.listdir(seg_dir) if f.lower().endswith(".jpg")))
        self.seg_dir = seg_dir
        self.idx = 0
        self.cls_id = CLASS_ID if cls_id is None else cls_id
        self.multi = multi                 # True: 一张图可以多个框（标志模式）
        self.cls_name = cls_name
        self.boxes = []                    # [(cls, x1,y1,x2,y2), ...]
        self.dirty = False
        self.cleared = False
        self.drag = None
        self.cur_pos = None
        self.inherit = inherit             # 同段连续帧继承上一张的框
        self.inherit_max_diff = inherit_max_diff
        self.inherited = False
        self.win = "label tool"

    # -- 坐标换算 --
    def to_img(self, x, y):
        return int(round(x / self.scale)), int(round(y / self.scale))

    def to_disp(self, x, y):
        return int(round(x * self.scale)), int(round(y * self.scale))

    def color_of(self, c):
        if self.multi:
            for name, cid in MARKER_CLASSES.items():
                if cid == c:
                    return MARKER_COLORS[name]
        return COLOR_BALL

    def load(self):
        name = self.files[self.idx]
        self.img = cv2.imread(os.path.join(self.seg_dir, name))
        if self.img is None:
            print("读图失败", name)
            return False
        self.name = name
        h, w = self.img.shape[:2]
        self.w, self.h = w, h
        self.boxes = read_boxes(os.path.join(self.seg_dir, os.path.splitext(name)[0] + ".txt"), w, h)
        self.dirty = False
        self.cleared = False
        self.inherited = False
        self.drag = None
        self.cur_pos = None
        if not self.boxes:
            self._inherit_prev()
        return True

    def _white_still_there(self, boxes):
        """框中心周围还有没有白线结构。

        比"整幅灰度差"可靠得多：有人从画面走过会让灰度差飙到 20+，
        但白线接头一动没动；反过来相机平移了就算灰度差很小，框也会落空。

        实测（T_2_001，17x17 邻域）：真接头 top>=70 占 4.2%、max 121；
        草地 0~0.3%、max<=70。所以用「强响应占比 >= 1% 或 max >= 90」。
        """
        gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        top = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, np.ones((15, 15), np.uint8))
        for (_c, x1, y1, x2, y2) in boxes:
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            r = 8
            patch = top[max(0, cy - r):cy + r + 1, max(0, cx - r):cx + r + 1]
            if patch.size == 0:
                return False
            strong = float((patch >= 70).mean())
            if strong < 0.01 and int(patch.max()) < 90:
                return False
        return True

    def _inherit_prev(self):
        """同段连续帧的框位置几乎不变，本帧还没标注时直接继承上一张。

        抽帧时同一段内的间隔只有零点几秒、三脚架没动，实测同一接头的位移
        通常只有几个像素，YOLO 完全能吃下；省掉重复拖框的时间。
        """
        if not self.inherit or self.idx == 0:
            return
        prev_name = self.files[self.idx - 1]
        pbox = read_boxes(os.path.join(self.seg_dir, os.path.splitext(prev_name)[0] + ".txt"),
                          self.w, self.h)
        if not pbox:
            return
        pimg = cv2.imread(os.path.join(self.seg_dir, prev_name))
        if pimg is None or pimg.shape != self.img.shape:
            return
        a = cv2.cvtColor(pimg, cv2.COLOR_BGR2GRAY).astype(np.int16)
        b = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY).astype(np.int16)
        if float(np.abs(a - b).mean()) > self.inherit_max_diff:
            return
        if not self._white_still_there(pbox):
            return
        self.boxes = list(pbox)
        self.inherited = True
        self.dirty = True          # 按 D 就落盘

    def save(self):
        """保存。保护：一个框都没有、又不是用户主动清空时，绝不覆盖磁盘上已有的标注。"""
        if not self.dirty:
            return
        if not self.boxes and not self.cleared:
            self.dirty = False
            return
        p = os.path.join(self.seg_dir, os.path.splitext(self.name)[0] + ".txt")
        write_boxes(p, self.boxes, self.w, self.h)
        if not self.boxes:
            print(f"  清空标注：{self.name}（这张会被训练跳过）")
        else:
            print(f"  保存：{self.name}  {len(self.boxes)} 个框")
        self.dirty = False

    def next(self, step=1):
        self.save()
        self.idx = (self.idx + step) % len(self.files)
        self.load()

    def render(self):
        disp = cv2.resize(self.img, None, fx=self.scale, fy=self.scale,
                          interpolation=cv2.INTER_LINEAR)
        for (c, x1, y1, x2, y2) in self.boxes:
            a = self.to_disp(x1, y1)
            b = self.to_disp(x2, y2)
            col = self.color_of(c)
            ov = disp.copy()
            cv2.rectangle(ov, a, b, col, -1)
            disp = cv2.addWeighted(ov, 0.10, disp, 0.90, 0)   # 极淡填充，不糊住目标
            cv2.rectangle(disp, a, b, col, 2)
            cv2.drawMarker(disp, a, (0, 220, 0), cv2.MARKER_SQUARE, 10, 2)
            cv2.drawMarker(disp, b, (0, 220, 0), cv2.MARKER_SQUARE, 10, 2)
            if self.multi:
                nm = next((k for k, v in MARKER_CLASSES.items() if v == c), str(c))
                cv2.putText(disp, nm, (a[0] + 3, max(14, a[1] - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2, cv2.LINE_AA)

        if self.drag and self.cur_pos:
            a = self.to_disp(*self.drag)
            b = self.to_disp(*self.cur_pos)
            cv2.rectangle(disp, a, b, (0, 255, 255), 1)

        # 顶部信息条（cv2.putText 不支持中文，全用英文，否则显示成 ????）
        bar = np.zeros((28, disp.shape[1], 3), np.uint8)
        if self.boxes:
            state = f"{len(self.boxes)} BOX"
            if not self.multi and len(self.boxes) == 1:
                _, x1, y1, x2, y2 = self.boxes[0]
                state += f"  {x2 - x1}x{y2 - y1}px"
            if self.inherited:
                state = "INHERITED " + state + "  (D=keep / X=wrong)"
        else:
            state = "CLEARED (skipped)" if self.cleared else "NO BOX -> drag to draw"
        if self.dirty:
            state += " *"
        if self.multi:
            cur = next((k for k, v in MARKER_CLASSES.items() if v == self.cls_id), "?")
            tag = f"[{cur}] "
        else:
            tag = f"[{self.cls_name}] " if self.cls_name else ""
        txt = f"[{self.idx + 1}/{len(self.files)}] {tag}{self.name}  {state}"
        cv2.putText(bar, txt, (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)

        hint = ("drag=add  R-click=del  1/2/3=class  D=next  A=prev  X=clear  Q=quit" if self.multi
                else "drag=draw  D=next  A=prev  X=clear  Q=quit")
        (tw, _), _ = cv2.getTextSize(hint, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.putText(bar, hint, (max(6, disp.shape[1] - tw - 8), 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
        return np.vstack([bar, disp])

    # -- 鼠标 --
    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drag = self.to_img(x, y)
            self.cur_pos = self.drag
        elif event == cv2.EVENT_MOUSEMOVE and self.drag is not None:
            self.cur_pos = self.to_img(x, y)
        elif event == cv2.EVENT_LBUTTONUP and self.drag is not None:
            p1, p2 = self.drag, self.to_img(x, y)
            self.drag = None
            self.cur_pos = None
            if abs(p2[0] - p1[0]) >= 6 and abs(p2[1] - p1[1]) >= 6:
                nb = (self.cls_id, min(p1[0], p2[0]), min(p1[1], p2[1]),
                      max(p1[0], p2[0]), max(p1[1], p2[1]))
                if self.multi:
                    self.boxes.append(nb)          # 标志模式：追加
                else:
                    self.boxes = [nb]              # 足球模式：替换
                self.cleared = False
                self.dirty = True
            else:
                print("  框太小，已忽略（要拖出至少 6x6 像素）")
        elif event == cv2.EVENT_RBUTTONUP and self.multi:
            px, py = self.to_img(x, y)
            for i, (c, x1, y1, x2, y2) in enumerate(self.boxes):
                if x1 <= px <= x2 and y1 <= py <= y2:
                    self.boxes.pop(i)
                    self.dirty = True
                    print(f"  删掉第 {i + 1} 个框")
                    break

    def run(self):
        if not self.files:
            print(f"{self.seg_dir} 里没有 jpg")
            return
        if not self.load():
            return
        cv2.namedWindow(self.win, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self.win, self.on_mouse)
        if self.multi:
            print(f"共 {len(self.files)} 张，类别 {self.cls_name}"
                  f"（可多个框）。拖拽加框 / 右键删框 / D 下一张 / A 上一张 / X 清空 / Q 退出")
        else:
            print(f"共 {len(self.files)} 张。拖鼠标画框 / D 下一张 / A 上一张 / X 清框 / Q 退出")
        while True:
            cv2.imshow(self.win, self.render())
            # 没按键时 waitKeyEx 返回 -1；早期版本用 waitKey()&0xFF 会把它变成 255，
            # 落在清框分支上，导致每 20ms 清一次框 → 按 D 就把空标注写盘。
            raw = cv2.waitKeyEx(20)
            if raw == -1:
                continue
            k = raw & 0xFF
            if k in (ord('q'), 27):
                self.save()
                break
            elif raw in (2555904, 83) or k in (ord('d'), 32):      # 右方向键 / d / 空格
                self.next(1)
            elif raw in (2424832, 81) or k == ord('a'):             # 左方向键 / a
                self.next(-1)
            elif raw in (3014656, 46) or k in (8, 127) or k == ord('x'):   # Delete/退格/x
                if self.boxes:
                    self.boxes = []
                    self.cleared = True
                    self.dirty = True
                    print(f"  已清空所有框（{self.name}），按 D 保存并继续")
            elif self.multi and k in (ord('1'), ord('2'), ord('3')):
                want = k - ord('0') - 1                     # 1=L  2=T  3=X
                for nm, cid in MARKER_CLASSES.items():
                    if cid == want:
                        self.cls_id = cid
                        print(f"  当前类别切成 {nm}")
                        break
        cv2.destroyAllWindows()
        print(f"已退出，最后停在第 {self.idx + 1} 张")


def check(seg_dir):
    """不开窗口，自检读写是否一致"""
    files = sorted(f for f in os.listdir(seg_dir) if f.lower().endswith(".jpg"))
    print(f"{seg_dir}: {len(files)} 张")
    if not files:
        return
    n_box = n_empty = 0
    tally = {}
    for f in files:
        txt = os.path.join(seg_dir, os.path.splitext(f)[0] + ".txt")
        if os.path.exists(txt):
            cls_list = [int(float(l.split()[0])) for l in open(txt, encoding="utf-8")
                        if len(l.split()) >= 5]
        else:
            cls_list = []
        if cls_list:
            n_box += 1
            for c in cls_list:
                tally[c] = tally.get(c, 0) + 1
        else:
            n_empty += 1
    for f in files[:5] + files[-3:] if len(files) > 8 else files:
        txt = os.path.join(seg_dir, os.path.splitext(f)[0] + ".txt")
        n = 0
        if os.path.exists(txt):
            n = sum(1 for line in open(txt, encoding="utf-8") if len(line.split()) >= 5)
        print(f"  {f}: {n} 个框" if n else f"  {f}: 无框")
    print(f"合计 {len(files)} 张：有框 {n_box}，无框 {n_empty}")
    if tally:
        inv = {v: k for k, v in MARKER_CLASSES.items()}
        print("各类别框数：", "  ".join(
            f"{inv.get(c, 'ball' if c == 0 else str(c))}={n}" for c, n in sorted(tally.items())))


def resolve_cls(seg_name, arg):
    """--cls auto 时按目录名前缀判断类别：L_3 -> L，T_1 -> T，X_2 -> X"""
    key = seg_name.split("_")[0].upper()
    if arg and arg.lower() != "auto":
        key = arg.upper()
    if key in MARKER_CLASSES:
        return MARKER_CLASSES[key], key
    if not arg:
        return None, ""
    if arg.lower() == "auto":
        sys.exit(f"段名 {seg_name} 不是 L_*/T_*/X_* 开头，猜不出类别。\n"
                 f"请显式指定：--cls L 或 --cls T 或 --cls X")
    sys.exit(f"不认识的类别 {arg}，只支持 {'/'.join(MARKER_CLASSES)}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("seg", nargs="?", default=None, help="段名，如 video2 / L_3")
    ap.add_argument("--scale", type=float, default=1.5)
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--empty", action="store_true", help="只标注还没有框的图片")
    ap.add_argument("--raw", default=RAW, help="图片根目录（默认 raw，标志数据用 raw_mark）")
    ap.add_argument("--cls", default=None,
                    help="标志模式：auto=按段名前缀，或直接写 L/T/X。不传=足球模式")
    ap.add_argument("--no-inherit", dest="inherit", action="store_false",
                    help="关闭「同段连续帧继承上一张的框」（默认开启）")
    ap.add_argument("--inherit-diff", dest="inherit_diff", type=float, default=15.0,
                    help="继承判据：与上一帧的灰度平均差上限，超过就不继承（默认 15.0）")
    args = ap.parse_args()

    root = args.raw
    if args.seg is None and args.check:
        # --check 不指定段时，遍历全部段汇总，不进交互输入
        segs = sorted(d for d in os.listdir(root) if os.path.isdir(os.path.join(root, d)))
        if not segs:
            print(f"{root}/ 下没有段目录")
            return
        tot_img = tot_box = 0
        for s in segs:
            d = os.path.join(root, s)
            n = len([f for f in os.listdir(d) if f.lower().endswith(".jpg")])
            nb = 0
            for f in os.listdir(d):
                if not f.lower().endswith(".txt"):
                    continue
                t = os.path.join(d, f)
                if os.path.getsize(t) == 0:
                    continue
                nb += sum(1 for line in open(t, encoding="utf-8") if len(line.split()) >= 5)
            tot_img += n
            tot_box += nb
            mark = "" if nb else "   <<< 一个框都没有"
            print(f"{s:<10} {n:4d} 张   {nb:4d} 个框{mark}")
        print("-" * 40)
        print(f"合计 {len(segs)} 段  {tot_img} 张  {tot_box} 个框")
        return

    if args.seg is None:
        segs = sorted(d for d in os.listdir(root) if os.path.isdir(os.path.join(root, d)))
        print(f"{root}/ 下可标注的段：")
        for s in segs:
            n = len([f for f in os.listdir(os.path.join(root, s)) if f.endswith(".jpg")])
            print(f"  {s:<12} {n:3d} 张")
        args.seg = input("\n输入段名（回车 = 第一段）：").strip() or segs[0]

    seg_dir = os.path.join(root, args.seg)
    if not os.path.isdir(seg_dir):
        print("没有这个段：", seg_dir)
        return

    if args.check:
        check(seg_dir)
        return

    cls_id, cls_name = (None, "")
    if args.cls:
        cls_id, cls_name = resolve_cls(args.seg, args.cls)

    files = None
    if args.empty:
        files = list_empty(seg_dir)
        if not files:
            print(f"{args.seg} 里没有空标注，全部都有框了")
            return
        print(f"{args.seg} 还有 {len(files)} 张没框：")
        for f in files:
            print("   ", f)

    Tool(seg_dir, args.scale, files, cls_id=cls_id,
         multi=args.cls is not None, cls_name=cls_name,
         inherit=args.inherit, inherit_max_diff=args.inherit_diff).run()


if __name__ == "__main__":
    main()
