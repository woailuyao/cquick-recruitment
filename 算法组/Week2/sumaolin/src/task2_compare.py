import cv2 as cv
from pathlib import Path
import numpy as np
import sys
import os
from datetime import datetime
SHOW = os.environ.get("W2_SHOW", "1") != "0"
NAME = "sumaolin"
ROOT = Path(__file__).parent.parent

def imwrite(path, img):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, buf = cv.imencode(path.suffix, img)
    if not ok:
        raise ValueError(f"编码失败：{path}")
    buf.tofile(str(path))
    print("修改图已保存到:", path)
    test = cv.imdecode(np.fromfile(str(path), np.uint8), cv.IMREAD_COLOR)
    if test.size == 0:
        sys.exit("修改图读取异常")
    print("修改图可读取")

def band(img, tag, msg):
    # 底部加一条自证信息，等价于终端 echo "姓名: ..."; date "+%Y-%m-%d %H:%M:%S"
    h, w = img.shape[:2]
    out = np.zeros((h+100, w, 3), np.uint8)
    out[:h] = img
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    cv.putText(out, f"name: {NAME}", (24, h+42), cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
    cv.putText(out, f"time: {now}", (24, h+84), cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
    cv.putText(out, tag, (760, h+42), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
    cv.putText(out, msg, (760, h+84), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
    print("姓名:", NAME)
    print(now)
    return out

# 基线参数，和 task1.py 完全一致
BASE = dict(lo=(0, 0, 120), hi=(180, 80, 255), area_min=5000, open_k=21, close_k=9)
AR_LO, AR_HI = 0.6, 1.7      # 长宽比筛选保持不变，保证只动一个参数
# 四个变体：第二个字典只覆盖要改的那一项，"一次只改一个主要参数"
VARIANTS = [
    ("A", "current params", dict(), "基线，和 task1 输出一致", "OK"),
    ("B", "wider HSV", dict(lo=(0, 0, 100), hi=(180, 120, 255)), "S 上限 80->120，V 下限 120->100", "FALSE POSITIVE"),
    ("C", "min area 500", dict(area_min=500), "最小面积 5000->500 px", "OK, more cands"),
    ("D", "no morphology", dict(open_k=0, close_k=0), "开运算和闭运算都关掉", "MISS"),
]

def run_one(img, hsv, cfg):
    mask = cv.inRange(hsv, cfg["lo"], cfg["hi"])
    if cfg["open_k"]:
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, cv.getStructuringElement(cv.MORPH_ELLIPSE, (cfg["open_k"], cfg["open_k"])))
    if cfg["close_k"]:
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, cv.getStructuringElement(cv.MORPH_ELLIPSE, (cfg["close_k"], cfg["close_k"])))
    cnts, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    area_min = cfg["area_min"]
    cands, drop_area, drop_ar = [], 0, 0
    for c in cnts:
        area = cv.contourArea(c)
        if area < area_min:
            drop_area += 1
            continue
        x, y, bw, bh = cv.boundingRect(c)
        if not (AR_LO <= bw/bh <= AR_HI):
            drop_ar += 1
            continue
        cands.append((area, c))
    cands.sort(key=lambda ac: ac[0], reverse=True)
    return mask, cnts, cands, area_min, drop_area, drop_ar

def render(img, cnts, cands):
    vis = img.copy()
    for area, c in cands[1:]:        # 其它通过筛选的候选也画出来，面积阈值一调小就能看到候选变多
        x, y, bw, bh = cv.boundingRect(c)
        cv.rectangle(vis, (x, y), (x+bw, y+bh), (255, 120, 0), 2)
    if not cands:
        cv.putText(vis, "NO TARGET", (40, 260), cv.FONT_HERSHEY_SIMPLEX, 2.2, (0, 0, 255), 5)
        c = max(cnts, key=cv.contourArea)   # 面积最大的原始轮廓描红：白线连成一张网，球被吞在里面
        cv.drawContours(vis, [c], -1, (0, 0, 255), 4)
        return vis, None
    area, c = cands[0]
    x, y, bw, bh = cv.boundingRect(c)
    m = cv.moments(c)
    if m["m00"] > 0:
        u, v = m["m10"]/m["m00"], m["m01"]/m["m00"]
    else:
        u, v = x+bw/2, y+bh/2
    cv.drawContours(vis, [c], -1, (0, 255, 0), 4)
    cv.rectangle(vis, (x, y), (x+bw, y+bh), (0, 255, 255), 2)
    cv.circle(vis, (int(u), int(v)), 9, (0, 0, 255), -1)
    cv.putText(vis, f"u={u:.0f}, v={v:.0f}", (x, max(210, y-12)), cv.FONT_HERSHEY_SIMPLEX, 1.3, (0, 0, 255), 3)
    return vis, (u, v, area)

img = cv.imdecode(np.fromfile(str(ROOT/"data"/"football_1.jpg"), np.uint8), cv.IMREAD_COLOR)
if img.size == 0:
    sys.exit("football_1 not found")
h, w = img.shape[:2]
hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
print("读取到 football_1", img.shape)

PW, PH = 960, 539          # 每格半分辨率，2x2 拼起来正好 1920x1078
panels = []
print(f"\n{'变体':<4}{'mask像素':>10}{'候选':>6}{'选中面积':>10}{'中心点':>16}  说明")
for key, name, over, note, verdict in VARIANTS:
    cfg = {**BASE, **over}
    mask, cnts, cands, area_min, drop_area, drop_ar = run_one(img, hsv, cfg)
    vis, res = render(img, cnts, cands)
    if res:
        u, v, area = res
        tail = f"u={u:.0f}, v={v:.0f}, area={int(area)}"
        print(f"{key:<4}{int(mask.sum()//255):>10}{len(cands):>6}{int(area):>10}{'(%.0f, %.0f)' % (u, v):>16}  {note}  [{verdict}]")
    else:
        tail = "NO TARGET"
        print(f"{key:<4}{int(mask.sum()//255):>10}{0:>6}{'-':>10}{'-':>16}  {note}  [{verdict}]")

    p = cv.resize(vis, (PW, PH), interpolation=cv.INTER_AREA)
    shade = p.copy()
    cv.rectangle(shade, (0, 0), (PW, 88), (0, 0, 0), -1)
    p = cv.addWeighted(shade, 0.6, p, 0.4, 0)
    cv.putText(p, f"{key}  {name}", (18, 34), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
    pv = f"S<={cfg['hi'][1]} V>={cfg['lo'][2]}  open{cfg['open_k']} close{cfg['close_k']}  area>={area_min:.0f}"
    cv.putText(p, pv, (18, 64), cv.FONT_HERSHEY_SIMPLEX, 0.62, (220, 220, 220), 2)
    cv.putText(p, f"-> {tail}   cand={len(cands)}   {verdict}", (18, 84), cv.FONT_HERSHEY_SIMPLEX, 0.62, (180, 255, 180), 2)
    panels.append(p)

grid = np.vstack([np.hstack([panels[0], panels[1]]), np.hstack([panels[2], panels[3]])])
grid = band(grid, "football_1  parameter compare", "A current / B wider HSV / C min area / D no morphology")
imwrite(ROOT/"outputs"/"task2"/"football_1_compare.png", grid)
if SHOW:
    cv.imshow("football_1_compare", grid)
    cv.waitKey(0)
    cv.destroyAllWindows()
