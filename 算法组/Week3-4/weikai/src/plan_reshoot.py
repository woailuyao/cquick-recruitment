"""重拍站位规划：基于 field.yaml v2（12 标志）+ 实测内参，计算候选站位的可见标志、投影尺寸、俯角覆盖。
运行：python plan_reshoot.py
"""
import math
import yaml

FIELD = yaml.safe_load(open("templates/field.yaml", encoding="utf-8"))
PTS = {k: v for k, v in FIELD.items()
       if k.startswith(("TCross", "LCross")) and isinstance(v, list)}

FX = 1554.66
FY = 1563.82
CX, CY = 681.26, 419.04
W, H = 1280, 720
HALF_H = math.degrees(math.atan((W / 2) / FX))   # 22.39°
HALF_V = math.degrees(math.atan((H / 2) / FY))   # 12.96°
MARK_M = 0.45          # 标志线段典型长度（m），用于估算投影像素


def evaluate(cam, yaw, h, pitch):
    """返回 (可见点数, 最近/最远距离, 最小投影px, 方位跨度, 俯角偏移范围)"""
    vis = []
    for name, (px, py) in PTS.items():
        dx, dy = px - cam[0], py - cam[1]
        d = math.hypot(dx, dy)
        off = math.degrees(math.atan2(dy, dx)) - yaw
        if abs(off) > HALF_H - 1.0:
            continue
        downtilt = math.degrees(math.atan2(h, d))   # 目标相对水平线向下角
        voff = downtilt - pitch                      # 相对光轴的垂直偏移
        if abs(voff) > HALF_V - 1.0:
            continue
        vis.append((name, d, off, voff, MARK_M * FX / d))
    if not vis:
        return None
    ang = [v[2] for v in vis]
    return dict(n=len(vis), dmin=min(v[1] for v in vis), dmax=max(v[1] for v in vis),
                proj=min(v[4] for v in vis), spread=max(ang) - min(ang),
                voff=(min(v[3] for v in vis), max(v[3] for v in vis)),
                names=[v[0] for v in vis])


def yaw_to(target, cam):
    return math.degrees(math.atan2(target[1] - cam[1], target[0] - cam[0]))


GOAL = (3.977, 0.0)

# 固定安装实测值（用户 2026-11-21 卷尺/几何法量测）
H_CAM, PITCH = 1.005, 15.0

CANDIDATES = [
    ("R1 边线中点朝球门",       (0.00, 2.60), GOAL),
    ("R2 中心朝右侧LCross4",   (0.00, 0.00), (3.084, -1.461)),
    ("R3 左前朝球门",          (-2.00, 1.50), GOAL),
    ("R4 右前斜朝右下拐角LCross6", (0.75, 1.25), (3.977, -2.991)),
    ("R5 右前近景朝左侧LCross2", (2.00, -1.00), (3.084, 1.461)),
    ("R7 中心朝左下L",          (0.00, 0.00), (2.077, -1.983)),
    ("R8 中线后段朝球门",       (-1.00, 0.50), GOAL),
]

print(f"视野: 水平 ±{HALF_H:.2f}°  垂直 ±{HALF_V:.2f}°   "
      f"固定安装假设 h={H_CAM}m pitch={PITCH}°")
print(f"{'站位':<18} {'yaw真值':>8} {'可见':>5} {'距离范围':>12} {'最小投影':>9} {'方位跨度':>9} {'垂直偏移':>14}")
print("-" * 90)
for name, cam, target in CANDIDATES:
    yaw = yaw_to(target, cam)
    r = evaluate(cam, yaw, H_CAM, PITCH)
    if r is None:
        print(f"{name:<18} {yaw:+8.2f} {'0':>5}  —— 全部标志不在画面内")
        continue
    print(f"{name:<18} {yaw:+8.2f} {r['n']:>3}/12  "
          f"{r['dmin']:5.2f}~{r['dmax']:5.2f}m  {r['proj']:7.0f}px  "
          f"{r['spread']:7.1f}°  {r['voff'][0]:+5.1f}~{r['voff'][1]:+5.1f}°")

print("\n--- 俯角敏感性：R4 (-3,1) 朝球门，不同 h/pitch 组合 ---")
cam, yaw = (-3.0, 1.0), yaw_to(GOAL, (-3.0, 1.0))
for h in (0.8, 1.0, 1.2, 1.5):
    row = []
    for p in (12, 15, 18, 21, 24):
        r = evaluate(cam, yaw, h, p)
        row.append(f"p{p}:{r['n'] if r else 0}")
    print(f"h={h:.1f}m  " + "  ".join(row))
