import sys
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

sys.path.insert(0, r'F:\robot\luohaoyang\week3_4\task2_markers')
from field_localize.field_map import FieldMap

fm = FieldMap.load(r'F:\robot\luohaoyang\week3_4\task2_markers\config\field_measured.yaml')

COL = {'LCross': '#1a7f37', 'TCross': '#1f6feb', 'XCross': '#c1121f', 'PenaltyPoint': '#b5179e'}
SHAPE = {'LCross': 'o', 'TCross': 's', 'XCross': '^', 'PenaltyPoint': 'D'}

XL, XR = -3.0, 3.0
YB, YT = -3.9, 3.9
GA_HALF, GA_Y = 1.35, 2.95
PA_HALF, PA_Y = 2.00, 2.00
R_CIRCLE = 0.75
GOAL_HALF, GOAL_DEPTH = 1.30, 0.45      # 假设值，图上标注"未测量"

fig, ax = plt.subplots(figsize=(11.5, 12.5))
ax.add_patch(plt.Rectangle((XL, YB), XR - XL, YT - YB,
                           facecolor='#2d5a2d', edgecolor='white', lw=2.4, zorder=1))

# 中线 / 中圈
ax.plot([XL, XR], [0, 0], color='white', lw=1.8, zorder=2)
ax.add_patch(plt.Circle((0, 0), R_CIRCLE, fill=False, ec='white', lw=1.6, zorder=2))
ax.plot([0], [0], marker='+', color='white', ms=10, mew=1.6, zorder=3)
ax.annotate('origin (0,0)', (0, 0), xytext=(0.85, -0.95), fontsize=8.5, color='#ffd166',
            arrowprops=dict(arrowstyle='-|>', color='#ffd166', lw=1.1), zorder=7)

for sgn in (+1, -1):
    ax.plot([-GA_HALF, GA_HALF], [sgn * GA_Y] * 2, color='white', lw=1.6, zorder=2)
    for sx in (-GA_HALF, GA_HALF):
        ax.plot([sx, sx], [sgn * GA_Y, sgn * YT], color='white', lw=1.6, zorder=2)
    ax.plot([-PA_HALF, PA_HALF], [sgn * PA_Y] * 2, color='white', lw=1.6, alpha=0.85, zorder=2)
    for sx in (-PA_HALF, PA_HALF):
        ax.plot([sx, sx], [sgn * PA_Y, sgn * YT], color='white', lw=1.6, alpha=0.85, zorder=2)
    # 球门（未测量，用虚线示意）
    y_out = sgn * (YT + GOAL_DEPTH)
    ax.plot([-GOAL_HALF, -GOAL_HALF], [sgn * YT, y_out], color='#ffd166', lw=2.2, ls='--', zorder=3)
    ax.plot([GOAL_HALF, GOAL_HALF], [sgn * YT, y_out], color='#ffd166', lw=2.2, ls='--', zorder=3)
    ax.plot([-GOAL_HALF, GOAL_HALF], [y_out, y_out], color='#ffd166', lw=2.2, ls='--', zorder=3)

for m in fm.markers:
    ax.plot(m.x, m.y, SHAPE[m.type], color=COL[m.type], ms=11, mec='white', mew=0.8, zorder=5)
    ax.annotate(m.id, (m.x, m.y), xytext=(0, 9), textcoords='offset points',
                ha='center', fontsize=7.5, color='white', zorder=6,
                bbox=dict(boxstyle='round,pad=0.15', fc=COL[m.type], ec='none', alpha=0.85))

# 坐标轴示意（画在场内左下空白处）
ax.annotate('', xy=(XL + 0.95, YB + 1.30), xytext=(XL + 0.12, YB + 1.30),
            arrowprops=dict(arrowstyle='-|>', color='#ffd166', lw=2.2))
ax.text(XL + 1.02, YB + 1.30, '+X', color='#ffd166', fontsize=12, va='center', fontweight='bold')
ax.annotate('', xy=(XL + 0.12, YB + 2.10), xytext=(XL + 0.12, YB + 1.30),
            arrowprops=dict(arrowstyle='-|>', color='#ffd166', lw=2.2))
ax.text(XL + 0.12, YB + 2.20, '+Y', color='#ffd166', fontsize=12, ha='center', fontweight='bold')
ax.text(XL + 0.12, YB + 1.12, 'axes', color='#ffd166', fontsize=8.5, ha='center')

# 测试点
poses = [('P1', 0.50, 3.60, -170, '#ff7b00', (-0.10, -0.45)),
         ('P2', -0.50, -3.60, 10, '#ff7b00', (0.10, -0.42)),
         ('P3', -0.90, 1.00, 120, '#ff7b00', (-0.60, -0.30)),
         ('B0 boot', -0.90, -1.00, -120, '#00b4d8', (-0.55, -0.22))]
for name, x, y, yaw, c, (ox, oy) in poses:
    th = math.radians(yaw)
    ax.annotate('', xy=(x + 0.8 * math.cos(th), y + 0.8 * math.sin(th)), xytext=(x, y),
                arrowprops=dict(arrowstyle='-|>', color=c, lw=2.8, mutation_scale=18), zorder=8)
    ax.plot(x, y, 'o', color=c, ms=9, mec='white', mew=1.2, zorder=9)
    ax.annotate(f'{name}  yaw={yaw}°', (x + ox, y + oy), ha='center', va='center',
                fontsize=8.5, color='black', zorder=10,
                bbox=dict(boxstyle='round,pad=0.22', fc=c, ec='white', alpha=0.95))

# 尺寸标注
ax.annotate('', xy=(XL, YB - 0.85), xytext=(XR, YB - 0.85),
            arrowprops=dict(arrowstyle='<->', color='#555555', lw=1.2))
ax.text(0, YB - 1.02, '6.00 m', ha='center', color='#555555', fontsize=10)
ax.annotate('', xy=(XR + 0.55, YB), xytext=(XR + 0.55, YT),
            arrowprops=dict(arrowstyle='<->', color='#555555', lw=1.2))
ax.text(XR + 0.75, 0, '7.80 m', va='center', rotation=90, color='#555555', fontsize=10)
ax.text(XR + 0.22, GA_Y, '0.95', va='center', color='#9be29b', fontsize=8)
ax.text(XR + 0.22, PA_Y, '1.90', va='center', color='#9be29b', fontsize=8)

ax.text(XL + 0.08, YT - 0.18, 'goal line', color='#9be29b', fontsize=8.5)
ax.text(XL + 0.08, GA_Y - 0.22, 'goal area  2.70 x 0.95', color='#9be29b', fontsize=8.5)
ax.text(XL + 0.08, PA_Y - 0.22, 'penalty area  4.00 x 1.90', color='#9be29b', fontsize=8.5)
ax.text(XL + 0.08, YT - 0.52,
        'dashed yellow = goal frame, NOT measured (guess)',
        color='#ffd166', fontsize=8.5)

# 待确认清单（放在图外底部）
confirm = ("CONFIRM THESE:   1) goals on the +Y/-Y ends?    "
           "2) goal line at y=+/-3.90, width 6.00 m?    "
           "3) goal area 2.70x0.95, penalty area 4.00x1.90?    "
           "4) centre circle radius 0.75 m (XCross at +/-0.75)?    "
           "5) any marker missing (posts, penalty spots)?")
fig.text(0.5, 0.012, confirm, ha='center', va='bottom', fontsize=8.6,
         color='#222', wrap=True,
         bbox=dict(boxstyle='round,pad=0.5', fc='#fff8dc', ec='#c9a227', lw=1.2))

handles = [Line2D([0], [0], marker=SHAPE[t], color='w', markerfacecolor=COL[t],
                  markersize=10, label=t, linestyle='None')
           for t in ('LCross', 'TCross', 'XCross') if fm.by_type(t)]
handles += [Line2D([0], [0], marker='o', color='w', markerfacecolor='#ff7b00',
                   markersize=10, label='test point P1/P2/P3', linestyle='None'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='#00b4d8',
                   markersize=10, label='boot point B0', linestyle='None')]
ax.legend(handles=handles, loc='lower right', fontsize=9, framealpha=0.92)

ax.set_title('C0 field map from measured markers  (origin = centre circle)\n'
             f'{len(fm.markers)} markers  |  L={len(fm.by_type("LCross"))}  '
             f'T={len(fm.by_type("TCross"))}  X={len(fm.by_type("XCross"))}',
             fontsize=12.5, pad=12)
ax.set_xlim(XL - 1.1, XR + 1.3)
ax.set_ylim(YB - 1.35, YT + 1.15)
ax.set_aspect('equal')
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.grid(alpha=0.15, color='white')
fig.patch.set_facecolor('#f4f4f4')
plt.subplots_adjust(left=0.09, right=0.95, top=0.93, bottom=0.10)
out = r'F:\robot\luohaoyang\week3_4\task2_markers\field_map_check.png'
plt.savefig(out, dpi=140, facecolor=fig.get_facecolor())
print('saved', out)