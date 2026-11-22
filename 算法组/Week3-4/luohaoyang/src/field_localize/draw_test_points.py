import sys
import math
import numpy as np
import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge
from matplotlib.lines import Line2D

sys.path.insert(0, r'F:\robot\luohaoyang\week3_4\task2_markers')
from field_localize.field_map import FieldMap
from field_localize.ground_projection import euler_to_R

K = np.array([[1612.90, 0, 669.6], [0, 1614.95, 352.1], [0, 0, 1.0]])
D = np.array([0.136, -0.9501, -0.0033, 0.0014, 0.0])
W, H = 1280, 720
fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
U0, U1 = (0 - cx) / fx, (W - cx) / fx
V0, V1 = (0 - cy) / fy, (H - cy) / fy
H_HALF = 21.5
MAXR = 3.6

fm = FieldMap.load(r'F:\robot\luohaoyang\week3_4\task2_markers\config\field_measured.yaml')
R_bc = euler_to_R(9.42, 3.13, 0.33)      # 今天标出来的外参
O = np.array([0.18, 0.025, 0.643])


def sees(x, y, yaw, rmin=1.3, rmax=MAXR):
    """可见标志：先在【未畸变归一化坐标】上判断是否落在像素范围内，
    再考虑畸变 —— 不能先投影，因为这台相机的 k2=-0.95 在视野边缘会把
    画面外的点"折"回画面内（校准板只覆盖了画面中心 11%，边缘模型不可信）。"""
    th = math.radians(yaw)
    c, s = math.cos(th), math.sin(th)
    R_cb = R_bc.T
    out = []
    for m in fm.markers:
        dx, dy = m.x - x, m.y - y
        bx, by = c * dx + s * dy, -s * dx + c * dy
        r = math.hypot(bx, by)
        if r < rmin or r > rmax:
            continue
        Xc = R_cb @ (np.array([bx, by, 0.0]) - O)
        if Xc[2] <= 0.2:
            continue
        xn, yn = Xc[0] / Xc[2], Xc[1] / Xc[2]
        if U0 <= xn <= U1 and V0 <= yn <= V1:
            out.append(m)
    return out


POSES = [('P1', -0.90, 1.10, 110, '#ff7b00'),
         ('P2', 2.70, 0.60, 100, '#e63946'),
         ('P3', 0.50, 3.80, -170, '#2a9d8f')]

XL, XR, YB, YT = -3.0, 3.0, -3.9, 3.9
GA_HALF, GA_Y, PA_HALF, PA_Y = 1.35, 2.95, 2.00, 2.00

fig, ax = plt.subplots(figsize=(12, 13))
ax.add_patch(plt.Rectangle((XL, YB), XR - XL, YT - YB, facecolor='#26402a',
                           edgecolor='white', lw=2.2, zorder=1))
ax.plot([XL, XR], [0, 0], color='white', lw=1.6, zorder=2)
ax.add_patch(plt.Circle((0, 0), 0.75, fill=False, ec='white', lw=1.5, zorder=2))
for sgn in (+1, -1):
    ax.plot([-GA_HALF, GA_HALF], [sgn * GA_Y] * 2, color='white', lw=1.4, zorder=2)
    for sx in (-GA_HALF, GA_HALF):
        ax.plot([sx, sx], [sgn * GA_Y, sgn * YT], color='white', lw=1.4, zorder=2)
    ax.plot([-PA_HALF, PA_HALF], [sgn * PA_Y] * 2, color='white', lw=1.4, alpha=0.8, zorder=2)
    for sx in (-PA_HALF, PA_HALF):
        ax.plot([sx, sx], [sgn * PA_Y, sgn * YT], color='white', lw=1.4, alpha=0.8, zorder=2)

for m in fm.markers:
    ax.plot(m.x, m.y, 'o', color='#9be29b', ms=6, zorder=3)

for name, x, y, yaw, col in POSES:
    ax.add_patch(Wedge((x, y), MAXR, yaw - H_HALF, yaw + H_HALF, width=MAXR,
                       facecolor=col, alpha=0.13, edgecolor=col, lw=1.2, zorder=4))
    seen = sees(x, y, yaw)
    for m in seen:
        ax.plot(m.x, m.y, 'o', mfc='none', mec=col, ms=15, mew=2.2, zorder=6)
    th = math.radians(yaw)
    ax.annotate('', xy=(x + 1.1 * math.cos(th), y + 1.1 * math.sin(th)), xytext=(x, y),
                arrowprops=dict(arrowstyle='-|>', color=col, lw=3, mutation_scale=20), zorder=7)
    ax.plot(x, y, 'o', color=col, ms=11, mec='white', mew=1.6, zorder=8)
    off = {'P1': (0.05, -0.62), 'P2': (-1.05, -0.30),
           'P3': (-0.95, -0.42), 'B0': (0.62, -0.60)}[name]
    ax.annotate(f'{name}  yaw={yaw}°\n{len(seen)} markers',
                (x + off[0], y + off[1]), ha='center', va='top', fontsize=9,
                color='black', zorder=9,
                bbox=dict(boxstyle='round,pad=0.3', fc=col, ec='white', alpha=0.95))

ax.plot([XL, XR], [YT, YT], color='white', lw=2.2, zorder=2)
ax.text(XL + 0.07, YT - 0.22, 'goal line  y=+3.90', color='#9be29b', fontsize=8.5)
ax.text(XL + 0.07, YB + 0.08, 'goal line  y=-3.90', color='#9be29b', fontsize=8.5)

ax.annotate('', xy=(XL + 1.0, YB + 1.15), xytext=(XL + 0.15, YB + 1.15),
            arrowprops=dict(arrowstyle='-|>', color='#ffd166', lw=2))
ax.text(XL + 1.06, YB + 1.15, '+X', color='#ffd166', fontsize=11, va='center', fontweight='bold')
ax.annotate('', xy=(XL + 0.15, YB + 1.95), xytext=(XL + 0.15, YB + 1.15),
            arrowprops=dict(arrowstyle='-|>', color='#ffd166', lw=2))
ax.text(XL + 0.15, YB + 2.02, '+Y', color='#ffd166', fontsize=11, ha='center', fontweight='bold')

handles = [Line2D([0], [0], marker='o', color='w', markerfacecolor='#9be29b',
                  markersize=7, label='marker' + f' ({len(fm.markers)})', linestyle='None')]
for name, _, _, _, col in POSES:
    handles.append(Line2D([0], [0], marker='o', color='w', markerfacecolor=col,
                          markersize=10, label=f'{name} pose + FOV', linestyle='None'))
handles.append(Line2D([0], [0], marker='o', color='w', markerfacecolor='none',
                      markeredgecolor='white', markersize=12, label='visible marker'))
ax.legend(handles=handles, loc='lower right', fontsize=9, framealpha=0.92)

ax.set_title('Where the three test points are (cone = ±21.5° horizontal FOV, radius 4 m)\n'
             'rings = markers actually inside the 1280x720 frame for that pose',
             fontsize=12, pad=12)
ax.set_xlim(XL - 0.5, XR + 0.5)
ax.set_ylim(YB - 0.5, YT + 0.5)
ax.set_aspect('equal')
ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
ax.grid(alpha=0.12, color='white')
fig.patch.set_facecolor('#f4f4f4')
plt.tight_layout()
out = r'F:\robot\luohaoyang\week3_4\task2_markers\test_points_map.png'
plt.savefig(out, dpi=135, facecolor=fig.get_facecolor())
print('saved', out)
for name, x, y, yaw, _ in POSES:
    seen = sees(x, y, yaw)
    print(f'{name} ({x:+.2f},{y:+.2f},{yaw:+4d}): ' +
          ', '.join(f'{m.id}@{math.hypot(m.x-x,m.y-y):.2f}' for m in seen))