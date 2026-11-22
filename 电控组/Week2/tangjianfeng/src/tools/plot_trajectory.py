#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
把 record_pose.py 录到的真实位姿 CSV 画成轨迹图（模拟 turtlesim 画布 0~11）。
用法: ./plot_trajectory.py pose.csv out.png
"""
import csv
import datetime
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# 中文字体
matplotlib.rcParams['font.sans-serif'] = [
    'Noto Sans CJK SC', 'Noto Serif CJK SC', 'WenQuanYi Zen Hei',
    'AR PL UMing CN', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False

GOALS = {1: (2.0, 3.0), 2: (5.5, 8.0), 3: (9.0, 4.0)}
COLORS = {1: '#1f77b4', 2: '#ff7f0e', 3: '#2ca02c'}


def load(path):
    xs, ys = [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            xs.append(float(row['x']))
            ys.append(float(row['y']))
    return xs, ys


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else 'pose.csv'
    out = sys.argv[2] if len(sys.argv) > 2 else 'trajectory.png'

    xs, ys = load(src)

    fig, ax = plt.subplots(figsize=(7.2, 7.2), dpi=150)
    # turtlesim 画布：0~11，原点在左下
    ax.set_xlim(0, 11)
    ax.set_ylim(0, 11)
    ax.set_aspect('equal')

    # 场地边框（模拟 turtlesim 画布边界）
    ax.add_patch(plt.Rectangle((0, 0), 11, 11, fill=False,
                               edgecolor='#555555', lw=2.5))
    for i in range(1, 11):
        ax.axvline(i, color='#eeeeee', lw=0.6, zorder=0)
        ax.axhline(i, color='#eeeeee', lw=0.6, zorder=0)

    # 真实走过的轨迹
    ax.plot(xs, ys, color='#d62728', lw=2.2, zorder=3,
            label='turtle1 实际轨迹')

    # 起点
    ax.plot(xs[0], ys[0], 'o', ms=9, mfc='white', mec='#444444',
            mew=2, zorder=5)
    ax.annotate('起点 (5.54, 5.54)', (xs[0], ys[0]),
                xytext=(8, -18), textcoords='offset points',
                fontsize=9, color='#333333')

    # 三个目标点
    for bid, (gx, gy) in GOALS.items():
        ax.add_patch(Circle((gx, gy), 0.5, fill=False, ls='--',
                            edgecolor=COLORS[bid], lw=1.2, zorder=4))
        ax.plot(gx, gy, 'X', ms=13, color=COLORS[bid], zorder=6)
        off = {1: (-12, -20), 2: (10, 8), 3: (10, -16)}[bid]
        ax.annotate(f'球{bid} ({gx:g}, {gy:g})', (gx, gy),
                    xytext=off, textcoords='offset points',
                    fontsize=10, color=COLORS[bid], fontweight='bold',
                    ha='right' if off[0] < 0 else 'left')

    # 终点
    ax.plot(xs[-1], ys[-1], 'o', ms=7, color='#d62728', zorder=6)

    ax.set_xlabel('x (场地坐标 / m)')
    ax.set_ylabel('y (场地坐标 / m)')
    ax.set_title('任务2 · 乌龟按 1→2→3 依次到达三个球位（闭环跟随）',
                 fontsize=12, pad=12)
    ax.legend(loc='upper left', framealpha=0.95)
    ax.grid(False)

    stamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    fig.text(0.99, 0.01, f'数据来源: 订阅 /turtle1/pose 实录 · 生成时间 {stamp}',
             ha='right', fontsize=7.5, color='#888888')

    fig.tight_layout()
    fig.savefig(out, bbox_inches='tight')
    print(f'已保存: {out}  轨迹点数={len(xs)}')


if __name__ == '__main__':
    main()
