# -*- coding: utf-8 -*-
"""把多张图拼成带标签的对照图（参数对比、批量结果都用它）。"""

from __future__ import annotations

from typing import List, Sequence

import cv2
import numpy as np

LABEL_H = 30          # 每张图上方的标签栏高度
GAP = 8               # 图与图之间的间隔
BAR_COLOR = (32, 32, 32)


def tile(image: np.ndarray, label: str, width: int = 480) -> np.ndarray:
    """缩放到固定宽度 + 顶部加一条标签栏，返回 BGR 图。"""
    if image.ndim == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    h, w = image.shape[:2]
    scale = width / float(w)
    resized = cv2.resize(image, (width, max(1, int(round(h * scale)))),
                         interpolation=cv2.INTER_AREA)
    out = cv2.copyMakeBorder(resized, LABEL_H, 0, 0, 0,
                             cv2.BORDER_CONSTANT, value=BAR_COLOR)
    cv2.putText(out, label, (8, LABEL_H - 9), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (255, 255, 255), 1, cv2.LINE_AA)
    return out


def stack_rows(rows: Sequence[Sequence[np.ndarray]]) -> np.ndarray:
    """把若干行拼起来，每行高度取该行最高者，其余用黑边补齐。"""
    if not rows:
        raise ValueError("没有可拼接的图片")

    out_rows: List[np.ndarray] = []
    for row in rows:
        row_h = max(img.shape[0] for img in row)
        padded = []
        for img in row:
            if img.shape[0] < row_h:
                img = cv2.copyMakeBorder(img, 0, row_h - img.shape[0], 0, 0,
                                         cv2.BORDER_CONSTANT, value=BAR_COLOR)
            padded.append(img)
            padded.append(np.full((row_h, GAP, 3), 0, dtype=np.uint8))
        out_rows.append(np.hstack(padded[:-1]))
    max_w = max(r.shape[1] for r in out_rows)
    fixed = []
    for r in out_rows:
        if r.shape[1] < max_w:
            r = cv2.copyMakeBorder(r, 0, 0, 0, max_w - r.shape[1],
                                   cv2.BORDER_CONSTANT, value=BAR_COLOR)
        fixed.append(r)
        fixed.append(np.zeros((GAP, max_w, 3), dtype=np.uint8))
    grid = np.vstack(fixed[:-1])
    return grid
