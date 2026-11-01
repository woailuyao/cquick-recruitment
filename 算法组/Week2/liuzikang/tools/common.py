#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""工具脚本共用的小函数：读图、存图、拼图、打印。"""
from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
IMG_EXTS = (".jpg", ".jpeg", ".png", ".bmp")


def setup_stdout() -> None:
    """让 Windows(GDK) 和 Ubuntu(UTF-8) 都能正常打印中文。"""
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")   # type: ignore[attr-defined]
        except (AttributeError, ValueError):
            pass


def imread_bgr(path: Path | str) -> np.ndarray | None:
    """读图。用 imdecode 而不是 imread，是为了支持中文路径（Windows 上 imread 会读不到）。"""
    try:
        data = np.fromfile(str(path), dtype=np.uint8)
    except OSError:
        return None
    if data.size == 0:
        return None
    img = cv2.imdecode(data, cv2.IMREAD_COLOR)
    return img


def imwrite_bgr(path: Path | str, img: np.ndarray) -> bool:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    ext = p.suffix if p.suffix else ".png"
    ok, buf = cv2.imencode(ext, img)
    if not ok:
        return False
    buf.tofile(str(p))
    return True


def list_images(folder: Path | str) -> List[Path]:
    folder = Path(folder)
    if not folder.is_dir():
        return []
    return sorted(p for p in folder.iterdir() if p.suffix.lower() in IMG_EXTS)


def load_json(path: Path | str) -> dict:
    p = Path(path)
    if not p.is_file():
        return {}
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return {}


def mask_to_bgr(mask: np.ndarray) -> np.ndarray:
    return cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)


def _font(size: int):
    try:
        return ImageFont.load_default(size=size)
    except TypeError:            # 老版本 Pillow 没有 size 参数
        return ImageFont.load_default()


def make_strip(panels: Sequence[Tuple[str, np.ndarray]], out_path: Path | str,
               panel_w: int = 620, label_h: int = 34) -> None:
    """把若干张图横向拼成一张，每张上面加一条英文标题。

    panels: [(标题, BGR 图), ...]
    """
    if not panels:
        return
    rendered = []
    for title, bgr in panels:
        h, w = bgr.shape[:2]
        panel_h = int(panel_w * h / max(1, w))
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        im = Image.fromarray(rgb).resize((panel_w, panel_h), Image.LANCZOS)
        canvas = Image.new("RGB", (panel_w, panel_h + label_h), (32, 32, 32))
        canvas.paste(im, (0, label_h))
        d = ImageDraw.Draw(canvas)
        d.text((8, 8), title, fill=(240, 240, 240), font=_font(18))
        rendered.append(canvas)

    total_w = sum(p.width for p in rendered) + 6 * (len(rendered) - 1)
    total_h = max(p.height for p in rendered)
    sheet = Image.new("RGB", (total_w, total_h), (32, 32, 32))
    x = 0
    for p in rendered:
        sheet.paste(p, (x, 0))
        x += p.width + 6
    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    sheet.save(out_path)


def make_grid(panels: Sequence[Tuple[str, np.ndarray]], out_path: Path | str,
              cols: int = 2, panel_w: int = 620, label_h: int = 34) -> None:
    """把图按 cols 列排成网格（参数对比用）。"""
    if not panels:
        return
    rendered = []
    for title, bgr in panels:
        h, w = bgr.shape[:2]
        panel_h = int(panel_w * h / max(1, w))
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        im = Image.fromarray(rgb).resize((panel_w, panel_h), Image.LANCZOS)
        canvas = Image.new("RGB", (panel_w, panel_h + label_h), (32, 32, 32))
        canvas.paste(im, (0, label_h))
        ImageDraw.Draw(canvas).text((8, 8), title, fill=(240, 240, 240), font=_font(18))
        rendered.append(canvas)

    rows = (len(rendered) + cols - 1) // cols
    cw = max(p.width for p in rendered)
    ch = max(p.height for p in rendered)
    sheet = Image.new("RGB", (cols * cw + 6 * (cols - 1), rows * ch + 6 * (rows - 1)), (32, 32, 32))
    for i, p in enumerate(rendered):
        r, c = divmod(i, cols)
        sheet.paste(p, (c * (cw + 6), r * (ch + 6)))
    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    sheet.save(out_path)
