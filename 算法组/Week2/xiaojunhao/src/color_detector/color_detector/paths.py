# -*- coding: utf-8 -*-
"""找图片目录。直接跑源码和 colcon 装好之后，两种情况的路径不一样。"""

from pathlib import Path


def package_root():
    return Path(__file__).resolve().parents[1]


def image_dir():
    # 装好之后图片在 share/color_detector/images 下
    try:
        from ament_index_python.packages import get_package_share_directory
        d = Path(get_package_share_directory("color_detector")) / "images"
        if d.is_dir():
            return d
    except Exception:
        pass
    # 没装或找不到，就用源码目录里的 data/images
    return package_root() / "data" / "images"


def result_dir():
    d = package_root() / "results"
    d.mkdir(parents=True, exist_ok=True)
    return d
