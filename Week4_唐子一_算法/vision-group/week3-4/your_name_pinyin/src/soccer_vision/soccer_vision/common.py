"""公共工具：YAML 读写、路径解析、坐标变换等。

所有脚本都从这里导入公共函数，避免重复代码。
"""

from __future__ import annotations

import os
from typing import Any, Dict

import numpy as np
import yaml


def load_yaml(path: str) -> Dict[str, Any]:
    """读取 YAML 文件，统一使用 UTF-8 编码。"""
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def save_yaml(path: str, data: Dict[str, Any]) -> None:
    """保存 YAML 文件（UTF-8，保留中文注释可读性）。"""
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(
            data, f, allow_unicode=True, sort_keys=False, default_flow_style=False
        )


def package_root() -> str:
    """返回工程包根目录（soccer_vision 包的上一级 src 的上一级……见具体调用）。

    约定：本文件位于 <个人目录>/src/soccer_vision/soccer_vision/common.py，
    包根为 <个人目录>。
    """
    # common.py -> soccer_vision(包目录) -> soccer_vision(外层) -> src -> 个人目录
    here = os.path.dirname(os.path.abspath(__file__))          # .../soccer_vision/soccer_vision
    return os.path.dirname(os.path.dirname(os.path.dirname(here)))  # .../<个人目录>


def res(*parts: str) -> str:
    """相对包根目录拼出资源路径。"""
    return os.path.join(package_root(), *parts)


# ---------------------------------------------------------------------------
# 坐标 / 位姿变换工具
# ---------------------------------------------------------------------------

def rot2d(theta: float) -> np.ndarray:
    """二维旋转矩阵。"""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]], dtype=float)


def se2_transform(points: np.ndarray, pose: tuple) -> np.ndarray:
    """把载体坐标系下的点 (N,2) 变换到地图坐标系。

    pose = (x, y, theta)，theta 为弧度。
    """
    x, y, theta = pose
    R = rot2d(theta)
    return points @ R.T + np.array([x, y])
