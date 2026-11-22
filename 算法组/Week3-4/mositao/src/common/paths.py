# -*- coding: utf-8 -*-
"""路径统一入口（第 3-4 周工程根 = 本文件的 ../../）。

两条硬规则，沿用第 1、2 周的做法：
1. 所有脚本都在项目根目录运行，读写一律用相对路径；
2. 任何 print / 日志都不出现 `E:\\...` 这类绝对路径，只显示以 PROJECT_LABEL 开头的相对路径。
"""

from __future__ import annotations

import os
from pathlib import Path

# 项目根：src/common/paths.py -> src/common -> src -> 项目根
ROOT = Path(__file__).resolve().parents[2]

# 终端里显示的项目名：相对仓库根 cquick-recruitment-main 的完整路径。
# 规范要求「相对路径的第一层必须是仓库目录名」，故默认写成仓库内的真实位置；
# 若日后挪动目录，改环境变量 W34_LABEL 即可，不必改代码。
PROJECT_LABEL = os.environ.get(
    "W34_LABEL", "cquick-recruitment-main/算法组/Week3-4/mositao"
)

CONFIG_DIR = ROOT / "config"
DATASET_DIR = ROOT / "dataset"
CALIB_DIR = ROOT / "calibration"
OUTPUT_DIR = ROOT / "outputs"
VIDEO_DIR = ROOT / "data" / "videos"
RUNS_DIR = ROOT / "runs"          # 训练产物，不提交仓库
WEIGHTS_DIR = ROOT / "weights"    # best.pt 等轻量权重存放处

# 运行期需要保证存在的目录
_RUNTIME_DIRS = (OUTPUT_DIR, RUNS_DIR, VIDEO_DIR)


def ensure_runtime_dirs() -> None:
    """按需创建运行期目录（不含 dataset / calibration，那两处由人工准备）。"""
    for d in _RUNTIME_DIRS:
        d.mkdir(parents=True, exist_ok=True)


def rel(path) -> str:
    """把任意路径转成以 PROJECT_LABEL 开头的相对路径字符串，用于打印与写 README。"""
    p = Path(path)
    try:
        p = p.resolve()
    except OSError:
        p = Path(path)
    try:
        r = p.relative_to(ROOT)
    except ValueError:
        # 不属于本项目时只保留末尾两段，绝不泄露绝对路径
        parts = p.parts[-2:] if len(p.parts) >= 2 else p.parts[-1:]
        return "/".join((PROJECT_LABEL,) + tuple(parts))
    return PROJECT_LABEL + "/" + r.as_posix()


def at(*parts: str) -> Path:
    """拼接项目根下的路径，例如 at("config", "field.yaml")。"""
    return ROOT.joinpath(*parts)
