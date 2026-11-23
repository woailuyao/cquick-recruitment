# -*- coding: utf-8 -*-
"""命令行参数助手：让 3 个脚本共用同一套 HSV / 面积 / 形态学开关。"""

from __future__ import annotations

import argparse

from .detector import (DEFAULT_HSV_LOWER, DEFAULT_HSV_UPPER,
                       DEFAULT_MIN_AREA, DEFAULT_MORPH_KERNEL, DetectorConfig)


def add_detector_args(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    parser.add_argument("--hsv-lower", nargs=3, type=int, default=list(DEFAULT_HSV_LOWER),
                        metavar=("H", "S", "V"), help="HSV 下界，默认 95 120 60")
    parser.add_argument("--hsv-upper", nargs=3, type=int, default=list(DEFAULT_HSV_UPPER),
                        metavar=("H", "S", "V"), help="HSV 上界，默认 130 255 255")
    parser.add_argument("--min-area", type=float, default=DEFAULT_MIN_AREA,
                        help="最小轮廓面积（像素平方），默认 500")
    parser.add_argument("--kernel", type=int, default=DEFAULT_MORPH_KERNEL,
                        help="形态学核尺寸（奇数），默认 5")
    parser.add_argument("--no-open", action="store_true", help="关闭开运算")
    parser.add_argument("--no-close", action="store_true", help="关闭闭运算")
    parser.add_argument("--no-morph", action="store_true", help="等价于同时关闭开运算和闭运算")
    return parser


def config_from_args(args: argparse.Namespace) -> DetectorConfig:
    use_open = not (args.no_open or args.no_morph)
    use_close = not (args.no_close or args.no_morph)
    return DetectorConfig(
        hsv_lower=tuple(args.hsv_lower),
        hsv_upper=tuple(args.hsv_upper),
        min_area=args.min_area,
        morph_kernel=args.kernel,
        use_open=use_open,
        use_close=use_close,
    )
