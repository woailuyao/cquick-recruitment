#!/usr/bin/env python3
"""生成 YOLO data.yaml（足球检测单类别，或标志检测多类别）。

用法（足球）：
    python make_data_yaml.py --data soccer_dataset --names ball --out soccer_dataset/data.yaml
用法（标志）：
    python make_data_yaml.py --data marker_dataset \
        --names LCross TCross XCross PenaltyPoint --out marker_dataset/data.yaml
"""

import argparse
import os

import yaml


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, help="数据集根目录")
    ap.add_argument("--names", nargs="+", required=True, help="类别名列表（顺序即 class_id）")
    ap.add_argument("--out", required=True, help="输出 data.yaml 路径")
    args = ap.parse_args()

    data_abs = os.path.abspath(args.data)
    cfg = {
        "path": data_abs,
        "train": "images/train",
        "val": "images/val",
        "test": "images/test",
        "nc": len(args.names),
        "names": args.names,
    }
    with open(args.out, "w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, allow_unicode=True, sort_keys=False)
    print(f"已生成 {args.out}，类别数 {len(args.names)}: {args.names}")


if __name__ == "__main__":
    main()
