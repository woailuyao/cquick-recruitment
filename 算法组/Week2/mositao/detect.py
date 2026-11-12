# -*- coding: utf-8 -*-
"""兼容入口：一次性运行 任务1 + 任务2。

任务已按要求拆分到 src/ 下，本文件只保留旧的一键入口（任意工作目录均可运行）：
    python detect.py                  # 任务1 + 任务2
    python detect.py --demo-no-target # 附带演示未检测到目标的失败分支

实际逻辑：
    src/task1.py  任务1：HSV 颜色分割与目标中心点检测
    src/task2.py  任务2：检测结果可视化与参数对比
"""
import argparse
import os
import sys

# 按本文件自身位置找 src/：不依赖当前工作目录，也不写死绝对路径
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "src"))

import task1  # noqa: E402
import task2  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description="一次性运行 任务1 + 任务2")
    parser.add_argument("--demo-no-target", action="store_true",
                        help="额外演示未检测到目标时的失败分支")
    args = parser.parse_args()

    if not os.path.isdir(task1.IMG_DIR):
        sys.exit(f"[错误] 找不到 {task1.rel_path(task1.IMG_DIR)}（数据目录应与 src/ 同级）")
    os.makedirs(task1.OUT_DIR, exist_ok=True)
    task1.run()
    print()
    task2.main()
    if args.demo_no_target:
        task1.demo_no_target()


if __name__ == "__main__":
    main()
