# -*- coding: utf-8 -*-
"""
把 prelabels/ 里的自动预标注复制到 raw/<段>/ 下，让 labelImg 一打开就有草稿框。

用法：
    python apply_prelabels.py            # 复制（已存在的 .txt 不覆盖）
    python apply_prelabels.py --force    # 覆盖已有 .txt（小心：会盖掉你手改过的）

说明：只复制 .txt，不动图片。想撤销就删掉 raw/<段>/ 里的 .txt 即可。
"""
import argparse
import os
import shutil

RAW = "raw"
PRE = "prelabels"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--force", action="store_true")
    ap.add_argument("--only", default=None, help="只处理某一段，例如 video1")
    args = ap.parse_args()

    if not os.path.isdir(PRE):
        print(f"找不到 {PRE}/，先跑 python prelabel_ball.py")
        return

    total_copy = total_skip = total_empty = 0
    for seg in sorted(os.listdir(PRE)):
        src = os.path.join(PRE, seg)
        dst = os.path.join(RAW, seg)
        if not os.path.isdir(src) or not os.path.isdir(dst):
            continue
        if args.only and seg != args.only:
            continue

        copied = skipped = empty = 0
        for f in sorted(os.listdir(src)):
            if not f.endswith(".txt"):
                continue
            s = os.path.join(src, f)
            d = os.path.join(dst, f)
            has_box = os.path.getsize(s) > 0
            if not has_box:
                # 预标注为空 = 模型没找到球（或找到的是静态杂物被过滤掉了）。
                # 必须把目标 .txt 也清空，否则 raw/ 里会残留上一版错误的框，
                # labelImg 里就会看到"框在别的东西上"。清空后 labelImg 不显示框，
                # 由人工按 W 自己画。split_dataset.py 会跳过 0 字节的标注。
                if os.path.exists(d):
                    open(d, "w").close()
                empty += 1
                continue
            if os.path.exists(d) and not args.force:
                skipped += 1
                continue
            shutil.copy2(s, d)
            copied += 1
        print(f"  {seg}: 复制 {copied} 个有框的 .txt，跳过已存在 {skipped}，"
              f"清空 {empty} 个（需手工画框）")
        total_copy += copied
        total_skip += skipped
        total_empty += empty

    print(f"\n合计：复制 {total_copy}，跳过 {total_skip}，无预标注 {total_empty}")
    print("下一步：python labelImg_launcher.py video1")
    print("在 labelImg 里逐张核对预标注框；漏检的补画，误检的删掉。")


if __name__ == "__main__":
    main()
