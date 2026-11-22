#!/usr/bin/env python3
"""把 makesense.ai 导出的 YOLO 标签包，自动放进对应的图片文件夹。

makesense.ai 导出的 zip 里一般是 labels/xxx.txt + classes.txt，
文件名和原图一致，所以直接按同名匹配即可。

用法（一条命令）：
  python tools\\import_labels.py --target C:\\...\\frames\\ball_v04
  python tools\\import_labels.py --target C:\\...\\frames\\ball_v04 --zip C:\\...\\labels_xxx.zip
不加 --zip 时自动找"下载"文件夹里最新的 labels_*.zip
"""
from __future__ import annotations

import argparse
import glob
import os
import shutil
import tempfile
import zipfile

IMG_EXT = (".jpg", ".jpeg", ".png", ".bmp")


def find_newest_zip(downloads: str):
    pats = ["labels_*.zip", "labels*.zip", "*.zip"]
    for p in pats:
        hits = [f for f in glob.glob(os.path.join(downloads, p))
                if "labels" in os.path.basename(f).lower()]
        if hits:
            return max(hits, key=os.path.getmtime)
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", required=True, help="要放标签的图片文件夹")
    ap.add_argument("--zip", default=None, help="导出包路径；不给就自动找下载目录里最新的")
    ap.add_argument("--downloads", default=os.path.join(os.path.expanduser("~"), "Downloads"))
    args = ap.parse_args()

    target = os.path.abspath(args.target)
    if not os.path.isdir(target):
        print(f"[error] 图片文件夹不存在：{target}")
        return 2

    imgs = {os.path.splitext(f)[0]: f for f in os.listdir(target)
            if f.lower().endswith(IMG_EXT)}
    if not imgs:
        print(f"[error] {target} 里没有图片")
        return 2
    print(f"[info] 目标文件夹有 {len(imgs)} 张图")

    zpath = args.zip or find_newest_zip(args.downloads)
    if not zpath or not os.path.exists(zpath):
        print("[error] 没找到导出包。请先在 makesense.ai 里 Actions -> Export Labels")
        print(f"        下载目录：{args.downloads}")
        return 3
    print(f"[info] 使用导出包：{zpath}")
    print(f"       大小 {os.path.getsize(zpath)/1024:.0f} KB  时间 "
          f"{__import__('time').strftime('%Y-%m-%d %H:%M', __import__('time').localtime(os.path.getmtime(zpath)))}")

    tmp = tempfile.mkdtemp(prefix="mslbl_")
    try:
        with zipfile.ZipFile(zpath) as z:
            z.extractall(tmp)
        txts = []
        for root, _, files in os.walk(tmp):
            for f in files:
                if f.lower().endswith(".txt"):
                    txts.append(os.path.join(root, f))
        if not txts:
            print("[error] 压缩包里没有 .txt，可能导出格式选错了（要选 YOLO）")
            return 4

        matched, unmatched, empty = 0, [], 0
        for t in txts:
            stem = os.path.splitext(os.path.basename(t))[0]
            if stem.lower() in ("classes", "predefined_classes", "notes"):
                continue                      # makesense 顺带导出的说明文件
            if stem not in imgs:
                unmatched.append(stem)
                continue
            with open(t, "r", encoding="utf-8", errors="ignore") as fh:
                if not fh.read().strip():
                    empty += 1
            shutil.copy2(t, os.path.join(target, stem + ".txt"))
            matched += 1

        total_now = len([f for f in os.listdir(target) if f.lower().endswith(".txt")])
        print(f"\n[result] 压缩包里共 {len(txts)} 个标签文件")
        print(f"         匹配并放入 {matched} 个")
        print(f"         空标签     {empty} 个（纯背景图，正常）")
        print(f"         名字对不上 {len(unmatched)} 个")
        if unmatched:
            for s in unmatched[:5]:
                print(f"           - {s}（文件夹里没有同名图片）")
        print(f"\n[result] 该文件夹现在有 {total_now} 个 .txt / {len(imgs)} 张图")

        # 抽查一个标签的格式
        for f in os.listdir(target):
            if f.lower().endswith(".txt") and f.lower() != "classes.txt":
                with open(os.path.join(target, f), encoding="utf-8") as fh:
                    line = fh.readline().strip()
                parts = line.split()
                ok = len(parts) == 5 and parts[0].isdigit() or (
                    len(parts) == 5 and parts[0].replace('.', '', 1).isdigit())
                print(f"[check] 抽查 {f}: {line!r}")
                print(f"        {'格式 OK（class cx cy w h，归一化）' if ok else '格式可疑，应该 5 个数且第 1 个是类别 id'}")
                break

        if matched == 0:
            print("\n[error] 一个都没匹配上。最常见原因：导出的图片名和文件夹里的不一致。")
            print("        重新导出前，别在网站上改图片文件名。")
            return 5
        return 0
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
