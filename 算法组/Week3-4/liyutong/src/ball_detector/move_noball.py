# -*- coding: utf-8 -*-
"""
把 prelabel_noball.txt 里列出的「画面中确实没有球」的帧移出 raw/，
移到 raw_无球帧/<段>/ 保留（不删除），避免数据集里混入无标注图。

用法：python move_noball.py
"""
import os
import shutil

RAW = "raw"
DST = "raw_无球帧"
LIST = "prelabel_noball.txt"


def main():
    if not os.path.isfile(LIST):
        print(f"找不到 {LIST}，先跑 python prelabel_ball.py")
        return
    with open(LIST, encoding="utf-8") as fp:
        items = [l.strip() for l in fp if l.strip() and not l.startswith("#")]
    print(f"待移出 {len(items)} 张无球帧")

    moved = 0
    for it in items:
        seg, name = it.split("/", 1)
        src_img = os.path.join(RAW, seg, name)
        if not os.path.isfile(src_img):
            continue
        # 该帧的预标注 .txt 一并移走（有框的说明预标注误检了，也一起走）
        stem = os.path.splitext(name)[0]
        dst_dir = os.path.join(DST, seg)
        os.makedirs(dst_dir, exist_ok=True)
        shutil.move(src_img, os.path.join(dst_dir, name))
        for ext in (".txt",):
            s = os.path.join(RAW, seg, stem + ext)
            if os.path.isfile(s):
                shutil.move(s, os.path.join(dst_dir, stem + ext))
        moved += 1

    print(f"已移动 {moved} 张到 {DST}/（原图仍在，只是不在训练集里）")
    print("\n各段剩余：")
    for seg in sorted(os.listdir(RAW)):
        d = os.path.join(RAW, seg)
        if os.path.isdir(d):
            print(f"  {seg}: {len([f for f in os.listdir(d) if f.endswith('.jpg')])} 张")


if __name__ == "__main__":
    main()
