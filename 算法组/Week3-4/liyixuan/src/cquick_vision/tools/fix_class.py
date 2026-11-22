#!/usr/bin/env python3
"""批量修正 YOLO 标签里的类别 id（比如误按了别的类别键）。

用法：
  python tools\\fix_class.py --dir C:\\...\\marker --from 2 --to 1
  python tools\\fix_class.py --dir ... --from 2 --to 0 --dry-run
"""
from __future__ import annotations

import argparse
import glob
import os


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    ap.add_argument("--from", dest="src_cls", type=int, required=True)
    ap.add_argument("--to", dest="dst_cls", type=int, required=True)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.dir, "*.txt")))
    files = [f for f in files if os.path.basename(f).lower() != "classes.txt"]
    if not files:
        print(f"[error] {args.dir} 里没有 .txt 标签")
        return 2

    changed_files = 0
    changed_lines = 0
    for p in files:
        with open(p, "r", encoding="utf-8") as f:
            lines = f.readlines()
        new_lines = []
        touched = False
        for ln in lines:
            t = ln.split()
            if t and t[0] == str(args.src_cls):
                t[0] = str(args.dst_cls)
                new_lines.append(" ".join(t) + "\n")
                touched = True
                changed_lines += 1
            else:
                new_lines.append(ln)
        if touched:
            changed_files += 1
            print(f"  {os.path.basename(p)}")
            if not args.dry_run:
                with open(p, "w", encoding="utf-8") as f:
                    f.writelines(new_lines)

    print(f"\n[fix] 类别 {args.src_cls} -> {args.dst_cls}："
          f"{changed_files} 个文件、{changed_lines} 行"
          f"{'（dry-run，未写入）' if args.dry_run else '（已写入）'}")
    if changed_files == 0:
        print("[fix] 没有找到需要修改的标签")

    # 复查
    from collections import Counter
    cnt = Counter()
    for p in files:
        with open(p, "r", encoding="utf-8") as f:
            for ln in f:
                t = ln.split()
                if t:
                    cnt[t[0]] += 1
    print("[check] 现在的类别分布：",
          dict(sorted(cnt.items())), f"共 {sum(cnt.values())} 个实例")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
