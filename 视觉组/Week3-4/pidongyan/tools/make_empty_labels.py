"""给负样本图片生成同名空 .txt（YOLO 约定：空标签 = 图中无目标）。

为什么需要：split_dataset.py 只收「图 + 同名 txt」的样本。负样本不画框，
但没有 txt 就会被当作"未标注"直接丢弃，等于白白扔掉压误检率的素材。

用法：
  # 默认只处理 empty_field_ 开头的图（防止把漏标的球图误当负样本）
  python tools/make_empty_labels.py

  # 指定目录与前缀
  python tools/make_empty_labels.py --dir frames_pool --prefix empty_field

  # 干跑，只报告不动文件
  python tools/make_empty_labels.py --dry-run
"""
import argparse
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default="frames_pool")
    ap.add_argument("--prefix", default="empty_field",
                    help="只对匹配该前缀的图生成空标签（安全阀）")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    d = (ROOT / args.dir).resolve()
    if not d.is_dir():
        raise SystemExit(f"目录不存在：{d}")

    made, skipped_existing, skipped_nonempty, no_img = 0, 0, 0, 0
    for img in sorted(d.iterdir()):
        if img.suffix.lower() not in IMG_EXTS:
            continue
        if args.prefix and not img.stem.startswith(args.prefix):
            no_img += 1
            continue
        lbl = img.with_suffix(".txt")
        if lbl.exists():
            if lbl.stat().st_size > 0:
                skipped_nonempty += 1
            else:
                skipped_existing += 1
            continue
        if not args.dry_run:
            lbl.write_text("", encoding="utf-8")
        made += 1

    print(f"目录        ：{d}")
    print(f"前缀过滤    ：{args.prefix or '（无，处理全部图片）'}")
    print(f"被前缀跳过  ：{no_img} 张（非负样本，未触碰）")
    print(f"已有空标签  ：{skipped_existing} 张（跳过）")
    print(f"已有非空标签：{skipped_nonempty} 张（⚠ 这类图不该用本工具，已跳过，请人工检查）")
    print(f"{'将生成' if args.dry_run else '已生成'}空标签：{made} 张")

    total_txt = sum(1 for p in d.glob('*.txt'))
    total_img = sum(1 for p in d.iterdir() if p.suffix.lower() in IMG_EXTS)
    print(f"当前统计    ：图 {total_img} 张 / txt {total_txt} 个"
          f"（两者相等即可进入 split_dataset.py）")


if __name__ == "__main__":
    main()
