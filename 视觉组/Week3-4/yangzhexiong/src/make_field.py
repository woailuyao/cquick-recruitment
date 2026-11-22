"""把卷尺实测数据变成 field.yaml。

测量方法（推荐，最不容易错）：
    以「我方底线 × 左边线」的交点作为量测原点：
      along  = 从我方底线往球门方向的水平距离（米）
      across = 从左边线往右的水平距离（米）

    转换到地图坐标（原点在场地中心，+X 指向球门，+Y 指向左侧）：
      x = along - L/2
      y = W/2 - across

用法：
    1. 复制模板：cp measure_template.csv measure.csv
    2. 用文本编辑器按实测填写
    3. python3 make_field.py --length 8.0 --width 6.0 --csv measure.csv
    4. 生成 config/field.yaml 和 outputs/field_preview.png

CSV 列：id,type,along_m,across_m
    type 只能是 LCross / TCross / XCross / PenaltyPoint
"""
import argparse
import csv
import os

import yaml


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--length", type=float, required=True, help="场地长（米），沿 X")
    ap.add_argument("--width", type=float, required=True, help="场地宽（米），沿 Y")
    ap.add_argument("--csv", default="measure.csv")
    ap.add_argument("--out", default="config/field.yaml")
    ap.add_argument("--date", default="")
    ap.add_argument("--author", default="yangzhexiong")
    args = ap.parse_args()

    if not os.path.exists(args.csv):
        print(f"找不到 {args.csv}")
        print("先复制模板：cp measure_template.csv measure.csv")
        return 1

    L, W = args.length, args.width
    markers = []
    with open(args.csv, "r", encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            mid = (row.get("id") or "").strip()
            typ = (row.get("type") or "").strip()
            if not mid or mid.startswith("#"):
                continue
            if not typ:
                continue
            try:
                along = float(row["along_m"])
                across = float(row["across_m"])
            except (KeyError, ValueError, TypeError):
                print(f"跳过 {mid}：along_m / across_m 不是数字")
                continue
            x = round(along - L / 2, 3)
            y = round(W / 2 - across, 3)
            markers.append({"id": mid, "type": typ, "x": x, "y": y, "yaw": 0.0})

    if len(markers) < 3:
        print(f"有效标志只有 {len(markers)} 个，至少需要 3 个")
        return 1

    data = {
        "meta": {
            "version": 1,
            "unit": "meter",
            "frame_id": "map",
            "origin_description": ("原点位于场地几何中心，X 轴正方向指向球门，"
                                   "Y 轴正方向指向面向 X 时的左侧，Z 轴向上，地面 Z=0"),
            "measured_by": args.author,
            "measured_date": args.date,
            "note": "本组场地非标准尺寸，坐标为卷尺实测；未直接使用官方 field.yaml",
        },
        "field": {
            "half_length": round(L / 2, 3),
            "half_width": round(W / 2, 3),
            "length": L,
            "width": W,
            "boundary": {
                "x_min": round(-L / 2, 3), "x_max": round(L / 2, 3),
                "y_min": round(-W / 2, 3), "y_max": round(W / 2, 3),
            },
        },
        "markers": markers,
        "constraints": {"min_marker_distance": 0.5},
    }

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)

    print(f"场地 {L} x {W} m，共 {len(markers)} 个标志：\n")
    print(f"{'id':<18}{'type':<14}{'along':>8}{'across':>8}{'-> x':>8}{'y':>8}")
    for m, row in zip(markers, range(len(markers))):
        print(f"{m['id']:<18}{m['type']:<14}"
              f"{m['x'] + L / 2:>8.3f}{W / 2 - m['y']:>8.3f}"
              f"{m['x']:>8.3f}{m['y']:>8.3f}")
    print(f"\n已写入 {args.out}")

    # 顺手校验 + 画图
    try:
        import field_map
        fm = field_map.load(args.out)
        problems, warns, _ = field_map.validate(fm)
        if problems:
            print("\n【错误】")
            for p in problems:
                print("  ✗", p)
        if warns:
            print("\n【提醒】")
            for w in warns:
                print("  !", w)
        if not problems and not warns:
            print("校验通过")
        out = field_map.plot(fm)
        print(f"俯视图：{out}")
    except Exception as exc:  # noqa: BLE001
        print(f"（校验/画图跳过：{exc}）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
