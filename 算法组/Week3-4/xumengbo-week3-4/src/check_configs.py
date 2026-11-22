"""校验 config/ 与 calibration/ 下所有 YAML 可解析且关键字段在位。"""
import os
import yaml

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = os.path.join(BASE, "config")
CAL = os.path.join(BASE, "calibration")

EXPECT = {
    "field_geometry.yaml": ["map_version", "field"],
    "field_theory.yaml": ["LCross1", "TCross1", "XCross1", "PenalityPoint"],
    "field.yaml": ["LCross1", "TCross1", "XCross1", "PenalityPoint"],
    "calib_board.yaml": ["pattern_cols", "pattern_rows", "square_size_m", "image_dir"],
    "camera_intrinsics.yaml": ["camera_matrix", "dist_coeffs", "reprojection_error"],
}


def main():
    ok = True
    for fn, keys in EXPECT.items():
        p = os.path.join(CFG, fn)
        if not os.path.exists(p):
            print(f"[缺文件] {fn}")
            ok = False
            continue
        with open(p, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        miss = [k for k in keys if k not in data]
        if miss:
            print(f"[字段缺] {fn}: {miss}")
            ok = False
        else:
            print(f"[OK] {fn}")

    p = os.path.join(CAL, "camera_extrinsics.yaml")
    if os.path.exists(p):
        with open(p, "r", encoding="utf-8") as f:
            yaml.safe_load(f)
        print("[OK] camera_extrinsics.yaml")
    else:
        print("[缺文件] camera_extrinsics.yaml")
        ok = False

    print("\n校验通过" if ok else "\n存在问题，请检查")


if __name__ == "__main__":
    main()
