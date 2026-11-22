import sys, math, os, yaml
import numpy as np
from pathlib import Path

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

REPO = Path(__file__).resolve().parent.parent
DATA = Path(os.environ.get("W34_DATA", REPO.parents[4] / "week34_data"))
FIELD = REPO / "config" / "field.yaml"

ALIAS = {"PenalityPoint": "PenaltyPoint"}

MEASURED = {
    "P1": {
        "optics": {"TCross4": 0.0, "TCross3": 0.0, "LCross2": 0.0},
        "dir": {"TCross4": 0.0, "TCross3": 0.0, "LCross2": 0.0},
        "optics_to_dir": 0.0,
    },
}


def load_field():
    raw = yaml.safe_load(FIELD.read_text(encoding="utf-8"))
    pts = {}
    for k, v in raw.items():
        name = ALIAS.get(k, k)
        pts[name] = np.array([float(v[0]), float(v[1])])
    return pts


def trilaterate(pts_xy, dists):
    names = [n for n in dists if dists[n] > 0 and n in pts_xy]
    if len(names) < 3:
        return None, len(names)
    ref = names[0]
    A, b = [], []
    for n in names[1:]:
        fi, fj = pts_xy[ref], pts_xy[n]
        A.append(2 * (fj - fi))
        b.append(float(np.dot(fj, fj) - np.dot(fi, fi) + dists[ref] ** 2 - dists[n] ** 2))
    A = np.array(A, dtype=float)
    b = np.array(b, dtype=float)
    sol, res, rank, sv = np.linalg.lstsq(A, b, rcond=None)
    err = float(np.sqrt(np.mean((A @ sol - b) ** 2))) if len(A) > 2 else 0.0
    return sol, err


def report(pt, pts_xy, data):
    O, errO = trilaterate(pts_xy, data["optics"])
    M, errM = trilaterate(pts_xy, data["dir"])
    print(f"\n===== {pt} =====")
    if O is None:
        print("  光心投影点定位失败：需要至少 3 个非零距离")
        return
    print(f"  光心地面投影 O = ({O[0]:.3f}, {O[1]:.3f}) m   三边残差 {errO:.3f}")
    if M is None:
        print("  方向点 M 定位失败，yaw 无法计算")
        return
    print(f"  方向点 M       = ({M[0]:.3f}, {M[1]:.3f}) m   三边残差 {errM:.3f}")
    d = M - O
    yaw = math.degrees(math.atan2(d[1], d[0])) % 360
    print(f"  |O->M| 实测 {data['optics_to_dir']:.3f} m / 坐标算 {float(np.linalg.norm(d)):.3f} m")
    print(f"  yaw = {yaw:.2f}°（从地图 +X 逆时针）")
    return {"x": round(float(O[0]), 3), "y": round(float(O[1]), 3), "yaw_deg": round(yaw, 2),
            "trilat_err_O": round(errO, 3), "trilat_err_M": round(errM, 3)}


def main():
    pts_xy = load_field()
    print(f"载入 field.yaml：{len(pts_xy)} 个特征点")
    print("可用点名:", ", ".join(sorted(pts_xy)))
    rows = {}
    for pt, data in MEASURED.items():
        r = report(pt, pts_xy, data)
        if r:
            rows[pt] = r
    if rows:
        out = Path(__file__).resolve().parent.parent / "calibration" / "testpoints_reference.yaml"
        out.write_text(yaml.safe_dump({"note": "由 field.yaml 特征点三边定位得到，独立于算法输出",
                                       "field_file": FIELD.name, "points": rows},
                                      sort_keys=False, allow_unicode=True), encoding="utf-8")
        print("\n写入", out)
    else:
        print("\nMEASURED 里还没填实测值，先填距离再跑")


if __name__ == "__main__":
    main()
