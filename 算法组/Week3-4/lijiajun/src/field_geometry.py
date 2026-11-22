import math
from pathlib import Path

import numpy as np
import yaml

TOL = 1e-6


def load_yaml(path):
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def load_intrinsics(path):
    d = load_yaml(path)
    K = np.array(d["camera_matrix"], dtype=float)
    dist = np.array(d["dist_coeffs"], dtype=float).ravel()
    return K, dist, d


def load_extrinsics(path, allow_preset=False):
    d = load_yaml(path)
    if d.get("calibrated", False):
        return float(d["height_m"]), float(d["pitch_deg"]), float(d["roll_deg"]), d
    if not allow_preset:
        raise RuntimeError(
            f"{Path(path).name}: calibrated=false，h/pitch/roll 尚未在最终安装状态下测量。"
            "重装相机后必须重测；确实要用名义安装值先跑通，显式传 allow_preset=True")
    vals = [d.get(f"preset_{k}") for k in ("height_m", "pitch_deg", "roll_deg")]
    if any(v is None for v in vals):
        raise RuntimeError(f"{Path(path).name}: 没有 preset_* 可用")
    d = dict(d)
    d["assumed"] = True
    return float(vals[0]), float(vals[1]), float(vals[2]), d


def load_field_points(path):
    d = load_yaml(path)
    return {k: np.array([float(v[0]), float(v[1])]) for k, v in d.items()}


def load_frame(path):
    d = load_yaml(path)
    return dict(origin=np.array(d["frame"]["origin_in_field_yaml"], dtype=float),
                rotation=np.array(d["transform_from_field_yaml"]["rotation"], dtype=float),
                features={k: np.array(v, dtype=float) for k, v in d["features_official"].items()},
                raw=d)


def to_official(points, frame):
    return {k: frame["rotation"] @ (v - frame["origin"]) for k, v in points.items()}


def check_frame_consistency(points_yaml, frame):
    converted = to_official(points_yaml, frame)
    published = frame["features"]
    rows = []
    worst = 0.0
    for k in sorted(published):
        err = float(np.linalg.norm(converted[k] - published[k]))
        worst = max(worst, err)
        rows.append((k, converted[k], published[k], err))
    return rows, worst


def _group(points, axis, fixed_index):
    groups = {}
    for name, p in points.items():
        key = round(float(p[fixed_index]), 4)
        groups.setdefault(key, []).append(name)
    lines = []
    for key, members in groups.items():
        if len(members) < 2:
            continue
        members = sorted(members, key=lambda n: points[n][axis])
        lo = float(points[members[0]][axis])
        hi = float(points[members[-1]][axis])
        lines.append(dict(axis="H" if fixed_index == 1 else "V", fixed=key,
                          lo=lo, hi=hi, members=members))
    return lines


def derive_map_lines(points, extend=0.0):
    lines = []
    for axis, fixed_index in ((0, 1), (1, 0)):
        for ln in _group(points, axis, fixed_index):
            if ln["axis"] == "H":
                p0 = np.array([ln["lo"] - extend, ln["fixed"]])
                p1 = np.array([ln["hi"] + extend, ln["fixed"]])
                ang = 0.0
            else:
                p0 = np.array([ln["fixed"], ln["lo"] - extend])
                p1 = np.array([ln["fixed"], ln["hi"] + extend])
                ang = math.pi / 2
            ln = dict(ln)
            ln["p0"], ln["p1"], ln["angle"] = p0, p1, ang
            ln["fam"] = ln["axis"]
            ln["name"] = f"{'y' if ln['axis']=='H' else 'x'}={ln['fixed']:g}"
            lines.append(ln)
    return lines


def line_normal(line):
    d = line["p1"] - line["p0"]
    n = np.array([-d[1], d[0]])
    n = n / np.linalg.norm(n)
    return n, float(n @ line["p0"])


def summarize_map(points, lines):
    out = ["## 地图线（由 field.yaml 共线点反推）", ""]
    out.append("| 线 | 方向 | 范围 | 端点数 | 反推依据 |")
    out.append("|---|---|---|---|---|")
    for ln in sorted(lines, key=lambda l: (l["axis"], l["fixed"])):
        out.append(f"| {ln['name']} | {ln['axis']} | {ln['lo']:.3f} .. {ln['hi']:.3f} "
                   f"| {len(ln['members'])} | {', '.join(ln['members'])} |")
    used = {m for ln in lines for m in ln["members"]}
    iso = sorted(set(points) - used)
    out.append("")
    out.append(f"孤立点（无法定义线）：{', '.join(iso) if iso else '无'}")
    return "\n".join(out)
