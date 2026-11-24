#!/usr/bin/env python3

import os
import re
import sys

import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = os.path.join(ROOT, "config")

MARKER_TYPES = {"LCross", "TCross", "XCross", "PenaltyPoint"}
ALIAS = {"PenalityPoint": "PenaltyPoint"}

def load(name):
    path = os.path.join(CFG, name)
    if not os.path.exists(path):
        return None
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)

def check_camera():
    d = load("camera.yaml")
    if not d:
        return False, "缺少 config/camera.yaml"
    try:
        K = np.array(d["camera_matrix"], dtype=float)
    except Exception:
        return False, "camera_matrix 格式不对（应为 3x3）"
    if K.shape != (3, 3):
        return False, f"camera_matrix 形状是 {K.shape}，应为 (3,3)"
    if K[0, 0] <= 0 or K[1, 1] <= 0:
        return False, "fx / fy 必须大于 0"
    dist = np.array(d.get("dist_coeffs", []), dtype=float).ravel()
    if dist.size < 4:
        return False, "dist_coeffs 至少要有 4 个数"
    rms = float(d.get("rms", 0.0))
    if rms <= 0:
        return False, "rms 为 0，说明还没真正标定（运行 calibrate_camera.py）"
    if rms > 2.0:
        return True, f"已标定，但重投影误差偏大：rms={rms:.2f} px（建议重拍）"
    return True, f"已标定，rms={rms:.2f} px，分辨率 {d.get('image_width')}x{d.get('image_height')}"

def check_extrinsics():
    d = load("camera_extrinsics.yaml")
    if not d:
        return False, "缺少 config/camera_extrinsics.yaml"
    R = np.array(d.get("R_base_camera", []), dtype=float)
    t = np.array(d.get("t_base_camera", []), dtype=float).ravel()
    if R.shape != (3, 3):
        return False, "R_base_camera 应为 3x3"
    if t.size != 3:
        return False, "t_base_camera 应为 3 个数"
    if np.allclose(R, np.eye(3)) and np.allclose(t, 0):
        return False, "R 和 t 还是占位值，需要实测外参"
    h = float(d.get("camera_height_m", 0.0))
    if h <= 0:
        return False, "camera_height_m 必须大于 0（相机离地高度）"
    return True, f"外参已填，相机高度 {h:.2f} m，俯仰 {d.get('pitch_deg')}°，滚转 {d.get('roll_deg')}°"

def check_field():
    path = os.path.join(CFG, "field.yaml")
    if not os.path.exists(path):
        return False, "缺少 config/field.yaml"
    if os.path.getsize(path) == 0:
        return False, "field.yaml 是空文件，先用 scripts/build_field_map.py 建图"
    d = load("field.yaml")
    if not d:
        return False, "field.yaml 内容为空，先用 scripts/build_field_map.py 建图"

    markers = d.get("markers")
    if isinstance(markers, list) and markers:
        bad = []
        types = set()
        for m in markers:
            if not isinstance(m, dict) or "type" not in m or "x" not in m or "y" not in m:
                bad.append(m)
                continue
            t = ALIAS.get(m["type"], m["type"])
            if t not in MARKER_TYPES:
                bad.append(m)
            types.add(t)
        if bad:
            return False, f"有 {len(bad)} 个标志点格式不对或类别不认识"
        return True, f"{len(markers)} 个标志点（简化格式），类别：{', '.join(sorted(types))}"

    pattern = re.compile(r"^(LCross|TCross|XCross|PenalityPoint|PenaltyPoint)\d*$")
    types = set()
    count = 0
    for key, val in d.items():
        if not isinstance(val, (list, tuple)) or len(val) != 2:
            continue
        m = pattern.match(str(key))
        if not m:
            continue
        count += 1
        types.add(ALIAS.get(m.group(1), m.group(1)))
    if count:
        return True, f"{count} 个标志点（官方扁平格式），类别：{', '.join(sorted(types))}"
    return False, "既没有 markers 列表，也不是官方扁平格式；请把官方 field.yaml 放进来或改解析"

def check_geometry():
    d = load("field_geometry.yaml")
    if not d:
        return False, "缺少 config/field_geometry.yaml（官方提供，记录原点/轴向/尺寸）"
    size = d.get("field_size") or {}
    if float(size.get("length_m", 0)) <= 0 or float(size.get("width_m", 0)) <= 0:
        return False, "field_size 里的 length_m / width_m 还是 0，需要按实测填"
    return True, f"场地 {size.get('length_m')} x {size.get('width_m')} m，单位 {d.get('unit')}"

def main():
    checks = [
        ("camera.yaml", check_camera),
        ("camera_extrinsics.yaml", check_extrinsics),
        ("field.yaml", check_field),
        ("field_geometry.yaml", check_geometry),
    ]
    ok_all = True
    for name, fn in checks:
        try:
            ok, msg = fn()
        except Exception as e:
            ok, msg = False, f"读取失败：{e}"
        flag = "[就绪]" if ok else "[待补]"
        print(f"{flag:6} {name:28} {msg}")
        ok_all = ok_all and ok

    print()
    if ok_all:
        print("配置齐全，可以跑 locate_field.py")
        return 0
    print("还有待补项；占位值下运行定位会得到没有意义的坐标。")
    return 1

if __name__ == "__main__":
    raise SystemExit(main())
