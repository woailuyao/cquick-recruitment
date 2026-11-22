# -*- coding: utf-8 -*-
"""从一张场地照片自动搜索外参 (高度 h, 俯角 pitch, 滚转 roll=0)。

原理:
    外参决定了每个标志像素投影到地面的位置。给定一组 (h, pitch)，投影出的
    地面点如果与 field.yaml 的某个 SE(2) 位姿吻合（标志数 ≥4、残差小），
    这组外参就是可疑的；在可行解里取残差最小的那组。

适用条件:
    - 画面里至少能看到 4 个 L/T 标志，且分布不要都在一条直线上；
    - 相机与测试架固定好之后拍这张照片，标定完不要再动相机；
    - 相机尽量端平（roll 先按 0 处理）。

用法:
    python calibrate_extrinsics_auto.py --image D:/field_calib.jpg
    python calibrate_extrinsics_auto.py --image D:/field_calib.jpg --write

建议先用手机水平仪量一个粗值，再用本脚本复核；两者偏差较大时以现场
手工值为准，并检查点的对应关系是否正确。
"""

import argparse
import os

import numpy as np

from field_map import FieldMap, PoseSolver, normalize_type
from ground_geometry import CameraModel, Extrinsics
from marker_refine import refine_pixel

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
DEFAULT_INTR = os.path.join(ROOT, "calibration", "camera_intrinsics.yaml")
DEFAULT_OUT = os.path.join(ROOT, "calibration", "camera_extrinsics.yaml")
DEFAULT_MODEL = os.path.join(ROOT, "weights", "marker_v3_best.pt")


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", required=True)
    ap.add_argument("--model", default=DEFAULT_MODEL)
    ap.add_argument("--intrinsics", default=DEFAULT_INTR)
    ap.add_argument("--field", default=os.path.join(ROOT, "config", "field_measured_near.yaml"))
    ap.add_argument("--conf", type=float, default=0.30)
    ap.add_argument("--imgsz", type=int, default=1280,
                    help="推理分辨率，训练时是 960；远处小标志用 1280/1920")
    ap.add_argument("--device", default="0")
    ap.add_argument("--min-markers", type=int, default=4)
    ap.add_argument("--height", type=float,
                    help="已知相机高度（卷尺量），固定高度只搜俯角，结果更稳")
    ap.add_argument("--write", action="store_true", help="写入 camera_extrinsics.yaml")
    ap.add_argument("--out", default=DEFAULT_OUT)
    return ap.parse_args()


def evaluate(cam, field, solver, types, uvs, height, pitch):
    ext = Extrinsics.from_angles(height, pitch, 0.0)
    proj = ext.project_to_ground(uvs, cam)
    idx = [i for i in range(len(uvs)) if proj["valid"][i]]
    if len(idx) < 4:
        return None
    res = solver.solve([types[i] for i in idx], proj["points"][idx])
    return res


def main():
    args = parse_args()
    import cv2
    from ultralytics import YOLO

    cam = CameraModel(args.intrinsics)
    field = FieldMap(args.field)
    solver = PoseSolver(field)
    model = YOLO(args.model)

    frame = cv2.imread(args.image)
    if frame is None:
        raise SystemExit(f"读不到图片: {args.image}")
    r = model.predict(frame, imgsz=args.imgsz, conf=args.conf, verbose=False,
                      device=args.device)[0]

    types, uvs, names = [], [], []
    for box, cf, ci in zip(r.boxes.xyxy.cpu().numpy(),
                           r.boxes.conf.cpu().numpy(),
                           r.boxes.cls.cpu().numpy()):
        name = r.names[int(ci)]
        mtype = normalize_type(name)
        if mtype is None:
            continue
        u, v, ok, how = refine_pixel(frame, box, mtype)
        types.append(mtype)
        uvs.append([u, v])
        names.append((name, float(cf), ok, how))
        print(f"检出 {name:7s} conf={cf:.2f} pixel=({u:.1f},{v:.1f}) refined={ok} {how}")
    uvs = np.array(uvs, dtype=float).reshape(-1, 2)
    if len(uvs) < args.min_markers:
        raise SystemExit(f"只检出 {len(uvs)} 个标志，至少需要 {args.min_markers} 个。"
                         "请换个能看到更多标志的位置重拍。")

    fixed_h = args.height
    if fixed_h is not None:
        print(f"固定高度 h={fixed_h:.3f} m，只搜索俯角")
        h_grid = [float(fixed_h)]
    else:
        h_grid = np.arange(0.3, 1.85, 0.1)

    candidates = []
    for h in h_grid:
        for p in np.arange(5.0, 45.1, 2.5):
            res = evaluate(cam, field, solver, types, uvs, h, p)
            if res is None or res["marker_count"] < args.min_markers:
                continue
            if not np.isfinite(res["residual"]) or res["residual"] > 0.30:
                continue
            candidates.append((res["residual"], h, p, res))
    if not candidates:
        raise SystemExit("没有找到与外参一致、且匹配 4 个以上标志的解。\n"
                         "可能原因: 标志太少/共线、点位不全、地面角度超出搜索范围。")

    candidates.sort(key=lambda c: (c[0], -c[3]["marker_count"]))
    rms, h0, p0, res0 = candidates[0]

    # 局部细化
    best = (rms, h0, p0, res0)
    if fixed_h is not None:
        h_refine = [float(fixed_h)]
    else:
        h_refine = np.arange(max(0.2, h0 - 0.12), h0 + 0.12, 0.02)
    for h in h_refine:
        for p in np.arange(max(1.0, p0 - 3.0), p0 + 3.0, 0.5):
            res = evaluate(cam, field, solver, types, uvs, h, p)
            if res is None or res["marker_count"] < args.min_markers:
                continue
            if not np.isfinite(res["residual"]):
                continue
            if res["residual"] < best[0]:
                best = (res["residual"], h, p, res)

    rms, h, p, res = best
    print("\n候选解（按残差排序，前 5 组）:")
    for rms_i, h_i, p_i, res_i in candidates[:5]:
        print(f"  h={h_i:.2f} m pitch={p_i:5.1f}°  n={res_i['marker_count']} "
              f"res={rms_i:.3f} m  pose=({res_i['x']:+.2f},{res_i['y']:+.2f},"
              f"{res_i['yaw']:+.1f}°)  {res_i['status']}")
    print(f"\n采用: h={h:.2f} m  pitch={p:.1f}°  "
          f"n={res['marker_count']}  res={rms:.3f} m  status={res['status']}")
    print(f"该位置的相机位姿: x={res['x']:+.2f} y={res['y']:+.2f} yaw={res['yaw']:+.1f}°")

    if args.write:
        import calibrate_extrinsics as ce
        ce.write_yaml(os.path.abspath(args.out), h, p, 0.0,
                      "单张图片自动搜索（4 个以上标志的 SE(2) 匹配）",
                      None, note=f"自动解残差 {rms:.3f} m，建议到现场再核对一次")
        print("已写入", os.path.abspath(args.out))
    else:
        print("加 --write 可写入 camera_extrinsics.yaml")


if __name__ == "__main__":
    main()