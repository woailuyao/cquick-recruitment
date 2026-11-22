# -*- coding: utf-8 -*-
"""手动点选标志点：绕过检测器，验证外参与定位链路。

为什么需要它：
    当前 marker 模型的训练集几乎全是“近距离、同一机位”的帧，
    在 66 cm / 13.3° 机位、3–5 m 距离上置信度只有 0.02–0.05，
    属于典型的外观域差，靠调阈值和对比度增强难以弥补。
    重训要几小时，所以现场先用“人眼点 4 个点”把几何链路跑通。

用法 1：标外参（至少 4 点，鼠标点十字交点）
    python manual_points.py --mode pnp --image D:\\vision_data\\diag\\frame.jpg ^
        --order TCross2,TCross1,LCross3,LCross1
    点完按回车/空格结束，右键撤销上一个点，Esc 取消。
    结果写到 docs/extrinsic_points.yaml，并直接打印 PnP 解算值。

用法 2：算位姿（至少 3 点，需要已有外参或命令行给角度）
    python manual_points.py --mode locate --image D:\\vision_data\\diag\\frame.jpg ^
        --order LCross3,TCross2,TCross1 --height 0.66 --pitch-down 13.3 --roll 0
"""

import argparse
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
sys.path.insert(0, HERE)

from field_map import FieldMap, PoseSolver, normalize_type      # noqa: E402
from ground_geometry import CameraModel, Extrinsics             # noqa: E402


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["pnp", "locate"], default="pnp")
    ap.add_argument("--image", required=True)
    ap.add_argument("--order", required=True,
                    help="点击顺序，逗号分隔，如 TCross2,TCross1,LCross3,LCross1")
    ap.add_argument("--field", default=os.path.join(ROOT, "config", "field_measured_near.yaml"))
    ap.add_argument("--intrinsics",
                    default=os.path.join(ROOT, "calibration", "camera_intrinsics.yaml"))
    ap.add_argument("--extrinsics",
                    default=os.path.join(ROOT, "calibration", "camera_extrinsics.yaml"))
    ap.add_argument("--height", type=float, help="overrides: 相机光心离地高度 m")
    ap.add_argument("--pitch-down", type=float, default=0.0, help="overrides: 俯角，向下为正")
    ap.add_argument("--roll", type=float, default=0.0, help="overrides: 滚转角")
    ap.add_argument("--out", default=os.path.join(ROOT, "docs", "extrinsic_points.yaml"))
    ap.add_argument("--no-write", action="store_true")
    return ap.parse_args()


def pick_points(img, names):
    """左键加点，右键撤销，回车/空格结束，Esc 取消。"""
    pts = []

    def canvas():
        d = img.copy()
        for i, (u, v) in enumerate(pts):
            cv2.circle(d, (u, v), 7, (0, 0, 255), -1)
            cv2.putText(d, f"{i + 1}:{names[i]}", (u + 9, v - 9),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        hint = f"{len(pts)}/{len(names)}  left=add  right=undo  Enter=done  Esc=cancel"
        cv2.putText(d, hint, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 255, 255), 2)
        return d

    def on_mouse(event, x, y, flags, _):
        if event == cv2.EVENT_LBUTTONDOWN and len(pts) < len(names):
            pts.append((x, y))
        elif event == cv2.EVENT_RBUTTONDOWN and pts:
            pts.pop()

    cv2.namedWindow("pick", cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback("pick", on_mouse)
    while True:
        cv2.imshow("pick", canvas())
        k = cv2.waitKey(20) & 0xFF
        if k in (13, 10, 32):
            break
        if k == 27:
            pts.clear()
            break
    cv2.destroyAllWindows()
    return pts


def write_points_yaml(path, items):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    lines = ["# 手动点选的已知标志（像素为原图坐标，未去畸变）", "points:"]
    for name, (u, v), (mx, my) in items:
        lines.append(f"  - type: {name}")
        lines.append(f"    pixel: [{u:.1f}, {v:.1f}]")
        lines.append(f"    map: [{mx:.3f}, {my:.3f}]")
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")


def main():
    args = parse_args()
    img = cv2.imread(args.image)
    if img is None:
        raise SystemExit(f"读不到图片: {args.image}")
    print(f"图片 {img.shape[1]}x{img.shape[0]}")

    names = [s.strip() for s in args.order.split(",") if s.strip()]
    field = FieldMap(args.field)
    table = {m["id"]: m for m in field.markers}
    for n in names:
        if n not in table:
            raise SystemExit(f"field.yaml 里没有 {n}。可选: {', '.join(sorted(table))}")

    need = 4 if args.mode == "pnp" else 3
    if len(names) < need:
        raise SystemExit(f"{args.mode} 模式至少需要 {need} 个点，当前 --order 给了 {len(names)} 个")

    pts = pick_points(img, names)
    if len(pts) < need:
        raise SystemExit(f"只点了 {len(pts)} 个点，{args.mode} 模式至少需要 {need} 个")
    pts = pts[:len(names)]

    items = []
    print("\n序号  名称        像素(u,v)          地图(x,y)")
    for name, (u, v) in zip(names, pts):
        m = table[name]
        items.append((name, (u, v), (m["x"], m["y"])))
        print(f"{len(items):>3}  {name:<10} ({u:6.1f},{v:6.1f})   "
              f"({m['x']:.3f},{m['y']:.3f})")

    cam = CameraModel(args.intrinsics)

    if args.mode == "pnp":
        if not args.no_write:
            write_points_yaml(args.out, items)
            print(f"\n已写入 {os.path.abspath(args.out)}")
            print(f"复算: python src\\calibrate_extrinsics.py --mode pnp --points \"{args.out}\" --dry-run")
        from calibrate_extrinsics import solve_pnp
        height, pitch, roll, yaw, rms, n, center = solve_pnp(cam, args.out) \
            if not args.no_write else (None,) * 7
        if height is not None:
            print(f"\nPnP: 高度 {height:.3f} m  俯角 {pitch:.2f}°  滚转 {roll:.2f}°")
            print(f"     重投影误差 {rms:.3f} px  （标定时航向 {yaw:.1f}°，不写入外参）")
            print(f"     相机地图坐标 ({center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f})")
            if rms > 3.0:
                print("     警告: 重投影误差偏大，检查点的对应关系或像素坐标是否点准")
        return

    # locate 模式
    if args.height is not None:
        ext = Extrinsics.from_angles(args.height, args.pitch_down, args.roll,
                                     source="命令行")
    else:
        ext = Extrinsics.from_yaml(args.extrinsics)
        if ext is None:
            raise SystemExit("外参未标定，先 --mode pnp 标定，或给 --height --pitch-down --roll")
    print(f"\n外参: h={ext.height:.3f} m  pitch={ext.pitch_down:.2f}°  "
          f"roll={ext.roll:.2f}°  ({ext.method})")

    uv = np.array([[u, v] for _, (u, v) in zip(names, pts)], dtype=float)
    ground = ext.project_to_ground(uv, cam)
    print("\n像素 -> 载体地面坐标:")
    for (name, (u, v), _map), xy, ok, d in zip(items, ground["points"],
                                               ground["valid"], ground["dist"]):
        s = f"({xy[0]:6.2f},{xy[1]:6.2f})  距离 {d:5.2f} m" if ok else "  无效（射线不落地）"
        print(f"  {name:<10} ({u:6.1f},{v:6.1f}) -> {s}")

    good = [i for i in range(len(names)) if ground["valid"][i]]
    if len(good) < 3:
        raise SystemExit("有效地面点不足 3 个，无法解位姿")

    types = [table[names[i]]["type"] for i in good]
    xy = ground["points"][good]
    solver = PoseSolver(field, min_inliers=3)
    res = solver.solve(types, xy)
    print(f"\n定位结果: valid={res['valid']}  status={res['status']}  "
          f"markers={res['marker_count']}")
    if res["valid"]:
        print(f"  x={res['x']:.3f} m  y={res['y']:.3f} m  yaw={res['yaw']:.2f}°  "
              f"residual={res['residual']:.3f} m")
    else:
        print(f"  reason={res.get('reason')}")


if __name__ == "__main__":
    main()
