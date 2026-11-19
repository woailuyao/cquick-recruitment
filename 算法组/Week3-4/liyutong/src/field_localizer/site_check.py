# -*- coding: utf-8 -*-
"""现场勘测：实时显示「这个机位能不能解出位姿」。

为什么需要它
    重录 gt 素材时，最大的风险是"录完回家才发现这个机位一个点都检不到"。
    这个脚本把验收提前到现场：右下角直接告诉你当前机位
      OK        -> 能解，可以录
      NO_MARKERS-> 点不够，走近点或把相机俯仰加大
      NO_MATCH  -> 点检到了但和地图配不上，通常是外参/高度不对

用法
    python site_check.py                 # 用摄像头 0
    python site_check.py --cam 1
    python site_check.py --source videos/xxx.mp4   # 先用已有视频演练一遍

判据（现场照这个走）
    status=OK 且 marker_count>=3 且至少跨 2 个类别 -> 这个机位有效，开录
    status=OK 但只有同一类 2 个点                 -> 能解但有 180° 二义，尽量再挪一点
    其余                                          -> 换机位

按键
    q 退出   s 存当前帧
"""
import argparse
import time

import cv2
import numpy as np

import localize as L


def put(img, text, org, scale=0.55, color=(255, 255, 255), thick=1):
    """cv2.putText 不支持中文，且草地/白线切换时纯色字会看不清，先描黑边。"""
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick + 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam", type=int, default=0)
    ap.add_argument("--source", default=None, help="给视频路径就回放，不给就用摄像头")
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--model", default="runs_marker/baseline/weights/best.pt")
    ap.add_argument("--height", type=float, default=None, help="覆盖相机高度（米）")
    ap.add_argument("--probe", type=int, default=0,
                    help="不开窗口，只处理 N 帧并在终端打印状态（出门前自检用）")
    args = ap.parse_args()

    from ultralytics import YOLO

    if not __import__("os").path.exists(args.model):
        raise SystemExit(f"找不到标志模型 {args.model}")
    mk = YOLO(args.model)
    K, dist = L.load_intrinsics()
    R_gc, h_extr = L.load_extrinsics()
    markers, goal, fld = L.load_field()
    field_dims = (float(fld["length_m"]), float(fld["width_m"]))
    height_m = args.height if args.height else h_extr
    print(f"相机高度 {height_m:.3f} m  conf {args.conf}  "
          f"类别 {mk.names}")

    cap = (cv2.VideoCapture(args.source) if args.source
           else cv2.VideoCapture(args.cam, cv2.CAP_DSHOW))
    if not cap.isOpened():
        raise SystemExit("视频源打不开；摄像头编号用 camera_check.py 确认")
    if not args.source:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    prev = None
    n_ok = n_all = 0
    t0, nfr, fps = time.time(), 0, 0.0
    print("按键：q 退出 / s 存当前帧")
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if frame.shape[1] != 640 or frame.shape[0] != 480:
            frame = cv2.resize(frame, (640, 480))

        r = mk.predict(frame, conf=args.conf, imgsz=args.imgsz, verbose=False)[0]
        det, shown = [], []
        for b in r.boxes:
            u = float((b.xyxy[0][0] + b.xyxy[0][2]) / 2)
            v = float((b.xyxy[0][1] + b.xyxy[0][3]) / 2)
            c = int(b.cls[0])
            det.append((mk.names[c], u, v))
            shown.append((c, float(b.conf[0]), *[int(x) for x in b.xyxy[0]]))

        res = L.localize(det, K, dist, R_gc, height_m, markers, goal,
                         field_dims=field_dims, prev=prev)
        if res["valid"]:
            prev = dict(x=res["x_camera_map"], y=res["y_camera_map"],
                        yaw=res["yaw_camera_map"])
        elif res["status"] in ("NO_MARKERS", "NO_MATCH"):
            prev = None

        n_all += 1
        if res["valid"]:
            n_ok += 1

        vis = frame.copy()
        # 画面中心十字丝：把某个已知标志点对到这儿，参考 yaw 就能解析算出来
        cv2.line(vis, (320 - 22, 240), (320 + 22, 240), (0, 255, 255), 1)
        cv2.line(vis, (320, 240 - 22), (320, 240 + 22), (0, 255, 255), 1)
        cv2.circle(vis, (320, 240), 7, (0, 255, 255), 1)
        for c, cf, x1, y1, x2, y2 in shown:
            col = [(0, 165, 255), (0, 210, 0), (255, 0, 255)][c]
            cv2.rectangle(vis, (x1, y1), (x2, y2), col, 2)
            put(vis, f"{mk.names[c]} {cf:.2f}", (x1, max(12, y1 - 5)), 0.45, col, 1)
            cv2.drawMarker(vis, ((x1 + x2) // 2, (y1 + y2) // 2), col,
                           cv2.MARKER_CROSS, 10, 1)

        # 顶部：状态大字
        ok_now = res["valid"]
        cls_set = sorted({d[0] for d in det})
        bar = np.zeros((70, vis.shape[1], 3), np.uint8)
        state = "OK  ->  RECORD THIS" if ok_now else res["status"]
        put(bar, state, (8, 26), 0.8, (0, 230, 0) if ok_now else (0, 0, 255), 2)
        put(bar, f"markers {len(det)}  types {''.join(cls_set) or '-'}  "
                 f"count {res['marker_count']}  valid {ok_now}", (8, 54), 0.55,
            (0, 255, 255), 1)
        put(bar, f"FPS {fps:.1f}   OK {n_ok}/{n_all}", (vis.shape[1] - 190, 26),
            0.5, (200, 200, 200), 1)
        if not ok_now and res["reason"]:
            put(bar, res["reason"][:60], (vis.shape[1] - 420, 54), 0.45, (150, 150, 255), 1)
        if ok_now:
            put(bar, f"({res['x_camera_map']:+.2f}, {res['y_camera_map']:+.2f}) "
                     f"yaw {res['yaw_camera_map']:+.1f}  res {res['residual']:.1f}px",
                (8, 66), 0.45, (0, 255, 0), 1)
        vis = np.vstack([bar, vis])

        nfr += 1
        if nfr % 10 == 0:
            fps = 10.0 / max(1e-6, time.time() - t0)
            t0 = time.time()

        if args.probe:
            if nfr % max(1, args.probe // 10) == 0 or nfr == 1:
                print(f"  帧{nfr:4d}  markers={len(det)} types={''.join(cls_set) or '-'}  "
                      f"status={res['status']:<10} valid={ok_now}  "
                      f"{'' if not ok_now else '(%+.2f,%+.2f) yaw%+.1f' % (res['x_camera_map'], res['y_camera_map'], res['yaw_camera_map'])}")
            if nfr >= args.probe:
                break
            continue

        cv2.imshow("site_check  q=quit  s=snapshot", vis)
        k = cv2.waitKeyEx(1)
        if k in (ord("q"), ord("Q"), 27):
            break
        if k in (ord("s"), ord("S")):
            p = time.strftime("site_%H%M%S.jpg")
            cv2.imwrite(p, frame)
            print("已保存", p)

    cap.release()
    cv2.destroyAllWindows()
    print(f"\n本轮 {n_all} 帧，有效解 {n_ok} 帧（{100.0 * n_ok / max(n_all, 1):.0f}%）")
    if n_ok == 0:
        print("这个机位不行。按顺序试：① 走近标志点（<2 m） ② 相机俯仰加大 ③ 换个有 2 类以上标志点的位置")


if __name__ == "__main__":
    main()
