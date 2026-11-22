#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
live_demo.py —— 实时一体化演示（考核验收界面）
左侧: 相机实时画面 + 足球检测框(任务1) + 场地标志检测框/特征点(任务2)
右侧: field.yaml 场地俯视图 + 相机位置/朝向箭头 + 状态栏
      (x_camera_map / y_camera_map / yaw_camera_map / marker_count / residual / valid / status / FPS)

用法:
  实时相机(需先把 USB 相机接入 WSL, 见 README):
    /root/yolo_env/bin/python tools/live_demo.py
  指定相机节点:
    /root/yolo_env/bin/python tools/live_demo.py --cam 0
  无相机彩排(用录像文件走通界面, 不能替代实时验收):
    /root/yolo_env/bin/python tools/live_demo.py --video xxx.mp4
  同时录制合成画面:
    /root/yolo_env/bin/python tools/live_demo.py --save live.avi
按 q 退出。
"""
import argparse, os, sys, time
import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import localize

BALL_MODEL = os.path.expanduser(
    "~/soccer_ws/soccer_dataset/runs/detect/my_train/real_ball_test/weights/best.pt")
BALL_COLOR = (0, 200, 0)        # BGR 绿


def draw_status(map_canvas, sol, fps, n_ball):
    """右侧地图下方叠加状态栏"""
    S = map_canvas.shape[1]
    bar_h = 150
    overlay = map_canvas.copy()
    cv2.rectangle(overlay, (0, S - bar_h), (S, S), (20, 20, 24), -1)
    cv2.addWeighted(overlay, 0.72, map_canvas, 0.28, 0, map_canvas)
    ins = sol["status"] in ("insufficient", "out_of_field")
    fx = "--" if ins else f"{sol['x']:+.2f} m"
    fy = "--" if ins else f"{sol['y']:+.2f} m"
    fyaw = "--" if ins else f"{sol['yaw']:+.1f} deg"
    fres = "--" if ins else f"{sol['residual']:.3f} m"
    lines = [
        (f"x_camera_map  = {fx}", (255, 255, 255)),
        (f"y_camera_map  = {fy}", (255, 255, 255)),
        (f"yaw_camera_map= {fyaw}", (255, 255, 255)),
        (f"marker_count={sol['marker_count']}  residual={fres}", (200, 200, 200)),
        (f"valid={str(sol['valid']).lower()}  status={sol['status']}",
         (0, 255, 0) if sol["valid"] else (0, 160, 255)),
        (f"FPS={fps:.1f}  balls={n_ball}", (200, 200, 200)),
    ]
    y = S - bar_h + 24
    for txt, c in lines:
        cv2.putText(map_canvas, txt, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.52, c, 1)
        y += 22
    return map_canvas


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam", type=int, default=0, help="相机索引(/dev/videoN 的 N)")
    ap.add_argument("--cam-w", type=int, default=320, help="相机采集宽(实测 320 最稳)")
    ap.add_argument("--cam-h", type=int, default=240, help="相机采集高")
    ap.add_argument("--keep-bad", action="store_true", help="不丢弃截断/纯色坏帧(调试用)")
    ap.add_argument("--video", default=None, help="彩排用视频文件(非实时验收)")
    ap.add_argument("--conf", type=float, default=0.15, help="标志检测阈值")
    ap.add_argument("--ball-conf", type=float, default=0.25, help="足球检测阈值")
    ap.add_argument("--imgsz", type=int, default=640, help="标志模型推理尺寸(fix5按640训练)")
    ap.add_argument("--save", default=None, help="同时把合成画面录成视频文件")
    ap.add_argument("--max-frames", type=int, default=0, help=">0 时只处理这么多帧(测试用)")
    ap.add_argument("--no-window", action="store_true", help="不弹窗(无显示环境测试用)")
    args = ap.parse_args()

    K, dist, W, H = localize.load_intrinsics()
    mp = localize.load_map()
    print(f"地图: {localize.MAP_FILE}  特征 { {k: len(v) for k, v in mp.items()} }")
    print(f"标志模型: {localize.MODEL}  (imgsz={args.imgsz}, conf={args.conf})")
    print(f"足球模型: {BALL_MODEL}")

    from ultralytics import YOLO
    if not os.path.exists(BALL_MODEL):
        print("!! 足球模型不存在, 只跑标志定位"); ball = None
    else:
        ball = YOLO(BALL_MODEL)

    if args.video:
        cap = cv2.VideoCapture(args.video)
        print(f"[彩排模式] 视频输入: {args.video} (不能替代实时相机验收)")
    else:
        cap = cv2.VideoCapture(args.cam)
        # ⚠️ usbipd 转发限制: MJPG 大帧会被截断(下半幅全黑), YUYV 则拿不到真实数据(纯绿).
        #    实测 320x240 MJPG 可 100% 完整传输; 更高的分辨率时好时坏.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.cam_w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.cam_h)
        print(f"[实时模式] 相机 /dev/video{args.cam}  请求 {args.cam_w}x{args.cam_h}")
    if not cap.isOpened():
        print("!! 打不开相机/视频。相机模式请先用 usbipd 把 USB 相机接入 WSL")
        sys.exit(1)

    # 相机实际分辨率可能低于标定的 1280x720 -> 按比例缩放内参(水平FOV不变)
    ok0, probe = cap.read()
    if not ok0:
        print("!! 相机读不到帧"); sys.exit(1)
    fh, fw = probe.shape[:2]
    if (fw, fh) != (W, H):
        s = fw / float(W)
        K = K.copy()
        K[0, 0] *= s; K[1, 1] *= s
        K[0, 2] = fw / 2.0; K[1, 2] = fh / 2.0
        print(f"[内参适配] 相机实际 {fw}x{fh}, 缩放系数 {s:.3f} -> "
              f"fx={K[0,0]:.1f} fy={K[1,1]:.1f} cx={K[0,2]:.0f} cy={K[1,2]:.0f}")
        W, H = fw, fh

    writer = None
    want_save = args.save is not None
    if want_save:
        print(f"录制 -> {args.save} (尺寸按首帧合成结果确定)")

    fps, nframe, n_bad = 0.0, 0, 0
    t_prev = time.time()
    t_start = t_prev

    def frame_ok(f):
        """坏帧检测: usbipd 截断(底部全黑)或纯色占位帧(绿屏)一律丢弃"""
        if f is None or f.size == 0:
            return False
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        h = g.shape[0]
        if float(f.std()) < 12:                      # 近乎纯色
            return False
        rows = (g.max(axis=1) < 8)
        nb = 0
        for y in range(h - 1, -1, -1):
            if rows[y]:
                nb += 1
            else:
                break
        return nb / h < 0.20                          # 底部不应有整片全黑

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if not args.keep_bad and not args.video and not frame_ok(frame):
            n_bad += 1
            continue                                  # 丢弃坏帧, 不污染检测与显示
        if frame.shape[1] != W or frame.shape[0] != H:
            frame = cv2.resize(frame, (W, H))

        # ---------- 任务1: 足球 ----------
        n_ball = 0
        if ball is not None:
            rb = ball.predict(frame, imgsz=640, conf=args.ball_conf,
                              device="cpu", verbose=False)[0]
            for b in rb.boxes:
                x1, y1, x2, y2 = [int(v) for v in b.xyxy[0].tolist()]
                cf = float(b.conf[0])
                cu, cv_ = (x1 + x2) // 2, (y1 + y2) // 2
                cv2.rectangle(frame, (x1, y1), (x2, y2), BALL_COLOR, 2)
                cv2.circle(frame, (cu, cv_), 3, BALL_COLOR, -1)
                cv2.putText(frame, f"ball {cf:.2f}", (x1, max(y1 - 18, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, BALL_COLOR, 1)
                cv2.putText(frame, f"({cu},{cv_})", (x1, max(y1 - 6, 24)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, BALL_COLOR, 1)
                n_ball += 1

        # ---------- 任务2: 标志 + 定位 ----------
        sol, dets, obs = localize.localize_image(
            frame, K, dist, mp, W, H, conf=args.conf, tol=0.20,
            refine=True, imgsz=args.imgsz)

        left = localize.draw_camera_view(frame, dets, sol)
        cv2.putText(left, f"{W}x{H}", (W - 110, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        right = localize.draw_map_view(mp, sol, obs)
        right = draw_status(right, sol, fps, n_ball)
        combo = localize.combine(left, right)

        if want_save and writer is None:          # 按实际合成尺寸懒初始化
            h, w = combo.shape[:2]
            writer = cv2.VideoWriter(args.save, cv2.VideoWriter_fourcc(*"mp4v"), 10.0, (w, h))
            print(f"录制尺寸 {w}x{h} -> {args.save}")

        dt = time.time() - t_prev
        t_prev = time.time()
        fps = 1.0 / dt if fps == 0 else 0.9 * fps + 0.1 * (1.0 / max(dt, 1e-6))
        nframe += 1
        print(f"frame {nframe:5d}  FPS={fps:4.1f}  ball={n_ball}  "
              f"x={sol['x']:+7.2f} y={sol['y']:+7.2f} yaw={sol['yaw']:+7.1f}  "
              f"n={sol['marker_count']} [{sol['status']}]", flush=True)

        if writer is not None:
            writer.write(combo)
        if not args.no_window:
            disp = combo
            if combo.shape[1] > 1560:            # 窗口太宽放不下屏幕时缩小显示(录制仍为原分辨率)
                sc = 1560.0 / combo.shape[1]
                disp = cv2.resize(combo, (int(combo.shape[1] * sc), int(combo.shape[0] * sc)))
            cv2.imshow("live demo  (q=quit)", disp)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        if args.max_frames and nframe >= args.max_frames:
            break

    cap.release()
    if writer is not None:
        writer.release()
        print(f"已保存: {args.save}")
    cv2.destroyAllWindows()
    elapsed = time.time() - t_start
    if nframe:
        print(f"共处理 {nframe} 帧, 耗时 {elapsed:.1f}s, 平均 FPS = {nframe / elapsed:.2f}"
              + (f"  (丢弃坏帧 {n_bad} 帧)" if n_bad else ""))
    else:
        print(f"无可用帧 (丢弃坏帧 {n_bad} 个)。相机可能被截断: 试试 "
              f"--cam-w 352 --cam-h 288")


if __name__ == "__main__":
    main()