#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ros_live.py —— ROS2 版实时演示（live_demo 的 ROS2 封装）
窗口与 live_demo 相同：左=相机画面+足球/标志检测，右=地图位姿+状态栏
同时发布 ROS2 话题:
  /soccer/ball              std_msgs/String        JSON: [{conf,bbox,center_u,center_v},...]
  /localization/field_pose  geometry_msgs/Pose2D   x,y(米), theta=yaw(度); 仅在解出位姿时发布
  /localization/status      std_msgs/String        JSON: {valid,marker_count,residual,status,fps}

用法(终端需先 source ROS):
  source /opt/ros/humble/setup.bash
  /root/yolo_env/bin/python tools/ros_live.py                 # 真实相机
  /root/yolo_env/bin/python tools/ros_live.py --video x.mp4   # 无相机彩排
另开一个终端看话题:
  source /opt/ros/humble/setup.bash
  ros2 node list
  ros2 topic list
  ros2 topic echo /localization/field_pose
  ros2 topic echo /localization/status
按 q 退出。
"""
import argparse, json, os, sys, time
import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import localize
import live_demo as ld          # 复用状态栏绘制

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String

BALL_MODEL = os.path.expanduser(
    "~/soccer_ws/soccer_dataset/runs/detect/my_train/real_ball_test/weights/best.pt")
BALL_COLOR = (0, 200, 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam", type=int, default=0)
    ap.add_argument("--cam-w", type=int, default=320, help="相机采集宽(实测 320 最稳)")
    ap.add_argument("--cam-h", type=int, default=240, help="相机采集高")
    ap.add_argument("--keep-bad", action="store_true", help="不丢弃截断/纯色坏帧(调试用)")
    ap.add_argument("--video", default=None, help="彩排用视频(不能替代实时验收)")
    ap.add_argument("--conf", type=float, default=0.15)
    ap.add_argument("--ball-conf", type=float, default=0.25)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--no-window", action="store_true")
    args = ap.parse_args()

    K, dist, W, H = localize.load_intrinsics()
    mp = localize.load_map()

    from ultralytics import YOLO
    ball = YOLO(BALL_MODEL) if os.path.exists(BALL_MODEL) else None
    if ball is None:
        print("!! 足球模型不存在, /soccer/ball 将发布空列表")

    # ---------------- ROS2 ----------------
    rclpy.init()
    node = Node("soccer_vision_node")
    pub_pose = node.create_publisher(Pose2D, "/localization/field_pose", 10)
    pub_ball = node.create_publisher(String, "/soccer/ball", 10)
    pub_status = node.create_publisher(String, "/localization/status", 10)
    print("ROS2 节点已启动: /soccer_vision_node")
    print("话题: /soccer/ball  /localization/field_pose  /localization/status")

    # ---------------- 视频源 ----------------
    if args.video:
        cap = cv2.VideoCapture(args.video)
        print(f"[彩排模式] 视频输入: {args.video}")
    else:
        cap = cv2.VideoCapture(args.cam)
        # ⚠️ usbipd 转发限制: MJPG 大帧会被截断(下半幅全黑), YUYV 拿不到真实数据(纯绿).
        #    实测 320x240 MJPG 可 100% 完整传输.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.cam_w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.cam_h)
        print(f"[实时模式] 相机 /dev/video{args.cam}  请求 {args.cam_w}x{args.cam_h}")
    if not cap.isOpened():
        print("!! 打不开相机/视频"); node.destroy_node(); rclpy.shutdown(); sys.exit(1)

    # 相机实际分辨率可能低于标定的 1280x720 -> 按比例缩放内参(水平FOV不变)
    ok0, probe = cap.read()
    if not ok0:
        print("!! 读不到帧"); node.destroy_node(); rclpy.shutdown(); sys.exit(1)
    fh, fw = probe.shape[:2]
    if (fw, fh) != (W, H):
        s = fw / float(W)
        K = K.copy()
        K[0, 0] *= s; K[1, 1] *= s
        K[0, 2] = fw / 2.0; K[1, 2] = fh / 2.0
        print(f"[内参适配] 相机实际 {fw}x{fh}, 缩放系数 {s:.3f} -> "
              f"fx={K[0,0]:.1f} fy={K[1,1]:.1f} cx={K[0,2]:.0f} cy={K[1,2]:.0f}")
        W, H = fw, fh

    fps, nframe, n_bad = 0.0, 0, 0
    t_prev = time.time(); t_start = t_prev

    def frame_ok(f):
        """坏帧检测: usbipd 截断(底部全黑)或纯色占位帧(绿屏)一律丢弃"""
        if f is None or f.size == 0:
            return False
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        h = g.shape[0]
        if float(f.std()) < 12:
            return False
        rows = (g.max(axis=1) < 8)
        nb = 0
        for y in range(h - 1, -1, -1):
            if rows[y]:
                nb += 1
            else:
                break
        return nb / h < 0.20

    try:
        while True:
            if not rclpy.ok():          # 被 Ctrl+C / kill 后干净退出, 不再发布
                break
            ok, frame = cap.read()
            if not ok:
                break
            if not args.keep_bad and not args.video and not frame_ok(frame):
                n_bad += 1
                continue
            if frame.shape[1] != W or frame.shape[0] != H:
                frame = cv2.resize(frame, (W, H))

            # ---- 任务1: 足球 ----
            balls = []
            if ball is not None:
                rb = ball.predict(frame, imgsz=640, conf=args.ball_conf,
                                  device="cpu", verbose=False)[0]
                for b in rb.boxes:
                    x1, y1, x2, y2 = [int(v) for v in b.xyxy[0].tolist()]
                    cf = float(b.conf[0])
                    cu, cvv = (x1 + x2) // 2, (y1 + y2) // 2
                    cv2.rectangle(frame, (x1, y1), (x2, y2), BALL_COLOR, 2)
                    cv2.circle(frame, (cu, cvv), 3, BALL_COLOR, -1)
                    cv2.putText(frame, f"ball {cf:.2f}", (x1, max(y1 - 18, 12)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, BALL_COLOR, 1)
                    cv2.putText(frame, f"({cu},{cvv})", (x1, max(y1 - 6, 24)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, BALL_COLOR, 1)
                    balls.append({"conf": round(cf, 3), "bbox": [x1, y1, x2, y2],
                                  "center_u": cu, "center_v": cvv})

            # ---- 任务2: 标志 + 定位 ----
            sol, dets, obs = localize.localize_image(
                frame, K, dist, mp, W, H, conf=args.conf, tol=0.20,
                refine=True, imgsz=args.imgsz)

            # ---- 发布话题 ----
            mb = String(); mb.data = json.dumps(balls)
            pub_ball.publish(mb)

            solved = sol["status"] not in ("insufficient", "out_of_field")
            if solved:                     # 只在有解时发位姿, 绝不发伪造位姿
                pm = Pose2D()
                pm.x = float(sol["x"]); pm.y = float(sol["y"])
                pm.theta = float(sol["yaw"])
                pub_pose.publish(pm)

            ms = String()
            ms.data = json.dumps({
                "valid": bool(sol["valid"]),
                "marker_count": int(sol["marker_count"]),
                "residual": (None if sol["status"] == "insufficient"
                             else round(float(sol["residual"]), 4)),
                "status": sol["status"],
                "solved": solved,
            })
            pub_status.publish(ms)

            # ---- 可视化 ----
            left = localize.draw_camera_view(frame, dets, sol)
            right = localize.draw_map_view(mp, sol, obs)
            right = ld.draw_status(right, sol, fps, len(balls))
            combo = localize.combine(left, right)

            dt = time.time() - t_prev; t_prev = time.time()
            fps = 1.0 / dt if fps == 0 else 0.9 * fps + 0.1 * (1.0 / max(dt, 1e-6))
            nframe += 1
            rclpy.spin_once(node, timeout_sec=0)

            print(f"frame {nframe:5d} FPS={fps:4.1f} ball={len(balls)} "
                  f"x={sol['x']:+7.2f} y={sol['y']:+7.2f} yaw={sol['yaw']:+7.1f} "
                  f"n={sol['marker_count']} [{sol['status']}]", flush=True)

            if not args.no_window:
                disp = combo
                if combo.shape[1] > 1560:
                    sc = 1560.0 / combo.shape[1]
                    disp = cv2.resize(combo, (int(combo.shape[1] * sc),
                                              int(combo.shape[0] * sc)))
                cv2.imshow("ros_live  (q=quit)", disp)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            if args.max_frames and nframe >= args.max_frames:
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        elapsed = time.time() - t_start
        if nframe:
            print(f"共处理 {nframe} 帧, 平均 FPS = {nframe / elapsed:.2f}"
                  + (f"  (丢弃坏帧 {n_bad} 帧)" if n_bad else ""))
        else:
            print(f"无可用帧 (丢弃坏帧 {n_bad} 个)。试试 --cam-w 352 --cam-h 288")


if __name__ == "__main__":
    main()