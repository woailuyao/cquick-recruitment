#!/usr/bin/env python3
"""实时分屏主程序：左 RGB（足球 + 场地标志）+ 右 field.yaml 地图定位。

用法：
  python apps/soccer_vision.py --camera 0
  python apps/soccer_vision.py --video data/test.mp4 --no-refine
  python apps/soccer_vision.py --camera 0 --ros2 --headless
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

import cv2                                    # noqa: E402
import numpy as np                            # noqa: E402

from cvlib.config import (                    # noqa: E402
    load_extrinsics, load_intrinsics, load_matching, sanity_check,
)
from cvlib.fieldmap import load_field         # noqa: E402
from cvlib.filter import PoseFilter           # noqa: E402
from cvlib.geom import angle_diff_deg, compose_pose, wrap_deg  # noqa: E402
from cvlib.matcher import match               # noqa: E402
from cvlib.perception import (                # noqa: E402
    YoloDetector, observations_from_detections,
)
from cvlib.ui import MapView, compose_split, draw_left, put  # noqa: E402


from cvlib.camera import open_capture             # noqa: E402  (Linux/Windows 通用)


def screen_size():
    """取屏幕的"逻辑"分辨率（不声明 DPI-aware，Windows 会返回缩放后的值）。

    你的屏幕是 2880x1920 物理、200% 缩放，逻辑分辨率 1440x960，
    窗口超过逻辑宽度就会被切掉。
    """
    try:
        import ctypes
        u = ctypes.windll.user32
        return int(u.GetSystemMetrics(0)), int(u.GetSystemMetrics(1))
    except Exception:
        return 1920, 1080


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="CQuick C0 单目检测 + 场地自定位")
    p.add_argument("--camera", default="0", help="相机索引或 /dev/videoX")
    p.add_argument("--video", default=None, help="用视频文件代替相机（调试用）")
    p.add_argument("--field", default=os.path.join(ROOT, "config", "field.yaml"))
    p.add_argument("--geometry", default=os.path.join(ROOT, "config", "field_geometry.yaml"))
    p.add_argument("--intrinsics", default=os.path.join(ROOT, "calibration", "camera_intrinsics.yaml"))
    p.add_argument("--extrinsics", default=os.path.join(ROOT, "calibration", "camera_extrinsics.yaml"))
    p.add_argument("--cfg", default=os.path.join(ROOT, "config", "matching.yaml"))
    p.add_argument("--conf-ball", type=float, default=None)
    p.add_argument("--conf-marker", type=float, default=None)
    p.add_argument("--device", default=None)
    p.add_argument("--no-refine", action="store_true", help="关闭标志几何细化（对比用）")
    p.add_argument("--no-ball", action="store_true", help="关闭足球检测")
    p.add_argument("--no-marker", action="store_true", help="关闭场地标志检测（只做任务1）")
    p.add_argument("--view", default="split", choices=["split", "ball"],
                   help="split=左右分屏；ball=只显示相机画面（任务1 录屏用）")
    p.add_argument("--ros2", action="store_true", help="同时发布 ROS2 话题")
    p.add_argument("--headless", action="store_true", help="不显示窗口（配合 --ros2）")
    p.add_argument("--max-frames", type=int, default=0)
    p.add_argument("--log", default=None, help="JSONL 日志路径")
    p.add_argument("--save-video", default=None, help="把分屏结果写成 mp4（调试/留证）")
    return p


def merge_obs(buf, dedup_m: float = 0.25, max_obs: int = 12):
    """把多帧的观测按"同类别 + 位置接近"去重合并。

    单帧检测会抖动（同一个标志有时检出、有时漏检），把最近几帧合并后
    观测数才稳定，定位才不会一闪而过。
    """
    merged = []
    for group in buf:
        for o in group:
            dup = False
            for u in merged:
                if u.type == o.type and math.hypot(o.x - u.x, o.y - u.y) < dedup_m:
                    dup = True
                    if o.confidence > u.confidence:      # 留置信度高的那个
                        u.x, u.y = o.x, o.y
                        u.sigma, u.weight = o.sigma, o.weight
                        u.confidence, u.pixel = o.confidence, o.pixel
                        u.distance = o.distance
                    break
            if not dup:
                merged.append(o)
    if len(merged) > max_obs:
        merged = sorted(merged, key=lambda o: -o.confidence)[:max_obs]
    return merged


def main() -> int:
    args = build_parser().parse_args()
    cfg = load_matching(args.cfg)
    dcfg = cfg.get("detector", {})
    rcfg = dict(cfg.get("refine", {}))
    gcfg = cfg.get("geometry", {})
    fcfg = cfg.get("filter", {})
    if args.no_refine:
        rcfg["enabled"] = False

    intr = load_intrinsics(args.intrinsics)
    extr = load_extrinsics(args.extrinsics)
    sanity_check(intr, extr)

    field = load_field(args.field, args.geometry)
    print("[map]", field.summary())
    if field.meta.get("map_version"):
        print("[map] version:", field.meta.get("map_version"))

    W, H = intr["width"], intr["height"]
    src = args.video if args.video else args.camera
    cap = open_capture(src, W, H, 30)
    if cap is None or not cap.isOpened():
        print(f"[error] 无法打开视频源 {src}")
        return 2
    got_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    got_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[cam] source={src} requested={W}x{H} actual={got_w}x{got_h} "
          f"fps={cap.get(cv2.CAP_PROP_FPS):.1f}")
    if (got_w, got_h) != (W, H):
        print("[warn] 实际分辨率与标定分辨率不一致，将缩放到标定尺寸；"
              "正确做法是让相机输出标定分辨率")

    ball_det = None
    # 自动选择推理设备：配置里写的是 "0"（GPU），但机器可能只有 CPU
    dev = args.device or str(dcfg.get("device", "0"))
    if dev.isdigit():
        try:
            import torch
            if not torch.cuda.is_available():
                print(f"[cfg] 没有可用 GPU（torch {torch.__version__}），自动改用 device=cpu")
                dev = "cpu"
        except Exception:
            dev = "cpu"

    if not args.no_ball:
        ball_det = YoloDetector(dcfg.get("ball_model", ""), 
                                conf=args.conf_ball or float(dcfg.get("conf_ball", 0.35)),
                                iou=float(dcfg.get("iou", 0.45)),
                                imgsz=int(dcfg.get("imgsz", 640)),
                                device=dev)
    marker_det = YoloDetector(dcfg.get("marker_model", ""),
                              conf=args.conf_marker or float(dcfg.get("conf_marker", 0.25)),
                              iou=float(dcfg.get("iou", 0.45)),
                              imgsz=int(dcfg.get("imgsz", 640)),
                              device=dev)
    if args.no_marker:
        marker_det.enabled = False

    pf = PoseFilter(float(fcfg.get("ema_alpha_xy", 0.5)),
                    float(fcfg.get("ema_alpha_yaw", 0.5)),
                    int(fcfg.get("max_frames_hold", 15)))
    view = MapView(field)

    # 观测累积缓冲：单帧检测会抖动（同一标志有时检出有时漏检），
    # 把最近几帧的观测按位置去重后合并再匹配，站位不动时观测数才稳定。
    obs_buf: deque = deque(maxlen=int(fcfg.get("obs_window", 4)))

    # 时间一致性门控参数：新解与当前轨迹差太多就不采纳，避免画面闪烁
    jump_xy = float(fcfg.get("jump_xy_m", 0.8))
    jump_yaw = float(fcfg.get("jump_yaw_deg", 25.0))
    jump_reinit = int(fcfg.get("jump_reinit_frames", 6))
    n_jump = 0
    ros = None
    if args.ros2:
        try:
            from cvlib.ros2_pub import Ros2Bridge
            ros = Ros2Bridge(frame_id="map")
        except Exception as e:
            print(f"[warn] ROS2 初始化失败，继续离线运行: {e}")

    log_f = None
    if args.log:
        os.makedirs(os.path.dirname(os.path.abspath(args.log)), exist_ok=True)
        log_f = open(args.log, "w", encoding="utf-8")

    writer = None
    if args.save_video:
        os.makedirs(os.path.dirname(os.path.abspath(args.save_video)), exist_ok=True)
        writer = None    # 首帧确定尺寸后再建

    # 相机外参的平面部分：相机光心相对载体的 (x, y, yaw)
    bc = (float(extr["t"][0]), float(extr["t"][1]), math.radians(extr["yaw_offset_deg"]))

    fps_hist: deque = deque(maxlen=30)
    last_loc_t = 0.0
    loc_hz = 0.0
    n_valid = 0
    n_frames = 0
    t_start = time.time()

    print("[run] 按 q 退出, r 重新初始化, s 保存截图, d 打印当前详细状态")
    try:
        while True:
            t0 = time.perf_counter()
            ok, frame = cap.read()
            if not ok:
                print("[warn] 读帧失败（相机断开或视频结束）")
                break
            if (frame.shape[1], frame.shape[0]) != (W, H):
                frame = cv2.resize(frame, (W, H))

            t_inf0 = time.perf_counter()
            ball_dets = ball_det(frame) if ball_det else []
            if dcfg.get("single_ball", False) and len(ball_dets) > 1:
                # 单球假设：只保留置信度最高的一个框，压掉球门网等误检
                ball_dets = [max(ball_dets, key=lambda d: d.conf)]
            marker_dets = marker_det(frame)
            infer_ms = (time.perf_counter() - t_inf0) * 1000.0

            obs, dbg = observations_from_detections(
                frame, marker_dets, intr, extr, gcfg, rcfg, field)

            # 累积最近几帧的观测再匹配，抑制单帧抖动
            if obs:
                obs_buf.append(obs)
            obs_use = merge_obs(obs_buf) if len(obs_buf) > 1 else obs

            prior = None if pf.lost else pf.state
            res = match(obs_use, field.points, cfg, prior=prior)

            # 时间一致性门控：只有 2 个内点时位姿在数学上必然完美拟合，
            # 但可能在几个等价解之间来回跳。要求新解与当前轨迹接近才接受，
            # 否则保持不动（等下一帧或换位置），这样画面才不会闪。
            accept = res.valid
            if accept and pf.state is not None:
                dxy = math.hypot(res.x - pf.state[0], res.y - pf.state[1])
                dyaw = abs(angle_diff_deg(math.degrees(res.theta),
                                          math.degrees(pf.state[2])))
                if dxy > jump_xy or dyaw > jump_yaw:
                    n_jump += 1
                    accept = False
                    if n_jump <= 3:
                        print(f"[track] 位姿跳变 {dxy:.2f} m / {dyaw:.1f}° "
                              f"-> 本帧不采纳（继续观察）")
                else:
                    n_jump = 0

            # 时间一致性门控之后：连续跳变多次就认为换位置了，允许重新收敛
            if n_jump >= jump_reinit:
                print(f"[track] 连续 {n_jump} 帧与轨迹不符，认为已移动到新位置，重新定位")
                pf.reset()
                n_jump = 0
                res2 = match(obs_use, field.points, cfg, prior=None)
                if res2.valid:
                    res = res2
                    accept = True

            pf.update(res.pose, accept)
            base_pose = pf.state                    # 滤波后位姿；失效超时后为 None
            cam_pose = compose_pose(base_pose, bc) if base_pose is not None else None
            if res.valid and accept:
                n_valid += 1
                now = time.time()
                if last_loc_t > 0:
                    inst = 1.0 / max(now - last_loc_t, 1e-3)
                    loc_hz = inst if loc_hz <= 0 else 0.9 * loc_hz + 0.1 * inst
                last_loc_t = now

            n_frames += 1
            fps_hist.append(time.perf_counter())
            fps = (len(fps_hist) - 1) / max(fps_hist[-1] - fps_hist[0], 1e-6) if len(fps_hist) > 1 else 0.0

            info = {
                "frame_id": n_frames,
                "width": W, "height": H, "fps": fps, "infer_ms": infer_ms,
                "n_ball": len(ball_dets), "n_obs": len(obs_use),
                "n_obs_frame": len(obs),
                "n_refined": sum(1 for e in dbg if e["method"] != "bbox_center"),
                "marker_count": res.marker_count,
                "residual": res.residual,
                "valid": accept, "status": res.status,
                "n_hypotheses": res.n_hypotheses,
                "rival": res.rival,
                "loc_hz": loc_hz,
                "x_camera_map": cam_pose[0] if cam_pose else float("nan"),
                "y_camera_map": cam_pose[1] if cam_pose else float("nan"),
                "yaw_camera_map": wrap_deg(math.degrees(cam_pose[2])) if cam_pose else float("nan"),
            }

            if ros is not None:
                if cam_pose is not None:
                    ros.publish_pose(cam_pose, res.valid, res.marker_count,
                                     res.residual, res.status, pf.state or res.pose)
                ros.publish_balls(ball_dets)

            if log_f is not None:
                rec = dict(info)
                rec["t"] = time.time()
                rec["inlier_ids"] = res.inlier_ids
                rec["obs"] = [{"type": o.type, "x": o.x, "y": o.y,
                               "d": o.distance, "sigma": o.sigma, "id": o.map_id}
                              for o in obs]
                log_f.write(json.dumps(rec, ensure_ascii=False) + "\n")

            if not args.headless:
                left = draw_left(frame, ball_dets, dbg, info)
                if args.view == "ball":
                    combo = left                      # 任务1 录屏：只看相机画面
                else:
                    right = view.render(pf.state, cam_pose, info)
                    combo = compose_split(left, right)
                # 窗口不能超出屏幕，否则右侧地图被切掉
                sw, sh = screen_size()
                MAXW, MAXH = max(900, min(sw - 80, 1380)), max(560, sh - 200)
                if combo.shape[1] > MAXW or combo.shape[0] > MAXH:
                    sc = min(MAXW / float(combo.shape[1]), MAXH / float(combo.shape[0]))
                    combo = cv2.resize(combo, (int(combo.shape[1] * sc),
                                               int(combo.shape[0] * sc)))
                if n_frames == 1:
                    print(f"[ui] 屏幕 {sw}x{sh} -> 显示窗口 "
                          f"{combo.shape[1]}x{combo.shape[0]}（左 RGB + 右地图）")
                if writer is None and args.save_video:
                    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                    writer = cv2.VideoWriter(args.save_video, fourcc, 20.0,
                                             (combo.shape[1], combo.shape[0]))
                if writer is not None:
                    writer.write(combo)
                cv2.imshow("CQuick C0 | left: RGB  right: field map", combo)
                k = cv2.waitKey(1) & 0xFF
                if k == ord("q"):
                    break
                if k == ord("r"):
                    pf.reset()
                    print("[run] 位姿先验已重置")
                if k == ord("s"):
                    path = os.path.join(ROOT, "logs", f"shot_{n_frames:05d}.jpg")
                    os.makedirs(os.path.dirname(path), exist_ok=True)
                    cv2.imwrite(path, combo)
                    print("[run] saved", path)
                if k == ord("d"):
                    print(json.dumps(info, ensure_ascii=False, indent=2))
                    for o in obs:
                        print(f"   obs {o.type:12s} ({o.x:6.2f},{o.y:6.2f}) "
                              f"D={o.distance:.2f}m sigma={o.sigma:.3f} id={o.map_id}")

            if args.max_frames and n_frames >= args.max_frames:
                break
            _ = t0
    finally:
        cap.release()
        if writer is not None:
            writer.release()
        if log_f is not None:
            log_f.close()
        if ros is not None:
            ros.shutdown()
        if not args.headless:
            cv2.destroyAllWindows()

    dt = time.time() - t_start
    print(f"\n[done] frames={n_frames} 用时={dt:.1f}s 平均={n_frames/max(dt,1e-6):.1f} fps "
          f"定位有效={n_valid}/{n_frames} ({100.0*n_valid/max(n_frames,1):.1f}%)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
