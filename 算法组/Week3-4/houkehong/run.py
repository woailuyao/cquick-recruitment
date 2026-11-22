# -*- coding: utf-8 -*-
"""
统一入口

三种模式：

  # 1) 合成演示（无需相机 / 无需标定 / 无需数据集，用来验证几何与匹配是否正确）
  python run.py demo --seconds 10 --save demo.mp4

  # 2) 真实相机（需要先标定，并训练好模型）
  python run.py live --camera 0 \
      --ball-weights weights/ball_best.pt \
      --marker-weights weights/marker_best.pt \
      --intrinsics calibration/camera_intrinsics.yaml \
      --extrinsics calibration/camera_extrinsics.yaml \
      --field config/field.yaml --geometry config/field_geometry.yaml

  # 3) 离线视频
  python run.py video --video test.mp4 --marker-weights weights/marker_best.pt

通用参数： --no-gui 不弹窗  --json 每帧打印状态 JSON  --frames N 限制帧数
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from src.app import FpsMeter, Pipeline, YoloDetector          # noqa: E402
from src.config import (ConfigError, check_field, load_camera,  # noqa: E402
                        load_extrinsics, load_field)
from src.demo import SyntheticScene                          # noqa: E402
from src.geometry import CameraModel, Extrinsics, pose_delta  # noqa: E402
from src.matching import FieldMatcher                        # noqa: E402

ROOT = os.path.dirname(os.path.abspath(__file__))


def _p(*parts):
    return os.path.join(ROOT, *parts)


def default_camera_index(fallback: int = 1) -> int:
    """
    从 config/camera.yaml 读 camera.device_index。

    为什么不在代码里写死：Windows 下 OpenCV 的设备号会因插拔/换口而变，
    写死会在换设备号后打不开相机。用 scripts/camera_select.py --set N 固定一次即可。
    """
    try:
        import yaml
        with open(_p("config", "camera.yaml"), "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        v = (data.get("camera") or {}).get("device_index")
        if v is not None:
            return int(v)
    except Exception:
        pass
    return fallback


class StatusClient:
    """
    把每帧状态以 JSON 行推给 WSL2 里监听的 scripts/ros_bridge_node.py，
    由它转成 ROS2 话题。

    方向很重要：**Windows 主动连 WSL**，不是反过来。
    WSL2 默认 NAT 网络下，Windows 连 WSL 里监听的端口有自动转发；
    而 WSL 连 Windows 的端口会被防火墙/NAT 挡住（实测确认）。
    """

    def __init__(self, port: int = 5056, host: str = "127.0.0.1"):
        import socket
        self.host = host
        self.port = port
        self.sock = None
        self.last_try = 0.0
        self.sent = 0
        print(f"[ros] 将把状态推送到 {host}:{port}（WSL 里先跑 ros_bridge_node.py）")

    def poll(self):
        """未连接时按 2 秒间隔重试，不阻塞主循环。"""
        if self.sock is not None:
            return
        import socket
        now = time.time()
        if now - self.last_try < 2.0:
            return
        self.last_try = now
        try:
            s = socket.create_connection((self.host, self.port), timeout=0.5)
            s.settimeout(1.0)
            self.sock = s
            print(f"[ros] 已连接到转发节点 {self.host}:{self.port}")
        except OSError:
            pass

    def send(self, obj: dict):
        self.poll()
        if self.sock is None:
            return
        data = (json.dumps(obj, ensure_ascii=False) + "\n").encode("utf-8")
        try:
            self.sock.sendall(data)
            self.sent += 1
        except OSError:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None
            print("[ros] 转发节点断开，稍后自动重连")

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None


def _ball_payload(dets):
    balls = []
    for bbox, label, conf in dets:
        if label != "ball":
            continue
        x1, y1, x2, y2 = [float(v) for v in bbox]
        balls.append({"label": label, "confidence": float(conf),
                      "bbox": [x1, y1, x2, y2]})
    return {"_kind": "ball", "balls": balls}


_BACKENDS = {
    "msmf": cv2.CAP_MSMF,
    "dshow": cv2.CAP_DSHOW,
    "any": cv2.CAP_ANY,
}


def _open_one(index: int, backend: str, width: int, height: int, fps: float):
    """按给定后端顺序尝试打开某个设备号，成功返回 cap。"""
    order = []
    if backend in _BACKENDS:
        order.append(backend)
    for name in ("msmf", "dshow", "any"):
        if name not in order:
            order.append(name)

    for name in order:
        cap = cv2.VideoCapture(index, _BACKENDS[name])
        if not cap.isOpened():
            cap.release()
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        ok, frame = cap.read()
        if not ok or frame is None:
            cap.release()
            continue
        # 曝光拨回自动，避免上一次手动曝光把帧率拖到 3 fps
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if (aw, ah) != (width, height):
            print(f"[相机] 注意：请求 {width}x{height}，实际协商 {aw}x{ah}")
        print(f"[相机] index={index} backend={name.upper()}  {aw}x{ah}")
        return cap
    return None


def open_camera(index: int, backend: str = "msmf",
                width: int = 1280, height: int = 720, fps: float = 30.0,
                scan_fallback: bool = True, max_index: int = 4):
    """
    打开相机并协商分辨率。

    **后端默认用 MSMF，这是本机实测的结论**：
    OpenCV 的 DirectShow 后端在 1280x720 下只能跑到 ~10 fps（换 MJPG、改分辨率、
    调请求帧率都没用），换成 Media Foundation (MSMF) 立刻 30 fps，
    内置相机和外接 USB 相机都一样。所以千万别用 DSHOW。

    打开后会把曝光拨回自动：手动曝光值一旦被驱动记住，
    下次打开相机会一直很暗/很慢（实测掉到 3 fps），必须显式改回自动。

    scan_fallback：配置的设备号打不开时（插拔后编号变了很常见），
    自动扫描 0..max_index 找第一个能用的，而不是直接报错退出。
    """
    cap = _open_one(index, backend, width, height, fps)
    if cap is not None:
        return cap

    if not scan_fallback:
        print(f"[相机] index={index} 打不开")
        return cv2.VideoCapture()

    print(f"[相机] index={index} 打不开（插拔后设备号可能变了），自动扫描其他编号 ...")
    for alt in range(max_index + 1):
        if alt == index:
            continue
        cap = _open_one(alt, backend, width, height, fps)
        if cap is not None:
            print(f"[相机] 已回退到 index={alt}。"
                  f"建议运行 scripts/camera_select.py --set {alt} 把它固定下来")
            return cap

    print(f"[相机] 0..{max_index} 全部打不开。"
          f"检查：USB 是否插好 / 是否被别的程序占用 / Windows 隐私设置")
    return cv2.VideoCapture()


def build(cfg):
    field = load_field(cfg.field, cfg.geometry)
    warns = check_field(field)
    for w in warns:
        print(f"[地图警告] {w}")

    camera = load_camera(cfg.intrinsics)
    extr = load_extrinsics(cfg.extrinsics)

    matcher = FieldMatcher(field.features, gate=cfg.gate,
                           max_residual=cfg.max_residual,
                           min_inliers=cfg.min_inliers)
    print(f"[地图] {field.source_path}  version={field.version}  "
          f"特征 {len(field.features)} 个")
    counts = {t: len(v) for t, v in field.by_type().items()}
    print(f"[地图] 类型分布 {counts}")
    print(f"[相机] 内参 fx={camera.fx:.2f} fy={camera.fy:.2f} cx={camera.cx:.2f} "
          f"cy={camera.cy:.2f}  分辨率 {camera.width}x{camera.height}")
    print(f"[相机] 畸变 [k1,k2,p1,p2,k3] = {np.round(camera.dist, 6).tolist()}")
    # 外参：细则 2.4 要求录屏里能看到相机安装参数，所以把高度/俯仰/滚转都打出来
    _pitch = _roll = float("nan")
    try:
        import yaml as _yaml
        with open(os.path.join(ROOT, "config", "camera.yaml"), "r", encoding="utf-8") as f:
            _ex = _yaml.safe_load(f).get("extrinsics", {})
        _pitch = float(_ex.get("pitch_deg", float("nan")))
        _roll = float(_ex.get("roll_deg", 0.0))
    except Exception:
        pass
    print(f"[外参] 安装高度 {extr.camera_height:.3f} m  俯仰 {_pitch:.3f}°  "
          f"滚转 {_roll:.2f}°  yaw_offset "
          f"{math.degrees(math.atan2(extr.R[1, 2], extr.R[0, 2])):.1f}°")
    print(f"[外参] 地面平面 Z=0；相机光心在载体坐标系中的位置 "
          f"t=({extr.t[0]:.3f}, {extr.t[1]:.3f}, {extr.t[2]:.3f}) m")
    return field, camera, extr, matcher


def make_trajectory(field):
    """
    按地图实际范围生成一条"沿内圈绕行 + 始终朝向场心"的轨迹。

    不写死坐标，这样换成官方 field.yaml（原点在场内、坐标有负数）也能直接用。

    为什么这样走：相机水平视场约 ±37°，站在边线附近往场内看时，
    对面边线上的同类标志会同时落进视野，正好构成可唯一求解的观测组。
    """
    minx, maxx, miny, maxy = field.bounds()
    cx = (minx + maxx) / 2.0
    cy = (miny + maxy) / 2.0
    rx = (maxx - minx) * 0.36
    ry = (maxy - miny) * 0.36

    def traj(t: float):
        w = 0.45
        x = cx + rx * math.cos(w * t)
        y = cy + ry * math.sin(w * t)
        yaw = math.degrees(math.atan2(cy - y, cx - x)) + 12.0 * math.sin(0.8 * t)
        return (x, y, yaw)

    traj.center = (cx, cy)
    traj.radii = (rx, ry)
    return traj


def run_demo(cfg):
    field, camera, extr, _ = build(cfg)
    scene = SyntheticScene(field, camera, extr, seed=cfg.seed,
                           drop_rate=cfg.drop_rate,
                           max_range=cfg.max_range)
    pipe = Pipeline(field, camera, extr,
                    FieldMatcher(field.features, gate=cfg.gate,
                                 max_residual=cfg.max_residual,
                                 min_inliers=cfg.min_inliers),
                    detector=None, use_filter=cfg.filter)

    fps_meter = FpsMeter()
    writer = None
    n = 0
    err_raw = []
    err_filt = []
    t0 = time.perf_counter()
    dt = 1.0 / max(1.0, cfg.fps)
    broadcaster = StatusClient(cfg.status_port) if cfg.status_port else None
    trajectory = make_trajectory(field)
    print(f"[演示轨迹] 场心={tuple(round(v, 2) for v in trajectory.center)} "
          f"半径={tuple(round(v, 2) for v in trajectory.radii)}")

    print("\n=== DEMO 模式（合成画面，用于验证几何与匹配，不是真实相机输入）===")
    print(f"{'frame':>5} {'GT(x,y,yaw)':>26} {'EST(x,y,yaw)':>26} {'dxy':>6} {'dyaw':>7}  status")

    try:
        while True:
            t = n * dt
            if cfg.frames and n >= cfg.frames:
                break
            if cfg.seconds and t > cfg.seconds:
                break

            gt = trajectory(t)
            bcx, bcy = trajectory.center
            ball = (bcx + 3.0 * math.sin(0.5 * t + 0.7),
                    bcy + 2.2 * math.cos(0.4 * t))
            frame, demo_dets, gt_pixels, _ = scene.render(gt, ball_map_xy=ball)
            dets = [(d.bbox, d.label, d.conf) for d in demo_dets]

            t_inf = time.perf_counter()
            out = pipe.process(frame, dets=dets)
            infer_ms = (time.perf_counter() - t_inf) * 1000.0
            fps_meter.tick()

            if broadcaster:
                broadcaster.poll()
                broadcaster.send({"_kind": "pose", **out.status})
                broadcaster.send(_ball_payload(dets))

            if out.carrier_pose is not None:
                dpos, dyaw = pose_delta(gt, out.carrier_pose)
                err_filt.append((dpos, dyaw))
                if out.carrier_pose_raw is not None:
                    err_raw.append(pose_delta(gt, out.carrier_pose_raw))
                if n % 5 == 0 or not cfg.no_gui:
                    print(f"{n:>5} {gt[0]:8.2f},{gt[1]:7.2f},{gt[2]:7.1f} "
                          f"{out.carrier_pose[0]:8.2f},{out.carrier_pose[1]:7.2f},"
                          f"{out.carrier_pose[2]:7.1f} {dpos:6.2f} {dyaw:7.1f}  "
                          f"{out.status['status']}")
            else:
                if n % 10 == 0:
                    print(f"{n:>5} {gt[0]:8.2f},{gt[1]:7.2f},{gt[2]:7.1f} "
                          f"{'--':>26} {'--':>6} {'--':>7}  {out.status['status']}")

            if cfg.json:
                rec = dict(out.status)
                rec["frame"] = n
                print("JSON " + json.dumps(rec, ensure_ascii=False))

            canvas = pipe.render(out, fps=fps_meter.fps, infer_ms=infer_ms,
                                 gt_pose=gt, extra_left=[f"DEMO t={t:5.2f}s (synthetic)"])

            if cfg.save_video:
                if writer is None:
                    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                    writer = cv2.VideoWriter(cfg.save_video, fourcc, cfg.fps,
                                             (canvas.shape[1], canvas.shape[0]))
                writer.write(canvas)
            if cfg.save_dir and n % max(1, cfg.save_every) == 0:
                os.makedirs(cfg.save_dir, exist_ok=True)
                cv2.imwrite(os.path.join(cfg.save_dir, f"demo_{n:05d}.jpg"), canvas)

            if not cfg.no_gui:
                cv2.imshow("K1 field localization (demo)", canvas)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    break
            n += 1
    finally:
        if broadcaster is not None:
            broadcaster.close()
        if writer is not None:
            writer.release()
        if not cfg.no_gui:
            cv2.destroyAllWindows()

    print(f"\n共 {n} 帧，耗时 {time.perf_counter() - t0:.1f}s")

    def _stat(name, rows):
        if not rows:
            return
        d = np.array(rows)
        print(f"{name}误差：位置 均值 {d[:,0].mean():.3f} m / 最大 {d[:,0].max():.3f} m"
              f"    航向 均值 {d[:,1].mean():.2f} deg / 最大 {d[:,1].max():.2f} deg")

    _stat("原始解（未滤波）", err_raw)
    _stat("滤波后输出　　", err_filt)
    if n:
        print(f"定位有效率 {len(err_filt) / n * 100:.1f}%  "
              f"（{len(err_filt)}/{n} 帧输出了有效位姿）")
        print("注：滤波后误差通常略大于原始解，这是滤波平滑带来的相位滞后，属正常现象；"
              "演示时可同时显示两者以便评审判断。")
    return 0


def _make_detector(cfg):
    # 单个 5 类模型（推荐）：一个模型同时出足球和场地标志，推理只跑一次
    if getattr(cfg, "weights", None) and not cfg.marker_weights:
        cfg.marker_weights = cfg.weights
    if not cfg.ball_weights and not cfg.marker_weights:
        return None
    return YoloDetector(ball_weights=cfg.ball_weights,
                        marker_weights=cfg.marker_weights,
                        conf=cfg.conf, imgsz=cfg.imgsz, device=cfg.device,
                        marker_every=getattr(cfg, "marker_every", 1),
                        ball_conf=getattr(cfg, "ball_conf", None),
                        marker_conf=getattr(cfg, "marker_conf", None),
                        marker_imgsz=getattr(cfg, "marker_imgsz", None),
                        marker_gamma=getattr(cfg, "marker_gamma", 0.7))


def _fit_canvas(canvas, max_w: int = 1260, max_h: int = 630):
    """
    把分屏画布等比缩小到能完整放进屏幕。

    为什么需要：左侧 1280 + 分隔 3 + 右侧面板 620 = 1903 px，
    在 1920 屏幕上（还要减去窗口边框和任务栏）右侧会被裁掉 ——
    实测录屏里 x/y/yaw 的数值全被切掉了，而那是任务二最关键的证据。

    max_w / max_h 传 0 表示不限制。
    """
    h, w = canvas.shape[:2]
    sx = (max_w / w) if (max_w and w > max_w) else 1.0
    sy = (max_h / h) if (max_h and h > max_h) else 1.0
    s = min(sx, sy)
    if s >= 1.0:
        return canvas
    return cv2.resize(canvas, (int(w * s), int(h * s)), interpolation=cv2.INTER_AREA)


def run_stream(cfg):
    field, camera, extr, matcher = build(cfg)
    detector = _make_detector(cfg)
    if detector is None:
        print("[错误] live/video 模式需要 --ball-weights 或 --marker-weights")
        return 2
    pipe = Pipeline(field, camera, extr, matcher, detector=detector,
                    use_filter=cfg.filter)
    # --task1：只演示足球检测，全屏显示相机画面，不画右侧场地地图
    pipe.show_map = not getattr(cfg, "task1", False)
    # 观测累积窗口：多帧融合，消除三脚架静置时的 valid 抖动
    pipe.obs_window = int(getattr(cfg, "obs_window", 6) or 0)

    if cfg.video:
        cap = cv2.VideoCapture(cfg.video)
    else:
        cap = open_camera(cfg.camera, cfg.backend, cfg.width, cfg.height, cfg.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        print(f"[错误] 打不开输入源: {cfg.video or cfg.camera}")
        return 3

    fps_meter = FpsMeter()
    writer = None
    n = 0
    # 统计累加器：结束时要报 FPS / 推理耗时 / 定位有效率（细则 2.10）
    stats = {"valid": 0, "invalid": 0, "fps_sum": 0.0, "fps_n": 0,
             "infer_sum": 0.0, "infer_n": 0}
    broadcaster = StatusClient(cfg.status_port) if cfg.status_port else None
    bridge = None
    if cfg.ros:
        from src.ros_bridge import RosBridge
        bridge = RosBridge()
    print("\n=== 实时模式 ===")
    try:
        while True:
            if cfg.frames and n >= cfg.frames:
                break
            ok, frame = cap.read()
            if not ok or frame is None:
                print("[警告] 取帧失败，结束")
                break

            t_inf = time.perf_counter()
            out = pipe.process(frame)
            infer_ms = (time.perf_counter() - t_inf) * 1000.0
            fps_meter.tick()

            # 统计（细则 2.10：FPS / 处理耗时 / 定位有效率）
            stats["fps_sum"] += fps_meter.fps
            stats["fps_n"] += 1
            stats["infer_sum"] += infer_ms
            stats["infer_n"] += 1
            if out.status.get("valid"):
                stats["valid"] += 1
            else:
                stats["invalid"] += 1

            if broadcaster:
                broadcaster.poll()
                broadcaster.send({"_kind": "pose", **out.status})
                broadcaster.send(_ball_payload(out.dets))
            if bridge is not None:
                bridge.publish_pose(out.status)
                bridge.publish_ball([(d[1], d[2], d[0]) for d in out.dets])
                bridge.spin_once()

            if cfg.json:
                rec = dict(out.status)
                rec["frame"] = n
                rec["fps"] = round(fps_meter.fps, 1)
                print("JSON " + json.dumps(rec, ensure_ascii=False))
            elif n % 15 == 0:
                st = out.status
                print(f"[{n:>5}] valid={st['valid']} status={st['status']:<22} "
                      f"markers={st['marker_count']} res="
                      f"{st.get('residual')} pose="
                      f"{st.get('x_camera_map'),st.get('y_camera_map'),st.get('yaw_camera_map')} "
                      f"fps={fps_meter.fps:.1f}")

            canvas = pipe.render(out, fps=fps_meter.fps, infer_ms=infer_ms,
                                 extra_left=[f"source={cfg.video or f'camera {cfg.camera}'}"])
            # 分屏画布默认 1280+3+620=1903 px，超过 1920 屏幕的可用宽度，
            # 窗口右侧（位姿数值面板）会被裁掉。这里按需等比缩小到能完整显示。
            canvas = _fit_canvas(canvas, cfg.fit_width, cfg.fit_height)

            if cfg.save_video:
                if writer is None:
                    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                    writer = cv2.VideoWriter(cfg.save_video, fourcc, cfg.fps,
                                             (canvas.shape[1], canvas.shape[0]))
                writer.write(canvas)
            if cfg.save_dir and n % max(1, cfg.save_every) == 0:
                os.makedirs(cfg.save_dir, exist_ok=True)
                cv2.imwrite(os.path.join(cfg.save_dir, f"frame_{n:05d}.jpg"), canvas)

            if not cfg.no_gui:
                cv2.imshow("K1 field localization", canvas)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    break
                if key == ord("h"):
                    # 在右侧面板上叠出字段说明，录屏讲解时按一下即可
                    pipe.show_help = not getattr(pipe, "show_help", False)
                if key == ord("s"):
                    # 存一张原始帧 + 一张带标注的画面。
                    # 原始帧很重要：可以离线用不同置信度/尺寸复跑，排查"为什么没检出"。
                    os.makedirs(os.path.join(ROOT, "snapshots"), exist_ok=True)
                    import time as _t
                    tag = _t.strftime("%Y%m%d_%H%M%S")
                    raw_p = os.path.join(ROOT, "snapshots", f"shot_{tag}_raw.jpg")
                    vis_p = os.path.join(ROOT, "snapshots", f"shot_{tag}_vis.jpg")
                    cv2.imwrite(raw_p, frame)
                    cv2.imwrite(vis_p, canvas)
                    print(f"[存图] {raw_p}")
                    print(f"[存图] {vis_p}")
            n += 1
    finally:
        cap.release()
        if broadcaster is not None:
            broadcaster.close()
        if bridge is not None:
            bridge.shutdown()
        if writer is not None:
            writer.release()
        if not cfg.no_gui:
            cv2.destroyAllWindows()
    print(f"\n共处理 {n} 帧")
    # 结束时打印统计。
    # 任务 2（有标志模型）才报「定位有效率、更新频率」——细则 2.10 要求的四项；
    # 任务 1 只跑足球检测，没有标志模型，报这些必然全是 0，会让人误读成失败。
    has_marker = bool(getattr(getattr(pipe, "detector", None), "marker_model", None))
    if n > 0 and (has_marker or stats["fps_n"] > 0):
        tot = stats["valid"] + stats["invalid"]
        print("=" * 60)
        print("本次运行统计" + ("（细则 2.10 要求记录的项目）" if has_marker else "（任务 1：实时检测）"))
        print("-" * 60)
        print(f"  总帧数        {n}")
        print(f"  平均 FPS      {stats['fps_sum'] / max(1, stats['fps_n']):.1f}")
        print(f"  平均推理耗时  {stats['infer_sum'] / max(1, stats['infer_n']):.1f} ms")
        if has_marker and tot > 0:
            print(f"  有效位姿帧数  {stats['valid']}  /  {tot}")
            print(f"  定位有效率    {stats['valid'] / tot * 100:.1f} %")
            print(f"  定位更新频率  {stats['valid'] / max(1e-6, n) * (stats['fps_sum'] / max(1, stats['fps_n'])):.1f} Hz"
                  f"  （有效率 × 平均 FPS）")
        print("=" * 60)
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description="K1 单目场地自定位 + 足球检测")
    ap.add_argument("mode", choices=["demo", "live", "video"], nargs="?", default="demo")

    # 输入
    ap.add_argument("--camera", type=int, default=None,
                    help="相机设备号。不指定时读 config/camera.yaml 的 camera.device_index，"
                         "再不行就用 1。插拔后编号可能变，用 scripts/camera_select.py 确认并固定")
    ap.add_argument("--backend", default="msmf", choices=["msmf", "dshow", "any"],
                    help="采集后端。**默认 msmf**：实测 dshow 只能 10 fps，msmf 才有 30 fps")
    ap.add_argument("--video", type=str, default=None)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)

    # 配置
    ap.add_argument("--field", default=_p("config", "field.yaml"))
    ap.add_argument("--geometry", default=_p("config", "field_geometry.yaml"))
    # 默认 None，解析后再按模式决定：
    #   demo 用虚拟相机配置（不受真实相机标定影响）
    #   live/video 用真实标定结果
    ap.add_argument("--intrinsics", default=None)
    ap.add_argument("--extrinsics", default=None)

    # 模型
    ap.add_argument("--weights", type=str, default=None,
                    help="★推荐：单个 5 类模型的权重（ball/LCross/TCross/XCross/PenaltyPoint），"
                         "只推理一次；分两个模型会让帧率减半")
    ap.add_argument("--ball-weights", type=str, default=None)
    ap.add_argument("--marker-weights", type=str, default=None)
    ap.add_argument("--conf", type=float, default=0.35,
                    help="两个模型共用的置信度（会被下面两个覆盖）")
    ap.add_argument("--ball-conf", type=float, default=0.70,
                    help="足球检测置信度。实测 0.70 时 Precision 0.971 / Recall 0.811（F1 最高）；"
                         "调低会把白色鞋子、白斑之类误检成球")
    ap.add_argument("--marker-imgsz", type=int, default=640,
                    help="标志模型单独使用的输入尺寸（默认 640）。场地标志比足球小得多，640 输入下远处 50 px 的标志会被压成 25 px 而漏检；配合 --marker-every 降频可抵消变慢")
    ap.add_argument("--marker-conf", type=float, default=0.32,
                    help="场地标志检测置信度。0.32 是端到端实测最优点：配合 "
                         "--marker-gamma 0.7，T 型密集的方位角有效位姿帧数从 34 提到 91。"
                         "调低会放进更多假阳性，反而把地图匹配毒化")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--marker-every", type=int, default=1,
                    help="标志模型每 N 帧推理一次（1=每帧都跑）。"
                         "两个模型分开时每帧都跑只有约 7 fps，设成 3 可回到约 12 fps，"
                         "标志在画面里变化慢，对定位几乎没有影响")
    ap.add_argument("--task1", action="store_true",
                    help="任务 1 演示模式：只跑足球模型，全屏显示相机画面（不画右侧场地地图）")
    ap.add_argument("--marker-gamma", type=float, default=0.7,
                    help="标志模型推理前的 gamma 校正，<1 提亮。0.7 为实测最优点，1.0=关闭")
    ap.add_argument("--fit-width", type=int, default=1260,
                    help="分屏画布最大宽度（像素），超过则等比缩小。0=不限制")
    ap.add_argument("--fit-height", type=int, default=630,
                    help="分屏画布最大高度（像素），0=不限制")
    ap.add_argument("--obs-window", type=int, default=0,
                    help="观测累积帧数（多帧融合）。0 = 只用当前帧（默认）。"
                         "三脚架静止时可设 6~8，把最近几帧的标志观测合并以补偿单帧漏检；"
                         "但实测也会累积少量误检，不一定更好，所以默认关闭")

    # 匹配阈值（评审可能现场改这些参数，必须能解释影响）
    ap.add_argument("--gate", type=float, default=0.6, help="单个特征匹配门限，米")
    ap.add_argument("--max-residual", type=float, default=0.35, help="允许的最大残差，米")
    ap.add_argument("--min-inliers", type=int, default=3,
                    help="最少内点数。默认 3：2 个标志虽然能解出唯一刚体变换，但对应关系有多解，"
                         "必然不可靠（实测同一位置两次解出相差 2.7 m 的位姿）。"
                         "设为 2 会明显更容易出有效位姿，但可能是错的")
    ap.add_argument("--filter", action="store_true", default=True)
    ap.add_argument("--no-filter", dest="filter", action="store_false")

    # ROS2 / 状态转发
    ap.add_argument("--ros", action="store_true",
                    help="直接发布 ROS2 话题（需已 source /opt/ros/humble/setup.bash）")
    ap.add_argument("--status-port", type=int, default=0,
                    help="把每帧状态以 JSON 行广播到该 TCP 端口，"
                         "供 WSL2 里的 scripts/ros_bridge_node.py 转发成 ROS2 话题")

    # 运行控制
    ap.add_argument("--frames", type=int, default=0)
    ap.add_argument("--seconds", type=float, default=0.0)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--no-gui", action="store_true")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--save-video", type=str, default=None)
    ap.add_argument("--save-dir", type=str, default=None)
    ap.add_argument("--save-every", type=int, default=10)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--drop-rate", type=float, default=0.0,
                    help="demo 模式随机丢弃特征的比例，用来测定位失败分支")
    ap.add_argument("--max-range", type=float, default=14.0)

    cfg = ap.parse_args(argv)

    if cfg.camera is None:
        cfg.camera = default_camera_index()

    # demo 是算法自检（合成画面 + 已知真值），跟手上这台真实相机的内参无关，
    # 所以默认用独立的一份虚拟相机配置，避免被真实相机的窄视场拖成
    # "每帧都看不到足够标志"。
    if cfg.mode == "demo":
        demo_cam = _p("config", "camera_demo.yaml")
        if cfg.intrinsics is None:
            cfg.intrinsics = demo_cam
        if cfg.extrinsics is None:
            cfg.extrinsics = demo_cam
    else:
        real_cam = _p("config", "camera.yaml")
        if cfg.intrinsics is None:
            cfg.intrinsics = real_cam
        if cfg.extrinsics is None:
            cfg.extrinsics = real_cam

    try:
        if cfg.mode == "demo":
            return run_demo(cfg)
        return run_stream(cfg)
    except ConfigError as e:
        print(f"\n[配置错误] {e}\n")
        return 1


if __name__ == "__main__":
    sys.exit(main())
