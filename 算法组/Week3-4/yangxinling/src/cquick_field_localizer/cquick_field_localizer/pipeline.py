#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pipeline.py —— 单帧流水线（检测 → 细化 → 投影 → 地图匹配 → 位姿）

把"一帧图像怎么变成 (x, y, yaw)"这条链路封装成一个类，
CLI（localize.py）和 ROS2 节点（localizer_node.py）都复用它，
这样离线调参和现场演示跑的是**同一份代码**，不会出现"离线对、现场错"。

链路（和任务书的公式一一对应）：

    1. YOLO 检测          → bbox + 类别
    2. 几何细化            → 真实交点像素 (u, v)
    3. 去畸变              → (u', v')
    4. d_c = K^-1 [u',v',1]            （相机系方向）
    5. d_b = R_bc · d_c                 （载体系方向）
    6. P_b = O_b + λ d_b, λ = -O_bz/d_bz （与 Z=0 地面求交）
    7. field.yaml 匹配 + RANSAC/SE(2)  → (X, Y, ψ)
"""

import json
import os
import time

import numpy as np
import yaml

from .detector import Detector, canonical_class
from .field_map import FieldMap, CLASS_NAMES
from .geometry import CameraModel, camera_from_configs, optical_axis_ground_point
from .marker_refine import refine_marker
from .pose_solver import Observation, solve as solve_pose

MARKER_CLASSES = (0, 1, 2, 3)


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def resolve_path(base_dir, p):
    """配置里的相对路径按"仓库根目录"解析（base_dir 传进来）。"""
    if not p:
        return p
    return p if os.path.isabs(p) else os.path.normpath(os.path.join(base_dir, p))


def find_config(path):
    """
    找配置文件：① 绝对路径 ② 当前目录 ③ $CQUICK_REPO_ROOT 下的相对路径。

    为什么需要这个：`ros2 run` 的当前目录是**你敲命令的地方**，
    不一定是仓库根目录。有了这条兜底，在哪跑都不会报 "config not found"。
    """
    if not path:
        return path
    if os.path.isabs(path):
        return path
    if os.path.isfile(path):
        return os.path.abspath(path)
    root = os.environ.get("CQUICK_REPO_ROOT", "")
    if root:
        cand = os.path.join(root, path)
        if os.path.isfile(cand):
            return os.path.abspath(cand)
    return os.path.abspath(path)


class LocalizerPipeline(object):
    """
    一帧进、一个结果字典出。

    参数：
        config_path : config/camera.yaml
        repo_root   : 仓库根目录（相对路径的基准）。默认由 config 路径推出来。
        enable_ball : 是否同时跑足球模型（任务1 的节点用得上）
    """

    def __init__(self, config_path, repo_root=None, enable_ball=False):
        self.config_path = find_config(config_path)
        self.repo_root = repo_root or os.path.dirname(os.path.dirname(self.config_path))
        cfg = load_yaml(self.config_path)
        self.cfg = cfg

        intr_path = resolve_path(self.repo_root, cfg.get("intrinsics", "calibration/camera_intrinsics.yaml"))
        extr_path = resolve_path(self.repo_root, cfg.get("extrinsics", "calibration/camera_extrinsics.yaml"))
        self.field_path = resolve_path(self.repo_root, cfg.get("field_map", "config/field.yaml"))
        geom_path = resolve_path(self.repo_root, cfg.get("field_geometry", "config/field_geometry.yaml"))

        self.intr_cfg = load_yaml(intr_path)
        self.extr_cfg = load_yaml(extr_path)
        self.camera = camera_from_configs(self.intr_cfg, self.extr_cfg)
        self.field = FieldMap.from_yaml(self.field_path)
        # 相机必须落在场地矩形内：(x0,y0,x1,y1)，由标志点包围盒给出
        try:
            self.field_bounds = self.field.bounds(0.0)
        except Exception:
            self.field_bounds = None
        self.geometry = load_yaml(geom_path) if os.path.isfile(geom_path) else {}

        self.detector = Detector(cfg.get("detector", {}), role="marker", base_dir=self.repo_root)
        self.ball_detector = (Detector(cfg.get("detector", {}), role="ball", base_dir=self.repo_root)
                              if enable_ball else None)

        self.refine_cfg = cfg.get("refine", {}) or {}
        self.solver_cfg = cfg.get("solver", {}) or {}

        self.last_pose = None
        self.initial_prior = None      # (x, y, yaw_deg, sig_xy, sig_yaw)：启动时的粗略位姿
        self._t_valid = None
        self._update_hz = 0.0
        self.stats = {"frames": 0, "status": {}, "residual": [], "fps": []}
        self._t_prev = None
        self._fps = 0.0

        hp = optical_axis_ground_point(self.camera.R_bc, self.camera.O_b)
        self.axis_ground = hp
        # 内参是按某个分辨率标定的；如果实际输入分辨率不同（例如推流被缩到 960x540），
        # 必须把 fx/fy/cx/cy 按比例缩放，否则"像素→地面"整体错位，位姿永远解不出来。
        calib_size = getattr(self.camera, "image_size", None)
        self.calib_size = tuple(calib_size) if calib_size else None
        self._adapted_size = self.calib_size
        self.scale_note = ""

    # ------------------------------------------------------------------
    def adapt_to_frame_size(self, width, height):
        """按实际帧尺寸缩放内参（只缩放一次；尺寸变化会重算）。返回是否做了缩放。"""
        size = (int(width), int(height))
        if not self.calib_size:
            self.calib_size = size
            self._adapted_size = size
            return False
        if size == self._adapted_size:
            return False
        cw, ch = self.calib_size
        if size == (cw, ch):
            self.camera.K = self.camera.K.copy()
            self._adapted_size = size
            return False
        sx, sy = size[0] / float(cw), size[1] / float(ch)
        K = self.camera.K.copy()
        K[0, 0] *= sx
        K[0, 1] *= sx
        K[0, 2] *= sx
        K[1, 0] *= sy
        K[1, 1] *= sy
        K[1, 2] *= sy
        import numpy as _np
        from .geometry import CameraModel as _CameraModel
        self.camera = _CameraModel(K, self.camera.D, self.camera.R_bc, self.camera.O_b,
                                   image_size=size,
                                   pitch_deg=getattr(self.camera, "pitch_deg", float("nan")),
                                   roll_deg=getattr(self.camera, "roll_deg", float("nan")))
        self._adapted_size = size
        self.scale_note = "内参已按 %dx%d → %dx%d 缩放 (%.3f, %.3f)" % (cw, ch, size[0], size[1], sx, sy)
        print("[cquick] %s" % self.scale_note, flush=True)
        return True

    # ------------------------------------------------------------------
    def set_initial_prior(self, x, y, yaw_deg, sig_xy=1.5, sig_yaw=45.0):
        """给定启动时的粗略位姿（±sig_xy 米 / ±sig_yaw 度）。

        作用：本场地关于中线左右镜像对称，单帧观测在没有先验时必然存在两个
        等价的镜面解（会被判为 ambiguous）。给出一个"我在场地哪个位置、朝哪边
        看"的粗先验即可唯一确定，后续帧继续用上一帧位姿跟踪。
        """
        self.initial_prior = (float(x), float(y), float(yaw_deg),
                              float(sig_xy), float(sig_yaw))
        return self.initial_prior

    # ------------------------------------------------------------------
    def describe(self):
        lines = ["配置: %s" % self.config_path,
                 "内参: fx=%.2f fy=%.2f cx=%.2f cy=%.2f" % tuple(
                     [self.camera.K[0, 0], self.camera.K[1, 1], self.camera.K[0, 2], self.camera.K[1, 2]]),
                 "外参: 高度=%.3f m 俯仰=%.1f° 滚转=%.1f°" % (
                     self.camera.height, self.extr_cfg.get("pitch_deg", 0.0),
                     self.extr_cfg.get("roll_deg", 0.0)),
                 self.field.summary()]
        if self.axis_ground:
            lines.append("光轴落地点: %.2f m 前 / %.2f m 左" % (self.axis_ground[0], self.axis_ground[1]))
        lines.append("检测器: backend=%s" % self.detector.backend)
        for w in self.detector.warnings:
            lines.append("  [警告] %s" % w)
        return "\n".join(lines)

    # ------------------------------------------------------------------
    def process(self, frame_bgr, frame_id=0, stamp=None):
        """
        返回 dict：
            frame_id, stamp, fps,
            detections: [{cls_id, cls_name, conf, bbox, uv, refine_ok, refine_quality, pixel_refined}]
            observations: [Observation]（成功投到地面的才留）
            pose: solve() 的返回
            obs_map: [(cls_id, map_x, map_y, is_inlier)] 用于右侧可视化
        """
        t0 = time.time()
        stamp = stamp or time.strftime("%Y-%m-%d %H:%M:%S")

        if frame_bgr is not None:
            self.adapt_to_frame_size(frame_bgr.shape[1], frame_bgr.shape[0])

        dets = self.detector.detect(frame_bgr)
        marker_dets = [d for d in dets if d.cls_id in MARKER_CLASSES]

        refine_on = bool(self.refine_cfg.get("enable", True))
        margin = float(self.refine_cfg.get("margin", 0.30))
        min_q = float(self.refine_cfg.get("min_quality", 0.05))

        recs = []
        for d in marker_dets:
            if refine_on:
                r = refine_marker(frame_bgr, d.bbox, d.cls_id, margin=margin)
            else:
                r = {"uv": d.center, "ok": False, "quality": 0.0,
                     "method": "disabled", "fallback_reason": "refine_disabled"}
            uv = r["uv"] if (r["ok"] and r["quality"] >= min_q) else d.center
            recs.append({"det": d, "uv": uv, "refine": r})

        # ---- 像素 → 地面 ----
        observations = []
        if recs:
            uvs = np.array([rec["uv"] for rec in recs], dtype=float)
            pts_b, valid, depth = self.camera.ground_from_pixels(uvs)
            for rec, p_b, ok, dep in zip(recs, pts_b, valid, depth):
                rec["p_body"] = (float(p_b[0]), float(p_b[1]))
                rec["ground_ok"] = bool(ok)
                rec["depth"] = float(dep)
                if not ok:
                    continue
                d = rec["det"]
                rec["obs_index"] = len(observations)
                observations.append(Observation(
                    d.cls_id, d.cls_name, rec["p_body"], pixel=rec["uv"],
                    confidence=d.conf, depth=rec["depth"]))

        # ---- 位姿求解（带上一帧先验）----
        sc = self.solver_cfg
        kwargs = dict(tol=float(sc.get("tol_m", 0.35)),
                      min_pair_sep=float(sc.get("min_pair_sep_m", 0.25)),
                      ambiguity_penalty=float(sc.get("ambiguity_penalty", 0.6)),
                      max_hypotheses=int(sc.get("max_hypotheses", 4000)),
                      bounds=getattr(self, "field_bounds", None),
                      min_inliers=int(sc.get("min_inliers", 3)),
                      prior_gate_sigma=float(sc.get("prior_gate_sigma", 3.0)),
                      l_as_t=bool(sc.get("l_as_t", False)))
        prior = None
        if sc.get("use_prior", True) and self.last_pose and self.last_pose.get("valid"):
            lp = self.last_pose
            prior = (lp["x"], lp["y"], lp["yaw_deg"], 0.6, 25.0)
        elif sc.get("use_prior", True) and getattr(self, "initial_prior", None):
            prior = self.initial_prior
        pose = solve_pose(observations, self.field, prior=prior, **kwargs)
        # 先验太紧导致失败（比如被搬动/瞬移），放开重搜一次。
        # 但 "prior_gate"（解与先验严重矛盾）**默认不重搜**——那种情况重搜出来的
        # 往往就是左右镜像的错误分支（实测出现航向偏 158°），宁可不给位姿。
        # 例外：连续 prior_gate 超过 reacquire_after 帧（说明相机真的被搬远了），
        # 才允许无先验重捕获；且只接受"唯一解"（solve 内部已排除 ambiguous）。
        if pose.get("reason") == "prior_gate":
            self._gate_frames = getattr(self, "_gate_frames", 0) + 1
        else:
            self._gate_frames = 0

        if prior is not None and not pose.get("valid") and \
                pose.get("status") in ("insufficient_markers", "high_residual") and \
                (pose.get("reason") != "prior_gate" or
                 self._gate_frames >= int(sc.get("reacquire_after", 8))):
            pose2 = solve_pose(observations, self.field, prior=None, **kwargs)
            if pose2.get("valid"):
                pose = pose2
                pose["recovered_without_prior"] = True
                pose["recovered_after_gate"] = bool(self._gate_frames)
                self._gate_frames = 0

        if pose.get("valid"):
            self.last_pose = pose
            # 位姿更新频率：只在"给出有效位姿"的帧之间计时
            t_valid = time.time()
            if self._t_valid is not None:
                dtv = max(1e-3, t_valid - self._t_valid)
                hz = 1.0 / dtv
                self._update_hz = hz if self._update_hz <= 0 else 0.9 * self._update_hz + 0.1 * hz
            self._t_valid = t_valid
        pose["update_hz"] = float(self._update_hz)

        # ---- 观测点投到 map（画图用）----
        obs_map = []
        inlier_obs = {i for (i, _j, _d) in pose.get("inliers", [])}
        # 观测 -> 地图特征名的对应（录屏要展示"匹配关系"，要写进表格）
        obs_match = {}
        names = self.field.names() if hasattr(self.field, "names") else []
        for (i, j, dist) in pose.get("inliers", []):
            nm = names[j] if 0 <= j < len(names) else "?"
            obs_match[i] = (nm, float(dist))
        if pose.get("valid"):
            for i, obs in enumerate(observations):
                P = self.camera.body_to_map(np.array([obs.p_body]), (pose["x"], pose["y"]),
                                            pose["yaw_deg"])[0]
                obs_map.append((obs.cls_id, float(P[0]), float(P[1]), i in inlier_obs))
        elif observations:
            for i, obs in enumerate(observations):
                obs_map.append((obs.cls_id, float("nan"), float("nan"), False))

        # ---- FPS / 统计 ----
        now = time.time()
        dt = (now - self._t_prev) if self._t_prev else 0.0
        self._t_prev = now
        if dt > 1e-6:
            self._fps = 0.9 * self._fps + 0.1 * (1.0 / dt) if self._fps else 1.0 / dt
        self.stats["frames"] += 1
        st = pose.get("status", "lost")
        self.stats["status"][st] = self.stats["status"].get(st, 0) + 1
        if pose.get("valid") and np.isfinite(pose.get("residual", float("nan"))):
            self.stats["residual"].append(float(pose["residual"]))
        self.stats["fps"].append(self._fps)

        out = {
            "frame_id": int(frame_id),
            "stamp": stamp,
            "fps": float(self._fps),
            "processing_ms": (time.time() - t0) * 1000.0,
            "detections": [{
                "cls_id": rec["det"].cls_id,
                "cls_name": rec["det"].cls_name,
                "raw_name": rec["det"].raw_name,
                "conf": rec["det"].conf,
                "bbox": list(rec["det"].bbox),
                "uv": [float(rec["uv"][0]), float(rec["uv"][1])],
                "refine_ok": bool(rec["refine"].get("ok")),
                "refine_method": rec["refine"].get("method"),
                "refine_quality": float(rec["refine"].get("quality", 0.0)),
                "ground_ok": bool(rec.get("ground_ok", False)),
                "p_body": list(rec.get("p_body", [float("nan"), float("nan")])),
                "depth_m": float(rec.get("depth", float("nan"))),
                "match_name": (obs_match.get(rec.get("obs_index", -1), ("", float("nan")))[0]
                               if rec.get("obs_index", -1) in obs_match else ""),
                "match_dist_m": (obs_match.get(rec.get("obs_index", -1), ("", float("nan")))[1]
                                 if rec.get("obs_index", -1) in obs_match else float("nan")),
            } for rec in recs],
            "map_info": {
                "path": getattr(self.field, "path", ""),
                "version": getattr(self.field, "version", ""),
                "shorthash": getattr(self.field, "shorthash", ""),
                "count": len(self.field.markers),
            },
            "camera_info": {
                "fx": float(self.camera.K[0, 0]), "fy": float(self.camera.K[1, 1]),
                "cx": float(self.camera.K[0, 2]), "cy": float(self.camera.K[1, 2]),
                "k1": float(self.camera.D[0]), "k2": float(self.camera.D[1]),
                "p1": float(self.camera.D[2]), "p2": float(self.camera.D[3]),
                "k3": float(self.camera.D[4]) if len(self.camera.D) > 4 else 0.0,
                "height_m": float(-self.camera.O_b[2]),
                "pitch_deg": float(getattr(self.camera, "pitch_deg", float("nan"))),
                "roll_deg": float(getattr(self.camera, "roll_deg", float("nan"))),
            },
            "observations": observations,
            "obs_map": obs_map,
            "pose": pose,
        }
        return out

    # ------------------------------------------------------------------
    def summary(self):
        """跑完一段视频后打印/写入报告的统计。"""
        fps = self.stats["fps"][5:] if len(self.stats["fps"]) > 5 else self.stats["fps"]
        res = self.stats["residual"]
        total = max(1, self.stats["frames"])
        return {
            "frames": self.stats["frames"],
            "fps_mean": float(np.mean(fps)) if fps else 0.0,
            "fps_p50": float(np.percentile(fps, 50)) if fps else 0.0,
            "status_counts": dict(self.stats["status"]),
            "valid_ratio": float(self.stats["status"].get("ok", 0)) / total,
            "residual_mean_m": float(np.mean(res)) if res else None,
            "residual_p95_m": float(np.percentile(res, 95)) if res else None,
        }


def detections_to_json(rec):
    """把 process() 结果压成一行 JSON（jsonl 用）。"""
    pose = rec["pose"]
    return json.dumps({
        "frame": rec["frame_id"],
        "stamp": rec["stamp"],
        "fps": round(rec["fps"], 2),
        "x": round(pose.get("x", 0.0), 4) if pose.get("valid") else None,
        "y": round(pose.get("y", 0.0), 4) if pose.get("valid") else None,
        "yaw": round(pose.get("yaw_deg", 0.0), 3) if pose.get("valid") else None,
        "valid": bool(pose.get("valid")),
        "marker_count": int(pose.get("marker_count", 0)),
        "residual": (round(float(pose["residual"]), 4)
                     if pose.get("residual") is not None and np.isfinite(pose.get("residual", float("nan")))
                     else None),
        "status": pose.get("status", "lost"),
        "n_det": len(rec["detections"]),
        "classes": sorted({d["cls_name"] for d in rec["detections"]}),
    }, ensure_ascii=False)
