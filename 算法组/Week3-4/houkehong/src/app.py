# -*- coding: utf-8 -*-
"""
主流程：相机/视频/合成 -> YOLO 检测 -> 标志几何细化 -> 地面投影 -> field.yaml 匹配
        -> 载体/相机位姿 -> 左右分屏可视化

用法见 run.py。
"""

from __future__ import annotations

import json
import math
import os
import time
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np

from . import viz
from .geometry import (CameraModel, Extrinsics, pose_delta, rot2, transform_points,
                       wrap_deg)
from .landmarks import refine_landmark
from .matching import FieldMatcher, MapFeature, MarkerObs, MatchResult

from .config import FIELD_MARKER_CLASSES

# 现场实际存在的标志类别 —— 统一在 src/config.py 的 FIELD_MARKER_CLASSES 里配置
MARKER_CLASSES = tuple(FIELD_MARKER_CLASSES)


# ---------------------------------------------------------------- 检测器

def _apply_gamma(frame, gamma: float):
    """
    对整帧做 gamma 校正（gamma<1 提亮中间调）。

    现场实测：白线偏淡、T 型竖臂只有几像素时，gamma=0.7 能明显改善检出。
    在 164427 那段低对比度方位角上，T 召回 0.55 -> 0.65，L 召回不降。
    """
    if not gamma or abs(gamma - 1.0) < 1e-6:
        return frame
    lut = np.array([((i / 255.0) ** gamma) * 255 for i in range(256)]).astype("uint8")
    return cv2.LUT(frame, lut)


class YoloDetector:
    """
    封装 ultralytics。支持两种配置：

      A) **单个 5 类模型（推荐）**
         一个模型同时输出 ball / LCross / TCross / XCross / PenaltyPoint，
         推理只跑一次。在本机 CPU 上这一点很重要：
         分成两个模型等于推理时间翻倍，实时帧率会从 ~15 fps 掉到 ~7 fps。

         用法： --weights weights/k1_5class_best.pt
         （也可以只传 --marker-weights，此时它会被当成合并模型使用）

      B) 两个模型分开
         --ball-weights ball_best.pt --marker-weights marker_best.pt
    """

    def __init__(self, ball_weights: Optional[str] = None,
                 marker_weights: Optional[str] = None,
                 conf: float = 0.35, imgsz: int = 640, device: str = "cpu",
                 classes: Optional[list] = None, marker_every: int = 1,
                 ball_conf: Optional[float] = None, marker_conf: Optional[float] = None,
                 marker_imgsz: Optional[int] = None, marker_gamma: float = 0.7):
        from ultralytics import YOLO
        self.conf = conf
        self.imgsz = imgsz
        self.device = device
        self.classes = classes
        # 标志模型推理前的 gamma 校正。0.7 是现场实测的最优点：
        # 白线偏淡、T 型竖臂很短时更易检出；1.0 = 关闭。
        self.marker_gamma = float(marker_gamma)
        # 足球和标志的置信度分开：
        #   足球模型实测 conf=0.70 时 Precision 0.971 / Recall 0.811（F1 最高），
        #   再低就会把白色鞋子、白斑之类误检成球。
        #   标志模型本身输出置信度偏低（0.2~0.6 很常见），阈值太高会大量漏检。
        self.ball_conf = conf if ball_conf is None else float(ball_conf)
        self.marker_conf = conf if marker_conf is None else float(marker_conf)
        # 标志模型可以单独用更大的输入尺寸：场地标志普遍比足球小，
        # 640 输入下 50 px 的标志会被压成 25 px，远处就检不出来。
        # 但这会变慢，所以配合 marker_every 降频使用。
        self.marker_imgsz = int(imgsz if marker_imgsz is None else marker_imgsz)
        # 标志模型每 N 帧推理一次（见 _run_marker）。1 = 每帧都跑。
        self.marker_every = max(1, int(marker_every))
        self._marker_cache = None
        self._mf = 0
        self.ball_model = YOLO(ball_weights) if ball_weights else None
        self.marker_model = YOLO(marker_weights) if marker_weights else None
        if self.ball_model is None and self.marker_model is None:
            raise ValueError("至少需要一个模型权重")

        # 只有标志模型时，它很可能就是一个合并的 5 类模型，
        # 这时必须也接受 ball，否则足球检测结果会被整条滤掉。
        self._marker_wants = set(MARKER_CLASSES)
        if self.ball_model is None:
            self._marker_wants.add("ball")

    def _run(self, model, frame, want, conf=None, imgsz=None, gamma=None):
        if model is None:
            return []
        # 标志模型前面的 gamma 预处理：现场白线偏淡、T 型竖臂只有几像素，
        # 实测 gamma=0.7 能把 164427 这种低对比度方位角的 T 召回从 0.55 提到 0.65，
        # 同时 L 召回不降。足球模型传 gamma=None，用原图。
        if gamma:
            frame = _apply_gamma(frame, gamma)
        res = model.predict(frame, imgsz=self.imgsz if imgsz is None else imgsz,
                            conf=self.conf if conf is None else conf,
                            device=self.device, verbose=False)[0]
        out = []
        names = res.names
        boxes = res.boxes
        if boxes is None:
            return out
        for i in range(len(boxes)):
            cls_id = int(boxes.cls[i].item())
            label = names.get(cls_id, str(cls_id))
            # 类别名归一化：容忍训练时写成 sports ball / l_cross 之类
            lb = label.strip().lower().replace("-", "_").replace(" ", "_")
            if lb in ("sports_ball", "ball"):
                label = "ball"
            elif lb in ("lcross", "l_cross", "l"):
                label = "LCross"
            elif lb in ("tcross", "t_cross", "t"):
                label = "TCross"
            elif lb in ("xcross", "x_cross", "x"):
                label = "XCross"
            elif lb in ("penaltypoint", "penalitypoint", "penalty_point"):
                label = "PenaltyPoint"

            if want is not None and label not in want:
                continue
            xyxy = boxes.xyxy[i].tolist()
            out.append((tuple(float(v) for v in xyxy), label,
                        float(boxes.conf[i].item())))
        return out

    def detect(self, frame):
        dets = self._run(self.ball_model, frame, {"ball"}, conf=self.ball_conf)
        dets += self._run_marker(frame)
        return dets

    def _run_marker(self, frame):
        """
        标志降频检测。

        标志在画面里变化很慢（相机移动时标志也是连续平移），没必要每帧都跑；
        而足球是运动的，必须每帧跑。所以这里让标志模型每 marker_every 帧才真正推理一次，
        中间帧复用上一次的结果。

        实测：双模型每帧都跑是 6.9 fps，改成标志每 3 帧跑一次后回到约 12~13 fps，
        对定位几乎没有影响（位姿更新率仍在 4~5 Hz）。
        """
        if self.marker_model is None:
            return []
        self._mf = getattr(self, "_mf", 0) + 1
        every = max(1, int(getattr(self, "marker_every", 1)))
        if every > 1 and self._mf % every != 0 and getattr(self, "_marker_cache", None) is not None:
            return list(self._marker_cache)
        out = self._run(self.marker_model, frame, self._marker_wants,
                        conf=self.marker_conf, imgsz=self.marker_imgsz,
                        gamma=self.marker_gamma)
        self._marker_cache = out
        return list(out)


@dataclass
class PoseFilter:
    """指数滤波；定位失效或跳变时复位，绝不无限沿用旧位姿（细则 2.10）。"""
    alpha: float = 0.35
    max_jump_m: float = 1.5
    max_jump_deg: float = 45.0
    _state: Optional[tuple] = None
    resets: int = 0

    def reset(self):
        self._state = None
        self.resets += 1

    def update(self, pose: Optional[tuple]) -> Optional[tuple]:
        if pose is None:
            self.reset()
            return None
        if self._state is None:
            self._state = pose
            return pose
        dpos, dyaw = pose_delta(self._state, pose)
        if dpos > self.max_jump_m or dyaw > self.max_jump_deg:
            self.reset()
            self._state = pose
            return pose
        a = self.alpha
        x = (1 - a) * self._state[0] + a * pose[0]
        y = (1 - a) * self._state[1] + a * pose[1]
        yaw = wrap_deg(self._state[2] + a * wrap_deg(pose[2] - self._state[2]))
        self._state = (x, y, yaw)
        return self._state


# ---------------------------------------------------------------- 主流程

@dataclass
class FrameOutput:
    frame: np.ndarray
    dets: list = field(default_factory=list)
    obs: list[MarkerObs] = field(default_factory=list)
    feature_points: list = field(default_factory=list)   # 几何细化后的 (u, v, label)
    gt_points: dict = field(default_factory=dict)
    match: Optional[MatchResult] = None
    carrier_pose: Optional[tuple] = None      # 滤波后
    carrier_pose_raw: Optional[tuple] = None
    camera_pose: Optional[tuple] = None
    status: dict = field(default_factory=dict)


class Pipeline:
    def __init__(self, field_map, camera: CameraModel, extrinsics: Extrinsics,
                 matcher: FieldMatcher, detector=None, use_filter: bool = True):
        self.field = field_map
        self.cam = camera
        self.ex = extrinsics
        self.matcher = matcher
        self.detector = detector
        self.filter = PoseFilter() if use_filter else None
        # 任务 1 只需要足球检测，关掉右侧地图面板，全屏显示相机画面
        self.show_map = True
        # 按 h 切换：在右侧面板上叠加字段说明（录屏讲解时用）
        self.show_help = False

        # 相机光轴在载体坐标系中的水平朝向角（载体 +X 逆时针为正）
        axis = extrinsics.R[:, 2]
        self.camera_yaw_offset_deg = math.degrees(math.atan2(axis[1], axis[0]))
        self._uid = 0
        self._map_cache = None      # (base_img, MapView)，懒加载并复用

        # ---- 观测累积（多帧融合）----
        # 三脚架静止时，标志检测会逐帧抖动：同一帧里有 3~5 个，下一帧可能只剩 1 个。
        # 把最近 obs_window 帧的观测在**载体系地面坐标**里合并，单帧漏检就被补上了。
        # 窗口取小一些（默认 6 帧 ≈ 0.3~0.4 s），即使相机小幅移动也不会引入误导。
        # 这不是伪造结果：所有点都来自真实检测，只是把时间维度上的重复观测合并。
        self.obs_window = 6
        self.obs_min_hits = 2       # 至少被看到这么多次才采用（压制一闪而过的误检）
        self._obs_buf = []          # [{'type','x','y','conf','age','hits'}]
        self._merge_dist = 0.12     # 同类且相距小于此值视为同一个标志，米

    def reset_obs(self):
        """清空多帧观测缓冲。移动三脚架后必须调用，否则旧位置的观测会污染新位姿。"""
        self._obs_buf = []

    def _accumulate(self, obs):
        """把本帧观测合并进累积缓冲，返回**经过时间确认**的观测集合。"""
        for o in obs:
            merged = False
            for b in self._obs_buf:
                if b["type"] == o.type and math.hypot(b["x"] - o.x, b["y"] - o.y) < self._merge_dist:
                    a = 0.4
                    b["x"] = (1 - a) * b["x"] + a * o.x
                    b["y"] = (1 - a) * b["y"] + a * o.y
                    b["conf"] = max(b["conf"], o.conf)
                    b["age"] = 0
                    b["hits"] += 1
                    merged = True
                    break
            if not merged:
                self._obs_buf.append({"type": o.type, "x": o.x, "y": o.y,
                                      "conf": o.conf, "age": 0, "hits": 1})

        for b in self._obs_buf:
            b["age"] += 1
        self._obs_buf = [b for b in self._obs_buf if b["age"] <= self.obs_window]

        if len(self._obs_buf) > 20:
            self._obs_buf.sort(key=lambda b: (-b["hits"], -b["conf"], b["age"]))
            self._obs_buf = self._obs_buf[:20]

        # 只采用"被反复看到"的点：真标志会稳定反复出现，误检往往只闪一次。
        # 这一条是压制误检堆积的关键，否则窗口内会越积越多假点，一直报 ambiguous。
        min_hits = max(1, int(self.obs_min_hits))
        out = []
        for i, b in enumerate(self._obs_buf):
            if b["hits"] < min_hits:
                continue
            out.append(MarkerObs(uid=i + 1, type=b["type"], x=b["x"], y=b["y"],
                                 conf=b["conf"], refined=True))
        return out

    # -------------------------------------------------- 观测 -> 位姿
    def build_observations(self, frame, dets):
        """
        标志检测框 -> 几何细化 -> 去畸变 -> 射线与地面求交 -> 载体系地面点。

        points 里每一项是 (u, v, label, x_b, y_b)：
        前三个是像素特征点，后两个是该点**转换到载体坐标系后的地面米制坐标**
        （细则 2.4 要求录屏能看到"标志从像素点到载体地面坐标的转换结果"）。
        射线没打到地面时 x_b / y_b 为 None。
        """
        obs, points = [], []
        for bbox, label, conf in dets:
            if label not in MARKER_CLASSES:
                continue
            ref = refine_landmark(frame, bbox, label)

            self._uid += 1
            uv = np.array([[ref.u, ref.v]], dtype=np.float64)
            dirs_cam = self.cam.pixel_to_ray_camera(uv)
            orig_b, dirs_b = self.ex.ray_to_carrier(dirs_cam)
            pts, valid = self.ex.ground_intersection(orig_b, dirs_b)
            if not valid[0]:
                # 射线没打到地面 -> 丢弃，绝不造点；但仍记下像素点便于看出是哪个框
                points.append((ref.u, ref.v, label, None, None))
                continue
            xb, yb = float(pts[0, 0]), float(pts[0, 1])
            points.append((ref.u, ref.v, label, xb, yb))
            obs.append(MarkerObs(uid=self._uid, type=label,
                                 x=xb, y=yb, conf=conf, refined=ref.ok))
        return obs, points

    def process(self, frame, dets=None, gt_points=None) -> FrameOutput:
        if dets is None:
            if self.detector is None:
                raise RuntimeError("没有检测器也没有外部传入 dets")
            dets = self.detector.detect(frame)

        obs, points = self.build_observations(frame, dets)
        # 多帧累积：单帧漏检由前几帧补上，消除 valid 抖动
        fused = self._accumulate(obs)
        match = self.matcher.solve(fused if fused else obs)

        raw = (match.x, match.y, match.yaw) if match.valid else None
        filtered = self.filter.update(raw) if self.filter else raw

        carrier = filtered
        camera_pose = None
        if carrier is not None:
            t_xy = np.array([[self.ex.t[0], self.ex.t[1]]])
            pc = transform_points(t_xy, math.radians(carrier[2]),
                                  np.array([carrier[0], carrier[1]]))[0]
            camera_pose = (float(pc[0]), float(pc[1]),
                           wrap_deg(carrier[2] + self.camera_yaw_offset_deg))

        status = {
            "valid": bool(match.valid),
            "status": match.status,
            "marker_count": int(match.marker_count),
            "inlier_count": int(match.inlier_count),
            "residual": float(match.residual) if np.isfinite(match.residual) else None,
            "hypotheses": int(match.hypotheses),
        }
        if carrier is not None:
            status.update({
                "x_carrier_map": round(float(carrier[0]), 3),
                "y_carrier_map": round(float(carrier[1]), 3),
                "yaw_carrier_map": round(float(carrier[2]), 2),
            })
        if camera_pose is not None:
            status.update({
                "x_camera_map": round(camera_pose[0], 3),
                "y_camera_map": round(camera_pose[1], 3),
                "yaw_camera_map": round(camera_pose[2], 2),
            })

        # obs 用融合后的集合：地图面板上的「特征匹配到的地图 ID」要能看到累积的点
        return FrameOutput(frame=frame, dets=dets, obs=fused if fused else obs,
                           feature_points=points,
                           gt_points=gt_points or {}, match=match,
                           carrier_pose=carrier, carrier_pose_raw=raw,
                           camera_pose=camera_pose, status=status)

    # -------------------------------------------------- 渲染
    def render(self, out: FrameOutput, fps: float = 0.0, infer_ms: float = 0.0,
               extra_left=None, gt_pose=None, copy_frame: bool = False) -> np.ndarray:
        # 每帧都从相机拿到新的 frame，就地画可以省掉一次 2.7 MB 的整帧拷贝
        # （实测 render 从 18 ms 降到约 10 ms）。需要保留原始帧时传 copy_frame=True。
        left = out.frame.copy() if copy_frame else out.frame
        viz.draw_detections(left, out.dets, feature_points=out.feature_points)

        h, w = left.shape[:2]
        viz.put(left, f"{w}x{h}  FPS {fps:5.1f}  infer {infer_ms:5.1f} ms",
                (10, h - 12), 0.5, viz.TEXT)
        n_marker = sum(1 for d in out.dets if d[1] in MARKER_CLASSES)
        n_ball = sum(1 for d in out.dets if d[1] == "ball")
        viz.put(left, f"detections: ball={n_ball} marker={n_marker}",
                (10, 24), 0.5, viz.TEXT)
        y = 46
        if extra_left:
            for line in extra_left:
                viz.put(left, line, (10, y), 0.45, viz.TEXT_DIM)
                y += 20
        if gt_pose is not None and out.carrier_pose is not None:
            dpos, dyaw = pose_delta(gt_pose, out.carrier_pose)
            viz.put(left, f"GT-ERR dxy={dpos:.3f}m dyaw={dyaw:.2f}deg",
                    (10, y), 0.5, (120, 220, 255))
        elif gt_pose is not None:
            viz.put(left, "GT-ERR: no valid pose", (10, y), 0.5, viz.BAD_COLOR)

        right = self._map_panel(out)
        if not getattr(self, "show_map", True):
            # 任务 1 只需要足球检测，右侧地图面板是干扰，直接返回左侧全幅画面
            return left
        return viz.compose_split(left, right, divider=3)

    def _map_panel(self, out: FrameOutput) -> np.ndarray:
        W, H = 620, 660
        matched = {fid for _, fid, _ in (out.match.matches if out.match else [])}
        valid = bool(out.match and out.match.valid)
        pose = out.camera_pose if out.camera_pose is not None else None

        lines = []
        st = out.status
        if pose is not None:
            lines.append(f"x_camera_map   {pose[0]:8.3f} m")
            lines.append(f"y_camera_map   {pose[1]:8.3f} m")
            lines.append(f"yaw_camera_map {pose[2]:8.2f} deg")
        else:
            lines.append("x/y/yaw_camera_map   --")
        if out.carrier_pose is not None:
            lines.append(f"carrier  {out.carrier_pose[0]:6.2f}, {out.carrier_pose[1]:6.2f}, "
                         f"{out.carrier_pose[2]:6.1f} deg")
        lines.append(f"markers {st.get('marker_count', 0)}  inliers {st.get('inlier_count', 0)}")
        res = st.get("residual")
        lines.append(f"residual {'--' if res is None else f'{res:.3f} m'}")
        status_txt = f"valid={str(st.get('valid')).lower():5s} status={st.get('status','--')}"

        # 静态底图只画一次，之后每帧只叠加动态部分（省 20~30 ms/帧）
        if getattr(self, "_map_cache", None) is None:
            base, view = viz.make_map_base(W, H, self.field)
            self._map_cache = (base, view)
        base, view = self._map_cache
        panel = viz.draw_map_overlay(base, view, self.field, matched_fids=matched,
                                     pose=pose, status_text=status_txt,
                                     valid=valid, extra_lines=lines)
        if getattr(self, "show_help", False):
            panel = self._draw_help(panel)
        return panel

    # 字段说明（按 h 叠加）。界面统一用 ASCII，避免 OpenCV 无法渲染中文。
    HELP_LINES = [
        ("x_camera_map", "camera optical centre X in map [m]"),
        ("y_camera_map", "camera optical centre Y in map [m]"),
        ("yaw_camera_map", "camera heading in map [deg], CCW from +X"),
        ("carrier", "carrier base x, y, yaw (differs by extrinsics)"),
        ("markers", "landmarks detected AND matched to the map"),
        ("inliers", "observations that voted for this pose"),
        ("residual", "map-fit residual [m] -- smaller is better"),
        ("", ""),
        ("valid=true", "pose solved and uniquely determined"),
        ("ambiguous", "several equally good fits -> refuse to guess"),
        ("insufficient_markers", "fewer than min_inliers landmarks"),
        ("high_residual", "observations do not fit the map"),
        ("", ""),
        ("green dot + arrow", "camera position & heading on the map"),
        ("#ID", "map feature that was matched this frame"),
        ("", ""),
        ("h key", "toggle this help   |   s key: save snapshot"),
    ]

    def _draw_help(self, panel):
        """半透明黑底 + 字段说明，叠在右侧面板上。"""
        h, w = panel.shape[:2]
        overlay = panel.copy()
        cv2.rectangle(overlay, (0, 0), (w, h), (12, 15, 20), -1)
        # 接近不透明：底图透出来会和文字叠在一起，看不清
        panel = cv2.addWeighted(overlay, 0.97, panel, 0.03, 0)
        y = 26
        viz.put(panel, "FIELD GUIDE  (press h to hide)", (12, y), 0.5, (120, 220, 255))
        y += 24
        for name, desc in self.HELP_LINES:
            if not name and not desc:
                y += 8
                continue
            if name:
                viz.put(panel, name, (12, y), 0.44, (255, 220, 120))
            if desc:
                viz.put(panel, desc, (12, y + 15), 0.40, (200, 200, 200))
                y += 15
            y += 17
            if y > h - 8:
                break
        return panel


# ---------------------------------------------------------------- 工具

class FpsMeter:
    def __init__(self, window: int = 30):
        self.t = []
        self.window = window

    def tick(self):
        now = time.perf_counter()
        self.t.append(now)
        if len(self.t) > self.window:
            self.t.pop(0)

    @property
    def fps(self) -> float:
        if len(self.t) < 2:
            return 0.0
        dt = self.t[-1] - self.t[0]
        return (len(self.t) - 1) / dt if dt > 0 else 0.0
