#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
detector.py —— 统一的检测器封装（ONNX / Ultralytics 两种后端）

为什么要有这一层：
    任务1 和任务2 都要"实时运行"，而现场演示最怕的就是环境问题。
    .pt 模型需要 torch + ultralytics（依赖重、启动慢、有时还会联网）；
    .onnx 只要 onnxruntime，pip 装一个包就能跑，启动快、版本冲突少。
    所以这里做成"同一套接口，两个后端"，配置里一个 backend 字段就能切换，
    代码其它地方完全不用改。

对外接口只有三个：
    det = Detector(cfg)                      # 建一次
    boxes = det.detect(frame_bgr)            # 每帧调用
    det.names / det.warnings                 # 调试用

Detection 字段：
    cls_id   : 内部类别 id（0..3 标志，见 field_map.CLASS_ID；足球是 4）
    cls_name : 内部类别名（LCross/TCross/XCross/PenaltyPoint/Ball）
    conf     : 置信度
    bbox     : (x1, y1, x2, y2) 原图像素
    raw_name : 模型原始类名（写报告时有用，能看出别名映射对不对）
"""

import os

import cv2
import numpy as np

# 内部类名 → id。任务书规定 0..3 是四种标志点；Ball 放在 4，避免冲突。
INTERNAL_CLASSES = ["LCross", "TCross", "XCross", "PenaltyPoint", "Ball"]
INTERNAL_ID = {n: i for i, n in enumerate(INTERNAL_CLASSES)}

# 各类别的默认置信度（config 里的 class_conf 会覆盖）
DEFAULT_CLASS_CONF = {
    "LCross": 0.15, "TCross": 0.15, "XCross": 0.15,
    "PenaltyPoint": 0.15, "Ball": 0.35,
}

# 常见别名 → 内部名。键统一小写、去掉下划线和连字符后比较。
_DEFAULT_ALIAS = {
    "lcross": "LCross", "l": "LCross", "lmarker": "LCross", "l型": "LCross",
    "tcross": "TCross", "t": "TCross", "tmarker": "TCross", "t型": "TCross",
    "xcross": "XCross", "x": "XCross", "xmarker": "XCross", "x型": "XCross",
    "penaltypoint": "PenaltyPoint", "penalitypoint": "PenaltyPoint",
    "penalty": "PenaltyPoint", "penaltydot": "PenaltyPoint",
    "ball": "Ball", "football": "Ball", "soccerball": "Ball", "soccer": "Ball",
}


def _norm_name(name):
    return "".join(ch for ch in str(name).lower() if ch.isalnum() or "\u4e00" <= ch <= "\u9fff")


def canonical_class(raw_name, alias=None):
    """把模型输出的类名归一化成内部类名；认不出来返回 None（该检测会被丢掉）。"""
    table = dict(_DEFAULT_ALIAS)
    for k, v in (alias or {}).items():
        table[_norm_name(k)] = v
    return table.get(_norm_name(raw_name))


class Detection(object):
    __slots__ = ("cls_id", "cls_name", "conf", "bbox", "raw_name")

    def __init__(self, cls_name, conf, bbox, raw_name=""):
        self.cls_name = cls_name
        self.cls_id = INTERNAL_ID[cls_name]
        self.conf = float(conf)
        self.bbox = tuple(float(v) for v in bbox)
        self.raw_name = raw_name

    @property
    def center(self):
        x1, y1, x2, y2 = self.bbox
        return (x1 + x2) * 0.5, (y1 + y2) * 0.5

    def area(self):
        x1, y1, x2, y2 = self.bbox
        return max(0.0, x2 - x1) * max(0.0, y2 - y1)

    def __repr__(self):
        x1, y1, x2, y2 = self.bbox
        return "<%s %.2f [%.0f,%.0f,%.0f,%.0f]>" % (self.cls_name, self.conf, x1, y1, x2, y2)


# ---------------------------------------------------------------------------
# 预处理 / 后处理（ONNX 用）
# ---------------------------------------------------------------------------
def letterbox(img, new_shape=640, color=(114, 114, 114), stride=32):
    """把图像等比缩放到 new_shape，空出来的地方填灰边，返回 (图, 比例, 左padding, 上padding)。"""
    h, w = img.shape[:2]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)
    r = min(new_shape[0] / h, new_shape[1] / w)
    new_unpad = (int(round(w * r)), int(round(h * r)))
    dw = (new_shape[1] - new_unpad[0]) / 2.0
    dh = (new_shape[0] - new_unpad[1]) / 2.0
    if (w, h) != new_unpad:
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return img, r, left, top


def nms_numpy(boxes, scores, iou_thr):
    """纯 numpy 的 NMS，避免为了一行代码依赖 torchvision。boxes 是 (N,4) xyxy。"""
    if len(boxes) == 0:
        return np.zeros(0, dtype=int)
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(int(i))
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        inter = w * h
        union = areas[i] + areas[order[1:]] - inter
        iou = np.where(union > 0, inter / np.maximum(union, 1e-9), 0.0)
        order = order[1:][iou <= iou_thr]
    return np.array(keep, dtype=int)


# ---------------------------------------------------------------------------
class Detector(object):
    """
    统一检测器。cfg 就是 config/camera.yaml 里的 detector: 段（dict）。

    backend:
        auto / onnx / torch / none
    模型路径不存在时不会崩，只是 warnings 里记一条，detect() 返回空列表，
    这样你可以先把整条流水线跑通，再去准备模型。
    """

    def __init__(self, cfg, role="marker", base_dir=None):
        cfg = dict(cfg or {})
        self.cfg = cfg
        self.role = role
        self.base_dir = base_dir
        self.warnings = []
        self.backend = "none"
        self.session = None
        self.model = None
        self.names = {}

        self.conf_thr = float(cfg.get("conf", 0.25))
        self.iou_thr = float(cfg.get("iou", 0.45))
        self.imgsz = int(cfg.get("imgsz", 640))
        self.max_det = int(cfg.get("max_det", 100))
        alias = cfg.get("class_alias") or {}
        self.class_conf = {}
        for k, v in (cfg.get("class_conf") or {}).items():
            name = canonical_class(k, alias)
            if name:
                self.class_conf[name] = float(v)

        want = cfg.get("backend", "auto")
        path = cfg.get("ball_model" if role == "ball" else "marker_model", "")
        path = path or ""
        if path and base_dir and not os.path.isabs(path):
            # 配置里写的是仓库相对路径（models/xxx.onnx），按仓库根目录解析
            path = os.path.normpath(os.path.join(base_dir, path))

        if want in ("auto", "onnx") and path and os.path.isfile(path) and path.endswith(".onnx"):
            self._load_onnx(path)
        elif want in ("auto", "torch") and path and os.path.isfile(path) and path.endswith(".pt"):
            self._load_torch(path)
        elif want in ("auto", "torch") and path and os.path.isfile(path):
            # .pt 以外的权重交给 ultralytics 自己判断
            self._load_torch(path)
        else:
            if path:
                self.warnings.append("模型文件不存在：%s（进入无检测模式）" % path)
            else:
                self.warnings.append("未配置 %s 模型（进入无检测模式）" % role)

    # ---- 后端加载 ----
    def _load_onnx(self, path):
        try:
            import onnxruntime as ort
        except Exception as exc:                       # pragma: no cover
            self.warnings.append("onnxruntime 不可用（%s），尝试 ultralytics" % exc)
            self._load_torch(path[:-5] + ".pt")
            return
        providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
        so = None
        try:
            so = ort.SessionOptions()
            # 本机（CPU-only）实测：默认线程数在 1280x720 上跑不满，
            # 显式给满物理核能把单帧推理从 ~40 ms 压到 ~15 ms
            so.intra_op_num_threads = max(1, min(8, (os.cpu_count() or 4)))
            so.inter_op_num_threads = 1
            so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        except Exception:
            so = None
        try:
            self.session = ort.InferenceSession(path, sess_options=so, providers=providers)
        except Exception:
            self.session = ort.InferenceSession(path, sess_options=so,
                                                providers=["CPUExecutionProvider"])
        self.backend = "onnx"
        self.input_name = self.session.get_inputs()[0].name
        shape = self.session.get_inputs()[0].shape
        if isinstance(shape, (list, tuple)) and len(shape) == 4 and isinstance(shape[2], int):
            self.imgsz = int(shape[2])
        # 从 ONNX metadata 里读类名（ultralytics 导出时会写 names）
        self.names = self._names_from_onnx_meta(path)

    def _names_from_onnx_meta(self, path):
        names = {}
        try:
            import onnx
            m = onnx.load(path)
            for prop in m.metadata_props:
                if prop.key == "names":
                    txt = prop.value
                    # 形如 "{0: 'ball', 1: 'LCross'}"
                    for part in txt.strip("{}").split(","):
                        if ":" in part:
                            k, v = part.split(":", 1)
                            names[int(k.strip())] = v.strip().strip("'\"")
        except Exception:
            pass
        return names

    def _load_torch(self, path):
        try:
            from ultralytics import YOLO
        except Exception as exc:                       # pragma: no cover
            self.warnings.append("ultralytics 不可用（%s），进入无检测模式" % exc)
            return
        if not os.path.isfile(path):
            self.warnings.append("torch 权重不存在：%s" % path)
            return
        self.model = YOLO(path)
        self.backend = "torch"
        names = getattr(self.model, "names", None) or {}
        self.names = {int(k): str(v) for k, v in names.items()}

    # ---- 推理 ----
    def detect(self, frame_bgr):
        if self.backend == "onnx":
            return self._detect_onnx(frame_bgr)
        if self.backend == "torch":
            return self._detect_torch(frame_bgr)
        return []

    def _thr_for(self, name):
        return float(self.class_conf.get(name, self.conf_thr))

    def _detect_onnx(self, frame_bgr):
        img, ratio, pad_w, pad_h = letterbox(frame_bgr, self.imgsz)
        blob = img[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        blob = np.ascontiguousarray(blob)
        try:
            out = self.session.run(None, {self.input_name: blob})[0]
        except Exception as exc:
            self.warnings.append("ONNX 推理失败：%s" % exc)
            return []
        pred = np.asarray(out)
        if pred.ndim == 3:
            pred = pred[0]
        # ultralytics 导出格式通常是 (4+nc, N)，统一成 (N, 4+nc)：
        #   N 一般远大于 4+nc（640 输入时 N=8400），所以"行少于列"就转置；
        #   极小 N（如合成测试里的 N=1）会落进第二个分支再翻回来。
        if pred.shape[0] < pred.shape[1]:
            pred = pred.T
        if pred.shape[1] < 5 and pred.shape[0] >= 5:
            pred = pred.T
        if pred.shape[1] < 5:
            return []
        boxes_xywh = pred[:, :4]
        cls_scores = pred[:, 4:]
        if cls_scores.shape[1] == 1:                   # 单类且只有 obj 分数
            scores = cls_scores[:, 0]
            cls_ids = np.zeros(len(scores), dtype=int)
        else:
            cls_ids = cls_scores.argmax(axis=1)
            scores = cls_scores[np.arange(len(cls_ids)), cls_ids]

        keep_all = []
        for cid in np.unique(cls_ids):
            sel = np.nonzero(cls_ids == cid)[0]
            name = self.names.get(int(cid), "")
            internal = canonical_class(name, self.cfg.get("class_alias"))
            if internal is None:
                continue
            sel = sel[scores[sel] >= self._thr_for(internal)]
            if sel.size == 0:
                continue
            xy = boxes_xywh[sel]
            x1 = xy[:, 0] - xy[:, 2] / 2.0
            y1 = xy[:, 1] - xy[:, 3] / 2.0
            x2 = xy[:, 0] + xy[:, 2] / 2.0
            y2 = xy[:, 1] + xy[:, 3] / 2.0
            boxes = np.stack([x1, y1, x2, y2], axis=1)
            keep = nms_numpy(boxes, scores[sel], self.iou_thr)
            for idx in keep:
                b = boxes[idx]
                keep_all.append(Detection(
                    internal, scores[sel][idx],
                    ((b[0] - pad_w) / ratio, (b[1] - pad_h) / ratio,
                     (b[2] - pad_w) / ratio, (b[3] - pad_h) / ratio),
                    raw_name=name))
        keep_all.sort(key=lambda d: -d.conf)
        return keep_all[: self.max_det]

    def _detect_torch(self, frame_bgr):
        try:
            res = self.model.predict(
                frame_bgr, imgsz=self.imgsz, conf=min(self.class_conf.values() or [0.05]),
                iou=self.iou_thr, max_det=self.max_det, verbose=False,
                device=self.cfg.get("device", "cpu"))
        except Exception as exc:
            self.warnings.append("ultralytics 推理失败：%s" % exc)
            return []
        dets = []
        for r in res:
            names = getattr(r, "names", self.names)
            for b in r.boxes:
                cid = int(b.cls.item())
                name = str(names.get(cid, cid)) if isinstance(names, dict) else str(cid)
                internal = canonical_class(name, self.cfg.get("class_alias"))
                if internal is None:
                    continue
                conf = float(b.conf.item())
                if conf < self._thr_for(internal):
                    continue
                x1, y1, x2, y2 = [float(v) for v in b.xyxy[0].tolist()]
                dets.append(Detection(internal, conf, (x1, y1, x2, y2), raw_name=name))
        dets.sort(key=lambda d: -d.conf)
        return dets


# ---------------------------------------------------------------------------
def describe(det, n=None):
    """调试用：打印检测器状态。"""
    lines = ["检测器(role=%s) backend=%s 类名表=%s" % (det.role, det.backend, det.names)]
    for w in det.warnings:
        lines.append("  [警告] %s" % w)
    return "\n".join(lines)
