# 角点检测封装：YOLO 推理、同类去重、亚像素细化
import cv2
import numpy as np
from ultralytics import YOLO

class MarkerDetector:
    def __init__(self, model_path, conf=0.4):
        self.model = YOLO(model_path)
        self.conf = conf
        self.names = self.model.names

    def detect(self, frame, refine=False):
        res = self.model(frame, conf=self.conf, verbose=False)[0]
        out = []

        for box in res.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            cls_id = int(box.cls[0])
            name = self.names.get(cls_id, str(cls_id))
            conf_v = float(box.conf[0])

            if refine:
                cx, cy = self._refine(frame, x1, y1, x2, y2)
                if cx is None:
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            else:
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

            out.append((name, cx, cy, conf_v))

        return out

    @staticmethod
    def _refine(frame, x1, y1, x2, y2):
        h, w = frame.shape[:2]
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        patch = frame[y1:y2, x1:x2]
        if patch.size == 0:
            return None, None

        gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
        pts = cv2.goodFeaturesToTrack(
            gray, maxCorners=1, qualityLevel=0.01, minDistance=10)
        if pts is None:
            return None, None

        crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
        pts = cv2.cornerSubPix(gray, pts, (5, 5), (-1, -1), crit)

        px, py = pts[0][0]
        return int(px) + x1, int(py) + y1

def load_model_names(data_yaml_path):
    import yaml
    with open(data_yaml_path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    return cfg.get("names", [])
