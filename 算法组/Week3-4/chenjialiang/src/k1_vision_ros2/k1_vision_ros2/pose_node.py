# ROS2 节点：场地定位，发布 /camera_pose 与 /pose_status
import math
import os
import sys

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

PKG_DIR = os.path.dirname(os.path.abspath(__file__))


def _find_geometry():
    roots = [PKG_DIR, os.getcwd()]
    for r in list(roots):
        cur = r
        for _ in range(8):
            roots.append(cur)
            nxt = os.path.dirname(cur)
            if nxt == cur:
                break
            cur = nxt
    for r in roots:
        for rel in ("geometry", os.path.join("src", "geometry")):
            cand = os.path.join(r, rel)
            if os.path.isfile(os.path.join(cand, "localize.py")):
                return cand
    return os.path.join(PKG_DIR, "geometry")


GEOM_DIR = _find_geometry()
if GEOM_DIR not in sys.path:
    sys.path.insert(0, GEOM_DIR)

from localize import (
    CAM_HEIGHT, CONF, CORNERS, DUP_DIST, FIELD, K, MIN_MARKERS, MODEL,
    PITCH_DEG, REFINE, corner_xy, pose_to_xy_yaw, refine_pixel,
    solve_pose, solve_pose_height,
)

from ultralytics import YOLO

class Locator:

    def __init__(self, model_path=MODEL):
        self.model = YOLO(model_path)
        self.last = None
        self.hl = FIELD["length_x"] / 2
        self.hw = FIELD["width_y"] / 2

    def detect(self, frame):
        res = self.model(frame, conf=CONF, verbose=False)[0]
        raw = []
        goal_hint = False
        for b in res.boxes:
            name = self.model.names[int(b.cls[0])]
            if name == "goal":
                goal_hint = True
                continue
            if name not in CORNERS:
                continue
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            raw.append([name, (x1 + x2) / 2, (y1 + y2) / 2, float(b.conf[0]),
                        x1, y1, x2, y2])
        raw.sort(key=lambda r: -r[3])
        keep = []
        for r in raw:
            if any(r[0] == q[0] and (r[1] - q[1]) ** 2 + (r[2] - q[2]) ** 2 < DUP_DIST ** 2
                   for q in keep):
                continue
            keep.append(r)

        if REFINE and keep:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            for r in keep:
                nx, ny, ok = refine_pixel(gray, r[4], r[5], r[6], r[7], r[1], r[2])
                if ok:
                    r[1], r[2] = nx, ny

        self.goal_hint = goal_hint
        return keep

    @staticmethod
    def pick_pair(items):
        fams = {}
        for d in items:
            nm = d[0]
            if nm.endswith("_left"):
                side, fam = "L", nm[:-5]
            else:
                side, fam = "R", nm[:-6]
            fams.setdefault(fam, {})[side] = d
        best, best_key = [], (-1, -1)
        for sides in fams.values():
            key = (1 if len(sides) == 2 else 0, len(sides))
            if key > best_key:
                best_key, best = key, list(sides.values())
        return best

    def solve(self, known):
        if len(known) < MIN_MARKERS:
            return None, None, None, -1.0, "insufficient_markers"

        cands = []
        for front in (True, False):
            p3h = [[*corner_xy(d[0], front), 0.0] for d in known]
            p2h = [[d[1], d[2]] for d in known]
            if len(known) >= 4:
                rv, tv, rs = solve_pose(p3h, p2h)
                if rv is None:
                    continue
                hx, hy, hyaw = pose_to_xy_yaw(rv, tv)
            else:
                hxy, hyaw, rs = solve_pose_height(p3h, p2h, CAM_HEIGHT, PITCH_DEG)
                if hxy is None:
                    continue
                hx, hy = hxy

            mw = np.mean(np.asarray(p3h, dtype=np.float64), axis=0)[:2]
            bearing = math.degrees(math.atan2(mw[1] - hy, mw[0] - hx))
            if abs((hyaw - bearing + 180.0) % 360.0 - 180.0) > 30.0:
                continue
            cands.append((front, (hx, hy), hyaw, rs))

        if not cands:
            return None, None, None, -1.0, "no_valid_hypothesis"

        if self.last is None and getattr(self, "goal_hint", False):
            cf = [c for c in cands if c[0]]
            if cf:
                cands = cf

        def rank(c):
            fr, xy, _, _ = c
            inside = 0 if (abs(xy[0]) <= self.hl and abs(xy[1]) <= self.hw) else 1
            dist = 0.0 if self.last is None else (
                (xy[0] - self.last[0]) ** 2 + (xy[1] - self.last[1]) ** 2)
            return (inside, dist)

        _, (x, y), yaw, res = min(cands, key=rank)

        if abs(x) > self.hl + 1.0 or abs(y) > self.hw + 1.0:
            return None, None, None, res, "out_of_field"

        self.last = (x, y)
        status = "ok" if res <= (8.0 if len(known) >= 4 else 1.0) else "high_residual"
        return x, y, yaw, res, status

class PoseNode(Node):
    def __init__(self):
        super().__init__("k1_vision_pose")
        self.declare_parameter("camera_url", "http://localhost:8000/stream")
        self.declare_parameter("model", "")
        url = self.get_parameter("camera_url").value
        path = self.get_parameter("model").value or MODEL

        self.loc = Locator(path)
        self.cap = cv2.VideoCapture(url)
        self.pub = self.create_publisher(PoseStamped, "/camera_pose", 10)
        self.pub_st = self.create_publisher(String, "/pose_status", 10)
        self.timer = self.create_timer(0.02, self.tick)
        self.n = 0
        self.get_logger().info(f"PoseNode 启动，图像源 {url}")

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            return
        if frame.shape[1] != 1280:
            frame = cv2.resize(frame, (1280, 720))

        known = self.loc.pick_pair(self.loc.detect(frame))
        x, y, yaw, res, status = self.loc.solve(known)

        valid = 1 if status == "ok" else 0
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        if valid:
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = float(CAM_HEIGHT)
            m.pose.orientation.z = math.sin(math.radians(yaw) / 2.0)
            m.pose.orientation.w = math.cos(math.radians(yaw) / 2.0)
        self.pub.publish(m)

        s = String()
        s.data = (f"valid={valid} status={status} markers={len(known)} "
                  f"residual={res:.2f} x={x if valid else float('nan'):+.2f} "
                  f"y={y if valid else float('nan'):+.2f} "
                  f"yaw={yaw if valid else float('nan'):+.1f}")
        self.pub_st.publish(s)

        self.n += 1
        if self.n % 30 == 0:
            self.get_logger().info(s.data)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main():
    rclpy.init()
    node = PoseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
