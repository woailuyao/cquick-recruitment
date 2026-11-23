"""Split view. The left pane uses the original distorted image pixels."""
import cv2
import numpy as np

CAMERA_WIDTH = 820
PANEL_WIDTH = 460
HEADER = 48
FOOTER = 44
BLUE = (245, 214, 93)
GREEN = (96, 242, 131)
ORANGE = (60, 171, 255)
INK = (232, 235, 239)


def label(image, value, point, color=INK, size=0.47, weight=1):
    cv2.putText(image, value, tuple(map(int, point)), cv2.FONT_HERSHEY_SIMPLEX,
                size, color, weight, cv2.LINE_AA)


class Display:
    def __init__(self, session, polylines):
        self.session = session
        self.ids = list(session.markers)
        self.polylines = polylines
        height, width = session.image_shape
        self.height = round(height * CAMERA_WIDTH / width)
        self.sx, self.sy = CAMERA_WIDTH / width, self.height / height
        coords = np.asarray([m["xyz"][:2] for m in session.markers.values()])
        lo, hi = coords.min(axis=0) - .55, coords.max(axis=0) + .55
        self.map_scale = min(202 / (hi[0] - lo[0]), (self.height - 32) / (hi[1] - lo[1]))
        self.map_center = (hi + lo) / 2
        self.list_left = CAMERA_WIDTH + 230
        self.row_height = 23

    def map_point(self, xy):
        x, y = (np.asarray(xy) - self.map_center) * self.map_scale
        return CAMERA_WIDTH + 117 + round(x), HEADER + self.height // 2 - round(y)

    def image_point(self, uv):
        return round(uv[0] * self.sx), HEADER + round(uv[1] * self.sy)

    def mouse(self, event, x, y, _flags=None, _data=None):
        s = self.session
        if event != cv2.EVENT_LBUTTONDOWN or s.frozen is None:
            return
        if CAMERA_WIDTH <= x < CAMERA_WIDTH + PANEL_WIDTH:
            if x >= self.list_left:
                index = (y - HEADER - 10) // self.row_height
                if 0 <= index < len(self.ids):
                    s.selected = self.ids[index]
            else:
                nearest = min(self.ids, key=lambda key: np.linalg.norm(
                    np.subtract((x, y), self.map_point(s.markers[key]["xyz"][:2]))))
                if np.linalg.norm(np.subtract((x, y), self.map_point(
                        s.markers[nearest]["xyz"][:2]))) <= 15:
                    s.selected = nearest
        elif 0 <= x < CAMERA_WIDTH and HEADER <= y < HEADER + self.height and s.selected:
            s.pick(s.selected, (x / self.sx, (y - HEADER) / self.sy))

    def render(self, fps, offline=False):
        s = self.session
        canvas = np.full((HEADER + self.height + FOOTER, CAMERA_WIDTH + PANEL_WIDTH, 3), 26, np.uint8)
        active = s.frozen or s.latest
        if active is not None:
            canvas[HEADER:HEADER + self.height, :CAMERA_WIDTH] = cv2.resize(
                active.image, (CAMERA_WIDTH, self.height), interpolation=cv2.INTER_AREA)
        label(canvas, "MANUAL INITIALIZATION + OPTICAL FLOW + MAP PnP", (12, 20), BLUE, .54, 1)
        mode = "OFFLINE STILL" if offline else "WINDOWS v2 LIVE"
        view_state = "FROZEN - click map ID, then image" if s.frozen else "CURRENT IMAGE"
        frame_number = f" | frame {active.number}" if active is not None else ""
        label(canvas, f"{mode} | {view_state}{frame_number}", (12, 40))
        label(canvas, "MEASURED MAP  +X right, +Y up", (CAMERA_WIDTH + 12, 20), BLUE)
        label(canvas, "CLICK MAP POINT OR ID", (self.list_left + 8, 40))
        for path in self.polylines:
            poly = np.array([self.map_point(p) for p in path["points"]], np.int32)
            cv2.polylines(canvas, [poly], False, (115, 127, 140), 1, cv2.LINE_AA)
        for i, key in enumerate(self.ids):
            used = key in s.points
            color = ORANGE if key == s.selected else GREEN if used else INK
            p = self.map_point(s.markers[key]["xyz"][:2])
            cv2.circle(canvas, p, 5, color, 1, cv2.LINE_AA)
            label(canvas, str(i + 1), (p[0] + 7, p[1] - 5), color, 0.39)
            y = HEADER + 10 + i * self.row_height
            if used or key == s.selected:
                cv2.rectangle(canvas, (self.list_left, y),
                              (canvas.shape[1] - 4, y + self.row_height - 1), (43, 49, 56), -1)
            label(canvas, f"{i + 1:02d} {key}", (self.list_left + 5, y + 16), color, .43)
            if used:
                uv = s.points[key]
                pt = self.image_point(uv)
                cv2.drawMarker(canvas, pt, color, cv2.MARKER_CROSS, 15, 1, cv2.LINE_AA)
                label(canvas, f"{i + 1:02d} {key}", (min(pt[0] + 7, CAMERA_WIDTH - 205), pt[1] - 8), color)
                projected = s.diagnostics.get("projected_pixels_by_id", {}).get(key)
                if s.pose is not None and projected is not None and 0 <= projected[0] < CAMERA_WIDTH / self.sx and 0 <= projected[1] < self.height / self.sy:
                    cv2.circle(canvas, self.image_point(projected), 3, ORANGE, 1, cv2.LINE_AA)
        lower = HEADER + self.height
        reproj = s.diagnostics.get("rms_px")
        reproj_text = "-" if reproj is None else f"{reproj:.2f}px"
        label(canvas, f"valid={str(s.pose is not None).lower()}   status={s.status}   "
              f"points={len(s.points)}   reproj={reproj_text}   FPS={fps:.1f}",
              (12, lower + 17), GREEN if s.pose is not None else ORANGE)
        if s.pose is not None:
            x, y, _ = s.pose["center"]
            yaw = s.pose["yaw_deg"]
            label(canvas, f"Camera X={x:.3f}m Y={y:.3f}m   Yaw={yaw:.1f}deg",
                  (CAMERA_WIDTH + 8, lower + 17), GREEN)
            start = self.map_point((x, y))
            tip = self.map_point(np.array([x, y]) + .46 * s.pose["direction"][:2] /
                                 np.linalg.norm(s.pose["direction"][:2]))
            # Camera may stand outside the plotted half. The numeric pose
            # stays visible and the arrow never overlaps the camera image.
            if all(CAMERA_WIDTH + 4 < p[0] < self.list_left and HEADER < p[1] < lower
                   for p in (start, tip)):
                cv2.arrowedLine(canvas, start, tip, GREEN, 2, cv2.LINE_AA, tipLength=.24)
                cv2.circle(canvas, start, 4, GREEN, -1)
        label(canvas, "Space: freeze / restart   Enter: track   Backspace: undo   R: reset   Q/Esc: exit",
              (12, lower + 36))
        if not s.pose and not s.frozen:
            label(canvas, "Press Space to initialize again", (CAMERA_WIDTH + 8, lower + 17), ORANGE)
        return canvas
