import argparse
from pathlib import Path

import cv2
import numpy as np


WINDOW = "marker_labeler"
CLASS_NAMES = ["LCross", "TCross", "XCross"]
COLORS = [(0, 255, 0), (255, 128, 0), (0, 0, 255)]
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}


class MarkerLabeler:
    def __init__(self, image_dir: Path, start: int = 0):
        self.image_dir = image_dir
        self.images = sorted(
            p for p in image_dir.iterdir() if p.suffix.lower() in IMAGE_SUFFIXES
        )
        if not self.images:
            raise SystemExit(f"没有图片: {image_dir}")
        self.index = max(0, min(start, len(self.images) - 1))
        self.image = None
        self.display = None
        self.scale = 1.0
        self.boxes = []
        self.current_class = 0
        self.drawing = False
        self.start_xy = None
        self.current_xy = None
        self.confirm_delete = False
        self.status = ""

    def load(self):
        path = self.images[self.index]
        image = cv2.imread(str(path))
        if image is None:
            raise SystemExit(f"读图失败: {path}")
        self.image = image
        h, w = image.shape[:2]
        self.scale = min(1.0, 1280 / w, 720 / h)
        self.display = cv2.resize(image, (int(w * self.scale), int(h * self.scale)))
        self.boxes = self.load_labels(path)
        self.drawing = False
        self.start_xy = None
        self.current_xy = None
        self.confirm_delete = False
        self.status = f"{path.name} boxes={len(self.boxes)} class={CLASS_NAMES[self.current_class]}"

    def label_path(self):
        return self.images[self.index].with_suffix(".txt")

    def load_labels(self, image_path: Path):
        label_path = image_path.with_suffix(".txt")
        if not label_path.exists():
            return []
        h, w = self.image.shape[:2]
        boxes = []
        for line in label_path.read_text(encoding="utf-8").splitlines():
            parts = line.split()
            if len(parts) != 5:
                continue
            try:
                cls = int(parts[0])
                cx, cy, bw, bh = map(float, parts[1:])
            except ValueError:
                continue
            if cls < 0 or cls >= len(CLASS_NAMES):
                continue
            x1 = int((cx - bw / 2) * w)
            y1 = int((cy - bh / 2) * h)
            x2 = int((cx + bw / 2) * w)
            y2 = int((cy + bh / 2) * h)
            boxes.append((cls, x1, y1, x2, y2))
        return boxes

    def save(self):
        path = self.images[self.index]
        if not self.boxes:
            path.with_suffix(".txt").unlink(missing_ok=True)
            return
        h, w = self.image.shape[:2]
        lines = []
        for cls, x1, y1, x2, y2 in self.boxes:
            x1, x2 = sorted((max(0, min(w - 1, x1)), max(0, min(w - 1, x2))))
            y1, y2 = sorted((max(0, min(h - 1, y1)), max(0, min(h - 1, y2))))
            if x2 - x1 < 4 or y2 - y1 < 4:
                continue
            cx = (x1 + x2) / 2 / w
            cy = (y1 + y2) / 2 / h
            bw = (x2 - x1) / w
            bh = (y2 - y1) / h
            lines.append(f"{cls} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")
        if lines:
            path.with_suffix(".txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
        else:
            path.with_suffix(".txt").unlink(missing_ok=True)

    def delete_current(self):
        path = self.images[self.index]
        path.unlink(missing_ok=True)
        path.with_suffix(".txt").unlink(missing_ok=True)
        del self.images[self.index]
        if not self.images:
            return False
        if self.index >= len(self.images):
            self.index = len(self.images) - 1
        self.load()
        return True

    def mouse(self, event, x, y, flags, param):
        if self.image is None:
            return
        ix = int(x / self.scale)
        iy = int(y / self.scale)
        h, w = self.image.shape[:2]
        ix = max(0, min(w - 1, ix))
        iy = max(0, min(h - 1, iy))

        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.start_xy = (ix, iy)
            self.current_xy = (ix, iy)
            self.confirm_delete = False
        elif event == cv2.EVENT_MOUSEMOVE and self.drawing:
            self.current_xy = (ix, iy)
        elif event == cv2.EVENT_LBUTTONUP and self.drawing:
            self.drawing = False
            if self.start_xy is not None:
                x1, y1 = self.start_xy
                x2, y2 = ix, iy
                if abs(x2 - x1) >= 4 and abs(y2 - y1) >= 4:
                    self.boxes.append((
                        self.current_class,
                        min(x1, x2), min(y1, y2),
                        max(x1, x2), max(y1, y2),
                    ))
            self.start_xy = None
            self.current_xy = None
            self.status = f"{self.images[self.index].name} boxes={len(self.boxes)}"

    def redraw(self):
        if self.image is None:
            return
        canvas = self.display.copy()
        for cls, x1, y1, x2, y2 in self.boxes:
            color = COLORS[cls % len(COLORS)]
            p1 = (int(x1 * self.scale), int(y1 * self.scale))
            p2 = (int(x2 * self.scale), int(y2 * self.scale))
            cv2.rectangle(canvas, p1, p2, color, 2)
            cv2.putText(canvas, CLASS_NAMES[cls], (p1[0], max(18, p1[1] - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
        if self.drawing and self.start_xy and self.current_xy:
            p1 = (int(self.start_xy[0] * self.scale), int(self.start_xy[1] * self.scale))
            p2 = (int(self.current_xy[0] * self.scale), int(self.current_xy[1] * self.scale))
            cv2.rectangle(canvas, p1, p2, (0, 255, 255), 2)
        cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 34), (0, 0, 0), -1)
        cv2.putText(canvas, self.status[:150], (8, 23),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        if self.confirm_delete:
            cv2.putText(canvas, "Press d again to DELETE", (20, canvas.shape[0] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        self.display_frame = canvas

    def run(self):
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(WINDOW, self.mouse)
        self.load()
        while True:
            self.redraw()
            cv2.imshow(WINDOW, self.display_frame)
            key = cv2.waitKey(20) & 0xFF
            if key in (27, ord("q")):
                self.save()
                break
            if key in (ord("1"), ord("2"), ord("3")):
                self.current_class = key - ord("1")
                self.status = f"class={CLASS_NAMES[self.current_class]}"
            elif key == ord("s"):
                self.save()
                self.status = f"Saved {self.images[self.index].name}"
            elif key == ord("n"):
                if not self.boxes:
                    self.status = "No box. Draw a box or press d to delete."
                else:
                    self.save()
                    if self.index < len(self.images) - 1:
                        self.index += 1
                        self.load()
            elif key == ord("p"):
                self.save()
                if self.index > 0:
                    self.index -= 1
                    self.load()
            elif key == ord("u"):
                if self.boxes:
                    self.boxes.pop()
                self.status = f"Undo. boxes={len(self.boxes)}"
            elif key == ord("c"):
                self.boxes = []
                self.status = "Cleared."
            elif key == ord("d"):
                if not self.confirm_delete:
                    self.confirm_delete = True
                    self.status = "Press d AGAIN to delete."
                else:
                    if not self.delete_current():
                        break
        cv2.destroyAllWindows()
        print(f"Finished. Remaining images: {len(self.images)}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--images", default="dataset/images/selected")
    parser.add_argument("--start", type=int, default=0)
    args = parser.parse_args()
    MarkerLabeler(Path(args.images), args.start).run()