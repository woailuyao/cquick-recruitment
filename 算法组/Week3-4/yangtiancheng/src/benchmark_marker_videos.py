import argparse
from collections import Counter
import json
from pathlib import Path
from time import perf_counter

import cv2
from ultralytics import YOLO


def iou(a, b):
    overlap = max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(
        0, min(a[3], b[3]) - max(a[1], b[1])
    )
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - overlap
    return overlap / union if union > 0 else 0


def benchmark(model, video, labels):
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open {video}")
    counts = Counter()
    checked = Counter()
    frame_count = 0
    seconds = 0
    while True:
        start = perf_counter()
        ok, frame = cap.read()
        if not ok:
            break
        prediction = model.predict(frame, imgsz=640, conf=0.25, verbose=False)[0]
        detections = prediction.boxes.data.cpu().tolist()
        seconds += perf_counter() - start
        counts.update(int(row[-1]) for row in detections)
        if frame_count in labels:
            checked["frames"] += 1
            height, width = frame.shape[:2]
            targets = []
            for line in labels[frame_count].read_text(encoding="utf-8-sig").splitlines():
                if not line.strip():
                    continue
                cls, x, y, w, h = map(float, line.split())
                targets.append((int(cls), ((x-w/2)*width, (y-h/2)*height,
                                           (x+w/2)*width, (y+h/2)*height)))
            for *box, confidence, cls in sorted(detections, key=lambda row: -row[-2]):
                options = [(iou(box, target), index) for index, (kind, target)
                           in enumerate(targets) if kind == int(cls)]
                score, index = max(options, default=(0, -1))
                checked["detections"] += 1
                if score >= 0.5:
                    checked["matched"] += 1
                    targets.pop(index)
                else:
                    checked["unmatched"] += 1
        frame_count += 1
    cap.release()
    if not frame_count:
        raise RuntimeError(f"No decoded frames in {video}")
    return {
        "video": str(video), "frames": frame_count, "fps": round(frame_count / seconds, 2),
        "detections": sum(counts.values()),
        "detections_by_class": {model.names[key]: val for key, val in counts.items()},
        "labelled_frame_check": dict(checked),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("weights")
    parser.add_argument("videos", nargs="+", type=Path)
    args = parser.parse_args()
    model = YOLO(args.weights)
    manifest = json.loads(Path("frames/manifest.json").read_text(encoding="utf-8"))
    reports = []
    for video in args.videos:
        labels = {}
        for item in manifest:
            if item["video"] == video.name:
                path = Path("frames/labels") / (Path(item["frame"]).stem + ".txt")
                if path.is_file():
                    labels[item["frame_index"]] = path
        report = benchmark(model, video, labels)
        reports.append(report)
        print(json.dumps(report, ensure_ascii=False), flush=True)
    Path("marker_video_metrics.json").write_text(
        json.dumps(reports, ensure_ascii=False, indent=2), encoding="utf-8"
    )


if __name__ == "__main__":
    main()
