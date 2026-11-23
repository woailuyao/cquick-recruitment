"""Compare all retained marker weights on the exact frozen frame, never on the screenshot."""
import hashlib
import json
from pathlib import Path

import cv2
import torch
from ultralytics import YOLO

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
SITE = ROOT / "site_review_20261119"
# These are visible targets in 21657 only, from independent inspection of its original pixels.
REFERENCES = {
    "7": (0, [624, 394, 722, 465]),
    "8": (1, [570, 322, 682, 410]),
    "11": (0, [1082, 568, 1231, 667]),
    "12": (1, [769, 338, 886, 415]),
}
NAMES = ("LCross", "TCross", "XCross", "PenaltyPoint")
META = json.loads((SITE / "comparison/formal_metrics.json").read_text())["models"]
COLORS = {"matched": (70, 235, 112), "misclassified": (60, 150, 255), "box_offset": (0, 216, 250),
          "missed": (60, 60, 238), "background_or_duplicate": (220, 128, 180)}


def overlap(a, b):
    w = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    h = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    cross = w * h
    return cross / ((a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - cross)


def diagnostics(boxes):
    results = {}
    for marker, (cls, reference) in REFERENCES.items():
        nearby = sorted([(overlap(reference, b["bbox"]), i) for i, b in enumerate(boxes)],
                        reverse=True)
        same = [item for item in nearby if boxes[item[1]]["class_id"] == cls and item[0] >= .10]
        if same and same[0][0] >= .50:
            outcome, pair = "matched", same[0]
        elif nearby and nearby[0][0] >= .50 and boxes[nearby[0][1]]["class_id"] != cls:
            outcome, pair = "misclassified", nearby[0]
        elif same:
            outcome, pair = "box_offset", same[0]
        else:
            outcome, pair = "missed", None
        results[marker] = dict(status=outcome, iou=pair[0] if pair else None,
                               box_index=pair[1] if pair else None)
    return results


def annotate(raw, name, boxes, results):
    canvas = cv2.copyMakeBorder(raw, 32, 0, 0, 0, cv2.BORDER_CONSTANT)
    cv2.putText(canvas, name + " | 21657 offline | GT left corners unscored | conf=.25",
                (8, 22), 0, .62, (224, 224, 224), 1, cv2.LINE_AA)
    used = {r["box_index"]:r["status"] for r in results.values() if r["box_index"] is not None}
    for marker, (_, ref) in REFERENCES.items():
        x1, y1, x2, y2 = ref
        status = results[marker]["status"]
        cv2.rectangle(canvas, (x1, y1 + 32), (x2, y2 + 32), (190,190,190), 1)
        cv2.putText(canvas, f"{marker} {status}", (x1, y2 + 49), 0, .48, COLORS[status], 1, cv2.LINE_AA)
    for index, b in enumerate(boxes):
        color = COLORS[used.get(index, "background_or_duplicate")]
        x1, y1, x2, y2 = [round(v) for v in b["bbox"]]
        cv2.rectangle(canvas, (x1, y1 + 32), (x2, y2 + 32), color, 2)
        cv2.putText(canvas, f"{NAMES[b['class_id']]} {b['confidence']:.2f}", (x1, y1 + 27),
                    0, .45, color, 1, cv2.LINE_AA)
    return canvas


def main():
    torch.set_num_threads(2)
    source = json.loads((HERE/"candidate_diagnostics.json").read_text())["freeze_0002_21657"]
    path = ROOT / source["source_raw"]
    assert hashlib.sha256(path.read_bytes()).hexdigest() == source["raw_sha256"]
    img = cv2.imread(str(path))
    assert img.shape == (720, 1280, 3)
    out = HERE / "field_comparison_21657"
    out.mkdir(exist_ok=False)
    payload = dict(source=source["source_raw"], sha256=source["raw_sha256"], conf=.25, iou=.70,
                   imgsz=640, device="cpu", references=REFERENCES, weights={},
                   scope="only selected visible four IDs on 21657, not frame 24839")
    for name, meta in META.items():
        weight = Path(meta["weight"])
        assert hashlib.sha256(weight.read_bytes()).hexdigest() == meta["sha256"]
        model = YOLO(str(weight))
        result = model.predict(img, conf=.25, iou=.70, imgsz=640, device="cpu", classes=[0,1,2], verbose=False)[0]
        boxes = [dict(bbox=b, class_id=int(c), confidence=s)
                 for b,c,s in zip(result.boxes.xyxy.tolist(),result.boxes.cls.tolist(),result.boxes.conf.tolist())]
        ratings = diagnostics(boxes)
        payload["weights"][name] = dict(weight=str(weight), sha256=meta["sha256"],
                                        boxes=boxes, reference_results=ratings)
        assert cv2.imwrite(str(out/(name+".png")), annotate(img, name, boxes, ratings))
        print(name, len(boxes), {k:(v["status"],round(v["iou"],3) if v["iou"] is not None else None)
                                for k,v in ratings.items()})
    (out/"comparison.json").write_text(json.dumps(payload,indent=2)+"\n")


if __name__ == "__main__":
    main()
