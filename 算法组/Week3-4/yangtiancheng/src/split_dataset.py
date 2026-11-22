#!/usr/bin/env python3
"""Split annotated marker frames into a YOLO dataset BY VIDEO.

Frames from one video go to exactly one split (train/val/test), so the test
set is truly independent -- the assessment spec explicitly forbids scattering
adjacent frames of one video across splits.

Rules enforced here:
  * Pure-negative videos (no labelled instances at all, e.g. the INVALID
    segment) are always assigned to train: they teach the model background
    and must not occupy val/test.
  * val and test must each contain at least one labelled instance,
    otherwise per-class metrics are meaningless. The assignment search
    tries many seeded orderings until the gates pass.
  * --seed genuinely changes the outcome (it drives the search order).

Input layout (produced by extract_frames.py + your annotation tool):
    frames_dir/
        manifest.json            (frame -> source video)
        images...  (*.jpg)
        labels/  *.txt           (YOLO format, one file per image)

Output layout:
    out_dir/
        images/{train,val,test}/*.jpg
        labels/{train,val,test}/*.txt
        data.yaml
        split_report.json        (per-split, per-class counts)

Usage:
    python3 split_dataset.py --frames ./frames --out ./marker_dataset
"""
import argparse
import json
import random
import shutil
from collections import Counter, defaultdict
from pathlib import Path

import yaml

CLASS_NAMES = {0: "LCross", 1: "TCross", 2: "XCross", 3: "PenaltyPoint"}


def video_of(frame_name, manifest):
    if frame_name in manifest:
        return manifest[frame_name]
    # Fall back to "<video_stem>_f<index>.jpg" naming from extract_frames.py.
    stem = Path(frame_name).stem
    return stem.rsplit("_f", 1)[0] if "_f" in stem else stem


def count_label_file(path):
    counts = Counter()
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line:
            counts[int(line.split()[0])] += 1
    return counts


def greedy_assign(order, video_frames, ratios, forced):
    total = sum(len(f) for f in video_frames.values())
    targets = {k: total * r for k, r in ratios.items()}
    counts = {k: 0 for k in ratios}
    assignment = {}
    for v, split in forced.items():
        assignment[v] = split
        counts[split] += len(video_frames[v])
    for v in order:
        if v in assignment:
            continue
        split = max(ratios, key=lambda k: targets[k] - counts[k])
        assignment[v] = split
        counts[split] += len(video_frames[v])
    return assignment


def score_assignment(assignment, video_frames, video_classes, ratios):
    """Higher is better: gate on positives, reward class coverage + ratios."""
    total = sum(len(f) for f in video_frames.values())
    frame_counts = Counter()
    classes_in = {}
    instances_in = Counter()
    for v, split in assignment.items():
        frame_counts[split] += len(video_frames[v])
        classes_in.setdefault(split, set()).update(video_classes[v])
        instances_in[split] += sum(video_classes[v].values())
    if (instances_in["train"] == 0 or instances_in["val"] == 0
            or instances_in["test"] == 0):
        return None  # gate failed: every split needs labelled instances
    present = {c for vc in video_classes.values()
               for c, n in vc.items() if n > 0}
    if not present <= classes_in.get("train", set()):
        return None  # gate failed: train must contain every class that
        # exists in the data, or the model can never learn the missing ones
    coverage = len(classes_in.get("val", set())) + len(classes_in.get("test", set()))
    deviation = sum(abs(frame_counts[s] / total - ratios[s]) for s in ratios)
    # Ratio fit dominates: a split that starves train of frames is useless
    # however many classes val/test happen to cover.
    return coverage * 2 - deviation * 5


def all_assignments(videos, forced, splits):
    """List every assignment of free videos to splits (forced ones fixed).

    Returns None when there are too many free videos to enumerate;
    the caller then falls back to greedy search.
    """
    free = [v for v in videos if v not in forced]
    if len(free) > 10:
        return None
    results = []
    def rec(i, current):
        if i == len(free):
            results.append(dict(current))
            return
        for split in splits:
            current[free[i]] = split
            rec(i + 1, current)
        del current[free[i]]
    rec(0, dict(forced))
    return results


def assign_videos(video_frames, video_classes, ratios, seed):
    forced = {}
    for v in video_frames:
        if sum(video_classes[v].values()) == 0:
            forced[v] = "train"
    rng = random.Random(seed)
    videos = sorted(video_frames, key=lambda v: -len(video_frames[v]))
    passing = []
    enumerated = all_assignments(videos, forced, list(ratios))
    if enumerated is not None:
        # Few videos: score EVERY legal assignment. A greedy pass cannot
        # be trusted here -- when the smallest split's target is smaller
        # than the smallest video, greedy never fills that split at all.
        for candidate in enumerated:
            score = score_assignment(candidate, video_frames,
                                     video_classes, ratios)
            if score is not None:
                passing.append((score, candidate))
    else:
        for attempt in range(300):
            order = list(videos)
            if attempt > 0:
                rng.shuffle(order)
            candidate = greedy_assign(order, video_frames, ratios, forced)
            score = score_assignment(candidate, video_frames,
                                     video_classes, ratios)
            if score is not None:
                passing.append((score, candidate))
    best = None
    if passing:
        best_score = max(s for s, _ in passing)
        # Among near-best splits (same class coverage, slightly worse ratio
        # fit is acceptable), let the seed pick -- so changing --seed really
        # can change the split, without ever accepting a bad one.
        near_best, seen_keys = [], set()
        for score, candidate in passing:
            if score >= best_score - 1.0:
                key = tuple(sorted(candidate.items()))
                if key not in seen_keys:
                    seen_keys.add(key)
                    near_best.append(candidate)
        best = rng.choice(near_best)
    if best is None:
        # No ordering satisfied the gates; fall back to the plain greedy
        # result so the caller still gets a dataset, but say so loudly.
        best = greedy_assign(videos, video_frames, ratios, forced)
        print("WARNING: no split satisfied the gates (val/test positives). "
              "Check split_report.json before training.")
    return best, forced


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", required=True,
                    help="directory with images, labels/ and manifest.json")
    ap.add_argument("--out", default="marker_dataset")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--ratios", default="0.7,0.2,0.1",
                    help="train,val,test frame ratios")
    args = ap.parse_args()

    frames_dir = Path(args.frames)
    labels_dir = frames_dir / "labels"
    if not labels_dir.is_dir():
        raise SystemExit(f"labels directory not found: {labels_dir} "
                         "(annotate first, YOLO .txt format)")
    manifest = {}
    manifest_path = frames_dir / "manifest.json"
    if manifest_path.exists():
        for item in json.loads(manifest_path.read_text(encoding="utf-8")):
            manifest[item["frame"]] = item["video"]

    images = sorted(p for p in frames_dir.iterdir()
                    if p.suffix.lower() in {".jpg", ".jpeg", ".png"})
    if not images:
        raise SystemExit(f"no images in {frames_dir}")

    video_frames = defaultdict(list)
    video_classes = defaultdict(Counter)
    missing_labels = []
    for img in images:
        label_path = labels_dir / (img.stem + ".txt")
        if not label_path.exists():
            missing_labels.append(img.name)
            continue
        video = video_of(img.name, manifest)
        video_frames[video].append(img)
        video_classes[video].update(count_label_file(label_path))
    if missing_labels:
        print(f"warning: {len(missing_labels)} images without labels skipped, "
              f"e.g. {missing_labels[:3]}")
    if len(video_frames) < 3:
        raise SystemExit("need videos in at least 3 groups to fill "
                         "train/val/test without leakage")

    parts = [float(x) for x in args.ratios.split(",")]
    ratios = {"train": parts[0], "val": parts[1], "test": parts[2]}
    assignment, forced = assign_videos(video_frames, video_classes,
                                       ratios, args.seed)
    if forced:
        print(f"pure-negative videos forced to train: {sorted(forced)}")

    out = Path(args.out)
    for split in ratios:
        (out / "images" / split).mkdir(parents=True, exist_ok=True)
        (out / "labels" / split).mkdir(parents=True, exist_ok=True)

    report = {"videos": {}, "forced_train": sorted(forced), "splits": {}}
    class_counts = {s: Counter() for s in ratios}
    frame_counts = Counter()
    for video, split in sorted(assignment.items()):
        report["videos"][video] = split
        for img in video_frames[video]:
            shutil.copy2(img, out / "images" / split / img.name)
            label_src = labels_dir / (img.stem + ".txt")
            shutil.copy2(label_src, out / "labels" / split / label_src.name)
            frame_counts[split] += 1
            class_counts[split].update(count_label_file(label_src))
    for split in ratios:
        report["splits"][split] = {
            "frames": frame_counts[split],
            "instances": {CLASS_NAMES[c]: class_counts[split][c]
                          for c in sorted(CLASS_NAMES)},
        }

    data_yaml = {
        "path": str(out.resolve()),
        "train": "images/train",
        "val": "images/val",
        "test": "images/test",
        "names": CLASS_NAMES,
    }
    (out / "data.yaml").write_text(
        yaml.safe_dump(data_yaml, allow_unicode=True, sort_keys=False),
        encoding="utf-8")
    (out / "split_report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report["splits"], ensure_ascii=False, indent=2))
    print(f"dataset written to {out} (data.yaml + split_report.json)")


if __name__ == "__main__":
    main()
