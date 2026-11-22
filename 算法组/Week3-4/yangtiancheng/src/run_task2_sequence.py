#!/usr/bin/env python3
"""Run task-2 localization over an ordered image sequence with continuity gating."""
import argparse
import csv
import json
import math
import subprocess
import sys
from pathlib import Path


def angle_delta_deg(a, b):
    return math.degrees(math.atan2(math.sin(math.radians(a-b)),
                                   math.cos(math.radians(a-b))))


def smooth_angle_deg(previous, current, alpha):
    delta = angle_delta_deg(current, previous)
    return previous + alpha * delta


def main():
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", nargs="+", required=True)
    ap.add_argument("--output-dir", required=True)
    ap.add_argument("--initial-prior", nargs=3, type=float, required=True,
                    metavar=("X", "Y", "THETA_DEG"))
    ap.add_argument("--intrinsics", default=str(here.parent / "camera_intrinsics.yaml"))
    ap.add_argument("--extrinsics",
                    default=str(here / "camera_extrinsics_H72p5_G140_IMG0024_provisional.yaml"))
    ap.add_argument("--field", default=str(here / "field_geometry_kid_full.yaml"))
    ap.add_argument("--threshold", type=float, default=.35)
    ap.add_argument("--alpha", type=float, default=.35, help="EMA smoothing factor")
    ap.add_argument("--max-jump-m", type=float, default=.5)
    ap.add_argument("--max-jump-deg", type=float, default=30.0)
    ap.add_argument("--allow-provisional", action="store_true")
    args = ap.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    prior = tuple(args.initial_prior)
    smoothed = prior
    rows = []

    for index, image in enumerate(args.images):
        image_path = Path(image)
        frame_dir = output_dir / image_path.stem
        command = [
            sys.executable, str(here / "run_task2_pipeline.py"),
            "--image", str(image_path),
            "--output-dir", str(frame_dir),
            "--intrinsics", args.intrinsics,
            "--extrinsics", args.extrinsics,
            "--field", args.field,
            "--threshold", str(args.threshold),
            "--prior", *(str(x) for x in prior),
        ]
        if args.allow_provisional:
            command.append("--allow-provisional")
        completed = subprocess.run(command, text=True, capture_output=True)
        result_path = frame_dir / f"{image_path.stem}_localization.json"
        if completed.returncode != 0 or not result_path.exists():
            rows.append({"index": index, "image": str(image_path), "accepted": False,
                         "reason": "pipeline_failed"})
            continue

        result = json.loads(result_path.read_text(encoding="utf-8"))
        solution = result["solutions"][result.get("selected_solution", 0)]
        raw = (solution["x_m"], solution["y_m"], solution["theta_deg"])
        jump_m = math.hypot(raw[0]-prior[0], raw[1]-prior[1])
        jump_deg = abs(angle_delta_deg(raw[2], prior[2]))
        # The initial prior may only encode the correct half/rough heading.
        # Allow a wider angular bootstrap on frame zero; subsequent frames use
        # the strict continuity gate.
        angle_limit = args.max_jump_deg * 2 if index == 0 else args.max_jump_deg
        accepted = jump_m <= args.max_jump_m and jump_deg <= angle_limit
        if accepted:
            if index == 0:
                smoothed = raw
            else:
                smoothed = (
                    smoothed[0] + args.alpha * (raw[0]-smoothed[0]),
                    smoothed[1] + args.alpha * (raw[1]-smoothed[1]),
                    smooth_angle_deg(smoothed[2], raw[2], args.alpha),
                )
            prior = smoothed
        rows.append({
            "index": index, "image": str(image_path), "accepted": accepted,
            "raw_x_m": raw[0], "raw_y_m": raw[1], "raw_theta_deg": raw[2],
            "smooth_x_m": smoothed[0], "smooth_y_m": smoothed[1],
            "smooth_theta_deg": smoothed[2], "jump_m": jump_m,
            "jump_deg": jump_deg, "rms_m": solution["rms_m"],
        })

    (output_dir / "sequence_summary.json").write_text(
        json.dumps(rows, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    fields = sorted({key for row in rows for key in row})
    with (output_dir / "sequence_summary.csv").open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    accepted_count = sum(bool(row.get("accepted")) for row in rows)
    print(f"frames={len(rows)} accepted={accepted_count}")
    print(f"summary={output_dir / 'sequence_summary.json'}")


if __name__ == "__main__":
    main()
