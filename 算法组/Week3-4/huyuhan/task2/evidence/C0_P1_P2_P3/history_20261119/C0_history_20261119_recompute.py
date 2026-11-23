#!/usr/bin/env python3
# Recompute auditable summary tables from the preserved 2026-11-19 static_segments.json.
# Run from the history_20261119 directory:
#   python3 C0_history_20261119_recompute.py

from pathlib import Path
import csv
import json
import math
import sys

HERE = Path(__file__).resolve().parent
SRC = HERE / "static_segments.json"

# These full-flow counters are transcribed from CONSOLIDATED_POSE_RESULT.md.
# They are NOT fixed-window detector recall: the source report says the denominator
# includes repeated/stale active records.
FULL_FLOW = {
    "P1": {"valid": 5748, "source_records": 15027, "wait": 132, "selection": 1253, "continuous_valid_segments": 1},
    "P2": {"valid": 3884, "source_records": 6221,  "wait": 201, "selection": 1814, "continuous_valid_segments": 5},
    "P3": {"valid": 1362, "source_records": 6729,  "wait": 85,  "selection": 293,  "continuous_valid_segments": 1},
}

def write_csv(path, rows, fieldnames):
    with path.open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)

def main():
    if not SRC.exists():
        sys.exit(f"Missing {SRC.name}. Put this script beside static_segments.json.")

    data = json.loads(SRC.read_text(encoding="utf-8"))
    summary = []
    conditional = []

    for point in ("P1", "P2", "P3"):
        d = data[point]
        flow = FULL_FLOW[point]
        selected = int(d["selected"])
        valid = int(d["valid"])
        rms = d["rms_px"]
        jitter = d["jitter_xy_m"]
        xyz = d["pose_xyz_m"]

        summary.append({
            "point": point,
            "run": d["run"],
            "recording": d["recording"],
            "first_frame": d["first_frame"],
            "last_frame": d["last_frame"],
            "fixed_window_selected_frames": selected,
            "fixed_window_valid_frames": valid,
            "fixed_window_valid_rate_pct": round(100.0 * valid / selected, 4) if selected else "",
            "pose_x_m": xyz[0],
            "pose_y_m": xyz[1],
            "pose_z_m": xyz[2],
            "yaw_deg": d["yaw_deg"],
            "window_capture_duration_s": d["window_capture_duration_s"],
            "loop_fps_median": d["loop_fps_median"],
            "processed_per_second": d["processed_per_second"],
            "rms_mean_px": rms["mean"],
            "rms_median_px": rms["median"],
            "rms_p95_px": rms["p95"],
            "jitter_mean_m": jitter["mean"],
            "jitter_median_m": jitter["median"],
            "jitter_p95_m": jitter["p95"],
            "full_flow_valid_records": flow["valid"],
            "full_flow_source_records": flow["source_records"],
            "full_flow_valid_source_record_ratio_pct": round(100.0 * flow["valid"] / flow["source_records"], 4),
            "wait_records": flow["wait"],
            "selection_records": flow["selection"],
            "continuous_valid_segments": flow["continuous_valid_segments"],
            "formal_error_status": "HALF_SIGNS_CONFIRMED_PLUSX_PLUSX; HISTORICAL_MANUAL_PNP_ONLY; NOT_CURRENT_AUTO_YOLO_C0",
        })

        for c in d["conditionals"]:
            conditional.append({
                "point": point,
                "camera_half": "+X" if c["camera_half"] == 1 else "-X",
                "A_half": "+X" if c["umbrella_half"] == 1 else "-X",
                "reference_x_m": c["reference_xy"][0],
                "reference_y_m": c["reference_xy"][1],
                "A_x_m": c["umbrella_xy"][0],
                "A_y_m": c["umbrella_xy"][1],
                "reference_yaw_deg": c["approx_yaw_deg"],
                "position_error_mean_m": c["position_error_m"]["mean"],
                "position_error_median_m": c["position_error_m"]["median"],
                "position_error_p95_m": c["position_error_m"]["p95"],
                "absolute_yaw_error_mean_deg": c["approx_abs_yaw_delta_deg"]["mean"],
                "absolute_yaw_error_median_deg": c["approx_abs_yaw_delta_deg"]["median"],
                "absolute_yaw_error_p95_deg": c["approx_abs_yaw_delta_deg"]["p95"],
                "status": ("CONFIRMED_FIELD_CONDITION_HISTORICAL_ONLY" if c["camera_half"] == 1 and c["umbrella_half"] == 1 else "ALTERNATIVE_FIELD_CONDITION_NOT_SELECTED"),
            })

    summary_fields = list(summary[0].keys())
    conditional_fields = list(conditional[0].keys())
    write_csv(HERE / "C0_history_20261119_stability_summary.csv", summary, summary_fields)
    write_csv(HERE / "C0_history_20261119_conditional_errors.csv", conditional, conditional_fields)

    print("PASS: parsed P1/P2/P3")
    print(f"PASS: wrote {len(summary)} rows to C0_history_20261119_stability_summary.csv")
    print(f"PASS: wrote {len(conditional)} rows to C0_history_20261119_conditional_errors.csv")
    print("INFO: +X/+X field signs are user-confirmed for P1/P2/P3; alternative rows remain for audit.")
    print("WARNING: full-flow valid/source-record ratios use the source report's denominator, which includes repeated/stale active records.")
    print("WARNING: historical manual PnP does not prove current-installation YOLO C0 accuracy.")
    print("WARNING: yaw reference is approximate; angular uncertainty is not quantified.")
    print("WARNING: P95 XY jitter is within-window stability, not absolute position accuracy.")

if __name__ == "__main__":
    main()
