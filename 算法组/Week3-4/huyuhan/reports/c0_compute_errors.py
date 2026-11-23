#!/usr/bin/env python3
"""Compute C0 XY/Yaw errors from independent reference and automatic estimates.
Required CSV columns: point, ref_x, ref_y, ref_yaw, est_x, est_y, est_yaw.
Use actual independent references in meters/degrees; do not invent values.
"""
import csv
import math
import sys
from pathlib import Path

COLUMNS = ("point", "ref_x", "ref_y", "ref_yaw", "est_x", "est_y", "est_yaw")

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 c0_compute_errors.py REAL_C0.csv")
        return 2
    file = Path(sys.argv[1])
    with file.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or not set(COLUMNS).issubset(reader.fieldnames):
            raise ValueError("Missing required CSV headers: " + ", ".join(COLUMNS))
        rows = list(reader)
    seen = set()
    print("point,error_xy_m,error_yaw_deg,within_0_5m_10deg")
    for row in rows:
        name = row["point"].strip()
        if name in seen:
            raise ValueError("Duplicate point " + name)
        seen.add(name)
        rx, ry, rt, ex, ey, et = [float(row[c]) for c in COLUMNS[1:]]
        if not all(map(math.isfinite, [rx, ry, rt, ex, ey, et])):
            raise ValueError("Nonfinite measurement at " + name)
        position = math.hypot(ex-rx, ey-ry)
        yaw = abs((et - rt + 180) % 360 - 180)
        print(f"{name},{position:.6f},{yaw:.6f},{position <= .5 and yaw <= 10}")
    if not {"P1", "P2", "P3"}.issubset(seen):
        print("WARNING: P1/P2/P3 not all present. No 3-point pass claim.",file=sys.stderr)
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
