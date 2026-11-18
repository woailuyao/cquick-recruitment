
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import yaml

BASE = os.path.dirname(os.path.abspath(__file__))
FIELD_PATH = os.path.normpath(os.path.join(BASE, "..", "..", "config", "field.yaml"))
OUT_PATH = os.path.normpath(os.path.join(BASE, "..", "..", "outputs", "field_map.png"))

ALIASES = {
    "PenalityPoint": "PenaltyPoint",
}

def load_field(path):
    raise NotImplementedError

def plot_field(points, out_path):
    raise NotImplementedError

def main():
    points = load_field(FIELD_PATH)
    print(f"读到 {len(points)} 个点")
    for name, (x, y) in points.items():
        print(f"  {name:16s} x={x:7.3f}  y={y:7.3f}")

    plot_field(points, OUT_PATH)
    print(f"图已存到 {OUT_PATH}")

if __name__ == "__main__":
    main()
