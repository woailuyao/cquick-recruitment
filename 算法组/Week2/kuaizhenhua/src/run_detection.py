"""在个人目录运行：python3 src/run_detection.py；默认检测公开照片。"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "color_target_vision"))
from color_target_vision.offline import main

if __name__ == "__main__":
    raise SystemExit(main(project_root=ROOT))
