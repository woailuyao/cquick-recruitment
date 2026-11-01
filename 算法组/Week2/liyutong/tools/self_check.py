"""Developer checks on synthetic images; these are NOT real-scene assessment evidence."""
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'color_target_detector'))
import cv2
import numpy as np
from color_target_detector.detector import DetectorConfig, detect
from color_target_detector.detect_images import main as run_images


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'dev_checks' / 'synthetic_only')
    args = parser.parse_args()
    out = args.output
    out.mkdir(parents=True, exist_ok=True)
    # Use known HSV rather than guessing a BGR orange.
    orange = tuple(int(x) for x in cv2.cvtColor(np.uint8([[[15, 220, 220]]]), cv2.COLOR_HSV2BGR)[0, 0])
    image = np.zeros((240, 320, 3), np.uint8)
    cv2.circle(image, (200, 120), 30, orange, -1)
    cv2.circle(image, (40, 40), 8, orange, -1)
    result = detect(image)
    assert result.detected and abs(result.u - 200) < .01 and abs(result.v - 120) < .01
    assert result.area > 2500
    blank = np.zeros_like(image)
    missed = detect(blank)
    assert not missed.detected and (missed.u, missed.v, missed.area) == (0, 0, 0)
    assert not detect(image, DetectorConfig(min_area=10000)).detected
    # Degenerate line has zero contour moment even with min_area=0.
    line = blank.copy()
    cv2.line(line, (20, 20), (100, 20), orange, 1)
    assert not detect(line, DetectorConfig(min_area=0)).detected
    red_hsv = np.zeros((60, 100, 3), np.uint8)
    red_hsv[10:30, 10:30] = (178, 255, 255)
    red_hsv[10:40, 50:80] = (2, 255, 255)
    red = cv2.cvtColor(red_hsv, cv2.COLOR_HSV2BGR)
    wrapped = detect(red, DetectorConfig(hsv_lower=(170, 100, 80), hsv_upper=(10, 255, 255)))
    assert wrapped.detected and abs(wrapped.u - 64.5) < .01
    assert wrapped.mask[20, 20] == 255 and wrapped.mask[20, 60] == 255
    for changes in ({'min_area': -1}, {'hsv_lower': (180, 0, 0)}, {'kernel_size': 0}):
        try:
            DetectorConfig(**changes)
        except ValueError:
            pass
        else:
            raise AssertionError(f'Invalid config accepted: {changes}')
    for mode in ('none', 'open', 'close', 'open_close'):
        assert detect(image, DetectorConfig(morphology=mode)).detected
    paths = []
    for name, pixels in [('合成目标.png', image), ('合成无目标.png', blank)]:
        good, encoded = cv2.imencode('.png', pixels)
        assert good
        path = out / name
        path.write_bytes(encoded.tobytes())
        paths.append(str(path))
    assert run_images(paths + ['--output', str(out / 'results')]) == 0
    report = json.loads((out / 'results' / 'results.json').read_text(encoding='utf-8'))
    assert [r['detected'] for r in report['results']] == [True, False]
    for record in report['results']:
        assert record['width'] == 320 and record['height'] == 240
        for kind in ('annotated', 'mask'):
            decoded = cv2.imdecode(np.frombuffer(Path(record[kind]).read_bytes(), np.uint8), cv2.IMREAD_UNCHANGED)
            assert decoded is not None and decoded.shape[:2] == (240, 320)
    assert run_images([str(out / 'missing.png'), '--output', str(out / 'expected_error')]) == 1
    for package in ('target_interfaces', 'color_target_detector'):
        ET.parse(ROOT / 'src' / package / 'package.xml')
    for source in (ROOT / 'src').rglob('*.py'):
        compile(source.read_text(encoding='utf-8'), str(source), 'exec')
    message = (ROOT / 'src' / 'target_interfaces' / 'msg' / 'TargetPosition.msg').read_text(encoding='utf-8')
    assert message.splitlines() == ['bool detected', 'float32 u', 'float32 v', 'float32 area']
    summary = {'kind': 'synthetic developer checks only', 'opencv': cv2.__version__, 'numpy': np.__version__,
               'checks': ['largest contour centroid', 'no target zeros', 'area filtering', 'zero moment',
                          'red hue wrap', 'invalid configuration', 'morphology modes', 'Unicode image IO',
                          'saved PNG decode', 'CLI unreadable input failure', 'Python syntax', 'XML', 'message schema'],
               'ros2_runtime_tested': False, 'real_photos_tested': False}
    (out / 'check_summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
