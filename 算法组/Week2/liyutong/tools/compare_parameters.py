"""Run controlled parameter comparisons on user-supplied images; never invent analysis."""
import argparse
import csv
from dataclasses import replace
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'color_target_detector'))
from color_target_detector.detector import DetectorConfig
from color_target_detector.detect_images import main as detect_images


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('images', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--hsv-lower', nargs=3, type=int, default=[5, 100, 80])
    parser.add_argument('--hsv-upper', nargs=3, type=int, default=[25, 255, 255])
    parser.add_argument('--min-area', type=float, default=100)
    parser.add_argument('--saturation-b', type=int, default=160)
    parser.add_argument('--area-b', type=float, default=1000)
    args = parser.parse_args()
    try:
        base = DetectorConfig(hsv_lower=args.hsv_lower, hsv_upper=args.hsv_upper, min_area=args.min_area)
        saturation = replace(base, hsv_lower=(base.hsv_lower[0], args.saturation_b, base.hsv_lower[2]))
        area = replace(base, min_area=args.area_b)
        if saturation == base or area == base:
            raise ValueError('B values must differ from baseline for both comparisons')
    except ValueError as exc:
        parser.error(str(exc))
    # Do not replace existing experiment evidence on accidental repeated commands.
    if args.output.exists():
        parser.error('output already exists; use a new directory for this experiment')
    args.output.mkdir(parents=True)
    rows = []
    failed = False
    for label, config in [('baseline', base), ('saturation_B', saturation), ('area_B', area)]:
        directory = args.output / label
        status = detect_images([*[str(p) for p in args.images], '--output', str(directory),
                                '--hsv-lower', *map(str, config.hsv_lower),
                                '--hsv-upper', *map(str, config.hsv_upper),
                                '--min-area', str(config.min_area)])
        failed = failed or status != 0
        report = directory / 'results.json'
        if not report.is_file():
            failed = True
            continue
        for record in json.loads(report.read_text(encoding='utf-8'))['results']:
            rows.append({'variant': label, 'input': record['input'], 'width': record.get('width', ''),
                         'height': record.get('height', ''), 'hsv_lower': list(config.hsv_lower),
                         'hsv_upper': list(config.hsv_upper), 'min_area': config.min_area,
                         **{key: record[key] for key in ('detected', 'u', 'v', 'area', 'error')},
                         'observed_error_or_change': '', 'explanation': ''})
    if rows:
        with (args.output / 'comparison.csv').open('w', encoding='utf-8-sig', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    print('Compare baseline against saturation_B and baseline against area_B separately.')
    print('Inspect original images, masks and annotations; fill observations and explanations yourself.')
    return 1 if failed else 0


if __name__ == '__main__':
    raise SystemExit(main())
