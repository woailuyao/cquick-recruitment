"""核验已生成的合成演示结果；不会重新运行算法或修改输入图片。

运行：python src/verify_demo.py
另一组结果：python src/verify_demo.py --output-dir outputs_python310

核验合成数据上的几何结果、故意失败案例、保存记录及 PNG 可读性。
通过核验不代表已经完成真实照片、摄像头或个人学习任务。
"""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from common import PROJECT_ROOT, read_image, write_json


def read_json(path):
    return Path(path).read_text(encoding='utf-8-sig')


def load_json(path):
    return json.loads(read_json(path))


def project_path(text):
    path = Path(text)
    return path if path.is_absolute() else PROJECT_ROOT / path


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=PROJECT_ROOT / 'outputs')
    args = parser.parse_args()
    output_dir = args.output_dir.resolve()
    checks, task3_metrics = [], []

    def check(name, passed, **details):
        checks.append({'check': name, 'passed': bool(passed), **details})

    def guarded(name, action):
        try:
            action()
        except Exception as exc:
            check(name, False, error=f'{type(exc).__name__}: {exc}')

    try:
        manifest = load_json(PROJECT_ROOT / 'data/demo_manifest.json')
        entries = manifest['images']
        check('manifest_has_five_synthetic_cases', manifest.get('synthetic') is True and len(entries) == 5,
              image_count=len(entries))
    except Exception as exc:
        manifest, entries = {}, []
        check('manifest_readable', False, error=f'{type(exc).__name__}: {exc}')

    source_by_name = {entry['filename']: project_path(entry['image_path']) for entry in entries}

    def verify_detection_aggregate():
        records = load_json(output_dir / 'task3/detections.json')
        names = [record['image'] for record in records]
        check('task3_aggregate_images', len(names) == len(set(names)) and set(names) == set(source_by_name),
              actual_images=names)
        for record in records:
            single = load_json(output_dir / 'task3' / Path(record['image']).stem / 'detections.json')
            check(f"task3_aggregate_matches_individual/{record['image']}", record == single)

    guarded('task3_aggregate_readable', verify_detection_aggregate)

    for index, entry in enumerate(entries):
        def verify_scene(entry=entry, index=index):
            stem = Path(entry['filename']).stem
            folder = output_dir / 'task3' / stem
            record = load_json(folder / 'detections.json')
            detected = read_image(folder / '04_mask_filtered.png', cv2.IMREAD_UNCHANGED)
            truth = read_image(project_path(entry['mask_path']), cv2.IMREAD_UNCHANGED)
            source = read_image(project_path(entry['image_path']))
            detections = record['detections']
            metric = {'image': entry['filename'], 'expected_candidate_count': entry['expected_candidate_count'],
                      'actual_candidate_count': len(detections), 'true_target_center_xy': entry['true_target_center_xy']}
            task3_metrics.append(metric)
            check(f'task3_count/{stem}', len(detections) == entry['expected_candidate_count']
                  and record['candidate_count'] == len(detections), **metric)
            shape_ok = detected.ndim == 2 and truth.ndim == 2 and detected.shape == truth.shape == source.shape[:2]
            check(f'task3_mask_shape/{stem}', shape_ok, detected_shape=list(detected.shape),
                  truth_shape=list(truth.shape), source_shape=list(source.shape))
            check(f'task3_mask_binary/{stem}', np.isin(detected, [0, 255]).all())
            check(f'task3_source_hash/{stem}', record['sha256'] == sha256(project_path(entry['image_path'])))
            if not shape_ok:
                return
            predicted, target = detected != 0, truth != 0
            overlap = int(np.count_nonzero(predicted & target))
            union = int(np.count_nonzero(predicted | target))
            metric.update({'mask_iou': round(overlap / union, 6) if union else None,
                           'mask_white_pixels': int(np.count_nonzero(predicted)),
                           'extra_pixels_outside_true_target': int(np.count_nonzero(predicted & ~target))})
            height, width = detected.shape
            for number, detection in enumerate(detections, start=1):
                cx, cy = detection['center_xy']
                x, y, w, h = detection['bbox_xywh']
                area = detection['contour_area_px2']
                values = (cx, cy, x, y, w, h, area)
                valid = (all(np.isfinite(value) for value in values) and area > 0 and w > 0 and h > 0
                         and x >= 0 and y >= 0 and x + w <= width and y + h <= height
                         and x - .001 <= cx <= x + w - 1 + .001
                         and y - .001 <= cy <= y + h - 1 + .001)
                check(f'task3_detection_geometry/{stem}/{number}', valid, detection=detection)
            if index < 3:
                center_error = (float(np.linalg.norm(np.asarray(detections[0]['center_xy'], dtype=float)
                                                     - np.asarray(entry['true_target_center_xy'], dtype=float)))
                                if len(detections) == 1 else None)
                metric['centroid_error_px'] = round(center_error, 6) if center_error is not None else None
                check(f'task3_normal_mask_iou/{stem}', union > 0 and overlap / union >= .97,
                      value=metric['mask_iou'], minimum=.97)
                check(f'task3_normal_centroid/{stem}', center_error is not None and center_error <= 2,
                      error_px=metric['centroid_error_px'], maximum_px=2)
            elif stem == '04_dark_fail':
                check('task3_dark_target_deliberate_miss', not detections and not np.any(predicted)
                      and np.any(target), mask_white_pixels=metric['mask_white_pixels'])
            elif stem == '05_same_color_fail':
                check('task3_same_color_deliberate_false_positive', len(detections) == 2
                      and metric['extra_pixels_outside_true_target'] > 0,
                      extra_pixels_outside_true_target=metric['extra_pixels_outside_true_target'])

        guarded(f"task3_scene_readable/{entry['filename']}", verify_scene)

    task1_summary = {}

    def verify_task1():
        metadata = load_json(output_dir / 'task1/metadata.json')
        records = metadata['images']
        output_count = 0
        names = [Path(record['source']).name for record in records]
        check('task1_image_count', metadata['image_count'] == len(entries) == len(records)
              and len(set(names)) == len(names) and set(names) == set(source_by_name),
              recorded_image_count=metadata['image_count'], actual_records=len(records))
        check('task1_readback_summary', metadata.get('all_saved_files_read_back_verified') is True)
        for record in records:
            folder = output_dir / 'task1' / record['output_subfolder']
            source_name = Path(record['source']).name
            check(f'task1_three_outputs/{source_name}', set(record['results']) == {'resized', 'crop', 'gray'})
            for name, saved in record['results'].items():
                restored = read_image(folder / saved['file'], cv2.IMREAD_UNCHANGED)
                output_count += 1
                check(f'task1_saved_png/{source_name}/{name}',
                      saved.get('read_back_ok') is True and saved.get('pixels_identical') is True
                      and list(restored.shape) == saved['shape'], shape=list(restored.shape))
            source = read_image(source_by_name[source_name])
            check(f'task1_source_dimensions/{source_name}', list(source.shape) == record['source_shape']
                  and source.shape[1] == record['width'] and source.shape[0] == record['height'])
        expected_count = len(entries) * 3
        check('task1_png_readback_count', output_count == expected_count,
              actual_count=output_count, expected_count=expected_count)
        task1_summary.update({'image_count': len(records), 'verified_output_png_count': output_count})

    guarded('task1_readable', verify_task1)
    task2_summary = {}

    def verify_task2():
        report = load_json(output_dir / 'task2/results.json')
        configurations = report['configurations']
        expected_names = {'baseline', 'gaussian_changed', 'threshold_changed', 'canny_changed', 'morphology_changed'}
        names = [config['name'] for config in configurations]
        check('task2_five_configurations', len(names) == 5 and set(names) == expected_names,
              actual_names=names)
        gray = read_image(output_dir / 'task2/02_grayscale.png', cv2.IMREAD_UNCHANGED)
        check('task2_grayscale', gray.ndim == 2 and list(gray.shape) == report['source_shape'][:2],
              shape=list(gray.shape))
        stage_count = 0
        for config in configurations:
            check(f"task2_four_stages/{config['name']}",
                  set(config['files']) == {'gaussian', 'threshold', 'canny', 'morphology'})
            for stage, filename in config['files'].items():
                restored = read_image(output_dir / 'task2' / filename, cv2.IMREAD_UNCHANGED)
                stage_count += 1
                check(f"task2_stage_shape/{config['name']}/{stage}", restored.ndim == 2 and restored.shape == gray.shape,
                      shape=list(restored.shape))
                if stage != 'gaussian':
                    check(f"task2_stage_binary/{config['name']}/{stage}", np.isin(restored, [0, 255]).all())
        check('task2_stage_count', stage_count == 20, actual_count=stage_count, expected_count=20)
        task2_summary.update({'configuration_count': len(configurations), 'verified_stage_png_count': stage_count,
                              'grayscale_shape': list(gray.shape)})

    guarded('task2_readable', verify_task2)
    source_hash_summary = {'status': 'skipped', 'reason': 'environment.json or source_sha256 not available'}

    def verify_source_hashes():
        env_path = output_dir / 'environment.json'
        if not env_path.is_file():
            return
        environment = load_json(env_path)
        hashes = environment.get('source_sha256')
        if hashes is None:
            return
        check('environment_hash_inventory', set(hashes) == set(source_by_name))
        for name, expected in hashes.items():
            actual = sha256(source_by_name[name])
            check(f'original_source_sha256/{name}', actual == expected,
                  expected_sha256=expected, actual_sha256=actual)
        source_hash_summary.update({'status': 'checked', 'reason': None, 'source_count': len(hashes)})

    guarded('source_hashes_readable', verify_source_hashes)
    png_paths = sorted(output_dir.rglob('*.png'))
    reopened, png_errors = 0, []
    for path in png_paths:
        try:
            restored = read_image(path, cv2.IMREAD_UNCHANGED)
            if restored.size == 0:
                raise ValueError('Empty image')
            reopened += 1
        except Exception as exc:
            png_errors.append({'file': str(path.relative_to(output_dir)), 'error': f'{type(exc).__name__}: {exc}'})
    check('all_output_pngs_reopen', bool(png_paths) and not png_errors,
          found_png_count=len(png_paths), reopened_png_count=reopened, errors=png_errors)
    passed = all(item['passed'] for item in checks)
    result = {
        'passed': passed,
        'scope': 'Synthetic demo geometry, deliberate failures, saved-output records and PNG readability; not real-world accuracy or personal learning completion.',
        'output_directory': str(output_dir),
        'check_count': len(checks),
        'failed_check_count': sum(not item['passed'] for item in checks),
        'png_count': len(png_paths),
        'reopened_png_count': reopened,
        'task1': task1_summary,
        'task2': task2_summary,
        'task3': task3_metrics,
        'source_hashes': source_hash_summary,
        'checks': checks,
    }
    write_json(output_dir / 'verification.json', result)
    print(f'Verification passed={passed}; checks={len(checks)}; PNGs reopened={reopened}/{len(png_paths)}')
    for metric in task3_metrics:
        print(f"{metric['image']}: candidates={metric['actual_candidate_count']}, IoU={metric.get('mask_iou')}, centroid_error_px={metric.get('centroid_error_px')}")
    for item in checks:
        if not item['passed']:
            print(f"FAILED: {item['check']} {item.get('error', '')}")
    print(f'Saved: {output_dir / "verification.json"}')
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()
