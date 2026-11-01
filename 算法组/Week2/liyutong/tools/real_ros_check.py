#!/usr/bin/env python3
"""Actual photo /target_position consistency check; source Humble + installed overlay.

Creates a NEW output directory. Uses installed nodes, no source import override.
60-second active deadline; only owned process groups are cleaned up.
This checks detector/ROS consistency, not real-world detection accuracy.
"""
import argparse
from datetime import datetime
import getpass
import hashlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
import platform
import pwd
import re
import signal
import subprocess
import sys
import time
import traceback

RECORD = re.compile(r'count=(\d+) detected=(True|False) u=([^ ]+) v=([^ ]+) area=([^ ]+) status=(\w+) invalid_count=(\d+)')


def require(value, message):
    if not value:
        raise RuntimeError(message)


def now():
    return datetime.now().astimezone().isoformat()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--images-dir', required=True)
    args = parser.parse_args()
    output = Path(args.output).absolute()
    output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    deadline = started + 60
    children, handles, messages = [], [], []
    node = ros = None
    summary = dict(status='RUNNING', started_local=now(), username=getpass.getuser(),
                   uid=os.getuid(), account=pwd.getpwuid(os.getuid()).pw_name,
                   account_gecos=pwd.getpwuid(os.getuid()).pw_gecos,
                   real_photos_tested=True, accuracy_assessed=False,
                   purpose='end-to-end consistency with detect(), not accuracy',
                   topic='/target_position', active_deadline_seconds=60,
                   commands=[], children=[], checks={})

    def header():
        return f'# 姓名: liyutong | actual_username={summary["account"]} uid={os.getuid()} local_timestamp={now()}\n'

    def write(name, text):
        (output / name).write_text(header() + text, encoding='utf-8')

    def remaining():
        left = deadline - time.monotonic()
        require(left > 0, '60-second active deadline exceeded')
        return left

    def spawn(name, command):
        handle = (output / (name + '.log')).open('w', encoding='utf-8', buffering=1)
        handle.write(header())
        handle.flush()
        handles.append(handle)
        proc = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT,
                                start_new_session=True, env=dict(os.environ, PYTHONUNBUFFERED='1'))
        children.append((name, proc))
        summary['children'].append(dict(name=name, pid=proc.pid, process_group=proc.pid, argv=command))
        return proc

    def capture(name, command):
        proc = spawn(name, command)
        result = proc.wait(timeout=min(12, remaining()))
        summary['commands'].append(dict(argv=command, returncode=result, log=name + '.log'))
        require(result == 0, f'{name} command exited {result}')
        return (output / (name + '.log')).read_text(encoding='utf-8')

    def records():
        return list(RECORD.finditer((output / 'subscriber.log').read_text(encoding='utf-8')))

    def spin_until(predicate):
        while not predicate():
            for name, proc in children:
                if name in ('publisher', 'subscriber'):
                    require(proc.poll() is None, f'{name} exited prematurely: {proc.returncode}')
            ros.spin_once(node, timeout_sec=min(.1, remaining()))

    def timeout(_signal, _frame):
        raise TimeoutError('60-second active deadline exceeded')

    previous = signal.signal(signal.SIGALRM, timeout)
    signal.setitimer(signal.ITIMER_REAL, remaining())
    try:
        require(sys.platform.startswith('linux'), 'Linux required')
        require(os.environ.get('ROS_DISTRO') == 'humble', 'Source ROS Humble first')
        require('/home/liyut/ros2_ws/install' in os.environ.get('AMENT_PREFIX_PATH', ''),
                'Source /home/liyut/ros2_ws/install/setup.bash first')
        os.environ.update(ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(output / 'ros_logs'),
                          RCUTILS_COLORIZED_OUTPUT='0', RCUTILS_LOGGING_USE_STDOUT='1')
        import cv2
        import numpy as np
        import rclpy
        from ament_index_python.packages import get_package_prefix
        from target_interfaces.msg import TargetPosition
        import color_target_detector.detector as detector
        ros = rclpy
        summary['versions'] = dict(python=sys.version, platform=platform.platform(),
                                   opencv=cv2.__version__, numpy=np.__version__, ROS_DISTRO=os.environ['ROS_DISTRO'])
        for package in ('rclpy', 'color-target-detector'):
            try:
                summary['versions'][package] = importlib.metadata.version(package)
            except importlib.metadata.PackageNotFoundError:
                import xml.etree.ElementTree as ET
                ros_package = package.replace('-', '_')
                package_xml = Path(get_package_prefix(ros_package)) / 'share' / ros_package / 'package.xml'
                summary['versions'][package] = ET.parse(package_xml).getroot().findtext('version')
            require(bool(summary['versions'][package]), f'Missing version for {package}')
        summary['installed'] = dict(detector_file=detector.__file__,
            detector_prefix=get_package_prefix('color_target_detector'),
            interface_prefix=get_package_prefix('target_interfaces'),
            ament_prefix_path=os.environ['AMENT_PREFIX_PATH'])
        require(cv2.__version__.split('.')[0] == '4', 'Expected OpenCV 4')
        require(TargetPosition.get_fields_and_field_types() ==
                {'detected': 'boolean', 'u': 'float', 'v': 'float', 'area': 'float'}, 'Interface schema mismatch')
        config = detector.DetectorConfig(hsv_lower=(5, 200, 80), hsv_upper=(25, 255, 255),
                                         min_area=100.0, morphology='none', kernel_size=3)
        summary['configuration'] = config.__dict__
        images_dir = Path(args.images_dir)
        require(images_dir.is_absolute() and str(images_dir).startswith('/mnt/'),
                'Image list must use absolute mounted Windows paths')
        expected = []
        paths = [str(images_dir / (name + '.jpg')) for name in ('clear', 'small', 'background', 'lighting', 'absent')]
        for path in paths:
            data = Path(path).read_bytes()
            image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
            require(image is not None, f'Cannot decode {path}')
            result = detector.detect(image, config)
            expected.append(dict(path=path, sha256=hashlib.sha256(data).hexdigest(), shape=list(image.shape),
                                 detected=bool(result.detected), u=result.u, v=result.v, area=result.area,
                                 float32={key: float(np.float32(getattr(result, key))) for key in ('u', 'v', 'area')}))
        summary['expected'] = expected
        require(any(x['detected'] for x in expected) and any(not x['detected'] for x in expected),
                'Input cycle must contain True and False detect() outcomes')
        write('expected_results.log', json.dumps(expected, indent=2))
        write('environment.log', json.dumps({key: summary[key] for key in ('username', 'account', 'account_gecos', 'versions', 'installed', 'configuration')}, indent=2))
        ros.init(args=[])
        node = ros.create_node('real_photo_consistency_observer')

        def receive(message):
            values = {key: getattr(message, key) for key in ('detected', 'u', 'v', 'area')}
            require(all(math.isfinite(values[key]) for key in ('u', 'v', 'area')), 'Non-finite ROS field')
            messages.append(dict(local_timestamp=now(), **values))

        subscription = node.create_subscription(TargetPosition, '/target_position', receive, 100)
        capture('interface_show', ['ros2', 'interface', 'show', 'target_interfaces/msg/TargetPosition'])
        spawn('subscriber', ['ros2', 'run', 'color_target_detector', 'target_subscriber'])
        spin_until(lambda: node.count_subscribers('/target_position') >= 2)
        require(node.count_publishers('/target_position') == 0, 'Unrelated publisher already on /target_position')
        spawn('publisher', ['ros2', 'run', 'color_target_detector', 'target_publisher', '--ros-args',
              '-p', 'images:=' + json.dumps(paths), '-p', 'frequency:=5.0', '-p', 'max_messages:=0',
              '-p', 'hsv_lower:=[5,200,80]', '-p', 'hsv_upper:=[25,255,255]',
              '-p', 'min_area:=100.0', '-p', 'morphology:=none', '-p', 'kernel_size:=3'])
        spin_until(lambda: len(messages) >= 20 and len(records()) >= 20)
        require(node.count_publishers('/target_position') == 1, 'Expected exactly one publisher')
        info = capture('topic_info', ['ros2', 'topic', 'info', '/target_position', '--verbose'])
        require('target_interfaces/msg/TargetPosition' in info, 'Wrong topic type')
        capture('topic_echo_once', ['ros2', 'topic', 'echo', '/target_position', 'target_interfaces/msg/TargetPosition', '--once'])

        def matches(actual, reference, printed=False):
            if actual['detected'] != reference['detected']:
                return False
            return all(abs(actual[key] - reference['float32'][key]) <=
                       max(abs(float(np.spacing(np.float32(reference['float32'][key])))), 0.00000051 if printed else 1e-7)
                       for key in ('u', 'v', 'area'))

        def check_cycle(samples, printed=False):
            offsets = [offset for offset in range(5) if all(
                matches(sample, expected[(index + offset) % 5], printed)
                for index, sample in enumerate(samples))]
            require(offsets, 'Received sequence differs from detect() photo cycle')
            require(any(x['detected'] for x in samples) and any(not x['detected'] for x in samples),
                    'Received sequence missing detection or miss')
            return offsets

        subscriber_samples = []
        parsed = records()
        for index, record in enumerate(parsed):
            require(int(record[1]) == index + 1 and int(record[7]) == 0, 'Invalid subscriber count/status')
            require(record[6] == ('detected' if record[2] == 'True' else 'no_target'), 'Invalid status')
            subscriber_samples.append(dict(detected=record[2] == 'True', u=float(record[3]), v=float(record[4]), area=float(record[5])))
        summary['checks'] = dict(observer_records=len(messages), subscriber_records=len(parsed),
            observer_cycle_offsets=check_cycle(messages), subscriber_cycle_offsets=check_cycle(subscriber_samples, True),
            float32_tolerance='one float32 ULP; console additionally permits 0.51e-6 decimal rounding',
            subscriber_invalid_count=0, both_true_false=True, interface_schema=True, topic_type=True)
        write('observer_messages.log', '\n'.join(json.dumps(message) for message in messages) + '\n')
        remaining()
        summary['status'] = 'PASS'
    except BaseException as exc:
        summary.update(status='FAIL', error=f'{type(exc).__name__}: {exc}')
        write('failure_traceback.log', traceback.format_exc())
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, previous)
        cleanup_errors = []
        for name, proc in children:
            try:
                os.killpg(proc.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            except OSError as exc:
                cleanup_errors.append(f'{name}: {exc}')
        grace = time.monotonic() + 3
        for name, proc in children:
            try:
                proc.wait(timeout=max(0, grace - time.monotonic()))
            except subprocess.TimeoutExpired:
                pass
        for name, proc in children:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            except OSError as exc:
                cleanup_errors.append(f'{name}: {exc}')
            try:
                proc.wait(timeout=.25)
            except subprocess.TimeoutExpired:
                cleanup_errors.append(f'{name}: launcher failed to reap')
        for handle in handles:
            handle.close()
        if node is not None:
            node.destroy_node()
        if ros is not None and ros.ok():
            ros.shutdown()
        summary['cleanup'] = dict(errors=cleanup_errors, children=[dict(name=name, pid=proc.pid, returncode=proc.returncode) for name, proc in children])
        if cleanup_errors:
            summary['status'] = 'FAIL'
        summary.update(finished_local=now(), elapsed_seconds=time.monotonic() - started)
        (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
        print(json.dumps(summary, indent=2))
    return 0 if summary['status'] == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
