#!/usr/bin/env python3
"""Real ROS 2 console smoke check using SYNTHETIC DEVELOPMENT fixtures only.

Run in Ubuntu after sourcing ROS Humble and the colcon install overlay:
  python3 tools/ros2_smoke_check.py --output ~/ros2_smoke_synthetic_001

The output path must not exist. No source-tree import override is used. This is
not real-photo assessment evidence. Requires installed color_target_detector,
target_interfaces, rclpy, rcl_interfaces, OpenCV, NumPy, and the ros2 executable.
The active phase has a 40-second deadline; spawned process groups get at most
3 seconds after SIGINT before SIGKILL. No root privileges are needed.
"""

import argparse
from datetime import datetime, timezone
import importlib.metadata
import json
import math
import os
from pathlib import Path
import platform
import re
import shutil
import signal
import subprocess
import sys
import time
import traceback
import uuid


ACTIVE_SECONDS = 40.0
LOG_RECORD = re.compile(
    r'count=(\d+) detected=(True|False) u=([^ ]+) v=([^ ]+) '
    r'area=([^ ]+) status=(\w+) invalid_count=(\d+)'
)


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def require(condition, description):
    if not condition:
        raise RuntimeError(description)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True,
                        help='New/nonexisting directory for synthetic fixtures and actual runtime logs')
    parser.add_argument('--default-topic', action='store_true',
                        help='Use /target_position without topic remapping; stop unrelated publishers first')
    args = parser.parse_args(argv)
    output = Path(args.output).expanduser().absolute()
    try:
        # mkdir without exist_ok also rejects a preexisting symlink or directory.
        output.mkdir(parents=True, exist_ok=False)
    except OSError as exc:
        print(f'FAIL: cannot create new output directory {output}: {exc}', file=sys.stderr)
        return 1

    started = time.monotonic()
    deadline = started + ACTIVE_SECONDS
    summary = {
        'status': 'RUNNING', 'kind': 'SYNTHETIC DEVELOPMENT ROS 2 SMOKE CHECK',
        'synthetic_fixtures': True, 'real_photos_tested': False,
        'final_real_photo_evidence': False, 'ros2_runtime_tested': False,
        'started_utc': utc_now(), 'output': str(output),
        'active_timeout_seconds': ACTIVE_SECONDS, 'cleanup_sigint_seconds': 3.0,
        'checks': {}, 'commands': [], 'children': [],
        'versions': {'python': sys.version, 'platform': platform.platform(),
                     'ROS_DISTRO': os.environ.get('ROS_DISTRO'),
                     'RMW_IMPLEMENTATION': os.environ.get('RMW_IMPLEMENTATION', '(default)'),
                     'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '(default)')},
    }
    children = []
    logs = []
    node = None
    ros = None
    ros_initialized = False
    alarm_installed = False
    previous_alarm_handler = None
    messages = []
    success = False

    def remaining():
        value = deadline - time.monotonic()
        require(value > 0, '40-second active phase deadline exceeded')
        return value

    def check_children():
        for name, proc in children:
            require(proc.poll() is None,
                    f'{name} exited unexpectedly ({proc.returncode}); inspect its log')

    def spin_until(predicate, description):
        while not predicate():
            check_children()
            ros.spin_once(node, timeout_sec=min(0.1, remaining()))
        remaining()

    def command_capture(command, filename):
        remaining()
        record = {'argv': command, 'started_utc': utc_now()}
        summary['commands'].append(record)
        tick = time.monotonic()
        try:
            result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, timeout=min(8.0, remaining()), check=False)
            (output / filename).write_text(result.stdout, encoding='utf-8')
            record.update(returncode=result.returncode, log=filename,
                          elapsed_seconds=time.monotonic() - tick)
            require(result.returncode == 0, f'{command!r} failed; inspect {filename}')
            return result.stdout
        except subprocess.TimeoutExpired as exc:
            text = exc.output or b''
            if isinstance(text, bytes):
                text = text.decode('utf-8', errors='replace')
            (output / filename).write_text(text, encoding='utf-8')
            record.update(timed_out=True, elapsed_seconds=time.monotonic() - tick)
            raise

    def subscriber_records():
        text = (output / 'subscriber.log').read_text(encoding='utf-8', errors='replace')
        return list(LOG_RECORD.finditer(text))

    try:
        require(sys.platform.startswith('linux'), 'Run this script inside Ubuntu/Linux, not Windows')
        require(os.environ.get('ROS_DISTRO') == 'humble', 'Source ROS 2 Humble before running')
        require(shutil.which('ros2') is not None, 'ros2 executable is missing from PATH')
        require(bool(os.environ.get('AMENT_PREFIX_PATH')), 'Source the colcon install overlay first')

        def alarm_timeout(_signum, _frame):
            raise TimeoutError('40-second active phase deadline exceeded')

        previous_alarm_handler = signal.signal(signal.SIGALRM, alarm_timeout)
        signal.setitimer(signal.ITIMER_REAL, remaining())
        alarm_installed = True
        # Limit this check and its children to local DDS discovery.
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        os.environ['ROS_LOG_DIR'] = str(output / 'ros_logs')
        import cv2
        import numpy as np
        import rclpy
        from ament_index_python.packages import get_package_prefix
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        from rcl_interfaces.srv import SetParameters
        from target_interfaces.msg import TargetPosition
        import color_target_detector

        ros = rclpy
        summary['versions'].update(opencv=cv2.__version__, numpy=np.__version__)
        for distribution in ('rclpy', 'color-target-detector'):
            try:
                summary['versions'][distribution] = importlib.metadata.version(distribution)
            except importlib.metadata.PackageNotFoundError:
                summary['versions'][distribution] = 'not exposed through Python distribution metadata'
        prefix = Path(get_package_prefix('color_target_detector'))
        summary['installed_package'] = {
            'prefix': str(prefix), 'module_file': str(color_target_detector.__file__),
            'interface_prefix': get_package_prefix('target_interfaces'),
            'ament_prefix_path': os.environ['AMENT_PREFIX_PATH'],
            'python_path': list(sys.path),
        }
        for executable in ('target_publisher', 'target_subscriber'):
            require((prefix / 'lib' / 'color_target_detector' / executable).is_file(),
                    f'Installed console executable missing: {executable}; build/source the overlay')
        fields = TargetPosition.get_fields_and_field_types()
        require(fields == {'detected': 'boolean', 'u': 'float', 'v': 'float', 'area': 'float'},
                f'Unexpected installed TargetPosition schema: {fields}')
        summary['checks']['installed_message_schema'] = fields

        fixtures = output / 'SYNTHETIC_DEV_FIXTURES_NOT_REAL_PHOTOS'
        fixtures.mkdir()
        (fixtures / 'README.txt').write_text(
            'Generated synthetic development fixtures only. NOT real-photo final evidence.\n'
            'Orange disks: largest centroid (200,120), radius 30; smaller disk (40,40).\n'
            'Blank: no target. min_area=10000 must reject both fixtures.\n', encoding='utf-8')
        orange = tuple(int(x) for x in cv2.cvtColor(
            np.uint8([[[15, 220, 220]]]), cv2.COLOR_HSV2BGR)[0, 0])
        target = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2.circle(target, (200, 120), 30, orange, -1)
        cv2.circle(target, (40, 40), 8, orange, -1)
        paths = []
        for name, image in (('synthetic_target.png', target),
                            ('synthetic_blank.png', np.zeros_like(target))):
            path = fixtures / name
            ok, encoded = cv2.imencode('.png', image)
            require(ok, f'Could not encode {name}')
            path.write_bytes(encoded.tobytes())
            paths.append(str(path))
        summary['fixtures'] = {'paths': paths, 'expected_centroid': [200.0, 120.0],
                               'synthetic': True, 'hsv_color': [15, 220, 220]}
        namespace = '/synthetic_smoke_' + uuid.uuid4().hex[:12]
        topic = '/target_position' if args.default_topic else namespace + '/target_position'
        summary.update(namespace=namespace, topic=topic, default_topic_without_remap=args.default_topic)
        ros.init(args=[])
        ros_initialized = True
        node = ros.create_node('smoke_observer', namespace=namespace)

        def receive(message):
            require(type(message) is TargetPosition, 'Received object is not installed TargetPosition')
            require(type(message.detected) is bool, 'detected is not a bool')
            values = (message.u, message.v, message.area)
            require(all(type(value) is float and math.isfinite(value) for value in values),
                    'u/v/area must be actual finite Python float fields')
            if message.detected:
                require(abs(message.u - 200.0) <= 0.01 and abs(message.v - 120.0) <= 0.01,
                        f'Unexpected synthetic centroid: {values}')
                require(2500.0 < message.area < 10000.0, f'Unexpected synthetic area: {message.area}')
            else:
                require(values == (0.0, 0.0, 0.0), f'Missed target fields are not all zero: {values}')
            messages.append({'elapsed_seconds': time.monotonic() - started,
                             'detected': message.detected, 'u': message.u,
                             'v': message.v, 'area': message.area})

        subscription = node.create_subscription(TargetPosition, topic, receive, 10)
        summary['subscription_type'] = 'target_interfaces/msg/TargetPosition'
        remaps = ['--ros-args', '-r', '__ns:=' + namespace]
        if not args.default_topic:
            remaps += ['-r', '/target_position:=' + topic]
        child_env = dict(os.environ, PYTHONUNBUFFERED='1', RCUTILS_COLORIZED_OUTPUT='0',
                         RCUTILS_LOGGING_USE_STDOUT='1')
        # Avoid middleware logs outside the newly created output directory.
        child_env['ROS_LOG_DIR'] = str(output / 'ros_logs')
        for name, executable, extra in (
            ('subscriber', 'target_subscriber', []),
            ('publisher', 'target_publisher',
             ['-p', 'images:=' + json.dumps(paths), '-p', 'frequency:=10.0',
              '-p', 'max_messages:=0', '-p', 'min_area:=100.0']),
        ):
            log = (output / (name + '.log')).open('w', encoding='utf-8', buffering=1)
            logs.append(log)
            command = ['ros2', 'run', 'color_target_detector', executable] + remaps + extra
            proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                    start_new_session=True, env=child_env)
            children.append((name, proc))
            summary['children'].append({'name': name, 'pid': proc.pid,
                                        'process_group': proc.pid, 'argv': command})
        spin_until(lambda: len(messages) >= 12, '12 baseline messages')
        baseline = list(messages[:12])
        require(any(m['detected'] for m in baseline), 'No True detection among 12 baseline messages')
        require(any(not m['detected'] for m in baseline), 'No False detection among 12 baseline messages')
        spin_until(lambda: len(subscriber_records()) >= 10, 'subscriber console log >=10 messages')
        require(node.count_publishers(topic) == 1, 'Expected exactly one publisher on isolated topic')
        require(node.count_subscribers(topic) >= 2, 'Parent and console subscriber must both be present')
        summary['checks']['baseline_12_messages'] = baseline
        summary['ros2_runtime_tested'] = True

        client = node.create_client(SetParameters, namespace + '/target_publisher/set_parameters')
        spin_until(client.service_is_ready, 'publisher SetParameters service discovery')

        def set_min_area(value):
            request = SetParameters.Request()
            request.parameters = [Parameter(name='min_area', value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=value))]
            tick = time.monotonic()
            future = client.call_async(request)
            spin_until(future.done, f'min_area={value} service result')
            response = future.result()
            require(response is not None and len(response.results) == 1,
                    'SetParameters did not return exactly one result')
            result = response.results[0]
            record = {'value': value, 'successful': bool(result.successful),
                      'reason': result.reason, 'elapsed_seconds': time.monotonic() - tick}
            summary['checks'].setdefault('dynamic_parameters', []).append(record)
            return result

        require(set_min_area(10000.0).successful, 'Valid dynamic min_area=10000.0 was rejected')
        # Drain already queued baseline samples for 0.8s (8 publish periods).
        settle_end = time.monotonic() + 0.8
        spin_until(lambda: time.monotonic() >= settle_end, 'in-flight sample settling')
        after_settle = len(messages)
        spin_until(lambda: len(messages) >= after_settle + 12, '12 post-update messages')
        filtered = messages[after_settle:after_settle + 12]
        require(all(not m['detected'] for m in filtered),
                'Dynamic min_area=10000.0 did not make all 12 post-settle samples misses')
        summary['checks']['post_update_12_misses'] = filtered
        require(not set_min_area(-1.0).successful, 'Invalid min_area=-1.0 was accepted')
        after_rejection = len(messages)
        spin_until(lambda: len(messages) >= after_rejection + 6, '6 messages after rejected update')
        require(all(not m['detected'] for m in messages[after_rejection:after_rejection + 6]),
                'Rejected min_area update changed active detection behavior')
        summary['checks']['rejected_update_preserved_filter'] = True

        interface_text = command_capture(
            ['ros2', 'interface', 'show', 'target_interfaces/msg/TargetPosition'], 'interface_show.log')
        for declaration in ('bool detected', 'float32 u', 'float32 v', 'float32 area'):
            require(declaration in interface_text, f'Interface CLI output missing {declaration}')
        topic_text = command_capture(['ros2', 'topic', 'info', topic, '--verbose'], 'topic_info.log')
        require('target_interfaces/msg/TargetPosition' in topic_text,
                'Topic info did not report the custom TargetPosition type')
        summary['checks']['interface_show'] = True
        summary['checks']['topic_info'] = True
        records = subscriber_records()
        require(len(records) >= 10 and int(records[-1].group(1)) >= 10,
                'Console subscriber has fewer than 10 received-message log records')
        require(all(int(record.group(7)) == 0 and record.group(6) != 'invalid' for record in records),
                'Console subscriber reported invalid messages')
        require(any(record.group(2) == 'True' for record in records) and
                any(record.group(2) == 'False' for record in records),
                'Console subscriber did not log both detections and misses')
        summary['checks']['console_subscriber'] = {
            'logged_records': len(records), 'count': int(records[-1].group(1)), 'invalid_count': 0,
            'both_detected_and_missed': True}
        check_children()
        remaining()
        success = True
    except BaseException as exc:
        summary['error'] = f'{type(exc).__name__}: {exc}'
        (output / 'failure_traceback.log').write_text(traceback.format_exc(), encoding='utf-8')
    finally:
        if alarm_installed:
            signal.setitimer(signal.ITIMER_REAL, 0)
            signal.signal(signal.SIGALRM, previous_alarm_handler)
        # Only groups created by Popen(start_new_session=True) above are signaled.
        # Signal every owned group, even if its ros2 launcher has already exited.
        cleanup_started = time.monotonic()
        cleanup_errors = []
        for name, proc in children:
            try:
                os.killpg(proc.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            except OSError as exc:
                cleanup_errors.append(f'{name} SIGINT: {exc}')
        grace_end = cleanup_started + 3.0
        for name, proc in children:
            try:
                proc.wait(timeout=max(0.0, grace_end - time.monotonic()))
            except subprocess.TimeoutExpired:
                pass
        # A launcher may exit before its console child: kill the remaining group,
        # rather than using launcher poll() as proof that every descendant stopped.
        for name, proc in children:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            except OSError as exc:
                cleanup_errors.append(f'{name} SIGKILL: {exc}')
        for name, proc in children:
            try:
                proc.wait(timeout=0.25)
            except subprocess.TimeoutExpired:
                cleanup_errors.append(f'{name} launcher did not reap after SIGKILL')
        for log in logs:
            log.close()
        try:
            if node is not None:
                node.destroy_node()
            if ros_initialized and ros.ok():
                ros.shutdown()
        except BaseException as exc:
            cleanup_errors.append(f'ROS cleanup: {exc}')
        if cleanup_errors:
            success = False
        summary['cleanup'] = {'elapsed_seconds': time.monotonic() - cleanup_started,
                              'errors': cleanup_errors,
                              'returncodes': {name: proc.returncode for name, proc in children}}
        summary['received_message_count'] = len(messages)
        (output / 'received_messages.json').write_text(
            json.dumps(messages, indent=2) + '\n', encoding='utf-8')
        summary.update(status='PASS' if success else 'FAIL', finished_utc=utc_now(),
                       elapsed_seconds=time.monotonic() - started)
        (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    print(f"{summary['status']}: SYNTHETIC DEVELOPMENT ONLY; {output / 'summary.json'}")
    if not success:
        print(summary.get('error', 'Cleanup failed; inspect summary.json'), file=sys.stderr)
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
