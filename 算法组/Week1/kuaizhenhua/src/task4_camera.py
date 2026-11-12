"""任务四：把任务三的颜色定位用于连续视频，并保存可复查的结果。

在项目根目录运行：
    python src/task4_camera.py --video data/videos/demo_color_motion.avi
    python src/task4_camera.py --camera 0

按空格暂停/继续，S 保存当前帧，Q 或 Esc 结束。
每次运行在 outputs/task4/ 下新建一个带时间的目录，不覆盖上次实验。
输出 annotated.avi、frames.jsonl、summary.json 和 snapshots/，各自含义见下方代码。
--no-display 用于程序核验，不代表本人已观看视频或做过摄像头实测。
"""

from __future__ import annotations

import argparse
from collections import Counter
from contextlib import contextmanager
from datetime import datetime, timedelta, timezone
import json
import math
import os
from pathlib import Path
import shutil
import time

import cv2

from common import PROJECT_ROOT, save_image, write_json
from task3_color_detect import detect, load_config

RESULT_WINDOW = 'Task4 - result | Space pause | S snapshot | Q quit'
MASK_WINDOW = 'Task4 - filtered mask'


def wait_for_key(delay_ms):
    """处理按键和窗口关闭；暂停时点窗口的叉号也能正常结束。"""
    key = cv2.waitKey(delay_ms) & 0xFF
    try:
        for name in (RESULT_WINDOW, MASK_WINDOW):
            if cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE) < 1:
                return ord('q')
    except cv2.error:
        # 部分后端在窗口已销毁时抛出异常，把这种情况按退出处理。
        return ord('q')
    return key


def build_parser():
    """输入来源明确二选一，避免误开启摄像头。"""
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--video', type=Path, help='输入视频；练习素材放在 data/videos/')
    source.add_argument('--camera', type=int, help='摄像头编号，例如 0；与 --video 二选一')
    parser.add_argument('--config', type=Path,
                        default=PROJECT_ROOT / 'src/configs/blue.json', help='HSV 与面积参数文件')
    parser.add_argument('--output-dir', type=Path, default=PROJECT_ROOT / 'outputs/task4',
                        help='任务四输出根目录，每次在其中创建新运行目录')
    parser.add_argument('--save-every', type=int, default=10,
                        help='每隔多少帧自动保存一组原帧、掩膜、标注图；默认 10')
    parser.add_argument('--max-frames', type=int, default=0,
                        help='最多处理多少帧；0 表示视频结束或手动退出')
    parser.add_argument('--no-display', action='store_true', help='不弹窗、不等待回放，用于程序核验')
    return parser


@contextmanager
def video_folder(path):
    """仅在打开编解码器时用相对文件名，兼容部分 Windows 中文目录问题。

    项目输入、输出路径在这之前均已转换成绝对路径，打开后立即恢复工作目录。
    此工具用于本脚本的单线程主流程，不应在多个线程中同时调用。
    """
    previous = Path.cwd()
    try:
        os.chdir(path.parent)
        yield path.name
    finally:
        os.chdir(previous)


def open_video(path):
    with video_folder(path) as filename:
        capture = cv2.VideoCapture(filename)
    return capture


def new_run_directory(parent, source_type):
    """只允许把任务四结果写在项目 outputs/task4 内，并避免同秒运行覆盖。"""
    parent = parent.resolve()
    allowed = (PROJECT_ROOT / 'outputs/task4').resolve()
    if parent != allowed and allowed not in parent.parents:
        raise ValueError('任务四结果请放在本项目 outputs/task4/ 内。')
    stamp = datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f')
    folder = parent / f'{source_type}_{stamp}'
    folder.mkdir(parents=True, exist_ok=False)
    return folder


def save_snapshot(folder, frame_index, frame, detection, saved):
    """一组图片放同一子目录，名称与逐帧记录中的帧号一致，便于对应查看。"""
    if frame_index in saved:
        return
    destination = folder / 'snapshots' / f'frame_{frame_index:06d}'
    save_image(destination / '01_input.png', frame)
    save_image(destination / '02_mask.png', detection['mask_filtered'])
    save_image(destination / '03_result.png', detection['result'])
    saved.add(frame_index)


def verify_saved_video(path, expected_frames):
    """视频编码器关闭后重读所有帧，确认保存的结果能解码且没有少帧。"""
    capture = open_video(path)
    decoded = 0
    try:
        if not capture.isOpened():
            raise RuntimeError('输出视频保存后无法重新打开。')
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            if frame is None or frame.size == 0:
                raise RuntimeError('输出视频包含空帧。')
            decoded += 1
    finally:
        capture.release()
    if decoded != expected_frames:
        raise RuntimeError(f'输出视频应有 {expected_frames} 帧，实际重读 {decoded} 帧。')
    return decoded


def main():
    parser = build_parser()
    args = parser.parse_args()
    if args.save_every < 1 or args.max_frames < 0:
        parser.error('--save-every 至少为 1，--max-frames 不得为负数。')
    if args.camera is not None and args.camera < 0:
        parser.error('摄像头编号不能为负数。')
    config_path = args.config.resolve()
    config = load_config(config_path)
    source_path = args.video.resolve() if args.video else None
    if source_path and not source_path.is_file():
        parser.error(f'视频不存在：{source_path}')
    source_type = 'video' if source_path else 'camera'
    folder = new_run_directory(args.output_dir, source_type)
    print(f'本次结果目录：{folder}', flush=True)
    print('空格：暂停/继续；S：保存当前帧；Q / Esc：结束。', flush=True)

    capture = None
    writer = None
    temporary_input = None
    frame_count = 0
    saved = set()
    histogram = Counter()
    end_reason = 'not_started'
    error_message = None
    saved_video_frames = 0
    width = height = 0
    fps = 15.0
    expected_input_frames = 0
    last_frame = last_detection = None
    started_at = datetime.now(timezone(timedelta(hours=8))).isoformat(timespec='seconds')
    started_clock = time.perf_counter()
    try:
        if source_path:
            # 中文文件名也先用 Python 复制成临时 ASCII 文件名；只读原视频。
            decoder_path = source_path
            if not source_path.name.isascii():
                temporary_input = folder / ('_decoder_input' + source_path.suffix)
                shutil.copyfile(source_path, temporary_input)
                decoder_path = temporary_input
            capture = open_video(decoder_path)
        else:
            capture = cv2.VideoCapture(args.camera)
        if not capture.isOpened():
            raise RuntimeError('无法打开输入。请检查视频路径/编码，或摄像头编号与权限。')
        measured_fps = capture.get(cv2.CAP_PROP_FPS)
        if math.isfinite(measured_fps) and 0 < measured_fps <= 240:
            fps = float(measured_fps)
        metadata_frames = capture.get(cv2.CAP_PROP_FRAME_COUNT)
        if source_path and math.isfinite(metadata_frames) and metadata_frames > 0:
            expected_input_frames = int(round(metadata_frames))

        # JSONL 一行一帧，视频较长时也无需把全部记录保存在内存中。
        with (folder / 'frames.jsonl').open('w', encoding='utf-8') as log:
            while True:
                frame_started = time.perf_counter()
                ok, frame = capture.read()
                if not ok:
                    if not source_path:
                        raise RuntimeError('摄像头读帧失败，已保留此前结果。')
                    if not frame_count:
                        raise RuntimeError('视频打开了但没有可解码的画面。')
                    if expected_input_frames and frame_count < expected_input_frames:
                        raise RuntimeError('视频提前停止解码，请检查文件是否损坏或编码是否受支持。')
                    end_reason = 'end_of_video'
                    break
                if frame is None or frame.size == 0:
                    raise RuntimeError('读到了空帧，停止处理。')
                if writer is None:
                    height, width = frame.shape[:2]
                    # MJPG/AVI 便于 OpenCV 跨环境读取，输出文件名固定放在本次目录内。
                    with video_folder(folder / 'annotated.avi') as filename:
                        writer = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*'MJPG'),
                                                 fps, (width, height))
                    if not writer.isOpened():
                        raise RuntimeError('无法创建输出视频，请检查磁盘空间和 MJPG 编码支持。')
                if frame.shape[:2] != (height, width):
                    raise RuntimeError('输入中途改变了画面尺寸，不能直接写入同一个视频。')

                # 先完成当前帧的检测和视频写入，再更新“已处理帧数”。
                # 检测过程中中断时，收尾仍使用上一张完整帧及其正确编号。
                next_index = frame_count + 1
                detection = detect(frame, config)
                count = len(detection['detections'])
                # 视频时间从 0 开始；摄像头时间采用实际经过的秒数。
                time_seconds = ((next_index - 1) / fps if source_path
                                else time.perf_counter() - started_clock)
                annotated = detection['result'].copy()
                status = f'Frame {next_index} | t={time_seconds:.1f}s | candidates={count}'
                cv2.putText(annotated, status, (10, height - 14), cv2.FONT_HERSHEY_SIMPLEX,
                            0.48, (0, 0, 255), 1, cv2.LINE_AA)
                writer.write(annotated)
                frame_count, last_frame, last_detection = next_index, frame, detection
                histogram[count] += 1
                log.write(json.dumps({
                    'frame_index': frame_count, 'time_seconds': round(time_seconds, 3),
                    'candidate_count': count, 'detections': detection['detections'],
                }, ensure_ascii=False) + '\n')
                if (frame_count - 1) % args.save_every == 0:
                    save_snapshot(folder, frame_count, frame, detection, saved)

                if not args.no_display:
                    cv2.imshow(RESULT_WINDOW, annotated)
                    cv2.imshow(MASK_WINDOW, detection['mask_filtered'])
                    # 按源视频帧率回放；较慢的机器可能无法达到源帧率，但不会主动跳帧。
                    elapsed_ms = (time.perf_counter() - frame_started) * 1000
                    delay_ms = max(1, round(1000 / fps - elapsed_ms)) if source_path else 1
                    key = wait_for_key(delay_ms)
                    if key == ord(' '):
                        while True:
                            key = wait_for_key(50)
                            if key in (ord('s'), ord('S')):
                                save_snapshot(folder, frame_count, frame, detection, saved)
                            if key in (ord(' '), ord('q'), ord('Q'), 27):
                                break
                    if key in (ord('s'), ord('S')):
                        save_snapshot(folder, frame_count, frame, detection, saved)
                    if key in (ord('q'), ord('Q'), 27):
                        end_reason = 'user_quit'
                        break
                if args.max_frames and frame_count >= args.max_frames:
                    end_reason = 'max_frames'
                    break
    except KeyboardInterrupt:
        end_reason = 'keyboard_interrupt'
    except Exception as exc:
        end_reason = 'error'
        error_message = f'{type(exc).__name__}: {exc}'
    finally:
        # 无论正常退出还是报错，都释放设备、写完视频文件并保存运行摘要。
        if capture is not None:
            capture.release()
        if writer is not None:
            writer.release()
        if not args.no_display:
            cv2.destroyAllWindows()
        if temporary_input is not None:
            temporary_input.unlink(missing_ok=True)
        try:
            if frame_count:
                save_snapshot(folder, frame_count, last_frame, last_detection, saved)
                saved_video_frames = verify_saved_video(folder / 'annotated.avi', frame_count)
        except Exception as exc:
            end_reason = 'error'
            error_message = (error_message + '; ' if error_message else '') + str(exc)
        summary = {
            'source_type': source_type,
            'source': str(source_path) if source_path else args.camera,
            'started_at': started_at, 'input_fps': fps,
            'frame_index_base': 1,
            'coordinate_system': '图像像素；左上角为原点，x 向右、y 向下。',
            'area_definition': '外轮廓几何面积，单位像素平方；不等于白色像素数量。',
            'input_frame_count_from_metadata': expected_input_frames,
            'frame_size_wh': [width, height], 'processed_frames': frame_count,
            'candidate_count_histogram': dict(sorted(histogram.items())),
            'config_path': str(config_path), 'config': config,
            'display_enabled': not args.no_display, 'end_reason': end_reason,
            'error': error_message, 'saved_snapshot_frames': sorted(saved),
            'output_video_frames_verified': saved_video_frames,
            'files': {'annotated_video': 'annotated.avi', 'frame_records': 'frames.jsonl',
                      'snapshots': 'snapshots/'},
            'snapshot_files': {'input': '01_input.png', 'mask': '02_mask.png',
                               'result': '03_result.png'},
            'scope': '程序执行记录；是否本人观察、是否实拍须另行记录，候选数量不等于准确率。',
        }
        write_json(folder / 'summary.json', summary)

    print(f'处理 {frame_count} 帧，保存 {len(saved)} 组代表帧；结束原因：{end_reason}')
    print(f'Saved task4 results: {folder}', flush=True)
    if error_message:
        raise SystemExit(error_message)


if __name__ == '__main__':
    main()
