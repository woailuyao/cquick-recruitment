"""生成任务四使用的合成练习视频，不调用摄像头，也不修改原有图片。

生成：python src/make_demo_video.py
重新生成：python src/make_demo_video.py --force

素材固定存放在 data/videos/，检测结果应由 task4_camera.py 写入 outputs/task4/。
视频是 640×480、10 FPS、120 帧的 MJPG AVI，共 12 秒。
四个阶段各 3 秒：移动目标、小目标、暗光漏检、同色干扰。
这段程序生成的数据仅用于验证流程，不代表真实摄像头测试。
"""

import argparse
from collections import Counter
from contextlib import contextmanager
import hashlib
import os
from pathlib import Path
import shutil
import tempfile

import cv2
import numpy as np

from common import PROJECT_ROOT, write_json
from make_demo_data import hsv_color, TARGET_HSV
from task3_color_detect import detect, load_config


WIDTH, HEIGHT = 640, 480
FPS = 10
FRAMES_PER_STAGE = 30
TOTAL_FRAMES = 4 * FRAMES_PER_STAGE
DEFAULT_SEED = 20261020
STAGES = (
    ('moving_target', '移动蓝色矩形，默认参数应检出一个候选。', 1),
    ('small_target', '移动的小蓝色圆，面积仍高于 500，默认参数应检出一个候选。', 1),
    ('dark_target', '蓝色矩形的 V=50，低于默认下限 70，预期漏检。', 0),
    ('same_color_distractor', '矩形是真实目标，右侧更大的蓝色圆为干扰，预期两个候选。', 2),
)


@contextmanager
def temporary_ascii_video_paths():
    """让 OpenCV 只接收 ASCII 相对路径，避开 Windows 中文视频路径问题。

    Python 的 chdir 支持中文目录。临时切换目录后，VideoWriter/VideoCapture
    使用 demo_color_motion.avi 即可，完成验证后再用 pathlib/shutil 复制到项目。
    finally 保证发生异常也恢复原工作目录，不影响后续文件保存。
    """
    previous = Path.cwd()
    # 临时目录放在素材目录内，便于受限环境运行；验证后自动清理。
    temp_parent = (PROJECT_ROOT / 'data/videos').resolve()
    temp_parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='_building_', dir=temp_parent) as temp_dir:
        if not Path(temp_dir).resolve().is_relative_to(temp_parent):
            raise RuntimeError('临时目录超出素材目录。')
        try:
            os.chdir(temp_dir)
            yield Path(temp_dir)
        finally:
            os.chdir(previous)


def make_frame(frame_index, rng):
    """按帧号绘制一张 BGR 图，并返回真实目标的位置，方便核对结果。"""
    stage_index = frame_index // FRAMES_PER_STAGE
    local_index = frame_index % FRAMES_PER_STAGE
    progress = local_index / (FRAMES_PER_STAGE - 1)
    blue = hsv_color(*TARGET_HSV)
    image = np.full((HEIGHT, WIDTH, 3), 235, np.uint8)

    if stage_index == 0:
        # 目标向右移动；橙色和绿色块是不会通过蓝色阈值的背景物。
        x, y, width, height = round(90 + 310 * progress), 200, 120, 90
        cv2.rectangle(image, (40, 345), (160, 410), hsv_color(18, 220, 220), -1)
        cv2.rectangle(image, (440, 345), (580, 410), hsv_color(60, 180, 185), -1)
        image[y:y + height, x:x + width] = blue
        center = [x + (width - 1) / 2, y + (height - 1) / 2]
        target_shape = 'rectangle'

    elif stage_index == 1:
        # 使用有符号整数相加，防止 uint8 背景加噪声后溢出。
        noise = rng.integers(-4, 5, (HEIGHT, WIDTH, 1), dtype=np.int16)
        image = np.clip(image.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        cx = round(200 + 240 * progress)
        cy = round(240 + 30 * np.sin(progress * np.pi * 2))
        radius = 24
        cv2.circle(image, (cx, cy), radius, blue, -1)
        x, y, width, height = cx - radius, cy - radius, 49, 49
        center = [cx, cy]
        target_shape = 'circle'

    elif stage_index == 2:
        image[:] = 90
        x, y, width, height = round(140 + 180 * progress), 210, 120, 90
        image[y:y + height, x:x + width] = hsv_color(110, 200, 50)
        center = [x + (width - 1) / 2, y + (height - 1) / 2]
        target_shape = 'rectangle'

    else:
        x, y, width, height = round(80 + 30 * progress), 220, 120, 90
        image[y:y + height, x:x + width] = blue
        # 干扰圆与目标保持分离，且面积更大；直接选最大轮廓会选错。
        cv2.circle(image, (480, 260), 75, blue, -1)
        center = [x + (width - 1) / 2, y + (height - 1) / 2]
        target_shape = 'rectangle'

    # OpenCV 内置字体使用 ASCII；画面清楚标明合成来源及当前阶段。
    cv2.rectangle(image, (0, 0), (WIDTH - 1, 65), (248, 248, 248), -1)
    cv2.putText(image, f'SYNTHETIC / {STAGES[stage_index][0]}', (14, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, (35, 35, 35), 1, cv2.LINE_AA)
    cv2.putText(image, f'Frame {frame_index + 1}/{TOTAL_FRAMES} | {frame_index / FPS:.1f} s',
                (14, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (35, 35, 35), 1, cv2.LINE_AA)
    return image, {
        'frame_index': frame_index,
        'time_seconds': round(frame_index / FPS, 1),
        'stage': STAGES[stage_index][0],
        'true_target_shape': target_shape,
        'true_target_center_xy_before_compression': center,
        'true_target_bbox_xywh_before_compression': [x, y, width, height],
    }


def verify_video(filename, config):
    """重读全部帧并逐帧检测，防止只生成了文件却没有有效视频内容。"""
    cap = cv2.VideoCapture(filename)
    decoded_counts = []
    try:
        if not cap.isOpened():
            raise RuntimeError('生成的视频无法打开。')
        if abs(cap.get(cv2.CAP_PROP_FPS) - FPS) > 0.01:
            raise RuntimeError('视频帧率与预设不符。')
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if frame.shape != (HEIGHT, WIDTH, 3):
                raise RuntimeError(f'解码后的帧尺寸异常：{frame.shape}')
            decoded_counts.append(len(detect(frame, config)['detections']))
    finally:
        cap.release()

    if len(decoded_counts) != TOTAL_FRAMES:
        raise RuntimeError(f'视频应有 {TOTAL_FRAMES} 帧，实际读到 {len(decoded_counts)} 帧。')
    for index, count in enumerate(decoded_counts):
        expected = STAGES[index // FRAMES_PER_STAGE][2]
        if count != expected:
            raise RuntimeError(f'第 {index} 帧候选数为 {count}，预期为 {expected}。')
    return decoded_counts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed', type=int, default=DEFAULT_SEED,
                        help='背景噪声随机种子，默认 20261020。')
    parser.add_argument('--force', action='store_true',
                        help='允许替换本脚本生成的同名 AVI 和 JSON。')
    args = parser.parse_args()
    if args.seed < 0:
        parser.error('--seed 必须为非负整数。')

    # 统一素材目录；脚本从任何工作目录启动，默认保存位置都不变。
    video_dir = PROJECT_ROOT / 'data/videos'
    destination = video_dir / 'demo_color_motion.avi'
    manifest_path = video_dir / 'demo_color_motion.json'
    if any(path.exists() for path in (destination, manifest_path)) and not args.force:
        parser.error('素材已存在；要重新生成请加 --force。不会覆盖其他视频。')
    if any(path.is_dir() for path in (destination, manifest_path)):
        parser.error('目标文件位置是目录，请检查 data/videos。')

    config = load_config(PROJECT_ROOT / 'src/configs/blue.json')
    rng = np.random.default_rng(args.seed)
    ground_truth = []
    with temporary_ascii_video_paths() as temp_dir:
        filename = 'demo_color_motion.avi'
        writer = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*'MJPG'),
                                 FPS, (WIDTH, HEIGHT))
        try:
            if not writer.isOpened():
                raise RuntimeError('MJPG 视频写入器无法打开，请检查 OpenCV 安装。')
            for index in range(TOTAL_FRAMES):
                frame, truth = make_frame(index, rng)
                writer.write(frame)
                ground_truth.append(truth)
        finally:
            # 释放写入器，才能完成 AVI 文件尾部并开始读取验证。
            writer.release()
        decoded_counts = verify_video(filename, config)
        video_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(temp_dir / filename, destination)

    stages = []
    for index, (name, description, expected) in enumerate(STAGES):
        first, stop = index * FRAMES_PER_STAGE, (index + 1) * FRAMES_PER_STAGE
        stages.append({
            'name': name,
            'description_zh': description,
            'frame_range_start_inclusive_end_exclusive': [first, stop],
            'time_range_seconds_start_inclusive_end_exclusive': [first / FPS, stop / FPS],
            'expected_candidate_count_default_config': expected,
            'observed_candidate_count_histogram': dict(Counter(decoded_counts[first:stop])),
        })
    write_json(manifest_path, {
        'synthetic': True,
        'provenance_zh': '由 OpenCV 几何绘图和 NumPy 生成的合成教学视频，不是实拍或摄像头记录。',
        'source_script': 'src/make_demo_video.py',
        'video_path': 'data/videos/demo_color_motion.avi',
        'sha256': hashlib.sha256(destination.read_bytes()).hexdigest(),
        'seed': args.seed,
        'codec': 'MJPG',
        'frame_size_wh': [WIDTH, HEIGHT],
        'fps': FPS,
        'frame_count': TOTAL_FRAMES,
        'duration_seconds': TOTAL_FRAMES / FPS,
        'normal_target_hsv_before_compression': list(TARGET_HSV),
        'dark_target_hsv_before_compression': [110, 200, 50],
        'coordinate_system': '像素坐标；左上角为原点，x 向右，y 向下；帧号从 0 开始。',
        'compression_note_zh': 'MJPG 有损压缩会略微改变颜色和边界；以下真值为压缩前绘图值。',
        'default_detector_config': config,
        'validation': {'all_frames_decoded': True, 'decoded_frame_count': len(decoded_counts),
                       'candidate_counts_match_expectations': True,
                       'camera_used': False, 'personal_practice_record': False},
        'stages': stages,
        'ground_truth_per_frame': ground_truth,
    })
    print(f'素材：{destination}')
    print(f'说明和逐帧真值：{manifest_path}')
    print(f'已重读 {TOTAL_FRAMES}/{TOTAL_FRAMES} 帧，尺寸和帧率正常。')
    print('四阶段默认候选数：1、1、0、2；已逐帧验证。')
    print('这是合成视频的程序测试，没有使用摄像头，也不是本人实操记录。')


if __name__ == '__main__':
    main()
