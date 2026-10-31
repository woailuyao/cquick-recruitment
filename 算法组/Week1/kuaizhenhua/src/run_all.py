"""一键运行必做任务一至三；不调用摄像头，不修改原始图片。"""

import argparse
import hashlib
import platform
from pathlib import Path
import subprocess
import sys

import cv2
import numpy as np
import matplotlib

from common import PROJECT_ROOT, list_images, write_json, safe_output_dir


def main():
    """按顺序运行三个脚本，核对原图不变，并保存运行环境记录。"""
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        '--input-dir', type=Path, default=PROJECT_ROOT / 'data/images'
    )
    p.add_argument(
        '--output-dir', type=Path, default=PROJECT_ROOT / 'outputs'
    )
    p.add_argument(
        '--config', type=Path, default=PROJECT_ROOT / 'src/configs/blue.json'
    )
    a = p.parse_args()

    # 整套练习至少使用三张图；输出目录不能与原图目录相交。
    paths = list_images(a.input_dir)
    if len(paths) < 3:
        p.error('Full-week run requires at least 3 images.')
    out = safe_output_dir(a.output_dir, a.input_dir)

    # 记录运行前的文件哈希，所有任务结束后据此检查原图是否改变。
    hashes = {
        str(path): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in paths
    }

    for name in ('task1_basics', 'task2_processing', 'task3_color_detect'):
        task = name.split('_')[0]
        # 使用当前解释器，避免三个脚本意外进入不同的 Python 环境。
        # 各任务分别写入 task1、task2、task3 子目录；不启动选做摄像头。
        cmd = [
            sys.executable,
            str(PROJECT_ROOT / 'src' / f'{name}.py'),
            '--input-dir',
            str(a.input_dir.resolve()),
            '--output-dir',
            str(out / task),
        ]
        if task == 'task3':
            cmd += ['--config', str(a.config.resolve())]

        # 某一任务失败时立即停止，避免把不完整的结果记成全部成功。
        subprocess.run(cmd, check=True)

    if any(
        hashlib.sha256(Path(path).read_bytes()).hexdigest() != value
        for path, value in hashes.items()
    ):
        raise RuntimeError('Source image changed during processing.')

    # 环境记录与结果放在同一输出目录，便于之后复现实验。
    # 程序执行通过不代表真实场景准确，也不替代本人看图与学习记录。
    write_json(out / 'environment.json', {
        'python': platform.python_version(),
        'opencv': cv2.__version__,
        'numpy': np.__version__,
        'matplotlib': matplotlib.__version__,
        'platform': platform.platform(),
        'source_images_unchanged': True,
        'input_count': len(paths),
        'source_sha256': {Path(k).name: v for k, v in hashes.items()},
        'scope': (
            'execution check, not a claim of real-world accuracy '
            'or completed personal learning'
        ),
    })
    print(
        'All required scripts completed; source images unchanged. '
        f'Results: {out}'
    )


if __name__ == '__main__':
    main()
