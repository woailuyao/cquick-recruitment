"""公用文件读写。默认路径跟随项目，支持 Windows 中文文件名。"""

from pathlib import Path
import json

import cv2
import numpy as np

# src 的上一级就是项目根目录，整体移动或解压后仍能找到默认素材与结果。
PROJECT_ROOT = Path(__file__).resolve().parents[1]
IMAGE_SUFFIXES = {'.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff', '.webp'}


def read_image(path, flags=cv2.IMREAD_COLOR):
    """读取图片并检查结果；默认返回 BGR 彩色数组，失败时给出路径信息。"""
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f'Image not found: {path}')

    # 普通路径使用课内的 imread；Windows 非 ASCII 路径使用同一解码器的替代入口。
    if str(path).isascii():
        image = cv2.imread(str(path), flags)
    else:
        # 先由 NumPy 读文件字节，再交给 OpenCV 解码，避免中文文件名兼容问题。
        image = cv2.imdecode(np.fromfile(path, dtype=np.uint8), flags)
    if image is None or image.size == 0:
        raise ValueError(f'Cannot decode image: {path}')
    return image


def save_image(path, image):
    """保存并重读图片；检查尺寸、通道数，PNG 还逐像素检查无损保存。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if str(path).isascii():
        ok = cv2.imwrite(str(path), image)
        if not ok:
            raise OSError(f'imwrite failed: {path}')
    else:
        # 用扩展名选择编码格式，再由 NumPy 写入中文路径。
        ok, encoded = cv2.imencode(path.suffix, image)
        if not ok:
            raise OSError(f'imencode failed: {path}')
        encoded.tofile(path)

    # IMREAD_UNCHANGED 保留原通道数，灰度图不会在检查时被转成三通道。
    restored = read_image(path, cv2.IMREAD_UNCHANGED)
    if restored.shape != image.shape:
        raise OSError(f'Image shape changed on saving: {path}')
    # PNG 应完整保留像素；JPEG 等有损格式不要求逐像素相同。
    if path.suffix.lower() == '.png' and not np.array_equal(restored, image):
        raise OSError(f'PNG roundtrip mismatch: {path}')


def list_images(folder):
    """按文件名排序列出目录内的图片，不递归查找子目录。"""
    folder = Path(folder)
    if not folder.is_dir():
        raise FileNotFoundError(f'Image folder not found: {folder}')
    paths = sorted(
        (
            p for p in folder.iterdir()
            if p.is_file() and p.suffix.lower() in IMAGE_SUFFIXES
        ),
        key=lambda p: p.name.casefold(),
    )
    if not paths:
        raise ValueError(f'No supported images in: {folder}')
    # 每张图片以 stem 建子目录，阻止同名不同扩展名相互覆盖。
    stems = [p.stem.casefold() for p in paths]
    if len(set(stems)) != len(stems):
        raise ValueError('Use unique image names (e.g. photo01.jpg and photo02.png).')
    return paths


def project_relative_records(value):
    """递归转换记录中的项目内绝对路径，使整个文件夹移动后仍可复现。

    只转换路径本身，不替换普通说明文字；项目外输入路径保持原值。
    运行时仍可使用绝对路径，写入 JSON 时统一为相对项目根的正斜杠路径。
    """
    if isinstance(value, dict):
        return {
            project_relative_records(key): project_relative_records(item)
            for key, item in value.items()
        }
    if isinstance(value, (list, tuple)):
        return [project_relative_records(item) for item in value]
    if isinstance(value, Path):
        value = str(value)
    if isinstance(value, str):
        candidate = Path(value)
        if candidate.is_absolute():
            try:
                return candidate.resolve().relative_to(PROJECT_ROOT.resolve()).as_posix()
            except ValueError:
                pass
    return value


def write_json(path, obj):
    """保存可读的 UTF-8 JSON，并把项目内部路径记录为相对路径。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(project_relative_records(obj), ensure_ascii=False, indent=2),
        encoding='utf-8',
    )


def safe_output_dir(folder, input_folder):
    """创建输出目录，禁止与素材目录相同或互相嵌套，避免覆盖与混入原图。"""
    folder, input_folder = Path(folder).resolve(), Path(input_folder).resolve()
    if (
        folder == input_folder or input_folder in folder.parents
        or folder in input_folder.parents
    ):
        raise ValueError(
            'Input and output folders must be separate; do not overwrite originals.'
        )
    folder.mkdir(parents=True, exist_ok=True)
    return folder


def montage(items, columns=3, tile_width=400):
    """items 为 (ASCII 标题, 图像)；等比例缩放，不改变原始输出。"""
    tile_height = 330
    tiles = []
    for label, img in items:
        # 统一成三通道，才能把灰度图与彩色图放在同一张对比图上。
        bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) if img.ndim == 2 else img
        tile = np.full((tile_height, tile_width, 3), 245, np.uint8)
        scale = min(tile_width / bgr.shape[1], (tile_height - 36) / bgr.shape[0])
        # 等比例适应格子宽高，顶部留 36 像素写标题，不拉伸原图比例。
        resized = cv2.resize(
            bgr,
            (
                max(1, round(bgr.shape[1] * scale)),
                max(1, round(bgr.shape[0] * scale)),
            ),
        )
        y, x = 36, (tile_width - resized.shape[1]) // 2
        tile[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
        cv2.putText(
            tile, label, (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
            .5, (45, 45, 45), 1, cv2.LINE_AA,
        )
        tiles.append(tile)

    # 最后一行不足 columns 个时补空白格，再先横向、后纵向拼接。
    while len(tiles) % columns:
        tiles.append(np.full_like(tiles[0], 245))
    return np.vstack([
        np.hstack(tiles[i:i + columns])
        for i in range(0, len(tiles), columns)
    ])
