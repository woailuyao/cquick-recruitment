from pathlib import Path

from _image_io import read_image, save_image

BASE = Path(__file__).resolve().parents[1]
image = read_image(BASE / "images" / "clear.jpg")

# 保留第 0～329 行、全部列，这个区域不包含网球
no_target = image[:330, :]
save_image(BASE / "images" / "no_target.jpg", no_target, jpeg_quality=95)
