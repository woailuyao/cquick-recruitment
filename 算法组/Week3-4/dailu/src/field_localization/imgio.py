#图像读写
"""带 Unicode 路径支持的图片读写。

Windows 上 cv2.imread / cv2.imwrite 遇到中文路径会静默失败（返回 None / False）
必须走 np.fromfile + cv2.imdecode 这条路。
"""
from pathlib import Path

import cv2
import numpy as np


def imread(path, flags=cv2.IMREAD_COLOR):
    p = str(path)
    try:
        data = np.fromfile(p, dtype=np.uint8)
    except OSError:
        return None
    if data.size == 0:
        return None
    return cv2.imdecode(data, flags)


def imwrite(path, img):
    p = Path(str(path))
    ext = p.suffix or ".png"
    ok, buf = cv2.imencode(ext, img)
    if not ok:
        return False
    p.parent.mkdir(parents=True, exist_ok=True)
    buf.tofile(str(p))
    return True


def project_root(start=None):
    """从脚本位置向上找，返回包含 config/camera.yaml 的项目根目录。

    这样无论脚本放在 src/ 还是 src/field_localization/，默认路径都不会错。
    """
    p = Path(str(start or __file__)).resolve()
    for up in [p.parent, *p.parents]:
        if (up / "config" / "camera.yaml").exists():
            return up
    for up in [p.parent, *p.parents]:
        if (up / "config").is_dir():
            return up
    return p.parents[1]