from pathlib import Path
import cv2

from io_utils import imread, imwrite

# 项目根目录：src 的上一级
BASE = Path(__file__).resolve().parents[1]
IMG_DIR = BASE / "data" / "images"
OUT_DIR = BASE / "outputs" / "task1"
OUT_DIR.mkdir(parents=True, exist_ok=True)

# 显式列出任务一使用的图片，避免 data/images 里新增任务三的图片后输出数量变化
IMAGE_NAMES = ["01.JPG", "02.JPG", "03.JPG"]

images = [IMG_DIR / n for n in IMAGE_NAMES]
missing = [x.name for x in images if not x.exists()]
if missing:
    raise SystemExit(f"缺少图片: {missing}  (目录: {IMG_DIR})")

for path in images:
    img = imread(str(path))          # OpenCV 用 BGR
    if img is None:                      # 路径错时只返回 None，不抛异常
        print(f"[跳过] 读取失败: {path.name}")
        continue

    h, w, c = img.shape                  # 注意顺序是 高、宽、通道
    print(f"\n{path.name}: 宽={w} 高={h} 通道={c}")
    print(f"  像素数={w * h}  像素值个数={w * h * c}  数据类型={img.dtype}")

    stem = path.stem

    # 1) 缩放：缩小一半，INTER_AREA 缩小时质量最好
    small = cv2.resize(img, None, fx=0.5, fy=0.5,
                     interpolation=cv2.INTER_AREA)

    # 2) 裁剪中心区域：先高后宽
    crop = img[h // 4:h * 3 // 4, w // 4:w * 3 // 4]

    # 3) 灰度
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 4) 写入并重新读取验证（原始图片不动）
    for tag, out_img in [("resized", small), ("crop", crop), ("gray", gray)]:
      out_path = OUT_DIR / f"{stem}_{tag}.png"
      ok = imwrite(str(out_path), out_img)
      back = imread(str(out_path))
      print(f"  {tag:8s} 保存={ok} 重新读取={back is not None} "
            f"形状={None if back is None else back.shape}")

