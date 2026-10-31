from datetime import datetime
import os
import cv2
import numpy as np

print(datetime.now())
print(datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
print(f"{datetime.now():%H:%M:%S}")
print("Pi Dongyan")

# 原图路径（BASE = 个人目录根；原图放在 data/images/，来源 Unsplash，作者 allec gomes）
BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
src = os.path.join(BASE, "data", "images", "allec-gomes-xnRg3xDcNnE-unsplash.jpg")

# 输出目录：outputs/task2/
out_dir = os.path.join(BASE, "outputs", "task2")
os.makedirs(out_dir, exist_ok=True)

MAX_SIDE = 1600          # 输出统一缩放的长边像素

# 读图
img = cv2.imread(src)
if img is None:
    print("cv2.imread 读取失败，尝试用 numpy 读取字节流")
    try:
        img = cv2.imdecode(np.fromfile(src, dtype=np.uint8), cv2.IMREAD_COLOR)
    except Exception as e:
        print(f"备用方案失败：{e}")
    if img is None:
        raise SystemExit(f"图片读取失败，请检查路径是否正确、文件是否存在：{src}")

print(f"原图尺寸 = {img.shape[1]} x {img.shape[0]}（宽 x 高）")

# 统一缩放，长边 1600
h, w = img.shape[:2]
scale = MAX_SIDE / max(h, w)
if scale < 1:
    img = cv2.resize(img, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
print(f"处理尺寸 = {img.shape[1]} x {img.shape[0]}（宽 x 高，长边缩到 {MAX_SIDE}）")


def save(name, image):
    """保存并检查结果不是空文件
    注意：提交目录里含中文，"Pi Dongyan" 上一级是"视觉组"，cv2.imwrite 写中文路径会失败，
    所以先用 imencode 编码成字节流，再用 numpy 的 tofile 写盘（与读图时用 fromfile 是同一个道理）
    """
    p = os.path.join(out_dir, name)
    ext = os.path.splitext(name)[1]
    ok, buf = cv2.imencode(ext, image)
    if not ok:
        print(f"保存失败（编码阶段）：{p}")
        return p
    buf.tofile(p)
    size = os.path.getsize(p) / 1024 if os.path.exists(p) else 0
    if size == 0:
        print(f"保存失败（文件为空）：{p}")
    else:
        print(f"已保存 {name:26} {size:8.1f} KB")
    return p


# 原图
save("01_original.png", img)

# 灰度化
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
print(f"灰度图形状 = {gray.shape} ")
save("02_gray.png", gray)

# BGR - HSV，
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
H, S, V = cv2.split(hsv)

H_vis = (H.astype(np.float32) / 179).astype(np.uint8)

bar = 55                                        # 顶部标题栏高度
panels = []
for ch, label in [(H_vis, "Hue 0-179"), (S, "Saturation"), (V, "Value")]:
    p = cv2.cvtColor(ch, cv2.COLOR_GRAY2BGR)
    p = cv2.copyMakeBorder(p, bar, 0, 0, 10, cv2.BORDER_CONSTANT, value=(0, 0, 0))
    cv2.putText(p, label, (12, 38), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
    panels.append(p)
hsv_vis = np.hstack(panels)
save("03_hsv_channels.png", hsv_vis)

# 阈值分割
thr_a = 120
_, bin_a = cv2.threshold(gray, thr_a, 255, cv2.THRESH_BINARY_INV)

thr_b = 195
_, bin_b = cv2.threshold(gray, thr_b, 255, cv2.THRESH_BINARY_INV)

ratio_a = 100 * (bin_a == 255).mean()
ratio_b = 100 * (bin_b == 255).mean()
print(f"阈值 A = {thr_a}：白色（判为前景）占 {ratio_a:.2f}%")
print(f"阈值 B = {thr_b}：白色（判为前景）占 {ratio_b:.2f}%   （阈值调高 → 变白的像素变多）")

save("04_threshold_a.png", bin_a)
save("05_threshold_b.png", bin_b)
