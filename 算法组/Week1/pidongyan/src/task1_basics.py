import os
import cv2
import numpy as np
from datetime import datetime


print(datetime.now())
print(datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
print(f"{datetime.now():%H:%M:%S}")
print("Pi Dongyan")

# 读图片（BASE = 个人目录根，脚本在任意机器、任意路径下 clone 后都能直接跑）
BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
path = os.path.join(BASE, "data", "images", "allec-gomes-xnRg3xDcNnE-unsplash.jpg")
USE_IMSHOW = True

img = cv2.imread(path)

# 是否读取成功
if img is None:
    print("cv2.imread 读取失败（Python 里中文字符串没问题，但 OpenCV 的 Windows 解码器不支持非 ASCII 路径）")
    print("改用 np.fromfile + cv2.imdecode 重试……")
    try:
        img = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
    except Exception as e:
        img = None
        print(f"备用方案也失败了：{e}")

    if img is None:
        # 明确提示，然后退出，不继续往下处理（否则后面报一个看不懂的错）
        raise SystemExit(f"图片读取失败，请检查路径是否正确、文件是否存在：{path}")

print("图片读取成功")
print(f"文件路径 = {path}")

# 输出宽、高
height, width = img.shape[:2]          # shape 的顺序是 (高, 宽, 通道)，不是 (宽, 高)
print(f"图像宽高 = 宽 {width} 像素, 高 {height} 像素")

# 数组形状
print(f"shape   = {img.shape}")

#  输出 dtype
print(f"dtype   = {img.dtype}")

# 打印一个像素的通道值
y, x = 0, 0
b, g, r = img[y, x]
print(f"像素 (x={x}, y={y}) 的通道值 = {img[y, x]}  B = {b}, G = {g}, R = {r}")

# 验证
pixel_bgr = np.uint8([[[b, g, r]]])              # 1x1 的 3 通道图，值就是上面这个像素
pixel_rgb = cv2.cvtColor(pixel_bgr, cv2.COLOR_BGR2RGB)
print(f"  同一个像素转成 RGB 后 = {pixel_rgb[0, 0]} ")


# 显示图片
out_dir = os.path.join(BASE, "outputs", "task1")
os.makedirs(out_dir, exist_ok=True)

if USE_IMSHOW:
    try:
        # 这张图有 7173x10041 像素，默认 WINDOW_AUTOSIZE 会开一个比屏幕还大的窗口，
        # 只能看到左上角一块淡黄背景，所以用 WINDOW_NORMAL + 指定初始尺寸
        cv2.namedWindow("strawberry", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("strawberry", 700, 980)
        cv2.imshow("strawberry", img)
        key = cv2.waitKey(0)             # 0 表示无限等待，直到有按键
        cv2.destroyAllWindows()
        print(f"按键退出，键值 = {key}")
    except cv2.error as e:
        # 无图形界面（headless 环境）时 imshow 会抛异常，改为保存文件
        out_path = os.path.join(out_dir, "read_image_display.png")
        cv2.imwrite(out_path, img)
        print(f"当前环境无法弹窗显示（{e}），已改为保存图片：{out_path}")
else:
    out_path = os.path.join(out_dir, "read_image_display.png")
    cv2.imwrite(out_path, img)
    print(f"USE_IMSHOW = False，已保存图片代替显示：{out_path}")
