from pathlib import Path
import cv2
import numpy as np

project = Path(__file__).resolve().parents[1]
image_path = project / "data" / "images" / "sample01.jpg"
save_dir = project / "outputs" / "task2"
save_dir.mkdir(parents=True, exist_ok=True)
img = cv2.imread(str(image_path))
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv2.imwrite(str(save_dir / "gray.png"), gray)
blur_5 = cv2.GaussianBlur(gray, (5, 5), 0)
blur_9 = cv2.GaussianBlur(gray, (9, 9), 0)
cv2.imwrite(str(save_dir / "blur_5x5.png"), blur_5)
cv2.imwrite(str(save_dir / "blur_9x9.png"), blur_9)
ret1, thresh_low = cv2.threshold(
    blur_5, 90, 255, cv2.THRESH_BINARY
)

ret2, thresh_high = cv2.threshold(
    blur_5, 160, 255, cv2.THRESH_BINARY
)
cv2.imwrite(str(save_dir / "thresh_90.png"), thresh_low)
cv2.imwrite(str(save_dir / "thresh_160.png"), thresh_high)
edges_1 = cv2.Canny(blur_5, 50, 150)
edges_2 = cv2.Canny(blur_5, 80, 200)
cv2.imwrite(str(save_dir / "edges_50_150.png"), edges_1)
cv2.imwrite(str(save_dir / "edges_80_200.png"), edges_2)
kernel = np.ones((3, 3), dtype=np.uint8)
opened = cv2.morphologyEx(
    thresh_low,
    cv2.MORPH_OPEN,
    kernel
)

closed = cv2.morphologyEx(
    thresh_low,
    cv2.MORPH_CLOSE,
    kernel
)
cv2.imwrite(str(save_dir / "morph_open.png"), opened)
cv2.imwrite(str(save_dir / "morph_close.png"), closed)
