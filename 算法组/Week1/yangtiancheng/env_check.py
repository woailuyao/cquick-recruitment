import cv2
import numpy as np

image = cv2.imread("data/images/sample01.jpg")
assert image is not None, "原图读取失败"
print("shape:", image.shape, "dtype:", image.dtype)

test = np.zeros((100, 200, 3), dtype=np.uint8)
cv2.circle(test, (100, 50), 20, (0, 255, 0), -1)
ok = cv2.imwrite("outputs/task1/env_check.png", test)
print("write ok:", ok)

read_back = cv2.imread("outputs/task1/env_check.png")
assert read_back is not None
print("read back shape:", read_back.shape)
