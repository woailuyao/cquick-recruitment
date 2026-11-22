import cv2
import os

out_dir = "output_task2"
if not os.path.exists(out_dir):
    os.mkdir(out_dir)

img_list = [
    "apple.jpg",
    "basketball.jpg",
    "football.jpg"
]

for name in img_list:
    img = cv2.imread(name)
    h, w = img.shape[:2]
    base_name = name.split(".")[0]

    # 1 缩放
    img_resize = cv2.resize(img, (w//2, h//2))
    cv2.imwrite(os.path.join(out_dir, f"{base_name}_1_resize.jpg"), img_resize)

    # 2 旋转45°
    center = (w//2, h//2)
    rot_mat = cv2.getRotationMatrix2D(center, 45, 1.0)
    img_rot = cv2.warpAffine(img, rot_mat, (w, h))
    cv2.imwrite(os.path.join(out_dir, f"{base_name}_2_rotate.jpg"), img_rot)

    # 3 灰度
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imwrite(os.path.join(out_dir, f"{base_name}_3_gray.jpg"), img_gray)

    # 4 二值化
    ret, img_bin = cv2.threshold(img_gray, 127, 255, cv2.THRESH_BINARY)
    cv2.imwrite(os.path.join(out_dir, f"{base_name}_4_binary.jpg"), img_bin)

    # 5 腐蚀
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
    img_erode = cv2.erode(img_bin, kernel)
    cv2.imwrite(os.path.join(out_dir, f"{base_name}_5_erode.jpg"), img_erode)

    # 6 膨胀
    img_dilate = cv2.dilate(img_bin, kernel)
    cv2.imwrite(os.path.join(out_dir, f"{base_name}_6_dilate.jpg"), img_dilate)

print("task2全部处理完成！三张图结果保存在 output_task2")


