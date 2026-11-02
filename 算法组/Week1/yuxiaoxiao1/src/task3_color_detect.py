import cv2
import numpy as np
import os

out_dir = "outputs/task3"
os.makedirs(out_dir, exist_ok=True)

lower_blue = np.array([100, 100, 50])
upper_blue = np.array([130, 255, 255])

image_names = ["picture__one.png", "picture__two.png", "picture_three.jpg"]

for name in image_names:
    img_path = os.path.join("data", "images", name)
    img = cv2.imread(img_path)


    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    #开运算
    kernel = np.ones((5, 5), np.uint8)
    mask_opening= cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask_opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    result_img = img.copy()


    min_area = 500

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > min_area:

            x, y, w, h = cv2.boundingRect(cnt)

            cv2.rectangle(result_img, (x, y), (x + w, y + h), (0, 255, 0), 2)

            center_x = x + w // 2
            center_y = y + h // 2

            cv2.circle(result_img, (center_x, center_y), 5, (0, 0, 255), -1)

            text = f"({center_x},{center_y}) Area:{int(area)}"
            cv2.putText(result_img, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            print(f"中心坐标: ({center_x}, {center_y}), 轮廓面积: {int(area)}")


    mask_path = os.path.join(out_dir, f"{name.split('.')[0]}_mask.png")
    cv2.imwrite(mask_path, mask_opening)

    result_path = os.path.join(out_dir, f"{name.split('.')[0]}_result.png")
    cv2.imwrite(result_path, result_img)



print("\n✅ 任务三所有图片处理完毕，请去 outputs/task3/ 查看结果。")