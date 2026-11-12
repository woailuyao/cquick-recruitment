import cv2
import numpy as np

def detect_img(img_path):
    img = cv2.imread(img_path)
    if img is None:
        print("找不到图片：", img_path)
        return

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 120, 70])
    upper_red = np.array([10, 255, 255])
    mask = cv2.inRange(hsv, lower_red, upper_red)

    kernel = np.ones((3,3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) == 0:
        print(img_path, "没有检测到目标")
        return

    max_con = max(contours, key=cv2.contourArea)
    M = cv2.moments(max_con)
    if M["m00"] < 1e-6:
        print(img_path, "异常，m00=0")
        return

    cx = M["m10"] / M["m00"]
    cy = M["m01"] / M["m00"]
    print(f"{img_path} 中心点：u={cx:.2f}, v={cy:.2f}")

    cv2.drawContours(img, [max_con], -1, (0,255,0), 2)
    cv2.circle(img, (int(cx), int(cy)), 4, (0,0,255), -1)

    out_name = "out_" + img_path
    cv2.imwrite(out_name, img)
    print("结果图已保存：", out_name)

if __name__ == "__main__":
    img_list = ["1.png", "2.png", "3.png"]
    for name in img_list:
        detect_img(name)
