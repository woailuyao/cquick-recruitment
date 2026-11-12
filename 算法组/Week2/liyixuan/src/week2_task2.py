import cv2
import numpy as np

# 读取图片
img = cv2.imread(r'D:\MyCode\data\images\1.jpeg')

# 定义一个画图的函数
def detect_and_draw(image, lower, upper, min_area):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    result_img = image.copy()
    
    if len(contours) > 0:
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        if area > min_area:
            x, y, w, h = cv2.boundingRect(largest_contour)
            u = x + w // 2
            v = y + h // 2
            # 画绿框
            cv2.rectangle(result_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # 画红点
            cv2.circle(result_img, (u, v), 5, (0, 0, 255), -1)
            # 写坐标文字
            text = f"u={u}, v={v}"
            cv2.putText(result_img, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            return result_img, f"找到了目标，坐标({u},{v})，面积{area}"
    return result_img, "未检测到有效目标"

# 实验 A：标准阈值（能画出框）
res_a, msg_a = detect_and_draw(img, [35, 43, 46], [90, 255, 255], 500)
cv2.imwrite(r'D:\MyCode\outputs\week2_param_A.jpg', res_a)
print(f"参数A结果: {msg_a}")

# 实验 B：严格阈值（可能画不出框或画得不一样）
res_b, msg_b = detect_and_draw(img, [40, 150, 100], [80, 255, 255], 500)
cv2.imwrite(r'D:\MyCode\outputs\week2_param_B.jpg', res_b)
print(f"参数B结果: {msg_b}")