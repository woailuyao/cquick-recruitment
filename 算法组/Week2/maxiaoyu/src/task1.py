import cv2
import numpy as np
import os

# 图片列表
img_names = ["apple.jpg", "basketball.jpg", "football.jpg"]

# HSV参数，可以微调
color_params = {
    "apple": {"low": np.array([0, 120, 80]), "high": np.array([10, 255, 255])},
    "basketball": {"low": np.array([10, 80, 80]), "high": np.array([25, 255, 255])},
    "football": {"low": np.array([0,0,0]), "high": np.array([180, 80, 80])}
}

# 建立output文件夹保存结果
out_dir = "output_task1"
if not os.path.exists(out_dir):
    os.mkdir(out_dir)

for name in img_names:
    img = cv2.imread(name)
    if img is None:
        print(f"图片 {name} 读取失败！")
        continue

    h, w = img.shape[:2]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 根据文件名选择HSV阈值
    if "apple" in name:
        mask = cv2.inRange(hsv, color_params["apple"]["low"], color_params["apple"]["high"])
    elif "basketball" in name:
        mask = cv2.inRange(hsv, color_params["basketball"]["low"], color_params["basketball"]["high"])
    elif "football" in name:
        mask = cv2.inRange(hsv, color_params["football"]["low"], color_params["football"]["high"])
    else:
        mask = np.zeros_like(hsv[:,:,0])

    # 形态学操作去噪
    kernel = np.ones((5,5), np.uint8)
    mask_clean = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_OPEN, kernel)

    # 找轮廓
    contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    res_img = img.copy()

    # 筛选面积，过滤小噪点
    valid_contours = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 5000:
            valid_contours.append(cnt)

    if len(valid_contours) == 0:
        print(f"{name}: 未检测到目标")
    else:
        for cnt in valid_contours:
            M = cv2.moments(cnt)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            print(f"{name} 目标中心坐标 u={cx}, v={cy}")
            cv2.drawContours(res_img, [cnt], -1, (0,255,0), 2)
            cv2.circle(res_img, (cx, cy), 5, (0,0,255), -1)

    # 保存图片
    cv2.imwrite(os.path.join(out_dir, f"mask_{name}"), mask)
    cv2.imwrite(os.path.join(out_dir, f"clean_mask_{name}"), mask_clean)
    cv2.imwrite(os.path.join(out_dir, f"result_{name}"), res_img)

print("task1全部处理完成！结果保存在 output_task1 文件夹")

