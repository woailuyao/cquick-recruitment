import cv2
import numpy as np
import os


def task3_extraction(image_path, nu,output_dir: str = "../outputs/task3"):
    # 1. 创建输出文件夹
    os.makedirs(output_dir, exist_ok=True)

    # 2. 读取图片
    img = cv2.imread(image_path)
    img = cv2.resize(img, (0, 0), fx=0.3, fy=0.3)
    if img is None:
        print(f"图片读取失败了，检查一下路径：{image_path}")
        return

    # 3. 转 HSV 并设定颜色范围
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 蓝色的 HSV 范围
    lower_blue = np.array([90, 50, 50])
    upper_blue = np.array([130, 255, 255])

    # 4. 生成掩膜 (Mask)
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # 5. 形态学处理（改善掩膜）
    # 开运算：先腐蚀后膨胀，去噪点
    kernel = np.ones((5, 5), np.uint8)
    mask_opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    # 闭运算：先膨胀后腐蚀，填补小孔
    mask_closed = cv2.morphologyEx(mask_opened, cv2.MORPH_CLOSE, kernel)

    # 6. 用处理后的掩膜去原图里抠图（提取目标）
    result = cv2.bitwise_and(img, img, mask=mask_closed)

    # 7. 寻找轮廓
    #返回两个值：contours, hierarchy
    contours, _ = cv2.findContours(mask_closed, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    # 复制一份原图用来画框，避免污染原图
    draw_img = img.copy()

    for cnt in contours:
        # 计算轮廓面积
        area = cv2.contourArea(cnt)

        # 按面积过滤噪点（比如面积小于 500 的不要）
        if area > 500:
            # 获取外接矩形的坐标和宽高
            x, y, w, h = cv2.boundingRect(cnt)

            # 画外接矩形（绿色框，线宽 2）
            cv2.rectangle(draw_img, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # 计算质心（中心点）
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                # 画中心点（红色实心圆）
                cv2.circle(draw_img, (cX, cY), 5, (0, 0, 255), -1)
                # 在中心点旁边写上面积
                cv2.putText(draw_img, f"Area: {int(area)}", (cX - 20, cY - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # 8. 保存结果
    cv2.imwrite(os.path.join(output_dir, f"1_mask_original_{nu}.jpg"), mask)
    cv2.imwrite(os.path.join(output_dir, f"2_mask_closed_{nu}.jpg"), mask_closed)
    cv2.imwrite(os.path.join(output_dir, f"3_extracted_object_{nu}.jpg"), result)
    cv2.imwrite(os.path.join(output_dir, f"4_detected_bounding_box_{nu}.jpg"), draw_img)

    print(f"结果已保存到 {output_dir} 里了。")

    # 9. 显示结果
    cv2.imshow("Original", img)
    cv2.imshow("Mask (After Morph)", mask_closed)
    cv2.imshow("Result (Extracted)", result)
    cv2.imshow("Bounding Box & Center", draw_img)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    task3_extraction('../data/images/03.png', 3)
    task3_extraction('../data/images/04.png',4)
    task3_extraction('../data/images/05.png',5)