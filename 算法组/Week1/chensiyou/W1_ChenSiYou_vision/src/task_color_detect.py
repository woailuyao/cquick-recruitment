import cv2 as cv
import numpy as np
import os

def task3_color_detect():
    save_root = "../outputs/task3"
    if not os.path.exists(save_root):
        os.makedirs(save_root)

    # 3张测试图片，和README记录对应
    img_list = [
        "../data/images/img1.jpg", # 正常光照，背景简单
        "../data/images/img2.jpg", # 光线偏暗，背景杂物多
        "../data/images/img3.jpg"  # 强光，画面存在反光
    ]

    # ========= HSV蓝色范围设置（任务要求解释这个范围）=========
    # OpenCV的HSV：H:0~179，S:0~255，V:0~255
    lower_blue = np.array([100, 60, 60])
    upper_blue = np.array([130, 255, 255])
    # 轮廓最小面积阈值，小于这个面积直接当成噪点过滤
    min_area =600

    for idx, img_path in enumerate(img_list):
        img = cv.imread(img_path)
        if img is None:
            print(f"图片读取失败：{img_path}，检查路径")
            continue
        img_copy = img.copy() # 复制原图用来画框，不改变原始图片

        # 1. BGR转HSV，inRange生成二值掩膜mask
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, lower_blue, upper_blue)

        # 2. 形态学开运算，去掉小噪点，优化掩膜
        kernel = np.ones((3,3), np.uint8)
        mask_clean = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel, iterations=1)

        # 查找轮廓，只找最外层轮廓
        contours, hierarchy = cv.findContours(mask_clean, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        print(f"\n=====第{idx+1}张图片=====")
        # 遍历轮廓，按面积过滤噪声
        for cnt in contours:
            area = cv.contourArea(cnt)
            if area < min_area:
                continue # 面积太小，跳过，当成噪点

            # 获取外接矩形
            x, y, w, h = cv.boundingRect(cnt)
            # 计算轮廓中心坐标
            center_x = x + w//2
            center_y = y + h//2

            # 在原图上绘制矩形和中心点
            cv.rectangle(img_copy, (x,y), (x+w, y+h), (0,255,0), 2)
            cv.circle(img_copy, (center_x, center_y), 4, (0,0,255), -1)
            # 打印坐标和面积
            print(f"目标中心坐标 x={center_x}, y={center_y}  | 轮廓面积={area:.1f}")

        # 新建子文件夹保存这一组图片
        img_save_dir = os.path.join(save_root, f"test_img{idx+1}")
        if not os.path.exists(img_save_dir):
            os.makedirs(img_save_dir)
        # 保存掩膜图 和 画好框的结果图（满足任务要求保存mask.png result.png）
        cv.imwrite(os.path.join(img_save_dir, "mask.png"), mask_clean)
        cv.imwrite(os.path.join(img_save_dir, "result.png"), img_copy)

        # 弹窗预览
        cv.imshow(f"mask_{idx+1}", mask_clean)
        cv.imshow(f"result_{idx+1}", img_copy)
        print("按任意键，进入下一张图片")
        cv.waitKey(0)
        cv.destroyAllWindows()

if __name__ == "__main__":
    print("开始任务三：颜色目标提取与轮廓定位")
    task3_color_detect()