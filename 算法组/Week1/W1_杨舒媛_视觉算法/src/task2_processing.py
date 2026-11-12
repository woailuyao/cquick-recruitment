import cv2
import numpy as np
import os

os.makedirs('../outputs/task2', exist_ok=True)
img = cv2.imread('../data/image/2.jpg')

if img is None:
    print("❌ 找不到图片！")
else:
    print("✅ 开始处理任务二...")

    # 设定一个固定弹窗的宽高，防止太大超出屏幕
    w, h = 400, 533

    # 1. 灰度
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imwrite('../outputs/task2/1_gray.jpg', gray)

    # 2. 高斯滤波对比
    gaussian_5 = cv2.GaussianBlur(gray, (5, 5), 0)
    gaussian_21 = cv2.GaussianBlur(gray, (21, 21), 0)
    cv2.imwrite('../outputs/task2/2_gaussian_5.jpg', gaussian_5)
    cv2.imwrite('../outputs/task2/2_gaussian_21.jpg', gaussian_21)

    # 3. 阈值分割对比
    _, thresh_100 = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
    _, thresh_150 = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)
    cv2.imwrite('../outputs/task2/3_threshold_100.jpg', thresh_100)
    cv2.imwrite('../outputs/task2/3_threshold_150.jpg', thresh_150)

    # 4. Canny 边缘检测
    canny = cv2.Canny(gray, 50, 150)
    cv2.imwrite('../outputs/task2/4_canny.jpg', canny)

    # 5. 形态学处理
    kernel = np.ones((5, 5), np.uint8)
    morph = cv2.morphologyEx(thresh_100, cv2.MORPH_OPEN, kernel)
    cv2.imwrite('../outputs/task2/5_morphology.jpg', morph)

    print("✅ 任务二处理完毕！7张图已保存到 outputs/task2/。")
    print("⚠️ 请连续按键盘任意键，对比观察每一组效果...")

    # 弹窗对比显示（全部强制缩小到 w x h）
    cv2.imshow('1. Original', cv2.resize(img, (w, h)))
    cv2.imshow('2. Gray', cv2.resize(gray, (w, h)))
    cv2.waitKey(0);
    cv2.destroyAllWindows()

    cv2.imshow('3. Gaussian 5x5 (Clear)', cv2.resize(gaussian_5, (w, h)))
    cv2.imshow('4. Gauss ian 21x21 (Blurry)', cv2.resize(gaussian_21, (w, h)))
    cv2.waitKey(0);
    cv2.destroyAllWindows()

    cv2.imshow('5. Threshold 100 (More White)', cv2.resize(thresh_100, (w, h)))
    cv2.imshow('6. Threshold 150 (More Black)', cv2.resize(thresh_150, (w, h)))
    cv2.waitKey(0);
    cv2.destroyAllWindows()

    cv2.imshow('7. Canny Edge (Lines)', cv2.resize(canny, (w, h)))
    cv2.waitKey(0);
    cv2.destroyAllWindows()

    print("🚀 运行结束！")