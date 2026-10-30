import cv2
import os


def task2_processing(image_path, output_dir: str = "../outputs/task2"):
    # 1. 确保输出目录存在
    os.makedirs(output_dir, exist_ok=True)

    # 2. 读取图片
    img = cv2.imread(image_path)
    img = cv2.resize(img, (0, 0), fx=0.3, fy=0.3)
    if img is None:
        print(f"图片读取失败了，检查一下路径：{image_path}")
        return

    # 3. 图像处理
    # (1) 灰度图
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # (2) 高斯滤波 (去噪)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # (3) 阈值分割 (二值化，大于127变白，小于变黑)
    # 注意：cv2.threshold 返回两个值，_ 是阈值，thresh 是处理后的图像
    _, thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)

    # (4) Canny 边缘检测
    edges = cv2.Canny(blurred, 50, 150)

    # (5) 形态学开运算 (先腐蚀后膨胀，去除小噪点)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    opened = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

    # 4. 保存这 5 张中间结果图
    cv2.imwrite(os.path.join(output_dir, "1_gray.jpg"), gray)
    cv2.imwrite(os.path.join(output_dir, "2_gaussian_blur.jpg"), blurred)
    cv2.imwrite(os.path.join(output_dir, "3_threshold.jpg"), thresh)
    cv2.imwrite(os.path.join(output_dir, "4_canny_edges.jpg"), edges)
    cv2.imwrite(os.path.join(output_dir, "5_morphology_open.jpg"), opened)

    print(f"5张对比图已存到 {output_dir} 里了。")

    # 5. 显示结果对比 (按任意键关闭)
    # 把原图也显示出来，方便你对比
    cv2.imshow("Original", img)
    cv2.imshow("1. Gray", gray)
    cv2.imshow("2. Gaussian Blur", blurred)
    cv2.imshow("3. Threshold", thresh)
    cv2.imshow("4. Canny Edges", edges)
    cv2.imshow("5. Morphology Open", opened)

    cv2.waitKey(0)
    cv2.destroyAllWindows()


# 调用
if __name__ == "__main__":
    # 这里换成你 data 文件夹里另一张图片的名字，比如 02.jpg
    task2_processing('../data/images/01.jpg')