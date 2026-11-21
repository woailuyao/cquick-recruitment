import cv2 as cv
import numpy as np
import os

# 第1组参数函数
def task2_group1():
    save_path = "../outputs/task2/group1"
    if not os.path.exists(save_path):
        os.makedirs(save_path)

    img = cv.imread("../data/images/img2.jpg")
    if img is None:
        print("图片读取失败！检查图片路径")
        return

    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    gauss = cv.GaussianBlur(gray, (7,7), sigmaX=1.2)
    ret, thresh = cv.threshold(gauss, 90, 255, cv.THRESH_BINARY)
    canny_edge = cv.Canny(gauss, threshold1=30, threshold2=120)
    kernel = np.ones((3,3), np.uint8)
    morph_open = cv.morphologyEx(thresh, cv.MORPH_OPEN, kernel)

    # 保存图片
    cv.imwrite(os.path.join(save_path, "gray.jpg"), gray)
    cv.imwrite(os.path.join(save_path, "gauss.jpg"), gauss)
    cv.imwrite(os.path.join(save_path, "thresh.jpg"), thresh)
    cv.imwrite(os.path.join(save_path, "canny.jpg"), canny_edge)
    cv.imwrite(os.path.join(save_path, "morph_open.jpg"), morph_open)

    # 显示窗口
    cv.imshow("src", img)
    cv.imshow("gray", gray)
    cv.imshow("gauss blur", gauss)
    cv.imshow("threshold", thresh)
    cv.imshow("canny edge", canny_edge)
    cv.imshow("morph open", morph_open)

    cv.waitKey(0)
    cv.destroyAllWindows()

# 第2组参数函数
def task2_group2():
    save_path = "../outputs/task2/group2"
    if not os.path.exists(save_path):
        os.makedirs(save_path)

    img = cv.imread("../data/images/img2.jpg")
    if img is None:
        print("图片读取失败！检查图片路径")
        return

    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # 第二组修改后的参数
    gauss = cv.GaussianBlur(gray, (7,7), sigmaX=1.2)
    ret, thresh = cv.threshold(gauss, 90, 255, cv.THRESH_BINARY)
    canny_edge = cv.Canny(gauss, threshold1=80, threshold2=200)
    kernel = np.ones((3,3), np.uint8)
    morph_open = cv.morphologyEx(thresh, cv.MORPH_OPEN, kernel)

    # 保存图片
    cv.imwrite(os.path.join(save_path, "gray.jpg"), gray)
    cv.imwrite(os.path.join(save_path, "gauss.jpg"), gauss)
    cv.imwrite(os.path.join(save_path, "thresh.jpg"), thresh)
    cv.imwrite(os.path.join(save_path, "canny.jpg"), canny_edge)
    cv.imwrite(os.path.join(save_path, "morph_open.jpg"), morph_open)

    # 显示窗口
    cv.imshow("src", img)
    cv.imshow("gray", gray)
    cv.imshow("gauss blur", gauss)
    cv.imshow("threshold", thresh)
    cv.imshow("canny edge", canny_edge)
    cv.imshow("morph open", morph_open)

    cv.waitKey(0)
    cv.destroyAllWindows()


# 【运行控制，二选一，注释掉另一个】
# task2_group1()   # 运行第一组，取消注释
task2_group2()   # 运行第二组，取消注释

