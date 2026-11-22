import cv2
import numpy as np
import os
image_list = ['color1.jpg', 'color2.jpg', 'color3.jpg']
input_folder = '../images'
output_folder = '../outputs/task3'
os.makedirs(output_folder, exist_ok=True)
for img_name in image_list:
    print(f"\n================ 正在处理: {img_name} ================")
    img_path = os.path.join(input_folder, img_name)
    img = cv2.imread(img_path)

    if img is None:
        print(f"读取失败，请检查路径: {img_path}")
        continue
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, 100, 100])
    #np.array([H,S,V])分别为:色相 即颜色种类 范围为0~179 饱和度(鲜艳程度)0~255 明度0~255(0为纯黑)
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    lower_orange = np.array([11, 100, 100])
    upper_orange = np.array([25, 255, 255])
    #到这里 上面定义了颜色的界限 下面就是用界限划定颜色的范围然后再合并
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask3 = cv2.inRange(hsv, lower_orange, upper_orange)
    mask = mask1 + mask2 + mask3
    kernel = np.ones((5, 5), np.uint8)
    mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  #去除背景上的小白点 一般只输入黑白图(二值图)进行处理
    mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel)
    #open是先腐蚀后膨胀 即先用5x5的矩阵去掉范围内有一个黑点的中心点(换成黑点) 再用5x5去掉范围内有一个白点的中心点 接着向右移动一个像素再进行检测 close相反 且检测永远只检测原图再决定改不改
    mask_save_path = os.path.join(output_folder, f'{os.path.splitext(img_name)[0]}_mask.png')
    cv2.imwrite(mask_save_path, mask_clean)
    print(f"已保存掩膜图: {mask_save_path}")
    contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #这个函数只认黑白图！后两个参数意思分别是 检索模式(只找最外面轮廓);轮廓近似法(只存拐点 或者说顶点) 接着函数返回两个值 轮廓坐标和层级关系(即谁套在谁里面)
    min_area = 500
    valid_contours = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)#算出当前轮廓的面积
        if area > min_area:
            valid_contours += 1
            # 获取外接矩形 (x, y 是左上角坐标，w 是宽，h 是高)
            x, y, w, h = cv2.boundingRect(cnt)#用一个最小正矩形把轮廓cnt框起来
            center_x = x + w // 2
            center_y = y + h // 2#这两行找到了中心点坐标 xy为左上角坐标
            print(f"   -> 发现目标! 中心坐标: ({center_x}, {center_y}), 面积: {area:.1f} 像素")
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)#左上点坐标 右下坐标 颜色 方框粗细
            cv2.circle(img, (center_x, center_y), 5, (255, 0, 0), -1)#中心坐标 半径 颜色 -1是指实心圆 2指空心圆
            #这两行本质给人看的 机器人只要知道中心坐标即在哪向哪即可

    if valid_contours == 0:
        print("未发现面积大于设定阈值的目标。")

    result_save_path = os.path.join(output_folder, f'{os.path.splitext(img_name)[0]}_result.jpg')
    cv2.imwrite(result_save_path, img)
    print(f" 已保存结果标注图: {result_save_path}")

print("\n任务三全部执行完毕！请去 outputs/task3 文件夹查看结果。")