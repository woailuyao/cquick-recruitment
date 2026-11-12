import cv2 as cv
import sys
image1 = cv.imread("data/images/color1.jpg")
if image1 is None:
    sys.exit("color1.jpg读取失败，请检查路径")
image2 = cv.imread("data/images/color2.jpg")
if image2 is None:
    sys.exit("color2.jpg读取失败，请检查路径")
image3 = cv.imread("data/images/color3.jpg")
if image3 is None:
    sys.exit("color3.jpg读取失败，请检查路径")

print("三张彩色图片读取成功")

hsv1 = cv.cvtColor(image1, cv.COLOR_BGR2HSV)
hsv2 = cv.cvtColor(image2, cv.COLOR_BGR2HSV)
hsv3 = cv.cvtColor(image3, cv.COLOR_BGR2HSV)

print("盒子位置1：", hsv1[700, 1100])
print("盒子位置2：", hsv1[900, 900])
print("盒子位置3：", hsv1[1000, 1100])
print("背景位置：", hsv1[200, 200])

lower_color = (105, 20, 50)
upper_color = (145, 255, 255)

mask1 = cv.inRange(hsv1, lower_color, upper_color)
mask2 = cv.inRange(hsv2, lower_color, upper_color)
mask3 = cv.inRange(hsv3, lower_color, upper_color)

print("掩膜1保存成功：", cv.imwrite("outputs/task3/color1/mask.png", mask1))
print("掩膜2保存成功：", cv.imwrite("outputs/task3/color2/mask.png", mask2))
print("掩膜3保存成功：", cv.imwrite("outputs/task3/color3/mask.png", mask3))

opened1 = cv.morphologyEx(mask1, cv.MORPH_OPEN, None)
cleaned1 = cv.morphologyEx(opened1, cv.MORPH_CLOSE, None)
opened2 = cv.morphologyEx(mask2, cv.MORPH_OPEN, None)
cleaned2 = cv.morphologyEx(opened2, cv.MORPH_CLOSE, None)
opened3 = cv.morphologyEx(mask3, cv.MORPH_OPEN, None)
cleaned3 = cv.morphologyEx(opened3, cv.MORPH_CLOSE, None)

print("处理图1保存成功：", cv.imwrite("outputs/task3/color1/cleaned.png", cleaned1))
print("处理图2保存成功：", cv.imwrite("outputs/task3/color2/cleaned.png", cleaned2))
print("处理图3保存成功：", cv.imwrite("outputs/task3/color3/cleaned.png", cleaned3))

contours1, hierarchy1 = cv.findContours( cleaned1, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
contours2, hierarchy2 = cv.findContours(cleaned2, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
contours3, hierarchy3 = cv.findContours(cleaned3, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

min_area = 5000
target_contours1 = []
for contour in contours1:
    area = cv.contourArea(contour)

    if area >= min_area:
        target_contours1.append(contour)
        print("保留的轮廓面积：", area)

print("筛选前的轮廓数量：", len(contours1))
print("筛选后的轮廓数量：", len(target_contours1))

target_contours2 = []
for contour in contours2:
    area = cv.contourArea(contour)
    if area >= min_area:
        target_contours2.append(contour)
        print("图片2保留的轮廓面积：", area)

print("图片2筛选前的轮廓数量：", len(contours2))
print("图片2筛选后的轮廓数量：", len(target_contours2))

target_contours3 = []
for contour in contours3:
    area = cv.contourArea(contour)
    if area >= min_area:
        target_contours3.append(contour)
        print("图片3保留的轮廓面积：", area)

print("图片3筛选前的轮廓数量：", len(contours3))
print("图片3筛选后的轮廓数量：", len(target_contours3))

result1 = image1.copy()
for contour in target_contours1:
    area = cv.contourArea(contour)
    x, y, w, h = cv.boundingRect(contour)
    cv.rectangle(  result1, (x, y), (x + w, y + h),(0, 255, 0), 3)
    M = cv.moments(contour)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        cv.circle(result1, (cx, cy), 7, (0, 0, 255), -1)
        print("图片1目标框：", x, y, w, h)
        print("图片1质心坐标：", cx, cy)
        print("图片1轮廓面积：", area)
print("标注图1保存成功：", cv.imwrite("outputs/task3/color1/result.jpg", result1,
    [cv.IMWRITE_JPEG_QUALITY, 90]))

result2 = image2.copy()
for contour in target_contours2:
    area = cv.contourArea(contour)
    x, y, w, h = cv.boundingRect(contour)
    cv.rectangle(result2, (x, y), (x + w , y + h),(0, 255, 0), 3)
    M = cv.moments(contour)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        cv.circle(result2, (cx, cy), 7, (0, 0, 255), -1)
        print("图片2目标框：", x, y, w, h)
        print("图片2质心坐标：", cx, cy)
        print("图片2轮廓面积：", area)
print("标注图2保存成功：", cv.imwrite(
    "outputs/task3/color2/result.jpg", result2, [cv.IMWRITE_JPEG_QUALITY, 90]))

result3 = image3.copy()
for contour in target_contours3:
    area = cv.contourArea(contour)
    x, y, w, h = cv.boundingRect(contour)
    cv.rectangle(result3, (x, y), (x + w , y + h),(0, 255, 0), 3)
    M = cv.moments(contour)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        cv.circle(result3, (cx, cy), 7, (0, 0, 255), -1)
        print("图片3目标框：", x, y, w, h)
        print("图片3质心坐标：", cx, cy)
        print("图片3轮廓面积：", area)
print("标注图3保存成功：", cv.imwrite(
    "outputs/task3/color3/result.jpg", result3, [cv.IMWRITE_JPEG_QUALITY, 90]))
