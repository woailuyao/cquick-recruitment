import cv2
import os
os.chdir(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

#读取图片
img=cv2.imread("data/images/ball.jpg")
#检查是否读取成功
if img is None:
    raise ValueError("Cannot read the image")
h,w,c= img.shape
print(f"h={h}, w={w}, c={c}")#打印图像宽、高和通道数。

#缩放、
result = cv2.resize(img,(50,50))
cv2.imshow("result",result)

#指定区域裁剪和
crop=img[0:100,0:100]
cv2.imshow("crop",crop)

#灰度转换
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv2.imshow("gray", gray)

cv2.waitKey(0)#显示或保存处理结果。
cv2.destroyAllWindows()


#将处理后的图片写入文件，检查保存文件是否能够重新读取。
cv2.imwrite("outputs/task1/result.jpg", result)
cv2.imwrite("outputs/task1/crop.jpg", crop)
cv2.imwrite("outputs/task1/gray.jpg", gray)

check =cv2.imread("outputs/task1/result.jpg")
if check is None:
    raise ValueError("Cannot read the image")
