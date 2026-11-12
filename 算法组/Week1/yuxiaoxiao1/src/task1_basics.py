import cv2
import os
import sys


out_dir = "outputs/task1"
os.makedirs(out_dir,exist_ok=True)

image_one = cv2.imread("data/images/picture__one.png")
image_two = cv2.imread("data/images/picture__two.png")


if image_one is None:
    print("picture__one读取失败")
    sys.exit()

if image_two is None:
    print("picture__two读取失败")
    sys.exit()

h1,w1,c1=image_one.shape
print(f"图1：宽{w1},高{h1},通道数{c1}")
h2,w2,c2=image_two.shape
print(f"图2：宽{w2},高{h2},通道数{c2}")

result_one = cv2.resize(image_one,(200,600))
result_two = cv2.resize(image_two,(200,600))

crop_one=image_one[150:300,200:400]
crop_two=image_two[150:500,200:400]

gray_one = cv2.cvtColor(image_one,cv2.COLOR_BGR2GRAY)
gray_two = cv2.cvtColor(image_two,cv2.COLOR_BGR2GRAY)


cv2.imwrite(os.path.join(out_dir, "result_one.png"), result_one)
cv2.imwrite(os.path.join(out_dir, "result_two.png"), result_two)
cv2.imwrite(os.path.join(out_dir, "crop_one.png"), crop_one)
cv2.imwrite(os.path.join(out_dir, "crop_two.png"), crop_two)
cv2.imwrite(os.path.join(out_dir, "gray_one.png"), gray_one)
cv2.imwrite(os.path.join(out_dir, "gray_two.png"), gray_two)

check_read = cv2.imread(os.path.join(out_dir, "result_one.png"))
if check_read is not None:
    print("重新读取成功")
else:
    print("重新读取失败")

cv2.imshow("image_one",image_one)
cv2.imshow("image_two",image_two)
cv2.imshow("result_one",result_one)
cv2.imshow("result_two",result_two)
cv2.imshow("crop_one",crop_one)
cv2.imshow("crop_two",crop_two)
cv2.imshow("gray_one",gray_one)
cv2.imshow("gray_two",gray_two,)
cv2.waitKey()
cv2.destroyAllWindows()