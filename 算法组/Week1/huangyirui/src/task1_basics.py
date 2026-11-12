# ======== 这里写你的代码（任务一：图像基础操作）========
#
# 留 言：
#   1. 本文件一行代码都没有，从当前这行之后开始写就行。
#   2. 运行：选中这个文件按 F5（配置已备好），或终端 python src/task1_basics.py
#   3. 读图写 "data/images/xxx.png"，存图写 "outputs/xxx.png"——工作目录已设为项目根。
#   4. data/images/ 里有一张 sample_scene.png（本项目本地生成），可以先拿它练手。
#
# 建议覆盖：读图与显示、保存、尺寸与通道数、BGR 与灰度转换、ROI 裁剪、缩放。
#
# —— 卡住了把报错原样贴给我。(๑•̀ㅂ•́)و✧
# ======================================================
import cv2

path = "data/images/sample_scene.png"#设置路径
img = cv2.imread(path)#根据path读取进img

print(img is None)#如果img可以被读取（非None），则返回False
if img is None:
    raise SystemExit("Fail to read the picture,please check the path:"+path)#如果img返回为None,则退出并返回路径提示

print(img.shape)#形状（高、宽、通道数）
print(img.dtype)#比特类型（uint8）


#打开和关闭窗口
cv2.imshow("original",img)
cv2.waitKey(0)
cv2.destroyAllWindows()


h,w = img.shape[:2]#读取img前两个数据（不读取通道数）

half = cv2.resize(img,(w//2,h//2))#把尺寸减半，存进half
print(half.shape)
cv2.imwrite("outputs/task1/sample1_half.jpg",half)#保存half


#打开和关闭窗口
cv2.imshow("original",half)
cv2.waitKey(0)
cv2.destroyAllWindows()


x,y = w//5,h//5
crop = img[y:y+h*3//5,x:x+w*3//5]#裁剪一块区域
print(crop.shape)
cv2.imwrite("outputs/task1/sample1_crop.jpg",crop)#保存crop


#打开和关闭窗口
cv2.imshow("original",crop)
cv2.waitKey(0)
cv2.destroyAllWindows()

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)#转灰度
cv2.imwrite("outputs/task1/sample1_gray.jpg",gray)#保存gray


#打开和关闭窗口
cv2.imshow("original",gray)
cv2.waitKey(0)
cv2.destroyAllWindows()

