

import cv2
import sys
import os

img = cv2.imread('data/images/test_blue_circle.jpg')
assert img is not None, "图片读取失败，检查路径"

print(img.shape)    # (高, 宽, 通道)  

h, w = img.shape[:2]                 # 取出高、宽（shape 是 (高,宽,通道)）
target_w = 300                       # 固定的宽度为300
target_h = int(h * target_w / w)     # 高度按原比例换算
resized = cv2.resize(img, (target_w, target_h))   # rsize 的参数是 (宽, 高)
print("缩放后", resized.shape)
ok1 = cv2.imwrite('outputs/task1/blue_resize.jpg', resized)

ch, cw = 200, 200                    # 想裁出的 高、宽
y1 = h // 2 - ch // 2                # 中心裁剪：从正中心上下各取一半
y2 = y1 + ch
x1 = w // 2 - cw // 2                # 左右各取一半
x2 = x1 + cw
cropped = img[y1:y2, x1:x2]          # 切片顺序：高在前，宽在后
print("裁剪后", cropped.shape)
ok2 = cv2.imwrite('outputs/task1/blue_crop.jpg', cropped)

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)    #BGR转为GRAY
print("灰度后", gray.shape)          # 只有 (高, 宽)，没有第 3 维
ok3 = cv2.imwrite('outputs/task1/blue_gray.jpg', gray)

print(ok1, ok2, ok3)                 # 三个 True 表示都存成功

# 把刚保存的图重新读回来，确认能正常读取
reread1 = cv2.imread('outputs/task1/blue_resize.jpg')
assert reread1 is not None, "重新读取失败，文件可能没存上"
reread2 = cv2.imread('outputs/task1/blue_crop.jpg')
assert reread2 is not None, "重新读取失败，文件可能没存上"
reread3 = cv2.imread('outputs/task1/blue_gray.jpg', cv2.IMREAD_GRAYSCALE) #cv2.imread 默认按彩色(3通道)​加载
assert reread3 is not None, "重新读取失败，文件可能没存上"
print(reread3.shape) 




#在第二张图片上重复操作
img2 = cv2.imread('data/images/test_green_rect.jpg')
assert img2 is not None, "图片读取失败，检查路径"

print(img2.shape)    # (高, 宽, 通道)  

h2, w2 = img2.shape[:2]             
target_w2 = 200                     
target_h2 = int(h2 * target_w2 / w2)     
resized2 = cv2.resize(img2, (target_w2, target_h2))   
print("缩放后", resized2.shape)
ok4 = cv2.imwrite('outputs/task1/green_resize.jpg', resized2)
          
y1 = h2 // 2 - ch // 2                
y2 = y1 + ch
x1 = w2 // 2 - cw // 2                
x2 = x1 + cw
cropped2 = img2[y1:y2, x1:x2]          
print("裁剪后", cropped2.shape)
ok5 = cv2.imwrite('outputs/task1/green_crop.jpg', cropped2)

gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)    
print("灰度后", gray2.shape)        
ok6 = cv2.imwrite('outputs/task1/green_gray.jpg', gray2)

print(ok4, ok5, ok6)                 