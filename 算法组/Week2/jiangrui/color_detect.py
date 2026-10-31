import cv2
import numpy as np
import matplotlib.pyplot as plt
#--------------------------------第一张测试图---------------------------------
print("----------------------现在处理第一张测试图-------------------------------")
input_path = 'images/test1.jpg'
img =cv2.imread(input_path)
#转HSV和提取红色
hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
mask1 = cv2.inRange(hsv,(0,43,46),(10,255,255))
mask2 = cv2.inRange(hsv,(140,43,46),(179,255,255))
mask = cv2.bitwise_or(mask1,mask2)
result =cv2.bitwise_and(img,img,mask = mask)
#形态学去噪
kernel = np.ones((5,5),np.uint8)
mask = cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)
#找轮廓
contours,_ =cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
result_img = img.copy()
count = 0
#遍历所有轮廓，找出面积大于50的轮廓
for cnt in contours:
    area1 = cv2.contourArea(cnt)
    if area1 > 50:
        count += 1
       #遍历中心点并可视化
        M = cv2.moments(cnt)
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00']) 
            cy = int(M['m01']/M['m00'])
            print(f"第{count}个目标:面积{area1},中心= ({cx},{cy})")
            #画图
            cv2.circle(result_img,(cx,cy),5,(0,255,0),2)
            cv2.drawContours(result_img,[cnt],-1,(255,0,0),2)
    
            
print(f"图1检测成功:共找到{count}个红色目标")

#显示
plt.figure(1,figsize=(10, 10))
plt.subplot(221)
plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))   # 原图
plt.title('Original')
plt.axis('off')

plt.subplot(222)
plt.imshow(mask, cmap='gray')                      # 掩码
plt.title('Mask')
plt.axis('off')

plt.subplot(223)
plt.imshow(cv2.cvtColor(result_img, cv2.COLOR_BGR2RGB))  # ← 带轮廓的结果图
plt.title('Contours')
plt.axis('off') 
#--------------------------------第二张测试图---------------------------------
print("-----------------------现在处理第二张测试图-------------------------------")
input_path2 = 'images/test2 (2).jpg'
img2 = cv2.imread(input_path2)

# 1. 转 HSV 空间
hsv2 = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)

# 2. 提取低饱和度区域（不管是黑还是白，它的饱和度S都很低；而草地S很高）
mask_ball = cv2.inRange(hsv2, np.array([0, 0, 0]), np.array([180, 60, 255]))

# 3. 先做开运算（擦掉草地里遗漏的零星噪点）
kernel_open = np.ones((5, 5), np.uint8)
mask_ball = cv2.morphologyEx(mask_ball, cv2.MORPH_OPEN, kernel_open)

# 4. 再做闭运算（把球身上的黑白缝隙填满，让球变成一个实心大饼）
kernel_close = np.ones((25, 25), np.uint8) # 把球糊成一个整体
mask_ball = cv2.morphologyEx(mask_ball, cv2.MORPH_CLOSE, kernel_close)

# 5. 找轮廓
contours2, _ = cv2.findContours(mask_ball, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
result_img2 = img2.copy()

best_cnt2 = None
max_area2 = 0

# 6. 筛选面积最大，且长宽比像足球的轮廓
for cnt2 in contours2:
    area2 = cv2.contourArea(cnt2)
    if area2 < 1000: # 过滤小噪点
        continue
    
    x, y, w, h = cv2.boundingRect(cnt2)
    aspect_ratio = float(w) / h
    
    # 足球的长宽比接近1
    if 0.7 <= aspect_ratio <= 1.3:
        if area2 > max_area2:
            max_area2 = area2
            best_cnt2 = cnt2

# 7. 画最终结果
if best_cnt2 is not None:
    x, y, w, h = cv2.boundingRect(best_cnt2)
    # 画绿框
    cv2.rectangle(result_img2, (x, y), (x + w, y + h), (0, 255, 0), 3)
    
    # 算质心
    M = cv2.moments(best_cnt2)
    if M['m00'] != 0:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        cv2.circle(result_img2, (cx, cy), 5, (0, 255, 0), -1)
        cv2.putText(result_img2, f"Centroid: ({cx}, {cy})", (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)
        print(f"图2检测成功：外接矩形面积={w*h}, 质心=({cx},{cy})")
else:
    print("图2未找到符合条件的目标。")

# 8. 画图显示
images = [img2, mask_ball, result_img2]
titles = ['Original2', 'Low Saturation Mask', 'Result2']
plt.figure(2, figsize=(15, 5))
for i in range(3):
    plt.subplot(1, 3, i + 1)
    if i == 0 or i == 2:
        plt.imshow(cv2.cvtColor(images[i], cv2.COLOR_BGR2RGB))
    else:
        plt.imshow(images[i], cmap='gray')
    plt.title(titles[i])
    plt.axis('off')
#--------------------------------第三张测试图---------------------------------
print("-----------------------现在处理第三张测试图-----------------------------")
input_path3 = 'images/test3.jpg'
img3 = cv2.imread(input_path3)
#转HSV
hsv_img3 = cv2.cvtColor(img3,cv2.COLOR_BGR2HSV)
#mask,提取蓝色
mask_img3 = cv2.inRange(hsv_img3,np.array([100,80,80]),np.array([130,255,255]))
#去噪
kernel3 = np.ones((5,5),np.uint8)
mask_img3 = cv2.morphologyEx(mask_img3,cv2.MORPH_OPEN,kernel3)
#找轮廓
contours3, _ = cv2.findContours(mask_img3, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#画轮廓
result_img3 = img3.copy()
best_cnt3 = None
max_area3 = 0
#遍历所有轮廓，找出面积大于50的轮廓
for cnt3 in contours3:
    area3 = cv2.contourArea(cnt3)
    if area3 < 50:
        continue
    if area3 > max_area3:
        max_area3 = area3
        best_cnt3 = cnt3
#输出中心点
if best_cnt3 is not None:
    M = cv2.moments(best_cnt3) 
    if M['m00']!= 0:
            cx3 = int(M['m10']/M['m00'])
            cy3 = int(M['m01']/M['m00'])
            cv2.circle(result_img3,(cx3,cy3),8,(0,255,0),2)
    else:
        cx3, cy3 = None, None
    cv2.drawContours(result_img3,[best_cnt3],-1,(0,255,0),8)  
    cv2.putText(result_img3,f"Area: {max_area3:.0f}",(cx3-50,cy3-20),cv2.FONT_HERSHEY_SIMPLEX,1.5,(0,255,0),2)  
    print(f"图3检测成功:最佳目标：面积={max_area3:.0f}, 中心=({cx3},{cy3})")
else:
    print("图3未找到符合条件的目标。")
#显示
plt.figure(3,figsize=(10, 10))
plt.subplot(221)
plt.imshow(cv2.cvtColor(img3, cv2.COLOR_BGR2RGB))   # 原图
plt.title('Original3')
plt.axis('off')

plt.subplot(222)
plt.imshow(mask_img3, cmap='gray')                      # 掩码
plt.title('Mask3')
plt.axis('off')

plt.subplot(223)
plt.imshow(cv2.cvtColor(result_img3, cv2.COLOR_BGR2RGB))  # ← 带轮廓的结果图
plt.title('Contours3')
plt.axis('off')
plt.show()