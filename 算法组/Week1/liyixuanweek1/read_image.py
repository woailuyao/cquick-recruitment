import cv2

# 读取图片（使用绝对路径，防止找不到文件）
img = cv2.imread(r'D:\MyCode\data\images\1.jpeg')

# 判断图片是否读取成功，防止程序直接崩溃
if img is None:
    print("读取失败，请检查路径是否正确！")
else:
    print("读取成功！")
    
    # shape 返回的是一个元组 (高度, 宽度, 通道数)
    print(f"图像宽度: {img.shape[1]}, 图像高度: {img.shape[0]}")
    print(f"图像形状 (shape): {img.shape}")
    
    # dtype 通常是 uint8，代表像素值范围是 0-255 的无符号整数
    print(f"数据类型 (dtype): {img.dtype}")
    
    # 打印坐标 (100, 100) 处的像素值
    # OpenCV 默认读取的顺序是 BGR（蓝、绿、红），而不是人类习惯的 RGB
    print(f"坐标(100,100)的像素值 (BGR): {img[100, 100]}")