import cv2
import os

# 准备输出目录
os.makedirs('W1/outputs/task1', exist_ok=True)

# 图片列表
image_paths = ['W1/data/images/IMG1.jpg', 'W1/data/images/IMG2.jpg']

# 循环处理
for path in image_paths:
    img = cv2.imread(path)
    if img is None:
        print(f"{path} 读取失败")
        continue

    name = os.path.splitext(os.path.basename(path))[0]
    
    # 打印尺寸
    h, w, c = img.shape
    print(f"{path} 宽:{w} 高:{h} 通道:{c}")

    # 缩放
    resized = cv2.resize(img, (400, 300))
    cv2.imwrite(f'W1/outputs/task1/{name}_resized.jpg', resized)
    check = cv2.imread(f'W1/outputs/task1/{name}_resized.jpg')
    if check is None:
        print(f"保存失败或无法重新读取: {f'W1/outputs/task1/{name}_resized.jpg'}")
    else:
        print(f"保存成功，可重新读取: {f'W1/outputs/task1/{name}_resized.jpg'}")
        
    # 裁剪
    cropped = img[100:300, 200:500]
    cv2.imwrite(f'W1/outputs/task1/{name}_cropped.jpg', cropped)
    check = cv2.imread(f'W1/outputs/task1/{name}_cropped.jpg')
    if check is None:
        print(f"保存失败或无法重新读取: {f'W1/outputs/task1/{name}_cropped.jpg'}")
    else:
        print(f"保存成功，可重新读取: {f'W1/outputs/task1/{name}_cropped.jpg'}")

    # 灰度
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imwrite(f'W1/outputs/task1/{name}_gray.jpg', gray)
    check = cv2.imread(f'W1/outputs/task1/{name}_gray.jpg')
    if check is None:
        print(f"保存失败或无法重新读取: {f'W1/outputs/task1/{name}_gray.jpg'}")
    else:
        print(f"保存成功，可重新读取: {f'W1/outputs/task1/{name}_gray.jpg'}")