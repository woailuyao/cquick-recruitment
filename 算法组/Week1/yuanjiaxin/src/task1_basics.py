import numpy as np
import cv2 as cv

def process_image(img_name):
    img_path = f"data/images/{img_name}"
    img = cv.imread(img_path)
    if img is None:
        print(f"{img_name} 读取失败")
        return
    
    h, w = img.shape[:2]
    c = img.shape[2] if len(img.shape) == 3 else 1
    print(f"[{img_name}] 宽: {w}, 高: {h}, 通道数: {c}")
    
    scale = 0.5
    width = int(w * scale)
    height = int(h * scale)
    resized = cv.resize(img, (width, height), interpolation=cv.INTER_AREA)
    cropped = img[h//4 : h*3//4, w//4 : w*3//4]
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    
    save_paths = [
        f"outputs/task1/{img_name}_resized.jpg", 
        f"outputs/task1/{img_name}_cropped.jpg", 
        f"outputs/task1/{img_name}_gray.jpg"
    ]
    cv.imwrite(save_paths[0], resized)
    cv.imwrite(save_paths[1], cropped)
    cv.imwrite(save_paths[2], gray)
    
    for p in save_paths:
        check = cv.imread(p)
        print(p, "重新读取成功" if check is not None else "重新读取失败")

if __name__ == "__main__":
    for name in ["img1.jpg", "img2.jpg"]:
        process_image(name)
