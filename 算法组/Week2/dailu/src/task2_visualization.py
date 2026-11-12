import cv2
import numpy as np
import os
import argparse

# 建一个文件夹
os.makedirs("outputs/task2", exist_ok=True)

def process(img, hsv_min, hsv_max, min_area):
    """核心检测 + 画图"""
    #1. 转成HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #2. 生成二值掩膜
    mask = cv2.inRange(hsv, np.array(hsv_min), np.array(hsv_max))
    #3. 形态学去噪点
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
    #4. 找轮廓并且按面积筛选
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid_contours = [c for c in contours if cv2.contourArea(c) > min_area]
    
    vis = img.copy()
    if not valid_contours:
        cv2.putText(vis, "NO TARGET", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        return vis, mask, False
    
    # 面积最大的轮廓
    best_c = max(valid_contours, key=cv2.contourArea)
    M = cv2.moments(best_c)
    u, v = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
    
    # 画标注
    cv2.drawContours(vis, [best_c], -1, (0, 255, 0), 2)
    cv2.circle(vis, (u, v), 6, (0, 0, 255), -1)
    cv2.putText(vis, f"u={u}, v={v}", (u+10, v-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    
    return vis, mask, True

def main():

    image_path = "data/task1_03.jpg"  
    
    img = cv2.imread(image_path)
    if img is None:
        print(f"图片读取失败！请检查路径和文件名对不对：{image_path}")
        return

    # 默认参数（橙黄色，请按需修改）
    p_min, p_max, p_area = [5, 120, 50], [22, 255, 255], 2500

    # 1. 正常检测
    vis_base, mask_base, _ = process(img, p_min, p_max, p_area)
    
    # 2. 对比检测（把最小面积调大 3 倍，大概率漏检）
    vis_comp, mask_comp, _ = process(img, p_min, p_max, p_area * 3)

    # --- 拼图 1：02_结果可视化.png (原图和标注图左右拼) ---
    gap = np.full((img.shape[0], 10, 3), 255, np.uint8)
    res_vis = np.hstack([img, gap, vis_base])
    cv2.imwrite("outputs/task2/02_visualization.png", res_vis)

    # --- 拼图 2：02_参数对比.png (基线 vs 面积变大，上下拼) ---
    def to_bgr(m): return cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)
    row1 = np.hstack([vis_base, gap, to_bgr(mask_base)])
    row2 = np.hstack([vis_comp, gap, to_bgr(mask_comp)])
    vgap = np.full((10, row1.shape[1], 3), 255, np.uint8)
    res_comp = np.vstack([row1, vgap, row2])
    cv2.imwrite("outputs/task2/02_comparision.png", res_comp)

    print(f"成功处理图片：{image_path}")
    print("ok")

if __name__ == "__main__":
    main()