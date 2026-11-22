import cv2
import os

# 图片路径 + 对应的HSV上下限
img_info = [
    {
        "path": r"data/images/blueberry.jpg",
        "lower": (90, 60, 40),
        "upper": (130, 255, 255)
    },
    {
        "path": r"data/images/soccer.jpg",
        "lower": (0, 0, 0),
        "upper": (180, 255, 80)
    },
    {
        "path": r"data/images/leaf.jpg",
        "lower": (35, 60, 40),
        "upper": (75, 255, 255)
    }
]

out_dir = "outputs/task3"
os.makedirs(out_dir, exist_ok=True)

for idx, info in enumerate(img_info):
    path = info["path"]
    lower = info["lower"]
    upper = info["upper"]
    img = cv2.imread(path)
    if img is None:
        print(f"图片 {path} 读取失败！")
        continue

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7,7))
    mask_clean = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    draw_img = img.copy()
    for cnt in contours:
        x,y,ww,hh = cv2.boundingRect(cnt)
        cv2.rectangle(draw_img, (x,y), (x+ww,y+hh), (0,255,0), 3)

    cv2.imwrite(os.path.join(out_dir, f"mask_{idx+1}.jpg"), mask)
    cv2.imwrite(os.path.join(out_dir, f"mask_clean_{idx+1}.jpg"), mask_clean)
    cv2.imwrite(os.path.join(out_dir, f"result_{idx+1}.jpg"), draw_img)

    cv2.imshow(f"img{idx+1}_mask", mask)
    cv2.imshow(f"img{idx+1}_clean", mask_clean)
    cv2.imshow(f"img{idx+1}_result", draw_img)

print("\n✅ task3 三张图片全部处理完成，结果保存在 outputs/task3")
cv2.waitKey(0)
cv2.destroyAllWindows()
