import cv2
import os

img_list = [
    r"data/images/blueberry.jpg",
    r"data/images/orange.jpg"
]

out_dir = "outputs/task1"
os.makedirs(out_dir, exist_ok=True)

for idx, path in enumerate(img_list):
    img = cv2.imread(path)
    if img is None:
        print(f"图片 {path} 读取失败！")
        continue

    h, w, c = img.shape
    print(f"\n===== 第{idx+1}张图：{path} =====")
    print(f"高度 h = {h}")
    print(f"宽度 w = {w}")
    print(f"通道数 c = {c}")

    img_copy = img.copy()
    x, y = 200, 200
    img_copy[y, x] = [255, 255, 255]

    save_name = f"draw_point_{idx+1}.jpg"
    cv2.imwrite(os.path.join(out_dir, save_name), img_copy)

    cv2.imshow(f"img_{idx+1}_origin", img)
    cv2.imshow(f"img_{idx+1}_draw", img_copy)

print("\n✅ task1 两张图片全部处理完成，结果在 outputs/task1")
cv2.waitKey(0)
cv2.destroyAllWindows()




