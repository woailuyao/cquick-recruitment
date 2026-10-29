from pathlib import Path
import cv2


project = Path(__file__).resolve().parents[1]


image_dir = project / "data" / "images"
save_dir = project / "outputs" / "task1"


image_names = ["sample01.jpg", "sample02.jpg"]

for name in image_names:
    image_path = image_dir / name

    # 读取图片
    img = cv2.imread(str(image_path))
    print("原图尺寸：", img.shape)

    # 1. 缩放固定
    small = cv2.resize(img,(640, 360),interpolation=cv2.INTER_AREA)

    # 2. 缩小比例
    half = cv2.resize(img,None,fx=0.5,fy=0.5,interpolation=cv2.INTER_AREA)

    # 3. 裁剪
    h, w = img.shape[:2]
    crop_h = min(320, h)
    crop_w = min(400, w)
    start_y = (h - crop_h) // 2
    start_x = (w - crop_w) // 2
    crop = img[start_y:start_y + crop_h,start_x:start_x + crop_w].copy()

    # 4. 彩色图转灰度图
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 名字
    short_name = image_path.stem

    # 把要保存的图片放进一个列表
    results = [
        (short_name + "_resized_640x360.png", small),
        (short_name + "_scaled_50pct.png", half),
        (short_name + "_crop_center.png", crop),
        (short_name + "_gray.png", gray),
    ]

    # 保存
    for save_name, pic in results:
        save_path = save_dir / save_name
        ok = cv2.imwrite(str(save_path), pic)
        again = cv2.imread(str(save_path), cv2.IMREAD_UNCHANGED)
        print("保存成功：", save_name, again.shape)
