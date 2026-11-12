import cv2
import numpy as np
from pathlib import Path


# 项目目录不写死，从当前脚本的位置向上查找。
project = Path(__file__).resolve().parents[1]
image_dir = project / "data" / "images"
save_dir = project / "outputs" / "task3"
save_dir.mkdir(parents=True, exist_ok=True)


# sample01 中蓝色三角形约为 H=109, S=194, V=246。
# 背景网格的 Hue 约为 97~104，因此把 Hue 下限提高到 108。
# 较高的饱和度下限 180 用于排除低饱和度的青色背景。
lower_blue = np.array([108, 180, 50])
upper_blue = np.array([120, 255, 255])


# 第二张图已经存在；如果第三张暗光测试图不存在，就用 sample01 生成。
base_path = image_dir / "sample01.jpg"
dark_path = image_dir / "sample03_dark.jpg"

if not dark_path.exists():
    base_img = cv2.imread(str(base_path))
    if base_img is None:
        raise ValueError("无法读取 sample01.jpg")
    dark_img = cv2.convertScaleAbs(base_img, alpha=0.35, beta=0)
    if cv2.imwrite(str(dark_path), dark_img) is False:
        raise OSError("无法生成 sample03_dark.jpg")


image_names = [
    "sample01.jpg",
    "sample02.jpg",
    "sample03_dark.jpg",
]

min_area = 1000
kernel = np.ones((3, 3), dtype=np.uint8)
records = ["图片,中心坐标x,中心坐标y,轮廓面积"]


for name in image_names:
    image_path = image_dir / name
    img = cv2.imread(str(image_path))

    if img is None:
        print(name, "图片打不开，跳过")
        records.append("{},找不到,找不到,0".format(name))
        continue

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # 开运算去掉零散噪点，闭运算填补目标内部的小洞。
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    good_contours = [
        c for c in contours
        if cv2.contourArea(c) >= min_area
    ]

    short_name = image_path.stem
    mask_path = save_dir / (short_name + "_mask.png")
    result_path = save_dir / (short_name + "_result.png")

    if not good_contours:
        result = img.copy()
        cv2.putText(
            result,
            "No blue target",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2,
        )
        cv2.imwrite(str(mask_path), mask)
        cv2.imwrite(str(result_path), result)
        records.append("{},找不到,找不到,0".format(name))
        print(name, "没有找到蓝色目标")
        continue

    target = max(good_contours, key=cv2.contourArea)
    area = cv2.contourArea(target)
    x, y, w, h = cv2.boundingRect(target)
    center_x = x + w // 2
    center_y = y + h // 2

    result = img.copy()
    cv2.rectangle(
        result,
        (x, y),
        (x + w, y + h),
        (0, 255, 0),
        2,
    )
    cv2.circle(
        result,
        (center_x, center_y),
        6,
        (0, 0, 255),
        -1,
    )
    cv2.putText(
        result,
        "area={:.0f}".format(area),
        (x, max(20, y - 8)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 0),
        2,
    )

    if cv2.imwrite(str(mask_path), mask) is False:
        raise OSError("掩膜保存失败：" + str(mask_path))
    if cv2.imwrite(str(result_path), result) is False:
        raise OSError("结果图保存失败：" + str(result_path))

    if cv2.imread(str(mask_path)) is None:
        raise OSError("掩膜无法读回：" + str(mask_path))
    if cv2.imread(str(result_path)) is None:
        raise OSError("结果图无法读回：" + str(result_path))

    records.append(
        "{},{},{},{:.2f}".format(name, center_x, center_y, area)
    )
    print(
        name,
        "中心坐标:", (center_x, center_y),
        "轮廓面积:", area,
    )


records_path = save_dir / "records.txt"
with records_path.open("w", encoding="utf-8") as file:
    file.write("\n".join(records) + "\n")

print("全部完成，结果保存在：", save_dir)
