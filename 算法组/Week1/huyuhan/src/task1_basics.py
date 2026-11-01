import cv2 as cv
import sys
for name in ("test1", "test2"):
    image=cv.imread(f"data/images/{name}.jpg")
    if image is None:
        sys.exit(f"读取失败：{name}.jpg")
    height,width, channels = image.shape
    print(f"{name}.jpg")
    print("高度：", height)
    print("宽度：", width)
    print("通道数：", channels)
    resized = cv.resize(image,(width // 2, height // 2),interpolation=cv.INTER_AREA)
    cropped = image[
        height // 4:3 * height // 4,
        width // 4:3 * width // 4]
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    results = (
        (f"{name}_resize.png", resized),
        (f"{name}_crop.png", cropped),
        (f"{name}_gray.png", gray))
    for filename, result in results:
        path = f"outputs/task1/{filename}"
        success = cv.imwrite(path, result)
        if not success:
            sys.exit(f"保存失败：{filename}")
        check = cv.imread(path, cv.IMREAD_UNCHANGED)
        if check is None:
            sys.exit(f"重新读取失败：{filename}")
        if check.shape != result.shape:
            sys.exit(f"保存前后存在不一致：{filename}")
        print(f"保存并验证成功：{filename}，shape={check.shape}")
print("任务一完成：两张原图的六张处理结果均已保存并验证。")