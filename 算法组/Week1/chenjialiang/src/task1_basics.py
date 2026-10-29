import cv2

# 调的时候改成 True 能一张张看，平时 False 一次跑完不卡
SHOW = False


def cv_show(name, image):
    if SHOW:
        cv2.imshow(name, image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def save(path, image):
    ok = cv2.imwrite(path, image)
    back = cv2.imread(path)
    print(f"  {path}  写入{'成功' if ok else '失败'}  回读{'成功' if back is not None else '失败'}")


img_1 = cv2.imread('data/images/大肥鱼.jpg')
img_2 = cv2.imread('data/images/小金鱼.jpg')
img_3 = cv2.imread('data/images/bird.jpg')


def step_1(image, name):
    h, w = image.shape[:2]
    print(f"{name}: 宽={w} 高={h} 通道数={image.shape[2]}")
    cv_show(name, image)


def step_23(image, name):
    # 放缩到一半
    image_copy_1 = image.copy()
    img_resize = cv2.resize(image_copy_1, None, fx=0.5, fy=0.5)
    cv_show(f"{name}-放缩", img_resize)
    save(f"outputs/task1/{name}-放缩.jpg", img_resize)

    # 裁左上角 200x200
    image_copy_2 = image.copy()
    img_crop = image_copy_2[0:200, 0:200]
    cv_show(f"{name}-裁剪", img_crop)
    save(f"outputs/task1/{name}-裁剪.jpg", img_crop)

    # 灰度
    image_copy_3 = image.copy()
    img_gray = cv2.cvtColor(image_copy_3, cv2.COLOR_BGR2GRAY)
    cv_show(f"{name}-灰度", img_gray)
    save(f"outputs/task1/{name}-灰度.jpg", img_gray)


print("=== 读图，打印宽高和通道数 ===")
step_1(img_1, "大肥鱼")
step_1(img_2, "小金鱼")
step_1(img_3, "bird")

print("=== 放缩 / 裁剪 / 灰度 ===")
step_23(img_1, "大肥鱼")
step_23(img_2, "小金鱼")
step_23(img_3, "bird")

print("=== 原图有没有被改掉 ===")
for p in ['data/images/大肥鱼.jpg', 'data/images/小金鱼.jpg', 'data/images/bird.jpg']:
    im = cv2.imread(p)
    print(f"  {p}  {im.shape}")
