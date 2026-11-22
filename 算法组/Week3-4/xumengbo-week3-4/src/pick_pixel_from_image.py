"""
外参自检 · 帧上点标记读像素（pick_pixel_from_image）
=====================================================
自动打开 IMAGE_DIR 目录下所有 jpg/png，可前后翻页；每张图独立记录
点击像素 (u, v)，翻页不会丢掉上一张已点的点。退出时按图打印可直接
粘进 check_extrinsics_projection.py 的 MARKERS 片段（真实 X/Y 留 0.0 占位）。

运行：python pick_pixel_from_image.py
按键：左键 加点；n 下一张；p 上一张；
      d 删当前图最后一点；r 清空当前图；q/Esc 退出并打印片段。
"""
import os
import cv2

# ===== 拍照存帧的目录（capture_check_frame.py 默认存这里）=====
IMAGE_DIR = r"D:\loc_demo_snapshots"
IMAGE_EXTS = (".jpg", ".jpeg", ".png")

# 收集目录下所有图片，按文件名排序
if os.path.isdir(IMAGE_DIR):
    images = sorted(
        os.path.join(IMAGE_DIR, f)
        for f in os.listdir(IMAGE_DIR)
        if f.lower().endswith(IMAGE_EXTS)
    )
else:
    images = []

# 每张图单独存一组点：{图序号: [(u, v), ...]}
picks = {i: [] for i in range(len(images))}
cur = 0    # 当前显示的图序号


def on_mouse(event, u, v, flags, param):
    """左键点击：往当前这张图的点列表里追加一个像素。"""
    global cur
    if event == cv2.EVENT_LBUTTONDOWN and images:
        picks[cur].append((u, v))


def draw(img):
    disp = img.copy()
    h = disp.shape[0]

    # 画出当前图已点的点：黄色十字 + 序号 + 像素坐标
    for i, (u, v) in enumerate(picks[cur]):
        cv2.drawMarker(disp, (u, v), (0, 255, 255),
                       cv2.MARKER_CROSS, 18, 2)
        cv2.putText(disp, f"P{i+1} ({u},{v})", (u + 10, v),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    # 顶部：第几张/总数 + 文件名
    cv2.putText(disp, f"img {cur+1}/{len(images)}  "
                       f"{os.path.basename(images[cur])}",
                (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
    # 底部操作提示
    cv2.putText(disp, "click add | n next | p prev | d del | r clear | q quit",
                (12, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (200, 200, 200), 1)
    return disp


def main():
    if not images:
        raise FileNotFoundError(
            f"目录里没有图片: {IMAGE_DIR}\n请先运行 capture_check_frame.py 拍帧。")

    win = "Pick Pixels"
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)

    global cur
    while True:
        img = cv2.imread(images[cur])
        if img is None:
            # 打不开就跳到下一张
            cur = (cur + 1) % len(images)
            continue
        cv2.imshow(win, draw(img))

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break
        elif key == ord("n"):                      # 下一张
            cur = (cur + 1) % len(images)
        elif key == ord("p"):                      # 上一张
            cur = (cur - 1) % len(images)
        elif key == ord("d") and picks[cur]:       # 删当前图最后一点
            picks[cur].pop()
        elif key == ord("r"):                       # 清空当前图
            picks[cur].clear()

    cv2.destroyAllWindows()

    # 退出汇总：逐张打印有点的图；真实 X/Y 先留 0.0，手填卷尺坐标
    print("\n==== 粘到 check_extrinsics_projection.py 的 MARKERS ====")
    for i, path in enumerate(images):
        pts = picks[i]
        if not pts:
            continue
        print(f"-- {os.path.basename(path)} --")
        print("MARKERS = [")
        for j, (u, v) in enumerate(pts):
            print(f'    ("P{j+1}", 0.0, 0.0, {u}, {v}),')
        print("]")
    print("\n注意：一次反算一般用同一帧上的多个点；"
          "把每行的 0.0, 0.0 改成卷尺量到的真实 X, Y。")


if __name__ == "__main__":
    main()
