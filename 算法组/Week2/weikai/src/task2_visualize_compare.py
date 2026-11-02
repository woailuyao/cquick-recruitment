"""任务 2：可视化与参数对比

运行（从 weikai/ 根目录，需先 conda activate cv_practice）：
    python src/task2_visualize_compare.py

复用 src/task1_color_detect.py 的 detect_target / draw_result，保证两任务检测逻辑一致。

产出（outputs/）：
    - task2_annotated_<color>.png   原图 + 轮廓/外接矩形 + 中心点 + 坐标文字
    - task2_compare_hsv.png        HSV 范围（宽 vs 窄）掩膜/候选数对比
    - task2_compare_minarea.png    最小面积（小 vs 大）检测对比（大阈值导致漏检）
    - task2_false_detection.png    误检示例（阈值过低，小噪点被当目标）
    - task2_miss_detection.png     漏检示例（目标过小 / 阈值过高）

坐标约定说明（验收点）：
    - 左上角原点，u 向右、v 向下；u/v 是“像素坐标”。
    - 像素坐标 ≠ 机器人真实世界坐标 / 米制距离，换算需相机标定，本周不要求。
"""
import os
import sys
import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")          # 无界面保存图片
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IMAGE_DIR = os.path.join(ROOT, "data", "images")
OUTPUT_DIR = os.path.join(ROOT, "outputs")

# 让本脚本能 import 同目录的 task1 模块
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from task1_color_detect import detect_target, draw_result, TARGETS, MIN_AREA, imread_safe, imwrite_safe


def _load(name):
    p = os.path.join(IMAGE_DIR, name)
    img = imread_safe(p)
    assert img is not None, "读取失败: " + p
    return img


def _list_images():
    return sorted([f for f in os.listdir(IMAGE_DIR)
                   if f.lower().endswith((".jpg", ".jpeg", ".png"))])


def _save_two(imgs, titles, suptitle, fname):
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.2))
    for ax, im, ti in zip(axes, imgs, titles):
        ax.imshow(cv2.cvtColor(im, cv2.COLOR_BGR2RGB))
        ax.set_title(ti, fontsize=10)
        ax.axis("off")
    fig.suptitle(suptitle, fontsize=12)
    fig.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, fname), dpi=110, bbox_inches="tight")
    plt.close(fig)


def visualize():
    """2.1 可视化：把轮廓/外接矩形/中心点/坐标文字画回原图并保存到文件。"""
    print("== 2.1 可视化（轮廓/外接矩形/中心点/坐标文字）==")
    t = TARGETS[0]                       # 目标：白色足球
    for fname in _list_images():
        img = _load(fname)
        res = detect_target(img, t["lower"], t["upper"])
        vis = draw_result(img, res)
        base = os.path.splitext(fname)[0]
        out = os.path.join(OUTPUT_DIR, "task2_annotated_%s.png" % base)
        imwrite_safe(out, vis)
        print("  %s: detected=%s (u,v)=(%.0f,%.0f) -> %s"
              % (fname, res["detected"], res["u"], res["v"], os.path.basename(out)))


def compare_hsv():
    """2.2 参数对比 A：HSV 范围（S/V 宽 vs 窄），每次只改一个变量。"""
    print("== 2.2 参数对比A：HSV 范围（S/V 宽 vs 窄）==")
    img = _load(_list_images()[0])
    t = TARGETS[0]                       # 白色目标：宽窄只改 S/V，H 保持全 0~180
    wide = detect_target(img, t["lower"], t["upper"])
    # 窄范围：S 上界 60->30、V 下界 180->210（球的阴影/偏灰部分会被切掉）
    narrow = detect_target(img, (t["lower"][0], 0, 210), (t["upper"][0], 30, 255))
    wpx = int(np.count_nonzero(wide["mask"]))
    npx = int(np.count_nonzero(narrow["mask"]))
    _save_two([wide["mask"], narrow["mask"]],
              ["wide S/V: %d px, %d cand" % (wpx, len(wide["candidates"])),
               "narrow S/V: %d px, %d cand" % (npx, len(narrow["candidates"]))],
              "HSV range (only S/V changed): wide tolerates light; narrow may cut/miss",
              "task2_compare_hsv.png")
    print("  宽范围: 掩膜=%d px 候选=%d 中心=(%.0f,%.0f)；窄范围: 掩膜=%d px 候选=%d 中心=(%.0f,%.0f)"
          % (wpx, len(wide["candidates"]), wide["u"], wide["v"],
             npx, len(narrow["candidates"]), narrow["u"], narrow["v"]))
    print("  -> S/V 取窄会让偏色/暗部像素丢失，掩膜变小、轮廓残缺，中心点随之偏移。")


def compare_minarea():
    """2.3 参数对比 B：最小轮廓面积（小 vs 大），每次只改一个变量。"""
    print("== 2.3 参数对比B：最小轮廓面积（小 vs 大）==")
    img = _load(_list_images()[0])
    t = TARGETS[0]
    small = detect_target(img, t["lower"], t["upper"], min_area=50)
    large = detect_target(img, t["lower"], t["upper"], min_area=80000)   # 大过目标面积(59781) -> 漏检
    _save_two([draw_result(img, small), draw_result(img, large)],
              ["min_area=50: detected=%s" % small["detected"],
               "min_area=80000: detected=%s (target dropped)" % large["detected"]],
              "Min area (only min_area changed): too large drops the real target",
              "task2_compare_minarea.png")
    print("  min_area=50 detected=%s；min_area=80000 detected=%s（阈值大过目标面积，球被当噪声丢弃）"
          % (small["detected"], large["detected"]))


def _make_false_image():
    """合成图：大白圆目标 + 多个白色小噪点（模拟反光噪点）。"""
    h, w = 400, 600
    img = np.full((h, w, 3), 120, np.uint8)          # 暗灰背景(V=120)，不在白色范围(V>=180)
    cv2.circle(img, (300, 200), 60, (255, 255, 255), -1)  # 主目标，面积 ~11300
    rng = np.random.default_rng(0)
    for _ in range(10):
        x = int(rng.integers(30, w - 30))
        y = int(rng.integers(30, h - 30))
        r = int(rng.integers(7, 11))                  # 小噪点半径 7~10（面积<500，合理阈值下被滤除）
        cv2.circle(img, (x, y), r, (255, 255, 255), -1)
    return img


def draw_candidates(img, lower, upper, min_area):
    """检测并把“所有”满足 min_area 的候选轮廓/外接矩形/中心点都画出来。

    用于误检/漏检的直观展示：阈值低时一堆方框、阈值合理时只剩一个。
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    vis = img.copy()
    n = 0
    for c in contours:
        if cv2.contourArea(c) >= min_area:
            n += 1
            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(vis, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.drawContours(vis, [c], -1, (0, 255, 0), 2)
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx, cy = M["m10"] / M["m00"], M["m01"] / M["m00"]
            else:
                cx, cy = x + w / 2.0, y + h / 2.0
            cv2.drawMarker(vis, (int(round(cx)), int(round(cy))), (0, 0, 255), cv2.MARKER_CROSS, 12, 2)
    return vis, n


def false_detection():
    """2.4 误检案例：阈值过低，小噪点被当作目标候选。"""
    print("== 2.4 误检案例 ==")
    img = _make_false_image()
    t = TARGETS[0]
    vis_low, n_low = draw_candidates(img, t["lower"], t["upper"], min_area=20)    # 阈值过低
    vis_high, n_high = draw_candidates(img, t["lower"], t["upper"], min_area=500)  # 合理阈值
    _save_two([vis_low, vis_high],
              ["min_area=20: %d boxes (false +)" % n_low,
               "min_area=500: %d box (correct)" % n_high],
              "False detection: too-low min_area treats noise as targets",
              "task2_false_detection.png")
    print("  min_area=20 框出 %d 个（含 %d 个噪点误判）；min_area=500 只框出 %d 个（正确）。"
          % (n_low, n_low - n_high, n_high))
    print("  结论：min_area 过小会把背景/噪点误判为目标，应取合理阈值。")


def _make_small_target_image():
    """合成图：仅一个很小的白色目标（面积小于默认 500）。"""
    h, w = 400, 600
    img = np.full((h, w, 3), 120, np.uint8)
    cv2.circle(img, (300, 200), 12, (255, 255, 255), -1)   # 小目标，面积 ~452 < 500
    return img


def miss_detection():
    """2.5 漏检/定位偏差案例：目标过小 / min_area 过高导致漏检。"""
    print("== 2.5 漏检/定位偏差案例 ==")
    img = _make_small_target_image()
    t = TARGETS[0]
    vis_high, n_high = draw_candidates(img, t["lower"], t["upper"], min_area=500)  # 默认阈值 -> 漏检
    vis_low, n_low = draw_candidates(img, t["lower"], t["upper"], min_area=50)    # 降低阈值 -> 检出
    _save_two([vis_high, vis_low],
              ["min_area=500: %d box (missed)" % n_high,
               "min_area=50: %d box (found)" % n_low],
              "Missed detection: small target dropped by high min_area",
              "task2_miss_detection.png")
    print("  min_area=500 框出 %d 个（小目标被当噪声漏掉）；min_area=50 框出 %d 个（检出）。"
          % (n_high, n_low))
    print("  说明：目标过小/被遮挡/光照过强，或阈值不匹配都会漏检或中心点偏移。")
    print("  提醒：u/v 是像素坐标，不能直接当作机器人运动指令（像素≠真实距离）。")


def main():
    print("==== 任务 2：可视化与参数对比 ====")
    visualize()
    compare_hsv()
    compare_minarea()
    false_detection()
    miss_detection()
    print("==== 完成，结果图见 outputs/ ====")


if __name__ == "__main__":
    main()
