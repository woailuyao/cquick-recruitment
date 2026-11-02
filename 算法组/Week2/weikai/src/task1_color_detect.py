"""任务 1：颜色目标检测（HSV 分割 + 轮廓 + 中心点）

运行（从 weikai/ 根目录，需先 conda activate cv_practice）：
    python src/task1_color_detect.py

输出：
    - 终端打印每张图的 detected / (u,v) / area / 候选数
    - outputs/task1_<name>_mask.png    二值掩膜
    - outputs/task1_<name>_result.png  原图 + 轮廓/外接矩形 + 中心点 + 坐标文字

坐标约定（与第二周验收一致）：
    - 图像左上角为原点 (0,0)
    - u = 横向像素（向右增大，对应列 x）
    - v = 纵向像素（向下增大，对应行 y）
    注意：这只是目标在图像中的位置，不是机器人真实世界坐标（转换需标定）。
"""
import os
import cv2
import numpy as np

# ============ 可调参数（README 需记录，评审可能现场修改） ============
# 目标颜色与 HSV 范围：目标是白色足球。
# 白色没有固定色相，HSV 里表现为 S(饱和度)低 + V(明度)高：
#   - S 上界取 80 排除草地的绿色（草地 S 很高，通常 >100）
#   - V 下界取 170 排除阴影里的暗色区域（球面暗部实测 V 约 150~190，故不能取太高）
TARGETS = [
    {"name": "white", "lower": (0, 0, 170), "upper": (180, 80, 255)},
]
MIN_AREA = 500          # 最小轮廓面积(像素平方)：小于此值当噪声丢弃
KERNEL_SIZE = 5         # 形态学开运算结构元边长

# 项目内相对路径（从仓库根目录 weikai/ 运行）
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IMAGE_DIR = os.path.join(ROOT, "data", "images")
OUTPUT_DIR = os.path.join(ROOT, "outputs")


def imread_safe(path):
    """读取图片，兼容 Windows 中文路径。

    cv2.imread 在路径含非 ASCII（如“视觉组”）时在某些环境会读不到文件，
    改用 np.fromfile 读字节 + cv2.imdecode 解析，稳定可靠。
    """
    data = np.fromfile(path, dtype=np.uint8)
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def imwrite_safe(path, img):
    """保存图片，兼容 Windows 中文路径。

    cv2.imwrite 在路径含非 ASCII 时同样不稳，改用 cv2.imencode 编码后 tofile 写字节。
    """
    ext = os.path.splitext(path)[1].lower() or ".png"
    ok, buf = cv2.imencode(ext, img)
    if not ok:
        return False
    buf.tofile(path)
    return True


def detect_target(img, lower, upper, min_area=MIN_AREA, kernel_size=KERNEL_SIZE):
    """对单张 BGR 图像做 HSV 颜色分割，返回目标信息字典。

    返回字段：detected / u / v / area / candidates(候选列表) / mask / contour
    约定：未检测到时 detected=False，u/v/area 全为 0（不能用未初始化坐标）。
    """
    result = {"detected": False, "u": 0.0, "v": 0.0, "area": 0.0,
              "radius": 0.0, "candidates": [], "mask": None, "contour": None}

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_np = np.array(lower, dtype=np.uint8)
    upper_np = np.array(upper, dtype=np.uint8)
    mask = cv2.inRange(hsv, lower_np, upper_np)

    # 形态学开运算（先腐蚀后膨胀）：去除掩膜上的细小噪点/孤立点
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    # 形态学闭运算（先膨胀后腐蚀）：把目标内部被贴片/阴影切碎的白色块连回整体
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    result["mask"] = mask

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 按面积过滤噪声，并收集所有候选（>=min_area）
    # 另外排除“贴边”轮廓：被图像边缘裁切的白色物体（球门柱、围栏杆、白线
    # 延伸出画面）不完整，其中心位置不可信，直接丢弃。
    h_img, w_img = mask.shape[:2]
    candidates = []
    for c in contours:
        a = cv2.contourArea(c)
        if a < min_area:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if x <= 0 or y <= 0 or x + w >= w_img or y + h >= h_img:
            continue                                  # 贴边轮廓，丢弃
        M = cv2.moments(c)
        if M["m00"] != 0:                       # 优先用轮廓矩(质心)
            cx, cy = M["m10"] / M["m00"], M["m01"] / M["m00"]
        else:                                    # m00==0 退回外接矩形中心，避免除零
            cx, cy = x + w / 2.0, y + h / 2.0
        candidates.append((float(a), float(cx), float(cy), c))
    candidates.sort(key=lambda t: t[0], reverse=True)   # 按面积从大到小

    result["candidates"] = [(a, cx, cy) for (a, cx, cy, _c) in candidates]
    if not candidates:
        return result                                # detected=False，u/v/area 保持 0

    # 多候选时选择“面积最大”的作为主目标（选择规则见 README）
    best = candidates[0]
    result["detected"] = True
    result["area"] = best[0]
    # 两级定位：颜色粗定位(白色质心) -> 霍夫圆精定位(球的几何中心)
    result["u"], result["v"], result["radius"] = refine_with_hough(
        img, best[3], best[1], best[2])
    result["contour"] = best[3]
    return result


def refine_with_hough(img, contour, cx, cy):
    """颜色粗定位之后，在候选的局部区域用霍夫圆变换做形状精定位。

    为什么需要这一步：纯颜色法的质心是「白色像素的质心」——球面黑色贴片
    和下半球阴影把白色区域挖掉一大块，质心会被拉向亮部，偏离真实球心。
    但球是圆的：用霍夫圆在候选附近找「圆形」，可以把中心修正到球的几何中心，
    这就是「颜色找哪里可能有球 + 形状确认球到底在哪」的特征融合。

    返回 (u, v, radius)。霍夫找不到合适圆时，退回「凸包 + 最小外接圆」——
    仍比裸质心稳（凸包能把被挖掉的白块缺口包回来），只是精度略低。
    """
    x, y, w, h = cv2.boundingRect(contour)
    r_est = max(w, h) / 2.0
    # 局部 ROI：外接矩形向外扩一个估计半径，给被阴影切掉的下半球留出空间
    pad = int(r_est)
    x0, y0 = max(0, x - pad), max(0, y - pad)
    x1 = min(img.shape[1], x + w + pad)
    y1 = min(img.shape[0], y + h + pad)
    roi = img[y0:y1, x0:x1]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 5)                  # 霍夫对噪声敏感，先平滑
    circles = cv2.HoughCircles(
        gray, cv2.HOUGH_GRADIENT, dp=1,
        minDist=max(2 * r_est, 10),                 # 局部区域只有一个球，间距放开
        param1=120,                                 # Canny 高阈值
        param2=30,                                  # 圆心累加器阈值：越小越容易检出圆
        minRadius=int(r_est * 0.5), maxRadius=int(r_est * 1.5))
    if circles is not None:
        circles = np.round(circles[0, :]).astype(float)
        # 局部可能有多个圆，取「离颜色质心最近」的那个：颜色结果做门控，防止跳到别的圆形物体上
        dist2 = (circles[:, 0] + x0 - cx) ** 2 + (circles[:, 1] + y0 - cy) ** 2
        bc = circles[int(np.argmin(dist2))]
        return float(bc[0] + x0), float(bc[1] + y0), float(bc[2])
    # 霍夫失败 -> 凸包 + 最小外接圆
    hull = cv2.convexHull(contour)
    (fx, fy), fr = cv2.minEnclosingCircle(hull)
    return float(fx), float(fy), float(fr)


def draw_result(img, result, color=(0, 255, 0)):
    """在副本上画：轮廓 + 外接矩形 + 中心点 + 坐标文字，返回绘制后图像。"""
    vis = img.copy()
    if result["detected"] and result["contour"] is not None:
        c = result["contour"]
        cv2.drawContours(vis, [c], -1, color, 2)
        x, y, w, h = cv2.boundingRect(c)
        cv2.rectangle(vis, (x, y), (x + w, y + h), (255, 0, 0), 2)
        u, v = int(round(result["u"])), int(round(result["v"]))
        r = int(round(result.get("radius", 0.0)))
        if r > 0:   # 霍夫圆/外接圆：形状精定位结果（黄色），能看出圆心是否在球心
            cv2.circle(vis, (u, v), r, (0, 255, 255), 2)
        cv2.drawMarker(vis, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 16, 2)
        cv2.circle(vis, (u, v), 4, (0, 0, 255), -1)
        text = "u=%d, v=%d, area=%.0f" % (u, v, result["area"])
        cv2.putText(vis, text, (x, max(y - 10, 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    else:
        cv2.putText(vis, "NOT DETECTED", (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    return vis


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    image_files = sorted([f for f in os.listdir(IMAGE_DIR)
                          if f.lower().endswith((".jpg", ".jpeg", ".png"))])
    if not image_files:
        print("未在 %s 找到测试图片" % IMAGE_DIR)
        return

    print("==== 任务 1：颜色目标检测 ====")
    target = TARGETS[0]                       # 目标：白色足球
    summary = []
    for fname in image_files:
        img_path = os.path.join(IMAGE_DIR, fname)
        img = imread_safe(img_path)
        assert img is not None, "读取失败: %s" % img_path   # 读图失败立即报错

        res = detect_target(img, target["lower"], target["upper"])
        base = os.path.splitext(fname)[0]

        # 先存掩膜，方便排查阈值是否合理
        imwrite_safe(os.path.join(OUTPUT_DIR, "task1_%s_mask.png" % base), res["mask"])

        if not res["detected"]:
            # 未检测到 -> 明确提示，u/v/area 用约定默认 0，不用未初始化坐标
            print("[%s] 未检测到目标（detected=False, u/v/area=0）" % fname)
            imwrite_safe(os.path.join(OUTPUT_DIR, "task1_%s_result.png" % base),
                         draw_result(img, res))       # 图上也会打印 NOT DETECTED
            summary.append((fname, False, 0, 0, 0))
        else:
            print("[%s] 颜色=%s detected=True (u,v)=(%.0f,%.0f) area=%.0f 候选数=%d"
                  % (fname, target["name"], res["u"], res["v"],
                     res["area"], len(res["candidates"])))
            imwrite_safe(os.path.join(OUTPUT_DIR, "task1_%s_result.png" % base),
                         draw_result(img, res))
            summary.append((fname, True, res["u"], res["v"], res["area"]))

    print("==== 汇总 ====")
    for (fname, det, u, v, area) in summary:
        if det:
            print("  %s: 成功 (u,v)=(%.0f,%.0f) area=%.0f" % (fname, u, v, area))
        else:
            print("  %s: 未检测到" % fname)
    print("结果图已保存到 outputs/（task1_*_mask.png / task1_*_result.png）")


if __name__ == "__main__":
    main()
