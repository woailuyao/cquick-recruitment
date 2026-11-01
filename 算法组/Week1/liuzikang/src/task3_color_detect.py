import csv
import os
import sys
import cv2
import numpy as np

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(ROOT, "data", "images")
OUT = os.path.join(ROOT, "outputs", "task3")

SAMPLES = ["cup_clean", "cup_top", "cup_latte", "cup_multi"]

HSV_LOWER = (80, 20, 40)
HSV_UPPER = (135, 255, 255)
MIN_AREA_RATIO = 0.02          
MIN_AREA_ABS = 800
OPEN_KERNEL = (5, 5)
CLOSE_KERNEL = (9, 9)

NOTES = {
    "cup_clean": "杯碟相连，选中杯碟整体，结果稳定",
    "cup_top": "杯碟分开，避开面积更大的碟子，选中杯子",
    "cup_latte": "咖啡豆和布是暖色，不进入蓝色掩膜；杯碟相连使 bbox 偏大",
    "cup_multi": "画面有杯子和两个碟子，中心规则选中杯子",
}


def read_image(path, flags=cv2.IMREAD_COLOR):
    data = np.fromfile(path, dtype=np.uint8)
    return cv2.imdecode(data, flags)


def save_image(path, image):
    ok, data = cv2.imencode(".png", image)
    if not ok:
        raise IOError("保存失败：" + path)
    data.tofile(path)

    back = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    if back is None:
        raise IOError("回读失败：" + path)


def detect(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    raw = cv2.inRange(hsv, np.array(HSV_LOWER, np.uint8), np.array(HSV_UPPER, np.uint8))

    k_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, OPEN_KERNEL)
    k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, CLOSE_KERNEL)
    cleaned = cv2.morphologyEx(raw, cv2.MORPH_OPEN, k_open)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, k_close)

    contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)
        candidates.append({
            "contour": contour,
            "area": area,
            "bbox": (x, y, w, h),
            "center": (x + w // 2, y + h // 2),
        })
    candidates.sort(key=lambda c: c["area"], reverse=True)

    h, w = image.shape[:2]
    min_area = max(MIN_AREA_ABS, int(h * w * MIN_AREA_RATIO))
    valid = [c for c in candidates if c["area"] >= min_area]

    selected = min(
        valid,
        key=lambda c: (c["center"][0] - w / 2) ** 2 + (c["center"][1] - h / 2) ** 2,
    ) if valid else None
    return raw, cleaned, candidates, valid, selected, min_area

def draw_result(image, selected, min_area):
    """在原图上画轮廓、外接矩形和中心点。"""
    result = image.copy()
    if selected is None:
        cv2.rectangle(result, (12, 12), (520, 92), (0, 0, 0), -1)
        cv2.putText(result, "NO TARGET", (24, 50), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (0, 0, 255), 2, cv2.LINE_AA)
        return result

    x, y, w, h = selected["bbox"]
    cx, cy = selected["center"]
    area = selected["area"]

    cv2.drawContours(result, [selected["contour"]], -1, (0, 255, 255), 2)
    cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)
    cv2.circle(result, (cx, cy), 7, (0, 0, 255), -1)
    cv2.line(result, (cx - 16, cy), (cx + 16, cy), (0, 0, 255), 2)
    cv2.line(result, (cx, cy - 16), (cx, cy + 16), (0, 0, 255), 2)

    cv2.rectangle(result, (12, 12), (520, 92), (0, 0, 0), -1)
    cv2.putText(result, f"center=({cx},{cy}) area={area:.0f}", (24, 45),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(result, f"bbox=({x},{y},{w},{h}) min_area={min_area}",
                (24, 77), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (200, 255, 200), 1, cv2.LINE_AA)
    return result

def main():
    os.makedirs(OUT, exist_ok=True)
    rows = []
    results = []
    images = {}

    for stem in SAMPLES:
        path = os.path.join(DATA, stem + ".jpg")
        image = read_image(path)
        if image is None:
            raise FileNotFoundError("读取失败：" + path)
        images[stem] = image

        raw, cleaned, candidates, valid, selected, min_area = detect(image)
        result = draw_result(image, selected, min_area)

        save_image(os.path.join(OUT, stem + "_mask_raw.png"), raw)
        save_image(os.path.join(OUT, stem + "_mask.png"), cleaned)
        save_image(os.path.join(OUT, stem + "_result.png"), result)

        if selected is None:
            print(f"[{stem}] 未检测到目标")
            rows.append([stem, "MISS", 0, "", "", "", "", "", "", "", "", NOTES[stem]])
            continue

        x, y, w, h = selected["bbox"]
        cx, cy = selected["center"]
        area = selected["area"]

        moments = cv2.moments(selected["contour"])
        mx = moments["m10"] / moments["m00"] if moments["m00"] else 0
        my = moments["m01"] / moments["m00"] if moments["m00"] else 0

        print(f"[{stem}] 中心=({cx},{cy}) 面积={area:.0f} bbox=({x},{y},{w},{h}) "
              f"候选={len(candidates)} 有效={len(valid)}")
        rows.append([stem, "OK", round(area, 1), cx, cy, x, y, w, h,
                     round(mx, 1), round(my, 1), NOTES[stem]])
        results.append(result)

    if "cup_clean" in images:
        _, clean_main, _, _, _, _ = detect(images["cup_clean"])
        save_image(os.path.join(OUT, "mask.png"), clean_main)
        save_image(os.path.join(OUT, "result.png"),
                   read_image(os.path.join(OUT, "cup_clean_result.png")))

    if "cup_multi" in images:
        image = images["cup_multi"]
        _, _, _, valid, selected, min_area = detect(image)
        largest = valid[0] if valid else None
        wrong = draw_result(image, largest, min_area)
        cv2.putText(wrong, "STRESS: largest area = saucer", (24, 130),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 255), 2, cv2.LINE_AA)
        save_image(os.path.join(OUT, "stress_largest_wrong.png"), wrong)
        if largest:
            print(f"[反例] 最大面积 center={largest['center']} area={largest['area']:.0f}")
        if selected:
            print(f"[正确] 中心规则 center={selected['center']} area={selected['area']:.0f}")

    if len(results) == 4:
        thumbs = [cv2.resize(r, (480, 480)) for r in results]
        sheet = np.vstack([np.hstack(thumbs[:2]), np.hstack(thumbs[2:])])
        save_image(os.path.join(OUT, "all_results.png"), sheet)

    csv_path = os.path.join(OUT, "task3_measurements.csv")
    with open(csv_path, "w", encoding="utf-8-sig", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["image", "status", "area", "center_x", "center_y",
                         "bbox_x", "bbox_y", "bbox_w", "bbox_h",
                         "centroid_x", "centroid_y", "notes"])
        writer.writerows(rows)

    print("完成，结果在：", OUT)

if __name__ == "__main__":
    main()