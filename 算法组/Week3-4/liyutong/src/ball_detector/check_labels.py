# -*- coding: utf-8 -*-
"""
标注质检脚本（考核 1.5 明确要求：随机抽查 30 张把标签框画回原图）
用法：conda activate calib && python check_labels.py
      默认检查 soccer_dataset/；也可以传路径：python check_labels.py raw
输出：qa_out/ 下画好框的图 + 终端统计报告
"""
import cv2
import os
import sys
import random
import numpy as np

TARGET = sys.argv[1] if len(sys.argv) > 1 else "soccer_dataset"
QA_DIR = "qa_out"
SPLITS = ["train", "val", "test"]

def imread_safe(path):
    data = np.fromfile(path, dtype=np.uint8)
    return cv2.imdecode(data, cv2.IMREAD_COLOR) if data.size else None

def imwrite_safe(path, img):
    ok, buf = cv2.imencode(os.path.splitext(path)[1], img)
    if ok:
        buf.tofile(path)
    return ok

def find_pairs(root):
    """返回 [(图片路径, 标签路径或None)]"""
    pairs = []
    for dirpath, _, files in os.walk(root):
        for f in files:
            if f.lower().endswith((".jpg", ".png")):
                img = os.path.join(dirpath, f)
                lbl = os.path.join(dirpath.replace("images", "labels"), os.path.splitext(f)[0] + ".txt")
                pairs.append((img, lbl if os.path.exists(lbl) else None))
    return pairs

def main():
    os.makedirs(QA_DIR, exist_ok=True)
    pairs = find_pairs(TARGET)
    if not pairs:
        print(f"{TARGET} 下没找到图片")
        return

    stats = {"图片数": len(pairs), "无标注": 0, "越界": 0, "零面积": 0, "类别": {}}
    checked = []
    for img_path, lbl_path in pairs:
        img = imread_safe(img_path)
        if img is None:
            print(f"  读不出：{img_path}")
            continue
        h, w = img.shape[:2]
        if lbl_path is None:
            stats["无标注"] += 1
            continue
        with open(lbl_path, "r", encoding="utf-8", errors="ignore") as f:
            lines = [l.split() for l in f if l.strip()]
        bad_here = False
        for parts in lines:
            if len(parts) < 5:
                bad_here = True
                continue
            cid = int(float(parts[0]))
            xc, yc, bw, bh = map(float, parts[1:5])
            stats["类别"][cid] = stats["类别"].get(cid, 0) + 1
            if not (0 <= xc <= 1 and 0 <= yc <= 1) or bw <= 0 or bh <= 0 or bw > 1 or bh > 1:
                stats["越界"] += 1
                bad_here = True
            if bw * h > w * 1.5:      # 框宽超过图像宽度太多的异常
                stats["零面积"] += 1
        # 画回去
        vis = img.copy()
        for parts in lines:
            if len(parts) < 5:
                continue
            cid = int(float(parts[0]))
            xc, yc, bw, bh = map(float, parts[1:5])
            x1 = int((xc - bw / 2) * w); y1 = int((yc - bh / 2) * h)
            x2 = int((xc + bw / 2) * w); y2 = int((yc + bh / 2) * h)
            color = (0, 255, 0) if cid == 0 else (0, 0, 255)
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
            cv2.putText(vis, f"{cid}", (x1, max(y1 - 5, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        out = os.path.join(QA_DIR, os.path.basename(img_path))
        imwrite_safe(out, vis)
        checked.append((img_path, len(lines), bad_here))

    print("=" * 52)
    print(f"抽查目录：{TARGET}")
    print(f"图片总数      : {stats['图片数']}")
    print(f"没有标签文件  : {stats['无标注']}")
    print(f"坐标越界/异常 : {stats['越界']}")
    print(f"类别分布      : {stats['类别']}   (0 必须是 ball)")
    if 0 not in stats["类别"]:
        print("⚠️ 一整个 0 类都没有，检查标注是否真的保存了")
    if any(k != 0 for k in stats["类别"]):
        print("⚠️ 出现了非 0 类别，考核只允许 ball（class_id 0）")

    # 泄漏检查：不同 split 之间是否有同名文件（同段视频的帧被拆开）
    if os.path.isdir(os.path.join(TARGET, "images")):
        names = {}
        for s in SPLITS:
            d = os.path.join(TARGET, "images", s)
            if os.path.isdir(d):
                for f in os.listdir(d):
                    names.setdefault(os.path.splitext(f)[0].rsplit("_", 1)[0], set()).add(s)
        leak = {k: v for k, v in names.items() if len(v) > 1}
        print(f"跨集合同名视频段（疑似泄漏）: {len(leak)} 个")
        for k, v in list(leak.items())[:5]:
            print(f"  {k} 同时出现在 {v}")
    print("=" * 52)
    print(f"画好框的图在 {QA_DIR}/ ，一张张过一眼，重点看漏标和框太大")

if __name__ == "__main__":
    main()
