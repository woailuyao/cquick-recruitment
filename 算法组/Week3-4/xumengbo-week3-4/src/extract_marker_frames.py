"""
任务 2 场地标志视频抽帧脚本（YOLO 数据集骨架）
=================================================
输入：D:\\W34_dataset\\marks\\ 下的 5 段标志视频（clip_02 ~ clip_06.avi）
输出：D:\\W34_dataset\\markers\\images\\{train,val,test}\\ 下的 jpg 图片
      （labels 目录已由骨架建好，抽帧阶段不生成标签，标签后续用标注工具写）

用法（必须用 Python 3.10 全路径解释器）：
  C:\\Users\\A\\AppData\\Local\\Programs\\Python\\Python310\\python.exe extract_marker_frames.py

划分规则（和任务 1 一致）：按独立视频片段划分，同一片段只能进一个划分，
不打散相邻帧；如需调整，改下面 SPLIT 字典即可。
"""
import os
import cv2
import numpy as np

# ===== 路径与采样参数（按需修改）=====
RAW = r"D:\W34_dataset\marks"          # 原始标志视频所在目录
ROOT = r"D:\W34_dataset\markers"       # 输出数据集根目录
SAMPLE_FPS = 4.0                        # 抽帧帧率：每秒取 4 帧（30fps 视频即每 7~8 帧取 1 帧）
DIFF_THRESH = 2.0                       # 相邻缩略图灰度差阈值，小于该值认为画面几乎没变，跳过
# 个别视频画面变化很慢时，可单独给更低阈值，例如：DIFF_BY_VIDEO = {"clip_04": 0.15}
DIFF_BY_VIDEO = {}

# 类别编号必须与 markers_data.yaml、classes.txt 一致（细则 2.3）
CLASS_NAMES = ["LCross", "TCross", "XCross"]

# 视频片段 -> 数据划分。5 段默认：3 段 train、1 段 val、1 段 test。
# 注意：同一片段不要同时出现在两个划分里。
SPLIT = {
    "clip_02": "train",
    "clip_03": "train",
    "clip_06": "train",
    "clip_05": "val",
    "clip_04": "test",
}


def ensure_dirs():
    """建目录、清空旧抽帧图片（保留已有标签）、写入类别清单 classes.txt。"""
    for sub in ("train", "val", "test"):
        img_dir = os.path.join(ROOT, "images", sub)
        lbl_dir = os.path.join(ROOT, "labels", sub)
        os.makedirs(img_dir, exist_ok=True)
        os.makedirs(lbl_dir, exist_ok=True)

        # 只删旧 jpg，不删 labels 里的 txt，避免误删已标注内容
        for fn in os.listdir(img_dir):
            if fn.lower().endswith(".jpg"):
                os.remove(os.path.join(img_dir, fn))

        # 每个划分目录都放一份 classes.txt，内容与 CLASS_NAMES 对齐
        with open(os.path.join(lbl_dir, "classes.txt"), "w", encoding="utf-8") as f:
            f.write("\n".join(CLASS_NAMES) + "\n")


def thumb(frame):
    """把帧缩成 64×64 灰度缩略图，用于快速判断相邻帧是否几乎相同。"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return cv2.resize(gray, (64, 64), interpolation=cv2.INTER_AREA).astype(np.float32)


def extract():
    """遍历 SPLIT 里的每段视频，按采样间隔 + 差分去重抽帧并保存。"""
    total = {}
    for stem, split in SPLIT.items():
        path = os.path.join(RAW, stem + ".avi")
        if not os.path.exists(path):
            print(f"{stem}: 文件不存在，跳过")
            continue

        cap = cv2.VideoCapture(path)
        if not cap.isOpened():
            print(f"{stem}: 打不开")
            continue

        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0          # 视频原始帧率
        step = max(1, int(round(fps / SAMPLE_FPS)))      # 每隔多少帧取 1 张
        diff_thresh = DIFF_BY_VIDEO.get(stem, DIFF_THRESH)
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

        kept = 0
        last_thumb = None
        idx = 0
        while idx < total_frames:
            cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
            ok, frame = cap.read()
            if not ok:
                break

            t = thumb(frame)
            # 和上一张保留帧比灰度差，变化太小就跳过，避免存大量重复背景
            if last_thumb is not None:
                diff = float(np.abs(t - last_thumb).mean())
                if diff < diff_thresh:
                    idx += step
                    continue

            # 文件名带片段名和帧号，保证全局唯一、可追溯到原视频
            name = f"{stem}_f{idx:06d}.jpg"
            ok2, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
            if ok2:
                with open(os.path.join(ROOT, "images", split, name), "wb") as f:
                    f.write(buf.tobytes())
                kept += 1
                last_thumb = t
            idx += step

        cap.release()
        total[split] = total.get(split, 0) + kept
        print(f"{stem} -> {split}: 抽 {kept} 张（视频共 {total_frames} 帧，间隔 {step} 帧）")

    return total


def main():
    ensure_dirs()
    extract()

    print("--- 汇总 ---")
    alln = 0
    for s in ("train", "val", "test"):
        n = len(os.listdir(os.path.join(ROOT, "images", s)))
        alln += n
        print(f"{s}: {n} 张")
    print(f"合计: {alln} 张")


if __name__ == "__main__":
    # Windows 多进程保护：训练/标注类脚本入口必须包 main() 并加这一行
    main()
