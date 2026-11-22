"""
离线验证：在一段原始视频上跑 best.pt 推理，不弹窗口，验证检测逻辑与实际效果。
输出：每帧检测统计 + 前若干张带框样例图（存纯英文目录）+ 平均推理耗时/FPS。
"""
import os
import sys
import time
import cv2
from ultralytics import YOLO

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from detect_realtime import draw_detections  # 复用实时脚本同一套画框逻辑

WEIGHTS = r"D:\W3-4_许勐搏_单目定位\runs\ball\train_v2\weights\best.pt"
VIDEO = r"D:\W34_dataset\raw\clip_02.avi"
OUT_DIR = r"D:\ball_demo_snapshots"
CONF = 0.35
IOU = 0.45
DEVICE = 0
STEP = 3            # 每隔几帧取一帧，加快验证
MAX_FRAMES = 40     # 最多验证多少帧
SAVE_EVERY = 3      # 每检测到几帧保存一张样例

os.makedirs(OUT_DIR, exist_ok=True)


def main():
    model = YOLO(WEIGHTS)
    cap = cv2.VideoCapture(VIDEO)
    if not cap.isOpened():
        raise RuntimeError(f"打不开视频: {VIDEO}")

    frame_idx = 0
    saved = 0
    infer_times = []
    total_det = 0

    while frame_idx < MAX_FRAMES:
        ok, frame = cap.read()
        if not ok:
            break
        if frame_idx % STEP != 0:
            frame_idx += 1
            continue

        t0 = time.perf_counter()
        results = model.predict(frame, conf=CONF, iou=IOU, device=DEVICE,
                                imgsz=640, verbose=False)
        infer_ms = (time.perf_counter() - t0) * 1000.0
        infer_times.append(infer_ms)

        # 复用实时脚本画框：返回 [{"name","conf","box","center"}, ...]
        det = draw_detections(frame, results)

        total_det += len(det)
        print(f"frame {frame_idx:3d}  infer {infer_ms:5.1f}ms  n={len(det)}  {det}")

        if det and saved < SAVE_EVERY:
            p = os.path.join(OUT_DIR, f"offline_f{frame_idx:03d}.jpg")
            cv2.imwrite(p, frame)
            saved += 1

        frame_idx += 1

    cap.release()

    avg_ms = sum(infer_times) / len(infer_times) if infer_times else 0
    print("==== 汇总 ====")
    print(f"验证帧数: {len(infer_times)}  检测目标总数: {total_det}")
    print(f"平均推理: {avg_ms:.1f} ms/帧  等效 FPS: {1000/avg_ms:.1f}")
    print(f"样例图目录: {OUT_DIR}")


if __name__ == "__main__":
    main()
