"""
从视频中抽帧用于相机标定
- 逐帧播放，空格保存当前帧为 PNG
- A / D 前后跳 1 秒
- 左右方向键前后跳 1 帧
- 保存出来的就是视频原始分辨率，保证和推理一致
"""
import cv2
import os
import sys

# ============================================================
# ===== 改这里 =====
# ============================================================
VIDEO_PATH = "task1_qipaige.mp4"     # 你的棋盘格视频文件
OUT_DIR    = "images/calib"        # 抽帧保存目录
# ============================================================

def main():
    if not os.path.exists(VIDEO_PATH):
        print(f"[ERROR] 找不到视频: {VIDEO_PATH}")
        return

    os.makedirs(OUT_DIR, exist_ok=True)

    cap = cv2.VideoCapture(VIDEO_PATH)
    if not cap.isOpened():
        print("[ERROR] 视频打不开")
        return

    # 视频原始尺寸 & 总帧数
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    print(f"[INFO] 视频: {W}x{H}  {total} 帧  {fps:.1f} fps")
    print(f"       抽帧会保存为 {W}x{H} PNG（与推理分辨率一致）")

    idx = 0
    saved = 0
    # 已有文件数量
    existing = [f for f in os.listdir(OUT_DIR) if f.lower().endswith(".png")]
    saved = len(existing)

    win = "Extract - SPACE=save  A/D=+-1s  <-/->=+-1frame  Q=quit"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    # 显示窗口和原始尺寸一致，避免缩放
    cv2.resizeWindow(win, min(W, 1280), min(H, 720))

    while True:
        cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        ok, frame = cap.read()
        if not ok:
            print(f"[INFO] 到达末尾 (idx={idx})")
            break

        # 检查尺寸一致（防御性）
        if frame.shape[1] != W or frame.shape[0] != H:
            print(f"[WARN] 帧尺寸异常: {frame.shape}")

        vis = frame.copy()
        cv2.putText(vis,
            f"frame={idx}/{total-1}  t={idx/fps:.2f}s  saved={saved}",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(vis,
            f"size={W}x{H}   SPACE=save  A/D=+-1s  <-/-> = +-1frame  Q=quit",
            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        cv2.imshow(win, vis)
        key = cv2.waitKey(0) & 0xFF

        if key == ord('q'):
            break
        elif key == 32:  # SPACE
            fn = os.path.join(OUT_DIR, f"calib_{saved:03d}.png")
            cv2.imwrite(fn, frame)   # ← 保存原始帧，不缩放
            print(f"[SAVE] {fn}  (frame {idx})")
            saved += 1
        elif key == ord('d'):
            idx = min(total - 1, idx + int(fps))
        elif key == ord('a'):
            idx = max(0, idx - int(fps))
        elif key == 83 or key == 3:   # 右方向键（不同平台码值可能不同）
            idx = min(total - 1, idx + 1)
        elif key == 81 or key == 2:   # 左方向键
            idx = max(0, idx - 1)
        else:
            pass

    cap.release()
    cv2.destroyAllWindows()
    print(f"\n[完成] 共保存 {saved} 张 → {OUT_DIR}/")
    print("        检查所有图片应为原始分辨率（脚本打印的 W x H）")


if __name__ == "__main__":
    main()