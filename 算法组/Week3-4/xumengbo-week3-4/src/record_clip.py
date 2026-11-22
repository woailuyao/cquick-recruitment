import os
import re
import cv2

OUT_DIR = r"D:\W34_dataset\marks"   # 拍足球时改回 D:\W34_dataset\raw
os.makedirs(OUT_DIR, exist_ok=True)


def next_index(d):
    nums = []
    for f in os.listdir(d):
        m = re.match(r"clip_(\d{2})\.avi", f)
        if m:
            nums.append(int(m.group(1)))
    return max(nums) + 1 if nums else 1

CAM_INDEX = 1
WIDTH, HEIGHT, FPS = 1280, 720, 30

cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_MSMF)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
cap.set(cv2.CAP_PROP_FPS, FPS)

real_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
real_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"实际分辨率: {real_w} x {real_h}")

writer = None
recording = False
idx = next_index(OUT_DIR) - 1
print(f"下一个文件将从 clip_{idx + 1:02d}.avi 开始，不会覆盖已有视频")

while True:
    ok, frame = cap.read()
    if not ok:
        print("读取失败，相机可能被其他程序占用")
        break

    if recording and writer is not None:
        writer.write(frame)

    disp = frame.copy()
    if recording:
        cv2.circle(disp, (30, 30), 10, (0, 0, 255), -1)
        cv2.putText(disp, "REC  clip_%02d" % idx, (48, 38),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    cv2.imshow("USB Camera  (s=开始/停止  q=退出)", disp)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('s'):
        if not recording:
            idx += 1
            path = os.path.join(OUT_DIR, f"clip_{idx:02d}.avi")
            writer = cv2.VideoWriter(
                path,
                cv2.VideoWriter_fourcc(*"MJPG"),
                FPS,
                (real_w, real_h),
            )
            recording = True
            print(f"开始录制 -> {path}")
        else:
            recording = False
            writer.release()
            writer = None
            print("停止录制")

    if key == ord('q'):
        break

cap.release()
if writer is not None:
    writer.release()
cv2.destroyAllWindows()
print("已退出")
