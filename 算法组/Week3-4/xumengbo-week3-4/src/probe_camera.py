import cv2
import os

OUT_DIR = r"D:\W34_dataset\probe"
os.makedirs(OUT_DIR, exist_ok=True)
BACKENDS = [("MSMF", cv2.CAP_MSMF), ("DSHOW", cv2.CAP_DSHOW)]


def save_jpg(path, image):
    ok, buf = cv2.imencode(".jpg", image)
    if not ok:
        print("    编码失败")
        return
    with open(path, "wb") as f:
        f.write(buf.tobytes())
    print(f"    已保存 {os.path.basename(path)}")


for name, backend in BACKENDS:
    print(f"=== 后端 {name} ===")
    for i in range(5):
        cap = cv2.VideoCapture(i, backend)
        if not cap.isOpened():
            print(f"  index {i}: 打不开")
            cap.release()
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        ok, frame = cap.read()
        if ok:
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(f"  index {i}: OK {w}x{h}")
            save_jpg(os.path.join(OUT_DIR, f"{name}_cam_{i}.jpg"), frame)
        else:
            print(f"  index {i}: 能打开但读不到画面")
        cap.release()
