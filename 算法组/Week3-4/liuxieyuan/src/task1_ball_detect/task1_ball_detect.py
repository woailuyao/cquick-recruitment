import cv2
from ultralytics import YOLO
import time

# =========================================================
# 已经为你填好了准确的模型路径，不要修改
# ======================================================
model_path = r"C:\Users\15403\Desktop\新建文件夹\runs\detect\train-2\weights\best.pt"
model = YOLO(model_path)

# 打开外接摄像头（如果黑屏或打不开，把括号里的 0 改成 1）
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW) 

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    print("⚠️ 警告：无法打开摄像头，请检查设备占用或修改代码中的摄像头索引（0或1）！")
    exit()

prev_time = time.time()
frame_count = 0

print("摄像头已启动！按 'q' 键退出画面。")

while True:
    success, frame = cap.read()
    if not success:
        print("无法读取摄像头画面！")
        break
        
    frame_count += 1

    # 进行 YOLO 推理
    results = model(frame, conf=0.5, imgsz=640, device='cpu')
    
    for r in results:
        boxes = r.boxes
        for box in boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            
            center_u = (x1 + x2) // 2
            center_v = (y1 + y2) // 2
            
            conf = float(box.conf[0])
            
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, (center_u, center_v), 5, (0, 0, 255), -1)
            
            label = f"ball {conf:.2f} ({center_u}, {center_v})"
            (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(frame, (x1, y1 - 25), (x1 + w, y1), (0, 255, 0), -1)
            cv2.putText(frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

    curr_time = time.time()
    fps = 1.0 / (curr_time - prev_time)
    prev_time = curr_time
    
    fps_text = f"FPS: {fps:.1f}"
    cv2.putText(frame, fps_text, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    
    time_str = time.strftime("%Y-%m-%d %H:%M:%S")
    time_text = f"Frame: {frame_count} | {time_str}"
    cv2.putText(frame, time_text, (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    cv2.imshow("CQuick Task 1 - Ball Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()