
import cv2
import time
from ultralytics import YOLO
model = YOLO(r"D:\Soccer_Dataset\runs\detect\train\weights\best.pt")
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    print("摄像头打开失败")
    exit()

print(f"✅ 摄像头已打开！分辨率: {cap.get(cv2.CAP_PROP_FRAME_WIDTH)} x {cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")

prev_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    
    results = model(frame, conf=0.5, verbose=False)
    annotated_frame = results[0].plot()

    
    current_time = time.time()
    fps = 1 / (current_time - prev_time)
    prev_time = current_time
    cv2.putText(annotated_frame, f"FPS: {fps:.1f}", (20, 50), 
                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)

    
    boxes = results[0].boxes
    if len(boxes) > 0:
        for box in boxes:
            
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            conf = float(box.conf[0])
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            
            
            print(f" 检测到球 -> 框: [{int(x1)},{int(y1)},{int(x2)},{int(y2)}], 中心点: ({cx},{cy}), 置信度: {conf:.2f}")

    
    cv2.imshow("CQuick Ball Detection", annotated_frame)

    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()