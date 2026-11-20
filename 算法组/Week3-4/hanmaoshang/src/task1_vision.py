import cv2
from ultralytics import YOLO
import time

# 加载你亲手训练的终极模型
model = YOLO('best.pt')

# 打开外接摄像头
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    print("❌ 摄像头打不开")
    exit()

print("✅ 实时检测已启动！按 'q' 退出。")

prev_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret: break

    # 交给 AI 识别
    results = model.predict(frame, conf=0.5, verbose=False)

    # 获取画了框的画面
    annotated_frame = results[0].plot()
    
    # 计算 FPS
    curr_time = time.time()
    fps = 1.0 / (curr_time - prev_time)
    prev_time = curr_time

    # 在画面上显示 FPS
    cv2.putText(annotated_frame, f"FPS: {fps:.1f}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("Real-Time Ball Detection", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()