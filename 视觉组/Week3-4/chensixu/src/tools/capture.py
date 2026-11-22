import cv2
import time
import os

# 注意：Windows 下用 0 通常就是默认摄像头
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW) 
os.makedirs("data", exist_ok=True)

if not cap.isOpened():
    print("❌ 无法打开摄像头！请检查是否被其他软件占用。")
    exit()

print("=====================================")
print("按键说明：")
print("按 's' : 拍照片（保存棋盘格标定图）")
print("按 'v' : 开始/停止录像（录制数据集）")
print("按 'q' : 退出程序")
print("=====================================")

frame_count = 0
recording = False
out = None

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ 读取不到画面！")
        break
        
    frame_display = cv2.resize(frame, (640, 480))
    cv2.imshow("Camera Capture", frame_display)
    
    key = cv2.waitKey(1) & 0xFF
    
    if key == ord('s'):
        frame_count += 1
        cv2.imwrite(f"data/img_{frame_count}.jpg", frame)
        print(f"✅ 已保存图片: data/img_{frame_count}.jpg")
        
    elif key == ord('v'):
        recording = not recording
        if recording:
            fourcc = cv2.VideoWriter_fourcc(*'XVID') # Windows 用 XVID 录像比较稳
            out = cv2.VideoWriter(f"data/video_{int(time.time())}.avi", fourcc, 30.0, (frame.shape[1], frame.shape[0]))
            print("🔴 开始录像...")
        else:
            if out is not None:
                out.release()
                out = None
            print("⏹️ 停止录像。")
            
    elif key == ord('q'):
        break
        
    if recording and out is not None:
        out.write(frame)

cap.release()
if out is not None:
    out.release()
cv2.destroyAllWindows()