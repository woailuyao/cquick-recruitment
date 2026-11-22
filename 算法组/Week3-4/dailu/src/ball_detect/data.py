import cv2 as cv
#1.打开摄像头
camera_id = 2
cap = cv.VideoCapture(camera_id)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv.CAP_PROP_FPS, 30)


if not cap.isOpened():
    print("打开不了摄像头，请检测路径!")
else:
    print("正常打开摄像头!")

    file_name = 'task1_qipaige.mp4'
    fourcc = cv.VideoWriter_fourcc(*'mp4v')
    out = cv.VideoWriter(file_name, fourcc, 30.0, (1280, 720))
    print(f"正在录制，将是平保存在：{file_name}")
    print("按下'q'键退出")
    while  True:
        ret, frame = cap.read()
        if not ret:
            break

        cv.imshow("Test Camera", frame)
        cv.circle(frame, (30,30), 10, (0,0,250), -1)
        out.write(frame)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
out.release()
cv.detroyAllWindows()