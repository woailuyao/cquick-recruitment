import cv2 as cv
import sys

cap = cv.VideoCapture("data/video/test.mp4")
if not cap.isOpened():
    sys.exit("视频打开失败，请检查路径")
lower_color = (105, 20, 50)
upper_color = (145, 255, 255)
min_area = 5000

cv.namedWindow("frame", cv.WINDOW_NORMAL)
cv.resizeWindow("frame", 360, 640)
cv.namedWindow("mask", cv.WINDOW_NORMAL)
cv.resizeWindow("mask", 360, 640)
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("视频播放结束或无法继续读取")
        break
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lower_color, upper_color)
    opened = cv.morphologyEx(mask, cv.MORPH_OPEN, None)
    cleaned = cv.morphologyEx(opened, cv.MORPH_CLOSE, None)
    contours, hierarchy = cv.findContours(cleaned, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv.contourArea(cnt)
        if area < min_area:
            continue
        x, y, w, h = cv.boundingRect(cnt)
        cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
        M = cv.moments(cnt)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv.circle(frame, (cx, cy), 7, (0, 0, 255), -1)
    cv.imshow("frame", frame)
    cv.imshow("mask", cleaned)
    if cv.waitKey(33) == ord("q"):
        break

cap.release()
cv.destroyAllWindows()