"""
探测外接 USB 相机暴露的对焦/变焦/曝光属性（只读，不修改设备）。
判断这台相机是否支持自动对焦开关。
"""
import cv2

CAM_INDEX = 1
cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_MSMF)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# OpenCV 相机属性编号 -> 中文名
PROPS = {
    "CAP_PROP_AUTOFOCUS":  cv2.CAP_PROP_AUTOFOCUS,   # 自动对焦开关
    "CAP_PROP_FOCUS":      cv2.CAP_PROP_FOCUS,       # 对焦位置
    "CAP_PROP_ZOOM":       cv2.CAP_PROP_ZOOM,        # 变焦
    "CAP_PROP_AUTO_EXPOSURE": cv2.CAP_PROP_AUTO_EXPOSURE,
    "CAP_PROP_EXPOSURE":    cv2.CAP_PROP_EXPOSURE,
    "CAP_PROP_BRIGHTNESS": cv2.CAP_PROP_BRIGHTNESS,
    "CAP_PROP_AUTO_WB":    cv2.CAP_PROP_AUTO_WB,
}

print("相机是否打开:", cap.isOpened())
for name, pid in PROPS.items():
    val = cap.get(pid)  # 读当前值；不支持的属性通常返回 0 或 -1
    print(f"{name:24s} (id={pid:2d}) = {val}")

# 再读一帧确认画面正常
ok, frame = cap.read()
print("读到画面:", ok, "尺寸:", None if not ok else frame.shape)
cap.release()
