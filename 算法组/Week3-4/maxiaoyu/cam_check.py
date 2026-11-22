import cv2
import time

def camera_test(device_id=0, target_w=1280, target_h=720, target_fps=30):
    cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)

    # 1. 检查能否成功打开设备
    if not cap.isOpened():
        print("【FAIL】无法打开相机设备")
        return
    print("【OK】相机设备打开成功")

    # 关键：优先设置MJPG编码，再配置分辨率
    fourcc = cv2.VideoWriter_fourcc(*"MJPG")
    cap.set(cv2.CAP_PROP_FOURCC, fourcc)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, target_w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, target_h)

    # 获取实际生效分辨率
    real_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    real_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print(f"目标分辨率：{target_w} × {target_h}")
    print(f"实际分辨率：{real_w} × {real_h}")
    if real_w == target_w and real_h == target_h:
        print("【OK】分辨率符合要求")
    else:
        print("【WARN】实际分辨率和目标不一致")

    # 标称帧率，文档提示该值不可靠，仅作参考
    nominal_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"相机标称FPS: {nominal_fps}")

    # 2. 连续取帧，计时统计真实帧率（考核要求）
    frame_count = 0
    start_time = time.time()
    test_duration = 5

    print("开始连续取帧测试（5s）...")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("【FAIL】读取帧失败，相机断开/被占用")
            break
        frame_count += 1
        now = time.time()
        if now - start_time >= test_duration:
            break

    elapsed = time.time() - start_time
    real_fps = frame_count / elapsed
    print(f"测试时长 {elapsed:.2f} s，一共读取 {frame_count} 帧")
    print(f"【真实采集FPS】{real_fps:.2f}")
    if abs(real_fps - target_fps) < 3:
        print("【OK】帧率接近标称30fps")
    else:
        print("【WARN】帧率偏离30fps较多")

    cap.release()
    print("相机资源已释放，自检结束")

if __name__ == "__main__":
    camera_test(0,1280,720,30)


