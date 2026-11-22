"""
任务 1 实时足球检测脚本
========================
运行（必须用 Python 3.10 全路径解释器，或 PyCharm 选 3.10 SDK）：
    python detect_realtime.py

输入切换：
    SOURCE = 1   -> 打开编号 1 的外接 USB 相机（MSMF 后端）
    SOURCE = "D:/xxx.mp4" -> 读取本地视频文件（先离线验证模型用）

画面输出（对照任务书 1.7）：
    - 检测框 x1,y1,x2,y2
    - 类别名 ball、置信度 confidence
    - 框中心像素坐标 (center_u, center_v)
    - 帧号 / 时间戳
    - 实时 FPS、单帧推理耗时
按键：q 或 Esc 退出；s 保存当前帧截图（存到纯英文目录，避免中文路径写失败）
"""
import os
import time
import cv2
from ultralytics import YOLO

# ===== 可调配置 =====
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # 项目根目录
WEIGHTS = os.path.join(_ROOT, "runs", "ball", "train_v2", "weights", "best.pt")
# 现场先跑 probe_camera.py：看哪个索引 OK 且画面是外接相机，把下面 SOURCE 改成该索引。
# 注意：必须用采集训练时的同一台外接 USB 相机，不要用内置相机录屏。
SOURCE = 1                 # 整数=相机编号；改成视频文件路径字符串即可读视频
CAM_BACKEND = cv2.CAP_MSMF  # Windows 外接 USB 相机用 MSMF；内置相机可换 CAP_DSHOW
IMG_W, IMG_H = 1280, 720   # 与采集/标定一致的分辨率
CONF = 0.35                # 置信度阈值：调高减少误检，调低减少漏检，可按实测调整
IOU = 0.45                 # NMS  IoU 阈值，去掉同一颗球上的重复框
DEVICE = 0                 # 0=用 NVIDIA GPU；无 GPU 改 "cpu"
SNAP_DIR = r"D:\ball_demo_snapshots"  # 截图保存目录（纯英文）

# ===== 画面方向校正（相机在三脚架上装倒/装斜时使用）=====
# FRAME_ROTATE：读到帧后先旋转。0=不转；180=上下颠倒转 180°；
#               90=顺时针转 90°；270(或 -90)=逆时针转 90°
# FRAME_FLIP：  旋转之后再翻转。0=不翻；1=左右镜像；-1=上下翻转
# 先在实时窗口里看清画面到底怎么倒，再选对应取值；校正只改画面，不改相机。
FRAME_ROTATE = 0
FRAME_FLIP = 0


def correct_frame(frame):
    """按上面的开关对原始帧做旋转/翻转校正，返回校正后的画面。"""
    # 1. 先按角度旋转
    if FRAME_ROTATE == 180:
        frame = cv2.rotate(frame, cv2.ROTATE_180)
    elif FRAME_ROTATE == 90:
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    elif FRAME_ROTATE == 270 or FRAME_ROTATE == -90:
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    # 2. 再按需要做镜像翻转（flipCode: 1=水平左右, 0=垂直上下）
    if FRAME_FLIP == 1:
        frame = cv2.flip(frame, 1)
    elif FRAME_FLIP == -1:
        frame = cv2.flip(frame, 0)
    return frame


def draw_detections(frame, results):
    """把检测结果画到帧上，返回本帧检测到的目标列表。"""
    r = results[0]
    boxes = r.boxes
    detected = []
    h = frame.shape[0]            # 画面高度，用于角点文字贴边防越界

    for b in boxes:
        # xyxy: 左上角 (x1,y1) 右下角 (x2,y2)，像素坐标
        x1, y1, x2, y2 = b.xyxy[0].tolist()
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        conf = float(b.conf[0])          # 置信度 0~1
        cls_id = int(b.cls[0])           # 类别编号，本任务恒为 0
        name = r.names[cls_id]           # 类别名 ball

        # 框中心像素坐标：u=水平中心，v=垂直中心
        u = (x1 + x2) / 2.0
        v = (y1 + y2) / 2.0

        # 画检测框（绿色）
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        # 框上方第一行：类别名 + 置信度
        cv2.putText(frame, f"{name} {conf:.2f}", (x1, max(y1 - 8, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        # 框下方第二行：检测框角点 x1,y1,x2,y2（任务书 1.7 要求输出）
        cv2.putText(frame, f"box:({x1},{y1})-({x2},{y2})", (x1, min(y2 + 18, h - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
        # 中心点（红色）+ 中心像素坐标
        cv2.circle(frame, (int(u), int(v)), 4, (0, 0, 255), -1)
        cv2.putText(frame, f"({u:.0f},{v:.0f})", (int(u) + 8, int(v) + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        detected.append({"name": name, "conf": round(conf, 3),
                         "box": [x1, y1, x2, y2], "center": [round(u, 1), round(v, 1)]})

    return detected


def main():
    os.makedirs(SNAP_DIR, exist_ok=True)

    # 1. 加载训练好的权重
    model = YOLO(WEIGHTS)

    # 2. 打开输入：整数按相机打开，字符串按视频文件打开
    if isinstance(SOURCE, int):
        cap = cv2.VideoCapture(SOURCE, CAM_BACKEND)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_H)
    else:
        cap = cv2.VideoCapture(SOURCE)

    if not cap.isOpened():
        raise RuntimeError("无法打开相机/视频：检查相机编号、是否被占用，或视频路径是否正确")

    frame_idx = 0
    fps_smooth = 0.0          # 平滑后的 FPS
    prev_t = time.perf_counter()

    while True:
        ok, frame = cap.read()
        if not ok:
            print("读不到画面（相机断开或视频已结束）")
            break
        frame = correct_frame(frame)   # 先做方向校正，再推理和画框
        frame_idx += 1

        # 3. 逐帧推理；verbose=False 关闭 ultralytics 每帧刷屏
        t0 = time.perf_counter()
        results = model.predict(
            frame, conf=CONF, iou=IOU, device=DEVICE, imgsz=640, verbose=False
        )
        infer_ms = (time.perf_counter() - t0) * 1000.0

        # 4. 画框并取本帧检测结果
        detected = draw_detections(frame, results)

        # 5. 计算 FPS（指数滑动平均，避免数字跳动太大）
        now = time.perf_counter()
        inst_fps = 1.0 / max(now - prev_t, 1e-6)
        prev_t = now
        fps_smooth = fps_smooth * 0.9 + inst_fps * 0.1

        # 6. 左上角信息栏：FPS / 推理耗时 / 帧号 / 分辨率 / 时间戳
        h, w = frame.shape[:2]
        ts = time.strftime("%H:%M:%S")
        info = (f"FPS {fps_smooth:.1f}  infer {infer_ms:.1f}ms  "
                f"frame {frame_idx}  {w}x{h}  {ts}")
        cv2.putText(frame, info, (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # 终端同步打印本帧检测（便于录屏留证）
        if detected:
            print(f"frame {frame_idx}  t={ts}  n={len(detected)}  {detected}")

        cv2.imshow("Ball Detection  (q: quit, s: save)", frame)

        # 7. 按键处理：1ms 延时刷新窗口
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:      # q 或 Esc 退出
            break
        if key == ord("s"):                    # s 保存截图
            snap = os.path.join(SNAP_DIR, f"frame_{frame_idx:05d}.jpg")
            cv2.imwrite(snap, frame)
            print(f"截图已保存: {snap}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
