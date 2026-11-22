
"""
任务 2 单目场地自定位 · 主链路 + 左右分屏界面（field_locator）
=================================================================
链路：相机帧 -> 标志检测 -> 几何细化取点 -> 去畸变反投影地面求交
      -> 地图匹配求位姿 -> 左 RGB / 右地图俯视分屏显示。

前置条件（未就位时脚本会明确提示，不伪造结果）：
  1. 标志模型权重：runs/markers/train/weights/best.pt（先跑 train_markers.py）
  2. 外参已现场实测：calibration/camera_extrinsics.yaml 三次测量非空
  3. 地图：联调先用 field_theory.yaml；定稿后切换为组长 C0 版 field.yaml

运行（Python 3.10 全路径）：
    python field_locator.py
按键：q / Esc 退出；s 保存当前分屏截图（纯英文目录）
"""
import os
import time
import cv2
import numpy as np
from ultralytics import YOLO

import loc_common
import landmark
import ground_projection
import map_matcher

# ===== 可调配置 =====
MARKER_WEIGHTS = os.path.join(loc_common.BASE_DIR, "runs", "markers", "train", "weights", "best.pt")
SOURCE = 1                       # 整数=相机编号；字符串=视频文件路径
CAM_BACKEND = cv2.CAP_MSMF
IMG_W, IMG_H = 1280, 720
CONF = 0.35
IOU = 0.45
DEVICE = 0
SNAP_DIR = r"D:\loc_demo_snapshots"

# 地图源：联调用推算草稿；定稿后改为 os.path.join(CONFIG_DIR, "field.yaml")
MAP_YAML = os.path.join(loc_common.CONFIG_DIR, "field_theory.yaml")

# 方向校正（与采集脚本保持一致）
FRAME_ROTATE = 0
FRAME_FLIP = 0

# 位姿滤波：EMA 系数；定位失败帧不沿用旧位姿
EMA_ALPHA = 0.4

# 分屏半幅尺寸：左右各 PANEL_W 宽，拼成 2*PANEL_W 宽，确保整幅落在屏幕可视宽度内。
PANEL_W, PANEL_H = 760, 720
WIN_NAME = "Field Locator  (q: quit, s: save)"


def correct_frame(frame):
    """与 detect_realtime.py 同一套方向校正。"""
    if FRAME_ROTATE == 180:
        frame = cv2.rotate(frame, cv2.ROTATE_180)
    elif FRAME_ROTATE == 90:
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    elif FRAME_ROTATE == 270 or FRAME_ROTATE == -90:
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    if FRAME_FLIP == 1:
        frame = cv2.flip(frame, 1)
    elif FRAME_FLIP == -1:
        frame = cv2.flip(frame, 0)
    return frame


def draw_left(frame, marks):
    """左画面：叠加标志框、特征点、类别与置信度。"""
    for m in marks:
        x1, y1, x2, y2 = m["box"]
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(frame, (int(m["u"]), int(m["v"])), 4, (0, 0, 255), -1)
        cv2.putText(frame, f"{m['type']} {m['conf']:.2f}", (x1, max(y1 - 8, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return frame


def draw_map(canvas, field_map, carrier_pose, status, marker_count, residual, valid):
    """右画面：场地俯视 + 相机图标与朝向箭头 + 状态读数。"""
    h, w = canvas.shape[:2]
    # 场地 7.8 x 6.0，留边距；X/Y 共用同一比例尺，避免圆被拉成椭圆、点位偏离圆周
    margin = 40
    scale = min((w - 2 * margin) / 7.8, (h - 2 * margin) / 6.0)

    def to_px(x, y):
        # 地图原点居中，+X 向右，+Y 向上（画布 y 向下，故取反）
        return (int(w / 2 + x * scale), int(h / 2 - y * scale))

    # 外框
    cv2.rectangle(canvas, to_px(-3.9, 3.0), to_px(3.9, -3.0), (255, 255, 255), 1)
    # 中线
    cv2.line(canvas, to_px(0, 3.0), to_px(0, -3.0), (255, 255, 255), 1)
    # 中圈
    cv2.circle(canvas, to_px(0, 0), round(0.72 * scale), (255, 255, 255), 1)
    # 地图点（按类别画小标记）
    for mtype, pts in field_map.markers.items():
        for name, x, y in pts:
            cv2.circle(canvas, to_px(x, y), 3, (180, 180, 180), -1)

    # 相机位姿图标 + 朝向箭头
    if carrier_pose is not None:
        cx, cy, yaw = carrier_pose
        px, py = to_px(cx, cy)
        a = np.radians(yaw)
        # 箭头长度按朝向在地图平面投影
        dx, dy = np.cos(a), np.sin(a)
        cv2.arrowedLine(canvas, (px, py),
                        (int(px + dx * 30), int(py - dy * 30)), (0, 255, 255), 2)
        cv2.circle(canvas, (px, py), 6, (0, 255, 255), -1)

    # 状态读数
    color = (0, 255, 0) if valid else (0, 0, 255)
    cv2.putText(canvas, f"status: {status}", (12, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)
    cv2.putText(canvas, f"markers: {marker_count}  residual: {residual}", (12, 44),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    # 载体在地图坐标系下的位姿读数；无效状态显示占位
    if carrier_pose is not None:
        cv2.putText(canvas, f"x: {cx:.2f}  y: {cy:.2f}  yaw: {yaw:.1f}",
                    (12, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1)
    else:
        cv2.putText(canvas, "x: --  y: --  yaw: --", (12, 64),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
    return canvas


def main():
    os.makedirs(SNAP_DIR, exist_ok=True)

    # 1. 加载内参 / 外参 / 地图
    intrinsics = loc_common.Intrinsics.from_config()
    extrinsics = loc_common.Extrinsics.from_config()
    field_map = loc_common.FieldMap.from_yaml(MAP_YAML)
    projector = ground_projection.GroundProjector(intrinsics, extrinsics)

    # 2. 加载标志模型；权重不存在则明确提示先训练
    if not os.path.exists(MARKER_WEIGHTS):
        raise SystemExit(
            f"标志模型权重不存在: {MARKER_WEIGHTS}\n"
            "请先完成标志标注并运行 train_markers.py 生成权重。")
    model = YOLO(MARKER_WEIGHTS)
    names = model.names

    # 3. 打开输入
    if isinstance(SOURCE, int):
        cap = cv2.VideoCapture(SOURCE, CAM_BACKEND)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_H)
    else:
        cap = cv2.VideoCapture(SOURCE)
    if not cap.isOpened():
        raise RuntimeError("无法打开相机/视频")

    frame_idx = 0
    fps_smooth = 0.0
    prev_t = time.perf_counter()
    filtered_pose = None     # 滤波后位姿；失败帧清空，不沿用旧值

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frame = correct_frame(frame)
        frame_idx += 1

        # 4. 检测 -> 取点 -> 地面投影 -> 匹配
        results = model.predict(frame, conf=CONF, iou=IOU, device=DEVICE,
                                imgsz=640, verbose=False)
        marks = landmark.landmark_list(results, names)
        projected = projector.project_many(marks)
        match_res = map_matcher.match(projected, field_map)

        # 5. 位姿滤波：仅在当前帧有效时做 EMA；无效则清空滤波位姿
        pose = match_res["carrier_pose"]
        if match_res["valid"] and pose is not None:
            if filtered_pose is None:
                filtered_pose = pose
            else:
                filtered_pose = tuple(
                    EMA_ALPHA * p + (1 - EMA_ALPHA) * f
                    for p, f in zip(pose, filtered_pose))
        else:
            filtered_pose = None

        # 6. 左右分屏：左帧对齐到半幅尺寸，右地图画布同尺寸，拼成整幅
        left = cv2.resize(frame.copy(), (PANEL_W, PANEL_H))
        left = draw_left(left, marks)
        right = np.zeros((PANEL_H, PANEL_W, 3), dtype=np.uint8)
        right = draw_map(right, field_map, filtered_pose, match_res["status"],
                         match_res["marker_count"], match_res["residual"],
                         match_res["valid"])
        combo_full = np.hstack([left, right])

        # FPS（画在 combo 上）
        now = time.perf_counter()
        inst_fps = 1.0 / max(now - prev_t, 1e-6)
        prev_t = now
        fps_smooth = fps_smooth * 0.9 + inst_fps * 0.1
        cv2.putText(combo_full, f"FPS {fps_smooth:.1f}",
                    (10, combo_full.shape[0] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # 窗口强制按 combo 位图尺寸显示，避免内容被屏幕裁掉
        cv2.namedWindow(WIN_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WIN_NAME, combo_full.shape[1], combo_full.shape[0])
        cv2.imshow(WIN_NAME, combo_full)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break
        if key == ord("s"):
            snap = os.path.join(SNAP_DIR, f"loc_{frame_idx:05d}.jpg")
            cv2.imwrite(snap, combo_full)
            print(f"截图已保存: {snap}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
