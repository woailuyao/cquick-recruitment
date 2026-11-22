"""
外参反算 · 实时画面点选自检（check_extrinsics_live）
====================================================
在实时相机画面上直接点击已知地面标记，程序用外参反算地面坐标，
和卷尺实测真实坐标对比，输出平面误差。纯投影，不依赖模型检测。

现场步骤：
  1. 地面放标记，卷尺量出真实 (X_true, Y_true)，填进下面 KNOWN；
  2. 运行：python check_extrinsics_live.py
  3. 在画面里点某个标记，再按数字键 1/2/3 绑定到对应已知点；
  4. 按 q 退出，控制台打印平均/最大误差。

按键：
  1..9  把最近一次点击绑定到 KNOWN 第 N 个点并反算
  d     删除最后一条结果
  r     清空全部结果
  s     保存当前画面到 D:\\loc_demo_snapshots
  q/Esc 退出并打印汇总
"""
import os
import sys
import math
import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import loc_common
import ground_projection

# ===== 现场填：已知标记真实坐标 (名称, 真实X m, 真实Y m) =====
# 名称用英文/数字，避免画面中文乱码；最多 9 个（对应按键 1..9）。
KNOWN = [
    ("P1", 0.0, 0.0),
    ("P2", 0.0, 2.0),
    ("P3", 0.0, 3.5),
]

SOURCE = 1                       # 整数=相机编号；内置摄像头试 0
CAM_BACKEND = cv2.CAP_MSMF
IMG_W, IMG_H = 1280, 720
FRAME_ROTATE = 0                # 0=不旋转；180=上下颠倒转 180°
FRAME_FLIP = 0
SNAP_DIR = r"D:\loc_demo_snapshots"

# 最近一次点击（像素，校正后画面坐标）
clicked = None                   # (u, v)
results = []                     # 已绑定结果列表


def correct_frame(frame):
    """与 field_locator.py 同一套方向校正，保证取点方向一致。"""
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


def on_mouse(event, u, v, flags, param):
    """鼠标左键点击，记录最近一次像素坐标。"""
    global clicked
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked = (u, v)


def bind_point(idx, projector):
    """把最近一次点击绑定到 KNOWN[idx]，反算地面点并记录。"""
    global clicked
    if clicked is None:
        return
    if idx < 0 or idx >= len(KNOWN):
        return
    name, xt, yt = KNOWN[idx]
    u, v = clicked
    res = projector.project_one(u, v)
    if res["valid"]:
        xb, yb = res["x_b"], res["y_b"]
        err = math.hypot(xb - xt, yb - yt)
        results.append({"name": name, "xt": xt, "yt": yt, "u": u, "v": v,
                        "xb": xb, "yb": yb, "err": err,
                        "valid": True, "reason": "ok"})
    else:
        results.append({"name": name, "xt": xt, "yt": yt, "u": u, "v": v,
                        "xb": None, "yb": None, "err": None,
                        "valid": False, "reason": res["reason"]})
    clicked = None


def draw(frame):
    """在画面上画最近点击、已绑定结果和操作提示。"""
    # 最近点击：黄色十字 + 像素坐标
    if clicked is not None:
        u, v = clicked
        cv2.drawMarker(frame, (u, v), (0, 255, 255),
                       cv2.MARKER_CROSS, 18, 2)
        cv2.putText(frame, f"last=({u},{v})", (u + 10, v),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    # 已绑定结果：逐行打印；误差 <=0.5m 绿色，否则红色
    y = 24
    for r in results:
        if r["valid"]:
            txt = (f"{r['name']} ({r['xt']:.2f},{r['yt']:.2f}) -> "
                   f"({r['xb']:.2f},{r['yb']:.2f}) err={r['err']*100:.1f}cm")
            color = (0, 255, 0) if r["err"] <= 0.5 else (0, 0, 255)
        else:
            txt = f"{r['name']} invalid: {r['reason']}"
            color = (0, 0, 255)
        cv2.putText(frame, txt, (12, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        y += 22

    # 底部操作提示
    cv2.putText(frame,
                "click marker, press 1..9 to bind | d del | r clear | s snap | q quit",
                (12, IMG_H - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
    return frame


def main():
    os.makedirs(SNAP_DIR, exist_ok=True)

    # 1. 加载内参 / 外参 / 投影器
    intrinsics = loc_common.Intrinsics.from_config()
    extrinsics = loc_common.Extrinsics.from_config()
    if not extrinsics.measured:
        print("extrinsics not measured (measured=False); fill measurements first.")
        return
    projector = ground_projection.GroundProjector(intrinsics, extrinsics,
                                                  field_half=(3.9, 3.0))

    # 2. 打开相机
    cap = cv2.VideoCapture(SOURCE, CAM_BACKEND)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_H)
    if not cap.isOpened():
        raise RuntimeError("cannot open camera; try SOURCE=0 (built-in).")

    win = "Extrinsics Live Check"
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)

    frame_idx = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frame = correct_frame(frame)
        frame_idx += 1
        draw(frame)
        cv2.imshow(win, frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break
        elif key == ord("d") and results:
            results.pop()
        elif key == ord("r"):
            results.clear()
        elif key == ord("s"):
            snap = os.path.join(SNAP_DIR, f"live_{frame_idx:05d}.jpg")
            cv2.imwrite(snap, frame)
            print("snapshot:", snap)
        elif ord("1") <= key <= ord("9"):
            bind_point(key - ord("1"), projector)

    cap.release()
    cv2.destroyAllWindows()

    # 3. 退出汇总
    errs = [r["err"] for r in results if r["valid"]]
    print("==== summary ====")
    for r in results:
        if r["valid"]:
            print(f"{r['name']:>4} true=({r['xt']:.2f},{r['yt']:.2f}) "
                  f"proj=({r['xb']:.2f},{r['yb']:.2f}) err={r['err']*100:.1f}cm")
        else:
            print(f"{r['name']:>4} invalid: {r['reason']}")
    if errs:
        print(f"mean={sum(errs)/len(errs)*100:.1f}cm  "
              f"max={max(errs)*100:.1f}cm  n={len(errs)}")
    else:
        print("no valid bound points.")


if __name__ == "__main__":
    main()
