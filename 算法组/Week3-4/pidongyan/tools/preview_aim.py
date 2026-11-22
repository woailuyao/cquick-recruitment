"""现场摆机位辅助：实时相机预览 + 画面中心十字，用来把光轴参考物调到正中心。

只预览，不写视频、不写任何观测点、不做检测，纯现场取景用。

用法（PowerShell 单行）：
  python tools/preview_aim.py --index 0

快捷键：q 退出，s 存一张预览图到 check_out/aim_preview_<时间>.jpg

现场判断标准（只有两条，围挡入画不算问题）：
  1) 白色小盒子（光轴落地点 P）落在红色十字中心圈内，偏差尽量 <30 px；
  2) 肉眼数得出 >=4 个白线交点（L 角 / T 交点 / 中圈 X / 点球点）。

为什么围挡/窗户占上半屏是正常的、不要去消除：
  h=0.55 m、白盒在 3.77 m 处 -> 光轴俯角 8.3°；
  720p 配合 fy≈1620 的竖直半视场是 12.5°，画面上缘的俯角约 -4.2°，
  必然拍到地平线以上的围挡。想把围挡压出画面需要俯角 >20°，
  但那样远处标志点全被挤出画面，且与标定外参/内参不一致，反而更糟。
"""
import argparse
import datetime as dt
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def open_cam(index, width, height, fourcc):
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        sys.exit(f"错误：打不开相机 index={index}。检查是否被其它程序占用，或换 --index。")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    return cap


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--index", type=int, default=0)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fourcc", default="MJPG")
    args = ap.parse_args()

    cap = open_cam(args.index, args.width, args.height, args.fourcc)
    print("预览中：白盒调进红色中心圈、数出 >=4 个白线交点（围挡入画属正常，不用消除）；q 退出，s 截图")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("[警告] 读帧失败，重试")
                continue
            vis = frame.copy()
            h, w = vis.shape[:2]
            cx, cy = w // 2, h // 2
            cv2.line(vis, (cx, 0), (cx, h), (0, 0, 255), 1)
            cv2.line(vis, (0, cy), (w, cy), (0, 0, 255), 1)
            cv2.circle(vis, (cx, cy), 14, (0, 0, 255), 2)
            cv2.circle(vis, (cx, cy), 40, (0, 0, 255), 1)
            cv2.putText(vis, "AIM: put white box here", (cx + 46, cy - 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
            # 顶部提示条
            cv2.rectangle(vis, (0, 0), (w, 34), (0, 0, 0), -1)
            cv2.putText(vis, "q=quit  s=shot   put white box in red circle   count >=4 crossings",
                        (10, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 255), 2)
            cv2.imshow("aim preview", vis)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                out = ROOT / "check_out" / (
                    "aim_preview_" + dt.datetime.now().strftime("%Y%m%d_%H%M%S") + ".jpg")
                out.parent.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(out), vis)
                print("截图：", out)
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
